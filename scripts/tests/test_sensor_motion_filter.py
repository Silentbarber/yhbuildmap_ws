#!/usr/bin/env python3
import pathlib
import sys
import hashlib
import json
import subprocess
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from filter_sensor_motion import add_votes, additional_mask, motion_free_votes, sensor_transform
from evaluate_independent_run import write_pcd
from filter_dynamic_map import read_map


def target_returns(source_point, target_origin, background_range=3):
    direction = source_point-target_origin
    direction /= np.linalg.norm(direction)
    tangent = np.cross(direction, [0, 0, 1])
    tangent /= np.linalg.norm(tangent)
    vertical = np.cross(direction, tangent)
    offsets = [(1, 0), (-.5, .866), (-.5, -.866)]
    rays = np.array([direction+.001*(a*tangent+b*vertical) for a, b in offsets])
    rays /= np.linalg.norm(rays, axis=1)[:, None]
    body = np.array([1., 0, 0])+target_origin
    return np.vstack((body, target_origin+background_range*rays))


class SensorMotionFilterTest(unittest.TestCase):
    def test_attached_surface_with_measured_empty_old_position_votes(self):
        point = np.array([[1., 0, 0]])
        origin = np.array([0., .6, 0])
        accepted, counts = motion_free_votes(point, target_returns(point[0], origin),
            np.eye(3), np.zeros(3), np.eye(3), origin)
        self.assertTrue(accepted[0], counts)

    def test_static_return_rejects_device_frame_alias(self):
        point = np.array([[1., 0, 0]])
        origin = np.array([0., .6, 0])
        target = np.vstack((target_returns(point[0], origin), point))
        accepted, _ = motion_free_votes(point, target, np.eye(3), np.zeros(3), np.eye(3), origin)
        self.assertFalse(accepted[0])

    def test_occlusion_is_not_empty_space(self):
        point = np.array([[1., 0, 0]])
        origin = np.array([0., .6, 0])
        accepted, counts = motion_free_votes(point, target_returns(point[0], origin, .5),
            np.eye(3), np.zeros(3), np.eye(3), origin)
        self.assertEqual(counts['motion_matches'], 1)
        self.assertFalse(accepted[0])

    def test_missing_ray_triangle_cannot_vote(self):
        point = np.array([[1., 0, 0]])
        origin = np.array([0., .6, 0])
        target = target_returns(point[0], origin)[:3]
        accepted, _ = motion_free_votes(point, target, np.eye(3), np.zeros(3), np.eye(3), origin)
        self.assertFalse(accepted[0])

    def test_sensor_rotation_and_fixed_translation_offset_cancel(self):
        a = Rotation.from_euler('xyz', [10, -20, 35], degrees=True).as_matrix()
        b = Rotation.from_euler('xyz', [-30, 15, 75], degrees=True).as_matrix()
        offset = np.array([.10, -.03, .05])
        position_a, position_b = np.array([1, 2, 3]), np.array([1.2, 1.8, 3.1])
        local = np.array([[.7, .2, -.1], [1., -.3, .2]])
        source = local@a.T + position_a + a@offset
        expected = local@b.T + position_b + b@offset
        matrix = sensor_transform(a, position_a+a@offset, b, position_b+b@offset)
        np.testing.assert_allclose(source@matrix[:3, :3].T+matrix[:3, 3], expected, atol=1e-12)

    def test_votes_need_distinct_time_bins_and_span(self):
        votes = np.zeros(1, np.uint16)
        last_bin = np.full(1, -1, np.int32)
        first, last = np.full(1, np.inf), np.full(1, -np.inf)
        for seconds in [.60, .61, .62]:
            add_votes(np.array([0, 0]), 1, seconds, votes, last_bin, first, last)
        self.assertEqual(votes[0], 1)
        mask = additional_mask(np.array([False]), np.array([1]), votes, first, last)
        self.assertFalse(mask[0])
        add_votes(np.array([0]), 2, 1.2, votes, last_bin, first, last)
        add_votes(np.array([0]), 3, 1.8, votes, last_bin, first, last)
        self.assertTrue(additional_mask(np.array([False]), np.array([1]), votes, first, last)[0])

    def test_repeatedly_supported_and_already_removed_points_are_protected(self):
        mask = additional_mask(np.array([False, False, True]), np.array([1, 4, 0]),
            np.array([4, 4, 4]), np.zeros(3), np.full(3, 3.), max_hits=3)
        np.testing.assert_array_equal(mask, [True, False, False])


class SensorMotionCommandTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temp.name)
        self.frontend, self.backend, self.visibility = [self.root/name for name in ['frontend', 'backend', 'visibility']]
        (self.frontend/'capture').mkdir(parents=True)
        self.backend.mkdir()
        self.visibility.mkdir()
        self.stamps = np.array([10., 10.6, 11.2, 11.8])
        origins = [np.array([0., float(stamp-10.), 0]) for stamp in self.stamps]
        poses = [[stamp, *origin, 0, 0, 0, 1] for stamp, origin in zip(self.stamps, origins)]
        for path in [self.frontend/'capture/trajectory.csv', self.backend/'trajectory.csv']:
            np.savetxt(path, poses, delimiter=',', header='stamp,x,y,z,qx,qy,qz,qw', comments='')
        (self.frontend/'ros_parameters.yaml').write_text('mapping:\n  extrinsic_est_en: false\n  extrinsic_T: [0,0,0]\n')
        (self.frontend/'evaluation.json').write_text('{"failures": []}')
        point = np.array([1., 0, 0])
        scans = [point[None]] + [target_returns(point, origin) for origin in origins[1:]]
        xyzi = [np.column_stack((scan, np.full(len(scan), 11.))) for scan in scans]
        np.savez_compressed(self.frontend/'capture/window.npz', points=np.concatenate(xyzi),
                            stamps=self.stamps, lengths=[len(scan) for scan in scans])
        (self.frontend/'capture/capture.json').write_text(json.dumps(dict(counts={'output_cloud': 4}, temporal_raw=['window.npz'])))
        (self.backend/'report.json').write_text(json.dumps(dict(source_run=str(self.frontend), input_frames=4)))
        self.points = np.array([[1, 0, 0, 11], [7, 7, 7, 22], [8, 8, 8, 33]], dtype=np.float32)
        write_pcd(self.backend/'optimized_3cm.pcd', self.points)
        write_pcd(self.visibility/'filtered_3cm.pcd', self.points[[0, 2]])
        self.hits = np.array([1, 0, 0], dtype=np.uint16)
        self.save_evidence()
        report = dict(frontend=str(self.frontend), backend=str(self.backend), input_frames=4,
                      settings={'require_angular_support': True})
        for name, key in [('optimized_3cm.pcd', 'map_sha256'), ('trajectory.csv', 'trajectory_sha256')]:
            report[key] = hashlib.sha256((self.backend/name).read_bytes()).hexdigest()
        (self.visibility/'report.json').write_text(json.dumps(report))
        self.output = self.root/'output'

    def save_evidence(self):
        np.savez_compressed(self.visibility/'point_evidence.npz', removed=np.array([False, True, False]),
                            hit_bins=self.hits, free_bins=np.array([0, 5, 0], dtype=np.uint16))

    def tearDown(self):
        self.temp.cleanup()

    def invoke(self):
        script = pathlib.Path(__file__).resolve().parents[1]/'filter_sensor_motion.py'
        return subprocess.run([sys.executable, str(script), str(self.frontend), str(self.backend),
            '--visibility-dir', str(self.visibility), '--output-dir', str(self.output)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def test_full_command_removes_known_moving_return_and_keeps_unknown(self):
        process = self.invoke()
        self.assertEqual(process.returncode, 0, process.stderr)
        report = json.loads((self.output/'report.json').read_text())
        self.assertEqual(report['additional_removed_points'], 1)
        self.assertEqual(report['input_frames'], 4)
        np.testing.assert_array_equal(read_map(self.output/'filtered_3cm.pcd'), self.points[[2]])
        np.testing.assert_array_equal(read_map(self.output/'removed_3cm.pcd'), self.points[[0, 1]])

    def test_full_command_protects_repeatedly_observed_point(self):
        self.hits[0] = 4
        self.save_evidence()
        process = self.invoke()
        self.assertEqual(process.returncode, 0, process.stderr)
        report = json.loads((self.output/'report.json').read_text())
        self.assertEqual(report['additional_removed_points'], 0)
        np.testing.assert_array_equal(read_map(self.output/'filtered_3cm.pcd'), self.points[[0, 2]])

    def test_foreign_trajectory_is_rejected_before_outputs(self):
        (self.backend/'trajectory.csv').write_text('foreign trajectory')
        process = self.invoke()
        self.assertNotEqual(process.returncode, 0)
        self.assertIn('Visibility source mismatch: trajectory.csv', process.stderr)
        self.assertFalse(self.output.exists())

    def test_unsafe_parent_mask_is_rejected_before_outputs(self):
        np.savez_compressed(self.visibility/'point_evidence.npz', removed=np.array([True, True, False]),
                            hit_bins=np.array([4, 0, 0]), free_bins=np.array([8, 5, 0]))
        write_pcd(self.visibility/'filtered_3cm.pcd', self.points[[2]])
        process = self.invoke()
        self.assertNotEqual(process.returncode, 0)
        self.assertIn('Visibility mask already removed repeatedly supported points', process.stderr)
        self.assertFalse(self.output.exists())


if __name__ == '__main__':
    unittest.main()
