#!/usr/bin/env python3
import pathlib
import sys
import hashlib
import json
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from audit_dynamic_motion import compare_pair, frame_points, full_source_regions, export_full_review, sensor_attachment_score
from evaluate_independent_run import write_pcd


class MotionAuditTest(unittest.TestCase):
    @staticmethod
    def irregular_surface():
        rng = np.random.default_rng(18)
        direction = rng.normal(size=(1800, 3))
        direction /= np.linalg.norm(direction, axis=1, keepdims=True)
        return direction * np.array([.28, .19, .32]) + rng.normal(0, .001, direction.shape)

    def test_translated_irregular_object_has_heldout_motion_support(self):
        source = self.irregular_surface()
        target = source + [.12, -.08, .05]
        result = compare_pair(source, target)
        self.assertTrue(result['motion_supported'], result)
        self.assertGreater(result['displacement_at_source_center_m'], .12)

    def test_fixed_surface_with_different_coverage_is_not_motion(self):
        source = self.irregular_surface()
        target = source[source[:, 0] > -.10]
        result = compare_pair(source[source[:, 0] < .18], target)
        self.assertFalse(result['motion_supported'], result)

    def test_planar_tangential_shift_does_not_prove_motion(self):
        x, y = np.meshgrid(np.linspace(-.3, .3, 35), np.linspace(-.3, .3, 35))
        source = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
        result = compare_pair(source, source + [.15, 0, 0])
        self.assertFalse(result['motion_supported'], result)
        self.assertFalse(result['gates']['full_geometry'])

    def test_sparse_samples_are_inconclusive(self):
        result = compare_pair(np.zeros((5, 3)), np.ones((5, 3)))
        self.assertEqual(result['status'], 'insufficient_geometry')
        self.assertFalse(result['motion_supported'])

    def test_noisy_planar_scan_patterns_cannot_supply_full_motion(self):
        rng = np.random.default_rng(8)
        xy = rng.uniform(-.35, .35, size=(1600, 2))
        source = np.column_stack((xy, rng.normal(0, .004, len(xy))))
        target = source + [.14, .02, 0]
        result = compare_pair(source, target)
        self.assertFalse(result['motion_supported'], result)
        self.assertFalse(result['gates']['nonplanar_shape'])

    def test_candidate_and_control_selections_are_explicit(self):
        points = np.array([[0, 0, 0, 1], [0, 0, .02, 2], [0, 0, .03, 3], [0, 0, .04, 4]], dtype=float)
        classes = np.array([0, 1, 2, 3])
        frame = dict(offset=0, count=4)
        candidate = dict(kind='removal_candidate', center=[0, 0, 0])
        control = dict(kind='persistent_planar_candidate', center=[0, 0, 0], normal=[0, 0, 1])
        np.testing.assert_array_equal(frame_points(points, classes, frame, candidate), points[[0, 2, 3], :3])
        np.testing.assert_array_equal(frame_points(points, classes, frame, control), points[[1, 3], :3])

    def test_known_sensor_motion_explains_attached_returns(self):
        source = self.irregular_surface()
        target = source + [.12, -.08, .05]
        comparison = compare_pair(source, target)
        before = dict(imu_rotation=np.eye(3), lidar_origin=[0, 0, 0])
        after = dict(imu_rotation=np.eye(3), lidar_origin=[.12, -.08, .05])
        result = sensor_attachment_score(source, target, before, after, comparison['fixed_world'])
        self.assertTrue(result['compatible'], result)

    def test_world_fixed_surface_is_not_sensor_attached(self):
        source = self.irregular_surface()
        comparison = compare_pair(source, source)
        before = dict(imu_rotation=np.eye(3), lidar_origin=[0, 0, 0])
        after = dict(imu_rotation=np.eye(3), lidar_origin=[.12, -.08, .05])
        result = sensor_attachment_score(source, source, before, after, comparison['fixed_world'])
        self.assertFalse(result['compatible'], result)


class FullSourceAuditTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temp.name)
        self.frontend, self.backend = self.root/'frontend', self.root/'backend'
        (self.frontend/'capture').mkdir(parents=True)
        self.backend.mkdir()
        self.stamps = np.array([10., 10.1, 10.2])
        old = [[stamp, 0, 0, 0, 0, 0, 0, 1] for stamp in self.stamps]
        new = [[stamp, .5, 1, 0, 0, 0, np.sqrt(.5), np.sqrt(.5)] for stamp in self.stamps]
        for path, poses in [(self.frontend/'capture/trajectory.csv', old), (self.backend/'trajectory.csv', new)]:
            np.savetxt(path, poses, delimiter=',', header='stamp,x,y,z,qx,qy,qz,qw', comments='')
        params = self.frontend/'ros_parameters.yaml'
        params.write_text('mapping:\n  extrinsic_est_en: false\n  extrinsic_T: [.1,0,0]\n')
        scan = np.array([[1, 0, 0, 11], [2, 0, 0, 22], [3, 0, 0, 33]], dtype=np.float32)
        self.expected = np.array([[.5, 2, 0, 11], [.5, 3, 0, 22], [.5, 4, 0, 33]])
        self.archive = self.frontend/'capture/window.npz'
        np.savez_compressed(self.archive, points=np.tile(scan, (3, 1)), stamps=self.stamps, lengths=[3, 3, 3])
        write_pcd(self.backend/'optimized_3cm.pcd', self.expected.astype(np.float32))
        self.recording = dict(id='sample', frontend='frontend', backend='backend', frames=3)
        for name, key in [('optimized_3cm.pcd', 'map_sha256'), ('trajectory.csv', 'trajectory_sha256')]:
            self.recording[key] = hashlib.sha256((self.backend/name).read_bytes()).hexdigest()
        self.report = dict(source_metadata_sha256={str(path.relative_to(self.frontend)):
            hashlib.sha256(path.read_bytes()).hexdigest() for path in [params, self.frontend/'capture/trajectory.csv']},
            source_archives=[dict(file='window.npz', sha256=hashlib.sha256(self.archive.read_bytes()).hexdigest())],
            regions=[dict(key='object', lower=[-5, -5, -5], upper=[5, 5, 5])])
        self.review = self.root/'review'
        (self.review/'object').mkdir(parents=True)
        self.expected.astype('<f4').tofile(self.review/'object/baseline.bin')
        np.array([2, 1, 3], dtype=np.uint8).tofile(self.review/'object/baseline_classes.bin')
        np.arange(3, dtype='<u4').tofile(self.review/'object/baseline_indices.bin')
        for name, removed in [('angular_guard', [True, False, False]), ('delivery', [True, False, True])]:
            folder = self.root/'results/dynamic_filtering'/name/'sample'
            folder.mkdir(parents=True)
            meta = {key: self.recording[key] for key in ['map_sha256', 'trajectory_sha256']}
            meta['settings'] = dict(require_angular_support=name == 'angular_guard', max_hit_bins=3)
            (folder/'report.json').write_text(json.dumps(meta))
            np.savez_compressed(folder/'point_evidence.npz', removed=np.array(removed))

    def tearDown(self):
        self.temp.cleanup()

    def test_full_rate_coordinates_classes_origin_and_frame_ids(self):
        samples, hashes = full_source_regions(self.root, self.recording, self.report, {'object': .1}, 1.)
        self.assertEqual(len(samples['object']), 3)
        for i, sample in enumerate(samples['object']):
            np.testing.assert_allclose(sample['points'], self.expected, atol=1e-12)
            np.testing.assert_array_equal(sample['classes'], [2, 1, 3])
            np.testing.assert_allclose(sample['frame']['lidar_origin'], [.5, 1.1, 0])
            self.assertEqual(sample['frame']['frame_index'], i)
            self.assertEqual(sample['frame']['chunk_frame'], i)
        self.assertIn(str(self.archive), hashes)

    def test_requested_time_window_excludes_other_frames(self):
        samples, _ = full_source_regions(self.root, self.recording, self.report, {'object': .1}, .05)
        self.assertEqual([sample['frame']['frame_index'] for sample in samples['object']], [1])

    def test_changed_source_archive_is_rejected(self):
        self.archive.write_bytes(b'changed captured output')
        with self.assertRaisesRegex(ValueError, 'Source provenance changed'):
            full_source_regions(self.root, self.recording, self.report, {'object': .1}, 1.)

    def test_incomplete_captured_inventory_is_rejected(self):
        self.recording['frames'] = 4
        with self.assertRaisesRegex(ValueError, 'Incomplete captured source inventory'):
            full_source_regions(self.root, self.recording, self.report, {'object': .1}, 1.)

    def test_full_window_export_keeps_source_indices_and_offsets(self):
        samples, _ = full_source_regions(self.root, self.recording, self.report, {'object': .1}, 1.)
        output = self.root/'export'
        hashes = export_full_review(output, self.review, self.report, samples)
        frames = json.loads((output/'object/frames.json').read_text())
        self.assertEqual([frame['offset'] for frame in frames], [0, 3, 6])
        packed = np.fromfile(output/'object/scan.bin', dtype='<f4').reshape(-1, 4)
        np.testing.assert_allclose(packed, np.tile(self.expected, (3, 1)), atol=1e-7)
        np.testing.assert_array_equal(np.fromfile(output/'object/source_point_indices.bin', dtype='<u4'),
                                     np.tile([0, 1, 2], 3))
        np.testing.assert_array_equal(np.fromfile(output/'object/scan_classes.bin', dtype=np.uint8),
                                     np.tile([2, 1, 3], 3))
        self.assertEqual((output/'object/baseline.bin').read_bytes(), (self.review/'object/baseline.bin').read_bytes())
        self.assertEqual(hashes['object/scan.bin'], hashlib.sha256((output/'object/scan.bin').read_bytes()).hexdigest())


if __name__ == '__main__':
    unittest.main()
