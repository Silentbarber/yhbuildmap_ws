#!/usr/bin/env python3
"""Physical support spacing must not be confused with angular coverage."""
import pathlib
import sys
import hashlib
import json
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from audit_ray_footprint import audit_record, compare_structure, count_bins, origin_motion_summary, ray_footprint_radius
from evaluate_independent_run import write_pcd
from filter_dynamic_map import ray_evidence


class RayFootprintTest(unittest.TestCase):
    def ring(self, radius):
        angles = np.arange(3)*2*np.pi/3
        scan = np.column_stack([radius*np.cos(angles), radius*np.sin(angles), np.full(3, 12.)])
        return np.asarray([[0., 0., 10.]]), scan

    def test_surrounding_rays_can_miss_a_thin_static_object(self):
        point, scan = self.ring(.07)
        hit, free = ray_evidence(point, scan, np.zeros(3), angle_deg=.4, require_angular_support=True)
        self.assertFalse(hit[0])
        self.assertTrue(free[0])
        # A 2 cm radius pole at this position is missed by all three rays.
        radius = ray_footprint_radius(point, scan, np.zeros(3), .4)
        self.assertGreater(radius[0], .05)
        self.assertLess(radius[0], .06)

    def test_tight_support_passes_physical_bound(self):
        point, scan = self.ring(.02)
        self.assertLess(ray_footprint_radius(point, scan, np.zeros(3), .4)[0], .02)

    def test_farthest_support_ray_controls_the_radius(self):
        point, scan = self.ring(.07)
        scan[0, :2] = 0
        self.assertGreater(ray_footprint_radius(point, scan, np.zeros(3), .4)[0], .05)

    def test_same_angle_has_larger_spacing_at_longer_range(self):
        point, scan = self.ring(.07)
        near = ray_footprint_radius(point, scan, np.zeros(3), .4)
        far = ray_footprint_radius(point*2, scan*2, np.zeros(3), .4)
        np.testing.assert_allclose(far, near*2)

    def test_missing_and_outside_cone_are_unknown(self):
        point, scan = self.ring(.07)
        self.assertTrue(np.isinf(ray_footprint_radius(point, scan[:2], np.zeros(3), .4)[0]))
        self.assertTrue(np.isinf(ray_footprint_radius(point, scan, np.zeros(3), .1)[0]))

    def test_rigid_coordinate_changes_preserve_spacing(self):
        point, scan = self.ring(.07)
        rotation = Rotation.from_euler('xyz', [.5, -.2, 1.1]).as_matrix()
        offset = np.asarray([4., -9., 3.])
        expected = ray_footprint_radius(point, scan, np.zeros(3), .4)
        actual = ray_footprint_radius(point@rotation.T+offset, scan@rotation.T+offset, offset, .4)
        np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_bin_restrictions_preserve_distinct_time_counting(self):
        counts, seen = np.zeros(2, np.uint16), np.full(2, -1, np.int32)
        first, last = np.full(2, np.inf), np.full(2, -np.inf)
        count_bins(np.asarray([0, 1]), 0, .1, counts, seen, first, last)
        count_bins(np.asarray([0]), 0, .4, counts, seen, first, last)
        count_bins(np.asarray([1]), 1, 2.1, counts, seen, first, last)
        np.testing.assert_array_equal(counts, [1, 2])
        np.testing.assert_allclose(first, [.1, .1])
        np.testing.assert_allclose(last, [.4, 2.1])

    def test_scan_origin_diagnostic_includes_rotating_extrinsic(self):
        poses = np.zeros((3, 8))
        poses[:, 0] = [0., .1, .2]
        poses[1:, 1] = 1.
        rotations = Rotation.from_euler('z', [0., np.pi, np.pi]).as_matrix()
        result = origin_motion_summary(poses, rotations, np.asarray([.5, 0., 0.]))
        # Translation and rotating lever arm cancel exactly here.
        self.assertEqual(result['approximately_100ms_pairs'], 2)
        np.testing.assert_allclose(result['origin_displacement_m_p50_p95_max'], 0., atol=1e-14)
        self.assertEqual(result['pairs_exceeding_5cm'], 0)

    def test_long_pose_gap_is_not_claimed_as_scan_displacement(self):
        poses = np.zeros((3, 8))
        poses[:, 0] = [0., .5, 1.]
        result = origin_motion_summary(poses, np.tile(np.eye(3), (3, 1, 1)), np.zeros(3))
        self.assertEqual(result['approximately_100ms_pairs'], 0)
        self.assertIsNone(result['origin_displacement_m_p50_p95_max'])


class RayFootprintReplayTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temp.name)
        frontend, backend = self.root/'frontend', self.root/'backend'
        capture = frontend/'capture'
        capture.mkdir(parents=True)
        backend.mkdir()
        visibility = self.root/'evidence/synthetic'
        visibility.mkdir(parents=True)
        review = self.root/'review/synthetic'
        review.mkdir(parents=True)
        params = frontend/'ros_parameters.yaml'
        params.write_text('mapping:\n  extrinsic_est_en: false\n  extrinsic_T: [0, 0, 0]\n')
        stamps = np.asarray([100., 102., 104., 106.])
        trajectory = 'stamp,x,y,z,qx,qy,qz,qw\n'+''.join('{},0,0,0,0,0,0,1\n'.format(s) for s in stamps)
        (capture/'trajectory.csv').write_text(trajectory)
        (backend/'trajectory.csv').write_text(trajectory)
        _, scan = RayFootprintTest().ring(.07)
        scan = np.column_stack([scan, np.ones(3)])
        np.savez(capture/'scans.npz', stamps=stamps, lengths=np.full(4, 3), points=np.tile(scan, (4, 1)))
        (capture/'capture.json').write_text(json.dumps(dict(counts=dict(output_cloud=4), temporal_raw=['scans.npz'])))
        points = np.asarray([[0., 0., 10., 5.], [0., 0., 12., 8.]], dtype=np.float32)
        write_pcd(backend/'optimized_3cm.pcd', points)
        write_pcd(visibility/'filtered_3cm.pcd', points[1:])
        self.saved = dict(removed=np.asarray([True, False]), hit_bins=np.asarray([0, 4], np.uint16),
            free_bins=np.asarray([4, 0], np.uint16), first_free=np.asarray([0., np.inf]),
            last_free=np.asarray([6., -np.inf]))
        np.savez(visibility/'point_evidence.npz', **self.saved)
        digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
        self.record = dict(id='synthetic', frontend='frontend', backend='backend', frames=4,
            map_sha256=digest(backend/'optimized_3cm.pcd'), trajectory_sha256=digest(backend/'trajectory.csv'))
        settings = dict(require_angular_support=True, frame_step=.3, evidence_bin=2., angle_deg=.4,
            hit_distance=.1, free_margin=.2, max_range=20., min_free_bins=4, free_ratio=.8, min_span=6., max_hit_bins=3)
        parent = dict(frontend=str(frontend), backend=str(backend), settings=settings, input_frames=4,
            evidence_frames=4, map_sha256=self.record['map_sha256'], trajectory_sha256=self.record['trajectory_sha256'])
        (visibility/'report.json').write_text(json.dumps(parent))
        source = dict(source_capture=str(capture), map_sha256=self.record['map_sha256'],
            trajectory_sha256=self.record['trajectory_sha256'],
            source_metadata_sha256={p: digest(frontend/p) for p in ['capture/trajectory.csv', 'capture/capture.json', 'ros_parameters.yaml']},
            source_archives=[dict(file='scans.npz', sha256=digest(capture/'scans.npz'))])
        (review/'report.json').write_text(json.dumps(source))
        self.output = self.root/'output'

    def tearDown(self):
        self.temp.cleanup()

    def invoke(self):
        return audit_record(self.record, self.root, self.root/'evidence', self.root/'review',
                            self.output, np.asarray([.02, .06, .10]), 1)

    def test_real_replay_reproduces_parent_and_bounds_physical_scale(self):
        result = self.invoke()
        self.assertTrue(result['parent_evidence_reproduced'])
        self.assertEqual(result['input_frames'], 4)
        self.assertEqual([row['remaining_deletion_candidates'] for row in result['comparisons']], [0, 1, 1])
        self.assertTrue(all(row['retained_controls_newly_deleted'] == 0 for row in result['comparisons']))
        self.assertEqual(list(self.output.glob('*.pcd')), [])

    def test_source_tamper_rejected_without_output(self):
        path = self.root/'frontend/capture/scans.npz'
        path.write_bytes(path.read_bytes()+b'changed')
        with self.assertRaisesRegex(ValueError, 'Source archive changed'):
            self.invoke()
        self.assertFalse(self.output.exists())

    def test_original_count_mismatch_cannot_silently_pass(self):
        self.saved['free_bins'][0] = 5
        np.savez(self.root/'evidence/synthetic/point_evidence.npz', **self.saved)
        with self.assertRaisesRegex(ValueError, 'Original evidence did not reproduce: free_bins'):
            self.invoke()
        self.assertFalse(self.output.exists())

    def test_geometry_comparison_preserves_untouched_map(self):
        self.invoke()
        result = compare_structure(self.record, self.root, self.root/'evidence', self.output)
        self.assertEqual(result['original_removed'], 1)
        self.assertEqual([row['restored_points'] for row in result['comparisons']], [1, 0, 0])
        self.assertEqual(list(self.output.glob('*.pcd')), [])

    def test_geometry_comparison_rejects_inconsistent_mask(self):
        self.invoke()
        path = self.output/'point_audit.npz'
        with np.load(path) as saved:
            arrays = {key: saved[key].copy() for key in saved.files}
        arrays['deletion_masks'][1, 0] = True
        np.savez(path, **arrays)
        with self.assertRaisesRegex(ValueError, 'Footprint classification mismatch'):
            compare_structure(self.record, self.root, self.root/'evidence', self.output)
        self.assertFalse((self.output/'structural_report.json').exists())


if __name__ == '__main__':
    unittest.main()
