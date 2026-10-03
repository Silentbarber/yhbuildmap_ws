#!/usr/bin/env python3
import pathlib
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from deskew_ray_origin import grouped_origins, interpolate_pose, point_absolute_times, point_origins


class DeskewRayOriginTest(unittest.TestCase):
    def setUp(self):
        self.trajectory = np.asarray([[10., 0., 0., 0.], [11., 1., 0., 0.], [12., 2., 0., 0.]])
        self.rotations = Rotation.from_euler('z', [0., 0., np.pi/2]).as_matrix()

    def test_offsets_are_anchored_at_last_return(self):
        np.testing.assert_allclose(point_absolute_times(12., [0., 50., 100.]), [11.9, 11.95, 12.])

    def test_translation_and_rotation_interpolate_lidar_origin(self):
        origins = point_origins(12., [0., 50., 100.], self.trajectory, self.rotations, [1., 0., 0.])
        np.testing.assert_allclose(origins,
                                   [[2.056434465, .987688341, 0.], [2.028459096, .996917334, 0.], [2., 1., 0.]],
                                   atol=1e-7)

    def test_grouped_origins_cover_each_point_once(self):
        groups = grouped_origins(12., [0., 2., 5., 10., 50., 100.], self.trajectory,
                                 self.rotations, [0., 0., 0.], group_ms=5.)
        ids = np.concatenate([item[0] for item in groups])
        np.testing.assert_array_equal(np.sort(ids), np.arange(6))
        self.assertEqual(len(groups), 5)

    def test_outside_trajectory_and_bad_offsets_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'outside'):
            point_origins(12.1, [0., 100.], self.trajectory, self.rotations, [0., 0., 0.])
        with self.assertRaisesRegex(ValueError, '250'):
            point_absolute_times(12., [0., 251.])


if __name__ == '__main__':
    unittest.main()
