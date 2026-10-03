#!/usr/bin/env python3
import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from audit_angular_visibility import angular_support


class AngularSupportTest(unittest.TestCase):
    def check_triangle(self, yz):
        neighbours = np.column_stack((np.ones(3), np.asarray(yz, dtype=float)))
        neighbours /= np.linalg.norm(neighbours, axis=1)[:, None]
        return bool(angular_support(np.array([[1., 0, 0]]), neighbours[None])[0])

    def test_surrounding_triangle_supports_direction(self):
        self.assertTrue(self.check_triangle([[.001, 0], [-.001, .001], [-.001, -.001]]))

    def test_one_sided_rays_do_not_support_direction(self):
        self.assertFalse(self.check_triangle([[.001, 0], [.002, .001], [.002, -.001]]))

    def test_collinear_neighbours_without_direct_ray_are_unknown(self):
        self.assertFalse(self.check_triangle([[-.001, 0], [.001, 0], [.002, 0]]))

    def test_exact_ray_is_supported(self):
        self.assertTrue(self.check_triangle([[0, 0], [.001, 0], [.002, 0]]))

    def test_triangle_orientation_does_not_change_support(self):
        triangle = [[.001, 0], [-.001, .001], [-.001, -.001]]
        self.assertEqual(self.check_triangle(triangle), self.check_triangle(triangle[::-1]))


if __name__ == '__main__':
    unittest.main()
