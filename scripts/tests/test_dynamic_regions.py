#!/usr/bin/env python3
import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from audit_dynamic_regions import point_classes, roi_mask


class TemporalRegionTest(unittest.TestCase):
    def test_nearest_association_does_not_label_missing_support_as_removed(self):
        removed = np.array([False, True, False])
        previous = np.array([False, True, True])
        classes = point_classes(np.array([.01, .01, .01, .5]), np.array([0, 1, 2, 1]), removed, previous)
        np.testing.assert_array_equal(classes, [1, 2, 3, 0])

    def test_local_view_uses_fixed_world_bounds(self):
        points = np.array([[0, 0, 0], [1, 1, 1], [1.01, 0, 0]])
        np.testing.assert_array_equal(roi_mask(points, dict(lower=[0, 0, 0], upper=[1, 1, 1])),
                                      [True, True, False])


if __name__ == '__main__':
    unittest.main()
