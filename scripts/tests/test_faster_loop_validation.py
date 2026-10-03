#!/usr/bin/env python3
"""Perfect overlap on one plane must not pass a full-pose loop check."""
import pathlib
import sys
import unittest
from types import SimpleNamespace

import numpy as np
import open3d as o3d

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from refine_independent_pose_graph import validate_loop, plane_information, observation_odometry_information


def scene(axes):
    coordinates = np.linspace(-2, 2, 35)
    first, second = np.meshgrid(coordinates, coordinates)
    clouds, normals = [], []
    for axis in axes:
        points = np.zeros((first.size, 3))
        tangent = [index for index in range(3) if index != axis]
        points[:, tangent[0]] = first.ravel()
        points[:, tangent[1]] = second.ravel()
        points[:, axis] = 3
        clouds.append(points)
        normal = np.zeros_like(points)
        normal[:, axis] = 1
        normals.append(normal)
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate(clouds)))
    cloud.normals = o3d.utility.Vector3dVector(np.concatenate(normals))
    cells = np.floor(np.asarray(cloud.points) / .4).astype(np.int64)
    training = cells.sum(axis=1) % 2 == 0
    return cloud, training


class LoopValidationTest(unittest.TestCase):
    def test_timestamp_jitter_is_not_a_missing_scan(self):
        rows=[dict(stamp=str(i*.1), features='4000', matched_range_rms_m='3',
                   normalized_yaw_information='.15', gx='0', gy='0', gz='-9.81') for i in range(1,11)]
        _, report=observation_odometry_information(rows,0,1.001,np.eye(3))
        self.assertEqual(report['missing_update_duration_s'],0)

    def test_weak_observations_permit_more_yaw_correction(self):
        row = dict(stamp='.1', features='4000', matched_range_rms_m='3',
                   normalized_yaw_information='.15', gx='0', gy='0', gz='-9.81')
        strong, _ = observation_odometry_information([row], 0, .1, np.eye(3))
        weak, _ = observation_odometry_information([dict(row, features='5', matched_range_rms_m='.4')], 0, .1, np.eye(3))
        self.assertGreater(strong[2, 2], weak[2, 2]*1000)
        np.testing.assert_allclose(strong[:2, :2], weak[:2, :2])

    def test_plane_information_keeps_unobservable_directions_weak(self):
        cloud, _ = scene([2])
        information = plane_information(cloud, cloud, np.eye(4))
        eigenvalues = np.linalg.eigvalsh(information)
        self.assertLess(eigenvalues[2], 1e-4)
        self.assertGreater(eigenvalues[3], 100)

    def test_single_plane_with_perfect_overlap_is_rejected(self):
        cloud, training = scene([2])
        report = validate_loop(cloud, cloud, training, SimpleNamespace(transformation=np.eye(4)))
        self.assertGreater(report['heldout_overlap'], .99)
        self.assertFalse(report['validation_passed'])

    def test_three_orthogonal_planes_with_consistent_pose_pass(self):
        cloud, training = scene([0, 1, 2])
        report = validate_loop(cloud, cloud, training, SimpleNamespace(transformation=np.eye(4)))
        self.assertTrue(report['validation_passed'], report)


if __name__ == '__main__':
    unittest.main()
