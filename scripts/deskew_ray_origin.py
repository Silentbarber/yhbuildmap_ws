#!/usr/bin/env python3
"""Interpolate per-return Livox ray origins from captured point times."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def interpolate_pose(trajectory, rotations, timestamps):
    """Interpolate world IMU pose at one or more absolute timestamps."""
    trajectory = np.asarray(trajectory, dtype=float)
    rotations = Rotation.from_matrix(np.asarray(rotations, dtype=float))
    query = np.asarray(timestamps, dtype=float).reshape(-1)
    if trajectory.ndim != 2 or trajectory.shape[1] < 4 or len(trajectory) != len(rotations):
        raise ValueError('Trajectory and rotation arrays are incompatible')
    if not np.isfinite(trajectory[:, :4]).all() or not np.isfinite(query).all():
        raise ValueError('Pose interpolation received nonfinite values')
    if len(trajectory) < 2 or np.any(np.diff(trajectory[:, 0]) <= 0):
        raise ValueError('Trajectory timestamps must be strictly increasing')
    if np.any((query < trajectory[0, 0]) | (query > trajectory[-1, 0])):
        raise ValueError('Point time lies outside the optimized trajectory')
    positions = np.column_stack([np.interp(query, trajectory[:, 0], trajectory[:, axis])
                                 for axis in (1, 2, 3)])
    slerp = Slerp(trajectory[:, 0], rotations)
    return positions, slerp(query).as_matrix()


def point_absolute_times(frame_end, point_time_ms):
    """Convert output curvature offsets (scan-start milliseconds) to epoch seconds."""
    offsets = np.asarray(point_time_ms, dtype=float).reshape(-1)
    if not len(offsets) or not np.isfinite(offsets).all():
        raise ValueError('Point times are empty or nonfinite')
    if np.any(offsets < -1e-4) or np.max(offsets) > 250.:
        raise ValueError('Point time offsets must be within a 250 ms scan')
    scan_start = float(frame_end)-float(np.max(offsets))/1000.
    return scan_start+offsets/1000.


def point_origins(frame_end, point_time_ms, trajectory, rotations, extrinsic_t):
    """Return one world LiDAR origin per deskewed output point."""
    times = point_absolute_times(frame_end, point_time_ms)
    extrinsic = np.asarray(extrinsic_t, dtype=float).reshape(-1)
    if extrinsic.shape != (3,) or not np.isfinite(extrinsic).all():
        raise ValueError('Invalid LiDAR-to-IMU translation')
    positions, pose_rotations = interpolate_pose(trajectory, rotations, times)
    return positions+np.einsum('nij,j->ni', pose_rotations, extrinsic)


def grouped_origins(frame_end, point_time_ms, trajectory, rotations, extrinsic_t, group_ms=5.):
    """Approximate per-point origins with short time groups for tractable ray tests."""
    if not .5 <= group_ms <= 20:
        raise ValueError('Origin group must be between 0.5 and 20 ms')
    times = point_absolute_times(frame_end, point_time_ms)
    origins = point_origins(frame_end, point_time_ms, trajectory, rotations, extrinsic_t)
    groups = np.floor((np.asarray(point_time_ms, dtype=float)-np.min(point_time_ms))/group_ms).astype(np.int32)
    labels = np.unique(groups)
    grouped = []
    for label in labels:
        ids = np.flatnonzero(groups == label)
        grouped.append((ids, np.median(origins[ids], axis=0), float(np.median(times[ids]))))
    return grouped
