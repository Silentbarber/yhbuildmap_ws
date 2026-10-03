#!/usr/bin/env python3
"""Cross-check repeat-scene rotation using multistart, held-out points and planes."""
import argparse
import csv
import json
import pathlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def cloud(points):
    value = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    value.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.3, max_nn=30))
    return value


def validate(run, output, map_dir=None):
    if (output / 'timeline.json').exists():
        gravity = np.array(json.loads((output / 'timeline.json').read_text())['initial_gravity_m_s2'])
    else:
        with (run / 'state_trace.csv').open() as file:
            initial = next(csv.DictReader(file))
        gravity = np.array([float(initial[key]) for key in ['gx', 'gy', 'gz']])
    paths = sorted((map_dir or run / 'maps').glob('window_*_3cm.pcd'))
    target_cloud = o3d.io.read_point_cloud(str(paths[1])).voxel_down_sample(.06)
    target_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.3, max_nn=30))
    later = o3d.io.read_point_cloud(str(paths[-1])).voxel_down_sample(.06)
    xyz = np.asarray(later.points)
    cells = np.floor(xyz / .4).astype(np.int64)
    training = ((cells[:, 0] + cells[:, 1] + cells[:, 2]) % 2) == 0
    train = cloud(xyz[training])
    test = xyz[~training]
    gravity /= np.linalg.norm(gravity)
    trials = []
    estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane(
        o3d.pipelines.registration.HuberLoss(k=.08))
    center = xyz.mean(axis=0)
    for initial_angle in [-10, 0, 10]:
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_rotvec(gravity * np.deg2rad(initial_angle)).as_matrix()
        transform[:3, 3] = center - transform[:3, :3] @ center
        for radius in [.8, .4, .2, .12]:
            result = o3d.pipelines.registration.registration_icp(train, target_cloud, radius, transform,
                estimation, o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
            transform = result.transformation
        rotation = Rotation.from_matrix(transform[:3, :3].copy()).as_rotvec()
        transformed = test @ transform[:3, :3].T + transform[:3, 3]
        tree = cKDTree(np.asarray(target_cloud.points))
        distance_before = tree.query(test, workers=2)[0]
        distance_after = tree.query(transformed, workers=2)[0]
        displacement = transform[:3, :3] @ center + transform[:3, 3] - center
        trials.append(dict(initial_rotation_deg=initial_angle, train_fitness=float(result.fitness),
                           train_rmse_m=float(result.inlier_rmse), rotation_deg=float(np.rad2deg(np.linalg.norm(rotation))),
                           gravity_axis_rotation_deg=float(np.rad2deg(rotation @ gravity)),
                           off_gravity_rotation_component_deg=float(np.rad2deg(np.linalg.norm(rotation - gravity * (rotation @ gravity)))),
                           centroid_displacement_m=float(np.linalg.norm(displacement)),
                           heldout_overlap_before_at_15cm=float((distance_before < .15).mean()),
                           heldout_overlap_after_at_15cm=float((distance_after < .15).mean()),
                           heldout_nn_median_before_m=float(np.median(distance_before)),
                           heldout_nn_median_after_m=float(np.median(distance_after)),
                           transform=transform.tolist()))
    best = max(trials, key=lambda r: (r['train_fitness'], -r['train_rmse_m']))
    transform = np.array(best['transform'])
    rotation_spread = max(np.rad2deg((Rotation.from_matrix(np.array(a['transform'])[:3, :3].copy()).inv() *
                                     Rotation.from_matrix(np.array(b['transform'])[:3, :3].copy())).magnitude())
                          for a in trials for b in trials)
    # First-generation reference planes only; fit these to no later geometry.
    planes = []
    remainder = target_cloud
    o3d.utility.random.seed(7)
    for index in range(5):
        if len(remainder.points) < 300:
            break
        model, inliers = remainder.segment_plane(.015, 3, 500)
        points = np.asarray(remainder.points)[inliers]
        normal = np.array(model[:3]); offset = float(model[3])
        axis = np.eye(3)[np.argmin(np.abs(normal))]
        first = np.cross(normal, axis); first /= np.linalg.norm(first)
        tangent = np.column_stack((first, np.cross(normal, first)))
        projected = points @ tangent
        low, high = np.percentile(projected, [5, 95], axis=0)
        # Pick matching later points once from the diagnostic fit, then retain the same
        # point indices for the native comparison to avoid changing samples between views.
        corrected = test @ transform[:3, :3].T + transform[:3, 3]
        uv = corrected @ tangent
        signed_after = corrected @ normal + offset
        mask = (uv >= low).all(axis=1) & (uv <= high).all(axis=1) & (np.abs(signed_after) < .06)
        native = test[mask] @ normal + offset
        if mask.sum() >= 100:
            planes.append(dict(reference_plane=index, reference_points=len(points),
                               reference_normal=normal.tolist(), later_matched_heldout_points=int(mask.sum()),
                               native_signed_median_m=float(np.median(native)),
                               native_p95_abs_m=float(np.percentile(np.abs(native), 95)),
                               diagnostic_p95_abs_m=float(np.percentile(np.abs(signed_after[mask]), 95))))
        remainder = remainder.select_by_index(inliers, invert=True)
    report = dict(map_directory=str(map_dir or run / 'maps'), reference_window=1, later_window=len(paths)-1, source_points=len(xyz),
                  heldout_points=len(test), split='alternating 40cm voxel blocks in native source coordinates',
                  trials=trials, pairwise_fitted_rotation_spread_deg=float(rotation_spread), reference_planes=planes,
                  scope='Relative scene consistency only. Fits use training points; heldout blocks validate transfer. Plane samples are selected by the fitted association and therefore are descriptive, not unbiased precision estimates. No output cloud is modified; no surveyed truth.')
    output.mkdir(parents=True, exist_ok=True)
    (output / 'return_alignment_validation.json').write_text(json.dumps(report, indent=2) + '\n')
    horizontal_a = np.cross(gravity, np.array([0., 1., 0.]))
    horizontal_a /= np.linalg.norm(horizontal_a)
    horizontal = np.column_stack((horizontal_a, np.cross(gravity, horizontal_a)))
    reference_xyz = np.asarray(target_cloud.points)
    corrected = xyz @ transform[:3, :3].T + transform[:3, 3]
    # Horizontal slice makes duplicated walls visible while leaving source PCD intact.
    height = reference_xyz @ gravity
    center_height = np.median(height)
    slab = .5
    ref = reference_xyz[np.abs(height - center_height) < slab]
    native = xyz[np.abs(xyz @ gravity - center_height) < slab]
    fitted = corrected[np.abs(corrected @ gravity - center_height) < slab]
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)
    coordinate_label = 'Optimized map coordinates' if map_dir else 'Native map coordinates'
    for ax, value, label in zip(axes, [native, fitted], [coordinate_label, 'Diagnostic rigid fit only']):
        xy = ref @ horizontal
        ax.scatter(xy[:, 0], xy[:, 1], s=1, color='black', alpha=.45, label='Earlier 20s map')
        xy = value @ horizontal
        ax.scatter(xy[:, 0], xy[:, 1], s=1, color='#d34325', alpha=.45, label='Last temporal map')
        ax.set_aspect('equal'); ax.set_title(label); ax.set_xlabel('Horizontal axis 1 [m]')
        ax.set_ylabel('Horizontal axis 2 [m]'); ax.legend(markerscale=5); ax.grid(alpha=.15)
        ref_xy = ref @ horizontal
        low, high = np.percentile(ref_xy, [1, 99], axis=0)
        ax.set_xlim(low[0] - .5, high[0] + .5); ax.set_ylim(low[1] - .5, high[1] + .5)
    fig.suptitle((map_dir.name if map_dir else run.parent.name) + ' | 1m diagnostic slice; full PCD unchanged')
    fig.savefig(output / 'building_return_overlay.png', dpi=160); plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=pathlib.Path)
    parser.add_argument('--output', type=pathlib.Path, required=True)
    parser.add_argument('--map-dir', type=pathlib.Path, help='Separate optimized temporal maps to validate')
    args = parser.parse_args()
    validate(args.run.resolve(), args.output.resolve(), args.map_dir.resolve() if args.map_dir else None)
