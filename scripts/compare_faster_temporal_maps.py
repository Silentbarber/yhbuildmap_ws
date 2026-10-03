#!/usr/bin/env python3
"""Compare whole temporal maps; fit diagnostics never modify displayed coordinates."""
import argparse
import csv
import json
import pathlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=pathlib.Path)
    parser.add_argument('--map', action='append', required=True, help='Label=directory')
    parser.add_argument('--output', type=pathlib.Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.run/'state_trace.csv').open() as file:
        row = next(csv.DictReader(file))
    gravity = np.array([float(row[k]) for k in ['gx', 'gy', 'gz']])
    gravity /= np.linalg.norm(gravity)
    first = np.cross(gravity, [0., 1., 0.])
    first /= np.linalg.norm(first)
    horizontal = np.column_stack((first, np.cross(gravity, first)))
    registration = o3d.pipelines.registration
    loss = registration.TransformationEstimationPointToPlane(registration.HuberLoss(.08))
    reports, all_clouds = [], []
    for value in args.map:
        label, directory = value.split('=', 1)
        paths = sorted(pathlib.Path(directory).glob('window_*_3cm.pcd'))
        clouds = [o3d.io.read_point_cloud(str(p)).voxel_down_sample(.08) for p in paths]
        if len(clouds) < 3:
            raise ValueError('Missing temporal maps')
        target = clouds[1]
        target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.3, max_nn=30))
        records = []
        for index, cloud in enumerate(clouds):
            before = registration.evaluate_registration(cloud, target, .15, np.eye(4))
            transformation = np.eye(4)
            for radius in [.6, .3, .15]:
                fit = registration.registration_icp(cloud, target, radius, transformation, loss,
                    registration.ICPConvergenceCriteria(max_iteration=40))
                transformation = fit.transformation
            after = registration.evaluate_registration(cloud, target, .15, transformation)
            records.append(dict(window=index, start_s_approximately=index*20,
                before_overlap=float(before.fitness), before_inlier_rmse_m=float(before.inlier_rmse),
                after_overlap=float(after.fitness), after_inlier_rmse_m=float(after.inlier_rmse),
                diagnostic_rotation_deg=float(np.rad2deg(Rotation.from_matrix(transformation[:3, :3].copy()).magnitude())),
                diagnostic_fit_has_native_overlap=bool(before.fitness >= .2 and after.fitness >= .2)))
        reports.append(dict(label=label, directory=directory, reference_window=1, comparisons=records))
        all_clouds.append(clouds)
        print(json.dumps(dict(label=label, comparisons=records)), flush=True)
    reference = np.asarray(all_clouds[0][1].points)
    center = float(np.median(reference @ gravity))
    fig, axes = plt.subplots(len(reports), 2, figsize=(14, 5*len(reports)), squeeze=False,
                             constrained_layout=True)
    for row, (report, clouds) in enumerate(zip(reports, all_clouds)):
        for col, windows in enumerate(([1, 2, 3, 5, 6], [1, 3, 6, len(clouds)-1])):
            for window, color in zip(windows, ['black', '#008ba3', '#ba347c', '#de8300', '#278a3b']):
                points = np.asarray(clouds[window].points)
                points = points[np.abs(points @ gravity - center) < .5]
                xy = points @ horizontal
                axes[row, col].scatter(xy[:, 0], xy[:, 1], s=.8, alpha=.4, color=color,
                                      label=f'{window*20}--{(window+1)*20}s')
            axes[row, col].set_aspect('equal')
            axes[row, col].set_title(report['label'] + (' | earlier scans' if col == 0 else ' | earlier + final scans'))
            axes[row, col].set_xlabel('Horizontal axis 1 [m]')
            axes[row, col].set_ylabel('Horizontal axis 2 [m]')
            axes[row, col].grid(alpha=.15)
            axes[row, col].legend(markerscale=5)
    # Identical limits include every plotted scan, not only the loop-closure area.
    for col in range(2):
        xlimits = [axes[row, col].get_xlim() for row in range(len(reports))]
        ylimits = [axes[row, col].get_ylim() for row in range(len(reports))]
        for row in range(len(reports)):
            axes[row, col].set_xlim(min(x[0] for x in xlimits), max(x[1] for x in xlimits))
            axes[row, col].set_ylim(min(y[0] for y in ylimits), max(y[1] for y in ylimits))
    fig.suptitle('Actual output coordinates; no diagnostic fit applied | complete 1m height slice')
    fig.savefig(args.output/'temporal_comparison.png', dpi=140)
    plt.close(fig)
    (args.output/'comparison.json').write_text(json.dumps(dict(maps=reports,
        scope='Rigid diagnostic fits and overlap are screening signals, not truth. Low-overlap fits are not interpretable; a good closure does not establish whole-map consistency.'), indent=2)+'\n')


if __name__ == '__main__':
    main()
