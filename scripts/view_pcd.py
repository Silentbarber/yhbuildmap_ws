#!/usr/bin/env python3
"""View one local PCD in an Open3D window."""
import argparse
import pathlib

import numpy as np
import open3d as o3d


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd', type=pathlib.Path)
    parser.add_argument('--voxel', type=float, default=0,
                        help='Optional display sampling in metres; leaves the source file intact')
    args = parser.parse_args()
    if not args.pcd.is_file():
        parser.error('PCD file does not exist')
    if args.voxel < 0:
        parser.error('voxel must be nonnegative')
    cloud = o3d.io.read_point_cloud(str(args.pcd))
    if cloud.is_empty():
        parser.error('PCD has no readable points')
    if args.voxel:
        cloud = cloud.voxel_down_sample(args.voxel)
    if not cloud.has_colors():
        height = np.asarray(cloud.points)[:, 2]
        low, high = np.percentile(height, [2, 98])
        scale = np.clip((height - low) / max(high - low, 1e-6), 0, 1)
        from matplotlib import colormaps
        cloud.colors = o3d.utility.Vector3dVector(colormaps['viridis'](scale)[:, :3])
    o3d.visualization.draw_geometries([cloud], window_name=args.pcd.name,
                                    width=1280, height=900)


if __name__ == '__main__':
    main()
