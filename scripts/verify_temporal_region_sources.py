#!/usr/bin/env python3
"""Reconstruct sampled review frames from captured source outputs and poses."""
import argparse
import csv
import hashlib
import io
import json
import pathlib

import numpy as np
import yaml
from scipy.spatial import cKDTree

from audit_dynamic_regions import point_classes
from filter_dynamic_map import correct_scan, load_trajectory, pose_index, read_map


ROOT = pathlib.Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=pathlib.Path, required=True)
    parser.add_argument('--review-root', type=pathlib.Path, required=True)
    parser.add_argument('--output', type=pathlib.Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a fresh output path')
    records = json.loads((ROOT/'config/dynamic_filtering_recordings.json').read_text())['recordings']
    checks = []
    for record in records:
        directory = args.review_root/record['id']
        report = json.loads((directory/'report.json').read_text())
        frontend, backend = args.data_root/record['frontend'], args.data_root/record['backend']
        for name, digest in report['source_metadata_sha256'].items():
            if hashlib.sha256((frontend/name).read_bytes()).hexdigest() != digest:
                raise ValueError('Source metadata changed: '+record['id']+'/'+name)
        for name, key in [('optimized_3cm.pcd', 'map_sha256'), ('trajectory.csv', 'trajectory_sha256')]:
            digest = hashlib.sha256((backend/name).read_bytes()).hexdigest()
            if digest != report[key] or digest != record[key]:
                raise ValueError('Baseline provenance mismatch')
        points = read_map(backend/'optimized_3cm.pcd')
        tree = cKDTree(points[:, :3])
        with np.load(args.data_root/'results/dynamic_filtering/angular_guard'/record['id']/'point_evidence.npz') as evidence:
            removed = evidence['removed'].copy()
        with np.load(args.data_root/'results/dynamic_filtering/delivery'/record['id']/'point_evidence.npz') as evidence:
            previous = evidence['removed'].copy()
        old, old_rotations = load_trajectory(frontend/'capture/trajectory.csv')
        new, new_rotations = load_trajectory(backend/'trajectory.csv')
        params = yaml.safe_load((frontend/'ros_parameters.yaml').read_text())
        translation = np.asarray(params['mapping']['extrinsic_T'])
        jobs = {}
        for region in report['regions']:
            folder = directory/region['key']
            baseline_ids = np.fromfile(folder/'baseline_indices.bin', dtype='<u4')
            baseline = np.fromfile(folder/'baseline.bin', dtype='<f4').reshape(-1, 4)
            if not np.array_equal(baseline, points[baseline_ids]):
                raise ValueError('Region baseline coordinates/intensity changed')
            expected_classes = point_classes(np.zeros(len(baseline_ids)), baseline_ids, removed, previous)
            if not np.array_equal(np.fromfile(folder/'baseline_classes.bin', dtype=np.uint8), expected_classes):
                raise ValueError('Region baseline class mismatch')
            frames = json.loads((folder/'frames.json').read_text())
            rows = list(csv.DictReader((folder/'frames.csv').open()))
            if len(rows) != record['frames'] or len(frames) != region['display_frames']:
                raise ValueError('Frame coverage mismatch')
            source = dict(points=np.fromfile(folder/'scan.bin', dtype='<f4').reshape(-1, 4),
                classes=np.fromfile(folder/'scan_classes.bin', dtype=np.uint8),
                indices=np.fromfile(folder/'source_point_indices.bin', dtype='<u4'),
                baseline_ids=np.fromfile(folder/'associated_baseline_indices.bin', dtype='<u4'),
                distances=np.fromfile(folder/'association_distances.bin', dtype='<f4'))
            expected = sum(frame['count'] for frame in frames)
            if any(len(array) != expected for array in source.values()):
                raise ValueError('Packed source array length mismatch')
            visible = [frame for frame in frames if frame['count']]
            for index in np.unique(np.linspace(0, len(visible)-1, min(4, len(visible)), dtype=int)):
                frame = visible[index]
                row = rows[frame['frame_index']]
                if row['chunk'] != frame['chunk'] or int(row['chunk_frame']) != frame['chunk_frame']:
                    raise ValueError('Full-frame/display index mismatch')
                jobs.setdefault(frame['chunk'], []).append((region['key'], frame, source))
        for archive in report['source_archives']:
            data = (frontend/'capture'/archive['file']).read_bytes()
            if hashlib.sha256(data).hexdigest() != archive['sha256']:
                raise ValueError('Captured source archive changed')
            if archive['file'] not in jobs:
                continue
            with np.load(io.BytesIO(data)) as chunk:
                cuts = np.r_[0, np.cumsum(chunk['lengths'])]
                for key, frame, packed in jobs[archive['file']]:
                    j = frame['chunk_frame']
                    stamp = float(chunk['stamps'][j])
                    if abs(stamp-frame['stamp']) > 1e-5:
                        raise ValueError('Source timestamp mismatch')
                    original = chunk['points'][cuts[j]:cuts[j+1]]
                    if len(original) != frame['source_scan_points']:
                        raise ValueError('Source scan count mismatch')
                    a, b = pose_index(old, stamp), pose_index(new, stamp)
                    xyz, origin = correct_scan(original, old[a, 1:4], old_rotations[a],
                                               new[b, 1:4], new_rotations[b], translation)
                    begin, end = frame['offset'], frame['offset']+frame['count']
                    ids = packed['indices'][begin:end]
                    actual = np.column_stack((xyz[ids], original[ids, 3])).astype('<f4')
                    if not np.array_equal(actual, packed['points'][begin:end]):
                        raise ValueError('Source XYZI reconstruction mismatch')
                    distances, nearest = tree.query(xyz[ids], workers=2)
                    if not np.array_equal(nearest, packed['baseline_ids'][begin:end]):
                        raise ValueError('Nearest baseline index mismatch')
                    np.testing.assert_allclose(distances, packed['distances'][begin:end], atol=1e-7, rtol=1e-6)
                    np.testing.assert_allclose(origin, frame['lidar_origin'], atol=1e-9)
                    expected_classes = point_classes(distances, nearest, removed, previous)
                    if not np.array_equal(expected_classes, packed['classes'][begin:end]):
                        raise ValueError('Source classification mismatch')
                    checks.append(dict(id=record['id'], region=key, frame=frame['frame_index'],
                                       stamp=stamp, checked_points=len(ids), xyzi_equal=True))
        print(record['id'], 'source samples verified', flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(passed=True, checks=checks,
        scope='Four nonempty display frame samples per selected region, exact reconstructed XYZI '
              'and nearest baseline associations; source metadata and all capture archive hashes checked. '
              'Not exhaustive per-point lineage or semantic motion truth.'), indent=2)+'\n')


if __name__ == '__main__':
    main()
