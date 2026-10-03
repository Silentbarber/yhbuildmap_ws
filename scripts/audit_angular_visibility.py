#!/usr/bin/env python3
"""Sample real scans to audit angular extrapolation in visibility evidence.

This diagnoses evidence geometry, not semantic motion or actual false deletions.
It never changes the delivered PCD files or the filter's classification.
"""
import argparse
import hashlib
import json
import pathlib

import numpy as np
from scipy.spatial import cKDTree

import yaml

from filter_dynamic_map import angular_support, correct_scan, load_trajectory, pose_index, ray_evidence, read_map


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=pathlib.Path, required=True)
    parser.add_argument('--delivery-root', type=pathlib.Path, required=True)
    parser.add_argument('--output', type=pathlib.Path, required=True)
    parser.add_argument('--frames', type=int, default=16)
    args = parser.parse_args()
    if args.frames < 2 or args.output.exists():
        parser.error('At least two sample frames and a fresh output path required')
    inventory = json.loads((pathlib.Path(__file__).resolve().parents[1] /
                            'config/dynamic_filtering_recordings.json').read_text())
    records = []
    for recording in inventory['recordings']:
        frontend = args.data_root / recording['frontend']
        backend = args.data_root / recording['backend']
        delivery = args.delivery_root / recording['id']
        report = json.loads((delivery / 'report.json').read_text())
        for filename, key in [('optimized_3cm.pcd', 'map_sha256'), ('trajectory.csv', 'trajectory_sha256')]:
            digest = hashlib.sha256((backend / filename).read_bytes()).hexdigest()
            if digest != recording[key] or digest != report[key]:
                raise ValueError('Baseline/evidence hash mismatch: ' + recording['id'] + '/' + filename)
        points = read_map(backend / 'optimized_3cm.pcd')
        with np.load(delivery / 'point_evidence.npz') as evidence:
            removed = evidence['removed'].copy()
        if removed.shape != (len(points),):
            raise ValueError('Evidence mask shape mismatch')
        parameters = yaml.safe_load((frontend / 'ros_parameters.yaml').read_text())
        extrinsic = np.asarray(parameters['mapping']['extrinsic_T'])
        original, old_rotations = load_trajectory(frontend / 'capture/trajectory.csv')
        optimized, new_rotations = load_trajectory(backend / 'trajectory.csv')
        capture = json.loads((frontend / 'capture/capture.json').read_text())
        target_frames = set(np.linspace(0, capture['counts']['output_cloud'] - 1,
                                       args.frames, dtype=int).tolist())
        # All removed candidates; fixed-seed retained controls, not static labels.
        rng = np.random.default_rng(20261003)
        retained = np.flatnonzero(~removed)
        controls = rng.choice(retained, min(len(retained), 10000), replace=False)
        ids = np.r_[np.flatnonzero(removed), controls]
        classes = removed[ids]
        totals = {name: dict(free_pairs=0, unsupported_pairs=0) for name in ['removed', 'retained_control']}
        unsupported = np.zeros(len(ids), dtype=bool)
        rows = []
        frame_index = 0
        for filename in capture['temporal_raw']:
            with np.load(frontend / 'capture' / filename) as chunk:
                lengths, stamps = chunk['lengths'], chunk['stamps']
                selected = [(j, frame_index + j) for j in range(len(stamps))
                            if frame_index + j in target_frames]
                frame_index += len(stamps)
                if not selected:
                    continue
                cloud = chunk['points']
                cuts = np.r_[0, np.cumsum(lengths)]
                for j, global_index in selected:
                    old, new = pose_index(original, stamps[j]), pose_index(optimized, stamps[j])
                    scan, origin = correct_scan(cloud[cuts[j]:cuts[j + 1]], original[old, 1:4],
                        old_rotations[old], optimized[new, 1:4], new_rotations[new], extrinsic)
                    delta = points[ids, :3].astype(float) - origin
                    ranges = np.linalg.norm(delta, axis=1)
                    eligible = np.flatnonzero((ranges > .35) & (ranges <= report['settings']['max_range']))
                    _, free = ray_evidence(points[ids[eligible], :3], scan, origin,
                        report['settings']['angle_deg'], report['settings']['hit_distance'],
                        report['settings']['free_margin'])
                    chosen = eligible[free]
                    scan_delta = scan - origin
                    scan_ranges = np.linalg.norm(scan_delta, axis=1)
                    valid = np.isfinite(scan_delta).all(axis=1) & (scan_ranges > .35)
                    directions = scan_delta[valid] / scan_ranges[valid, None]
                    if len(chosen):
                        query = delta[chosen] / ranges[chosen, None]
                        _, neighbours = cKDTree(directions).query(query, k=3, workers=2)
                        supported = angular_support(query, directions[neighbours])
                    else:
                        supported = np.zeros(0, dtype=bool)
                    unsupported[chosen[~supported]] = True
                    row = dict(frame=global_index, seconds=float(stamps[j] - optimized[0, 0]))
                    for name, mask in [('removed', classes[chosen]), ('retained_control', ~classes[chosen])]:
                        counts = dict(free_pairs=int(mask.sum()), unsupported_pairs=int((mask & ~supported).sum()))
                        row[name] = counts
                        for key, value in counts.items():
                            totals[name][key] += value
                    rows.append(row)
        records.append(dict(id=recording['id'],sample_frames=len(rows),
            map_sha256=report['map_sha256'],trajectory_sha256=report['trajectory_sha256'],
            candidate_points=int(classes.sum()),retained_control_points=int((~classes).sum()),
            totals=totals,unique_removed_candidates_with_unsupported_vote=int((unsupported & classes).sum()),
            unique_retained_controls_with_unsupported_vote=int((unsupported & ~classes).sum()),frames=rows))
        print(json.dumps(dict(id=recording['id'],totals=totals)), flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(
        scope='Sampled frame-point pairs. Unsupported means angular extrapolation, not a proven false deletion. '
              'No classification or PCD modification; final bin-count changes require full evidence recomputation.',
        recordings=records), indent=2) + '\n')


if __name__ == '__main__':
    main()
