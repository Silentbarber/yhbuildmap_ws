#!/usr/bin/env python3
"""Experimental sensor-motion supplement to an angular-guard visibility map.

Known device motion must explain a return better than a fixed-world model,
and measured rays must independently pass behind its old map position.
Unobserved or occluded locations are not empty. This is offline cleaning,
not a semantic body detector, and the existing trajectory is never changed.
"""
import argparse
from collections import deque
import csv
import hashlib
import json
import pathlib
import resource
import time

import numpy as np
import yaml
from scipy.spatial import cKDTree

from evaluate_independent_run import write_pcd
from filter_dynamic_map import correct_scan, load_trajectory, pose_index, ray_evidence, read_map


def sensor_transform(source_rotation, source_origin, target_rotation, target_origin):
    matrix = np.eye(4)
    relative = target_rotation @ source_rotation.T
    matrix[:3, :3] = relative
    matrix[:3, 3] = target_origin - relative @ source_origin
    return matrix


def motion_free_votes(points, measured, source_rotation, source_origin, target_rotation, target_origin,
                      match_distance=.04, world_distance=.10, min_motion=.10, free_margin=.10):
    accepted = np.zeros(len(points), dtype=bool)
    counts = dict(candidates=len(points), motion_matches=0, free_matches=0)
    if not len(points):
        return accepted, counts
    valid = np.isfinite(measured).all(axis=1) & (np.linalg.norm(measured-target_origin, axis=1) > .35)
    measured = measured[valid]
    if len(measured) < 3:
        return accepted, counts
    tree = cKDTree(measured)
    matrix = sensor_transform(source_rotation, source_origin, target_rotation, target_origin)
    predicted = points @ matrix[:3, :3].T + matrix[:3, 3]
    fixed_distance = tree.query(points, workers=2)[0]
    attached_distance = tree.query(predicted, workers=2)[0]
    matched = ((attached_distance <= match_distance) & (fixed_distance >= world_distance)
               & (np.linalg.norm(predicted-points, axis=1) >= min_motion))
    ids = np.flatnonzero(matched)
    counts['motion_matches'] = len(ids)
    if len(ids):
        _, free = ray_evidence(points[ids], measured, target_origin, angle_deg=.7,
            hit_distance=.05, free_margin=free_margin, require_angular_support=True)
        accepted[ids[free]] = True
        counts['free_matches'] = int(free.sum())
    return accepted, counts


def add_votes(ids, group, seconds, votes, last_bin, first, last):
    ids = np.unique(ids)
    fresh = ids[last_bin[ids] != group]
    votes[fresh] += 1
    last_bin[ids] = group
    first[ids] = np.minimum(first[ids], seconds)
    last[ids] = np.maximum(last[ids], seconds)


def additional_mask(base_removed, hits, votes, first, last, min_bins=3, min_span=1., max_hits=1):
    return (~base_removed & (hits <= max_hits) & (votes >= min_bins) & (last-first >= min_span))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frontend', type=pathlib.Path)
    parser.add_argument('backend', type=pathlib.Path)
    parser.add_argument('--visibility-dir', type=pathlib.Path, required=True)
    parser.add_argument('--output-dir', type=pathlib.Path, required=True)
    parser.add_argument('--frame-step', type=float, default=.3)
    parser.add_argument('--max-source-range', type=float, default=1.5)
    parser.add_argument('--evidence-bin', type=float, default=.5)
    parser.add_argument('--min-votes', type=int, default=3)
    parser.add_argument('--min-span', type=float, default=1.)
    parser.add_argument('--max-hit-bins', type=int, choices=[0, 1, 2, 3], default=1)
    args = parser.parse_args()
    if not .2 <= args.frame_step <= .5 or not .75 <= args.max_source_range <= 2:
        parser.error('frame-step must be .2--.5s and max-source-range .75--2m')
    if not .3 <= args.evidence_bin <= 1 or args.min_votes < 3 or not 1 <= args.min_span <= 10:
        parser.error('evidence-bin must be .3--1s, min-votes >=3 and min-span 1--10s')
    output = args.output_dir.resolve()
    if output.exists():
        parser.error('Use a fresh output directory')
    started = time.monotonic()
    frontend, backend, visibility = args.frontend.resolve(), args.backend.resolve(), args.visibility_dir.resolve()
    backend_report = json.loads((backend/'report.json').read_text())
    prior = json.loads((visibility/'report.json').read_text())
    if pathlib.Path(backend_report['source_run']).resolve() != frontend:
        raise ValueError('Backend belongs to a different frontend')
    if (pathlib.Path(prior['frontend']).resolve() != frontend or pathlib.Path(prior['backend']).resolve() != backend
            or not prior['settings'].get('require_angular_support')):
        raise ValueError('Expected same-run angular-guard visibility evidence')
    if json.loads((frontend/'evaluation.json').read_text()).get('failures'):
        raise ValueError('Incomplete frontend cannot be filtered')
    provenance = {}
    for name, key in [('optimized_3cm.pcd', 'map_sha256'), ('trajectory.csv', 'trajectory_sha256')]:
        digest = hashlib.sha256((backend/name).read_bytes()).hexdigest()
        if digest != prior[key]:
            raise ValueError('Visibility source mismatch: '+name)
        provenance[key] = digest
    params = yaml.safe_load((frontend/'ros_parameters.yaml').read_text())
    if params['mapping']['extrinsic_est_en']:
        raise ValueError('Changing extrinsics require per-frame records')
    extrinsic = np.asarray(params['mapping']['extrinsic_T'], dtype=float)
    if extrinsic.shape != (3,) or not np.isfinite(extrinsic).all():
        raise ValueError('Invalid LiDAR-to-IMU translation')
    old, old_rotations = load_trajectory(frontend/'capture/trajectory.csv')
    new, new_rotations = load_trajectory(backend/'trajectory.csv')
    capture = json.loads((frontend/'capture/capture.json').read_text())
    frames = capture['counts']['output_cloud']
    if frames != prior['input_frames'] or frames != backend_report['input_frames']:
        raise ValueError('Capture and visibility frame counts differ')
    if (new[-1, 0]-new[0, 0])/args.evidence_bin >= np.iinfo(np.uint16).max:
        raise ValueError('Recording exceeds evidence counter capacity')
    points = read_map(backend/'optimized_3cm.pcd')
    with np.load(visibility/'point_evidence.npz') as saved:
        arrays = {key: saved[key].copy() for key in saved.files}
    base_removed, hits = arrays['removed'], arrays['hit_bins']
    if any(array.shape != (len(points),) for array in arrays.values()) or base_removed.dtype != np.bool_:
        raise ValueError('Invalid visibility evidence shape or mask type')
    if np.any(base_removed & (hits >= 4)):
        raise ValueError('Visibility mask already removed repeatedly supported points')
    if not np.array_equal(read_map(visibility/'filtered_3cm.pcd'), points[~base_removed]):
        raise ValueError('Visibility PCD differs from its evidence mask')
    map_xyz = points[:, :3].astype(float)
    map_tree = cKDTree(map_xyz)
    votes = np.zeros(len(points), dtype=np.uint16)
    last_bin = np.full(len(points), -1, dtype=np.int32)
    first, last = np.full(len(points), np.inf), np.full(len(points), -np.inf)
    eligible = ~base_removed & (hits <= args.max_hit_bins)
    queue, rows, source_hashes = deque(), [], {}
    lags = [.6, 1.2, 1.8]
    last_selected, input_frames, selected = -np.inf, 0, 0
    for filename in capture['temporal_raw']:
        path = frontend/'capture'/filename
        source_hashes[filename] = hashlib.sha256(path.read_bytes()).hexdigest()
        with np.load(path) as archive:
            cuts = np.r_[0, np.cumsum(archive['lengths'])]
            scans = archive['points']
            for j, stamp in enumerate(archive['stamps']):
                input_frames += 1
                a, b = pose_index(old, stamp), pose_index(new, stamp)
                if stamp-last_selected < args.frame_step:
                    continue
                last_selected = stamp
                raw = scans[cuts[j]:cuts[j+1]]
                measured, origin = correct_scan(raw, old[a, 1:4], old_rotations[a], new[b, 1:4],
                                               new_rotations[b], extrinsic)
                rotation = new_rotations[b]
                seconds = float(stamp-new[0, 0])
                group = int(seconds/args.evidence_bin)
                while queue and stamp-queue[0]['stamp'] > max(lags)+.18:
                    queue.popleft()
                used = set()
                row = dict(stamp=float(stamp), seconds=seconds, source_candidates=0,
                           source_pairs=0, motion_matches=0, free_matches=0)
                for lag in lags:
                    if not queue:
                        break
                    source = min(queue, key=lambda item: abs(stamp-item['stamp']-lag))
                    if abs(stamp-source['stamp']-lag) > .18 or source['stamp'] in used:
                        continue
                    used.add(source['stamp'])
                    ids = source['ids']
                    accepted, counts = motion_free_votes(map_xyz[ids], measured, source['rotation'], source['origin'], rotation, origin)
                    add_votes(ids[accepted], group, seconds, votes, last_bin, first, last)
                    row['source_pairs'] += 1
                    row['source_candidates'] += len(ids)
                    row['motion_matches'] += counts['motion_matches']
                    row['free_matches'] += counts['free_matches']
                distances = np.linalg.norm(measured-origin, axis=1)
                near = measured[(distances > .35) & (distances <= args.max_source_range)]
                if len(near):
                    distance, nearest = map_tree.query(near, workers=2)
                    ids = np.unique(nearest[(distance <= .04) & eligible[nearest]])
                    ids = ids[np.linalg.norm(map_xyz[ids]-origin, axis=1) <= args.max_source_range]
                else:
                    ids = np.empty(0, dtype=np.int64)
                queue.append(dict(stamp=float(stamp), origin=origin, rotation=rotation, ids=ids))
                rows.append(row)
                selected += 1
        print('sensor motion: {} / {} source frames'.format(input_frames, frames), flush=True)
    if input_frames != frames:
        raise ValueError('Incomplete captured input')
    added = additional_mask(base_removed, hits, votes, first, last, args.min_votes, args.min_span, args.max_hit_bins)
    removed = base_removed | added
    output.mkdir(parents=True)
    write_pcd(output/'filtered_3cm.pcd', points[~removed])
    write_pcd(output/'removed_3cm.pcd', points[removed])
    write_pcd(output/'additional_removed_3cm.pcd', points[added])
    arrays.update(removed=removed, sensor_removed=added, sensor_bins=votes, sensor_first=first, sensor_last=last)
    np.savez_compressed(output/'point_evidence.npz', **arrays)
    with (output/'frame_evidence.csv').open('w') as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    result = dict(status='experimental_sensor_motion_supplement_requires_scene_review',
        frontend=str(frontend), backend=str(backend), visibility_dir=str(visibility), **provenance,
        visibility_evidence_sha256=hashlib.sha256((visibility/'point_evidence.npz').read_bytes()).hexdigest(),
        source_archives_sha256=source_hashes, input_frames=input_frames, evidence_frames=selected,
        baseline_points=len(points), filtered_points=int((~removed).sum()), removed_points=int(removed.sum()),
        prior_removed_points=int(base_removed.sum()), additional_removed_points=int(added.sum()),
        motion_matched_returns=sum(row['motion_matches'] for row in rows),
        ray_confirmed_returns=sum(row['free_matches'] for row in rows),
        sensor_vote_points=int((votes > 0).sum()), supported_points_removed=int(((hits >= 4) & removed).sum()),
        additional_supported_points_removed=int(((hits >= 4) & added).sum()), trajectory_unchanged=True,
        settings={key: value for key, value in vars(args).items() if key not in ['frontend', 'backend', 'visibility_dir', 'output_dir']},
        source_lags_s=lags, lag_tolerance_s=.18, match_distance_m=.04, world_distance_m=.10,
        min_motion_m=.10, ray_free_margin_m=.10, angular_support_required=True,
        filter_mode='Offline exact baseline subset, no extra map downsampling or spatial crop. '
                    'Only near-sensor, weakly supported source associations are considered for additional removal.',
        detector='Known sensor motion plus independent measured-ray empty-space evidence; no semantic labels or ICP fit',
        limitations='Geometry aliasing, glass, pose errors and angular interpolation remain possible. '
                    'Zero added points is valid and does not establish absence of residual dynamics. No precision/recall ground truth.',
        wall_seconds=time.monotonic()-started, peak_rss_mb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)
    result['output_bytes'] = sum(path.stat().st_size for path in output.iterdir() if path.is_file())
    (output/'report.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({key: result[key] for key in ['input_frames', 'evidence_frames', 'additional_removed_points', 'wall_seconds']}, indent=2))


if __name__ == '__main__':
    main()
