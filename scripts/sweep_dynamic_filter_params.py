#!/usr/bin/env python3
"""Compare removal thresholds using already acquired visibility evidence.

This never replays a bag and never writes a PCD. It reports parameter
sensitivity so a candidate is not selected from deletion count alone.
"""
import argparse
import hashlib
import json
import pathlib

import numpy as np

from audit_dynamic_filter import planar_sample_ids
from filter_dynamic_map import read_map, removal_mask


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_MIN_FREE = [4, 6, 8]
DEFAULT_RATIOS = [.8, .9, 1.0]
DEFAULT_SPANS = [6, 10, 20]
DEFAULT_HITS = [0, 1, 3]


def static_patch_counts(points, removed, patches_path):
    if not patches_path.exists():
        return []
    records = []
    for patch in json.loads(patches_path.read_text()):
        anchor = np.asarray(patch['anchor'])
        normal = np.asarray(patch['normal'])
        tangent = np.asarray(patch['tangent'])
        projected = (points[:, :3] - anchor) @ tangent
        residual = points[:, :3] @ normal + patch['offset']
        mask = (np.linalg.norm(projected, axis=1) < .35) & (np.abs(residual) < .03)
        records.append(dict(key=patch['key'], points=int(mask.sum()),
                            removed=int((mask & removed).sum())))
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=pathlib.Path, required=True)
    parser.add_argument('--evidence-root', type=pathlib.Path, required=True,
                        help='Root containing <id>/point_evidence.npz and report.json')
    parser.add_argument('--output', type=pathlib.Path, required=True)
    parser.add_argument('--min-free', type=int, nargs='+', default=DEFAULT_MIN_FREE)
    parser.add_argument('--ratio', type=float, nargs='+', default=DEFAULT_RATIOS)
    parser.add_argument('--span', type=float, nargs='+', default=DEFAULT_SPANS)
    parser.add_argument('--max-hit', type=int, nargs='+', default=DEFAULT_HITS)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a fresh output path')
    if (any(value < 2 for value in args.min_free) or any(not .5 <= value <= 1 for value in args.ratio)
            or any(not np.isfinite(value) or value < 2 for value in args.span)
            or any(value < 0 for value in args.max_hit)):
        parser.error('At least two free bins, ratio .5--1, finite span >=2s and nonnegative max-hit required')
    inventory = json.loads((ROOT/'config/dynamic_filtering_recordings.json').read_text())
    settings = [(free, ratio, span, hit) for free in args.min_free for ratio in args.ratio
                for span in args.span for hit in args.max_hit]
    if len(settings) > 500:
        parser.error('At most 500 parameter combinations')
    recordings = []
    for record in inventory['recordings']:
        backend = args.data_root / record['backend']
        evidence = args.evidence_root / record['id']
        report = json.loads((evidence/'report.json').read_text())
        map_path = backend/'optimized_3cm.pcd'
        map_hash = hashlib.sha256(map_path.read_bytes()).hexdigest()
        if map_hash != record['map_sha256'] or map_hash != report['map_sha256']:
            raise ValueError('Map provenance mismatch: '+record['id'])
        trajectory_hash=hashlib.sha256((backend/'trajectory.csv').read_bytes()).hexdigest()
        if trajectory_hash != record['trajectory_sha256'] or trajectory_hash != report['trajectory_sha256']:
            raise ValueError('Trajectory provenance mismatch: '+record['id'])
        if report['input_frames'] != record['frames'] or not report['settings']['require_angular_support']:
            raise ValueError('Unexpected evidence frame count or angular support setting')
        points = read_map(map_path)
        with np.load(evidence/'point_evidence.npz') as arrays:
            required = {key: arrays[key].copy() for key in
                        ['hit_bins', 'free_bins', 'first_free', 'last_free']}
        if any(array.shape != (len(points),) for array in required.values()):
            raise ValueError('Evidence shape mismatch: '+record['id'])
        planar_ids=planar_sample_ids(points)
        candidates = []
        for free, ratio, span, hit in settings:
            removed = removal_mask(required['hit_bins'], required['free_bins'],
                                   required['first_free'], required['last_free'],
                                   free, ratio, span, hit)
            patches = static_patch_counts(points, removed,
                backend.parent/'coverage_geometry_v5_validation/surface_patches/patches.json')
            candidates.append(dict(min_free_bins=free, free_ratio=ratio, min_span_s=span,
                max_hit_bins=hit, removed_points=int(removed.sum()),
                removed_fraction=float(removed.mean()),
                planar_sample_points=len(planar_ids),planar_sample_removed=int(removed[planar_ids].sum()),
                supported_points_removed=int(((required['hit_bins']>=4)&removed).sum()),
                static_patch_removed=sum(p['removed'] for p in patches),
                static_patch_points=sum(p['points'] for p in patches),
                static_patches=patches))
        recordings.append(dict(id=record['id'], bag=record['bag'], map_sha256=map_hash,
                               trajectory_sha256=trajectory_hash,
                               evidence_sha256=hashlib.sha256((evidence/'point_evidence.npz').read_bytes()).hexdigest(),
                               baseline_points=len(points), settings=report['settings'], candidates=candidates))
        print(record['id'], 'swept', len(candidates), 'parameter combinations', flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(status='threshold_sensitivity_only',
        source='angular-support evidence; no bag replay or PCD writes',
        combinations=[dict(min_free_bins=a, free_ratio=b, min_span_s=c, max_hit_bins=d)
                      for a,b,c,d in settings], recordings=recordings,
        interpretation='Deletion counts and static patch removal are internal sensitivity '
                       'signals, not dynamic precision/recall or static ground truth.'), indent=2)+'\n')


if __name__ == '__main__':
    main()
