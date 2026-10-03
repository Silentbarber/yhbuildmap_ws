#!/usr/bin/env python3
"""Audit physical ray spacing in angular-guard deletion candidates.

Replays the original evidence samples, then bounds the perpendicular distance
from a candidate to each of its three supporting measured rays. This diagnoses
interpolation scale, not semantic motion or confirmed static false deletions.
No delivered map or trajectory is modified.
"""
import argparse
import hashlib
import json
import pathlib
import resource
import time

import numpy as np
import yaml
from scipy.spatial import cKDTree

from filter_dynamic_map import correct_scan, load_trajectory, pose_index, ray_evidence, read_map, removal_mask


def ray_footprint_radius(points, measured, origin, angle_deg):
    """Maximum distance to the three forward supporting rays, in metres."""
    scan = measured-origin
    ranges = np.linalg.norm(scan, axis=1)
    valid = np.isfinite(scan).all(axis=1) & (ranges > .35)
    directions = scan[valid]/ranges[valid, None]
    radii = np.full(len(points), np.inf)
    if not len(points) or len(directions) < 3:
        return radii
    delta = points-origin
    target_range = np.linalg.norm(delta, axis=1)
    query = delta/np.maximum(target_range[:, None], 1e-12)
    cone = 2*np.sin(np.deg2rad(angle_deg)/2)
    _, nearest = cKDTree(directions).query(query, k=3, distance_upper_bound=cone, workers=2)
    available = (nearest < len(directions)).all(axis=1) & (target_range > .35)
    ids = np.flatnonzero(available)
    if len(ids):
        rays = directions[nearest[ids]]
        projection = np.einsum('nki,ni->nk', rays, delta[ids])
        perpendicular = delta[ids, None, :]-projection[:, :, None]*rays
        radii[ids] = np.linalg.norm(perpendicular, axis=2).max(axis=1)
        radii[ids[(projection <= 0).any(axis=1)]] = np.inf
    return radii


def count_bins(ids, group, seconds, counts, last_bin, first, last):
    fresh = ids[last_bin[ids] != group]
    counts[fresh] += 1
    last_bin[ids] = group
    first[ids] = np.minimum(first[ids], seconds)
    last[ids] = seconds


def origin_motion_summary(poses, rotations, extrinsic):
    origins = poses[:, 1:4]+np.einsum('nij,j->ni', rotations, extrinsic)
    gaps = np.diff(poses[:, 0])
    displacement = np.linalg.norm(np.diff(origins, axis=0), axis=1)
    eligible = (gaps > .075) & (gaps < .125)
    values = displacement[eligible]
    return dict(trajectory_poses=len(poses), approximately_100ms_pairs=int(eligible.sum()),
        median_pair_interval_s=float(np.median(gaps[eligible])) if len(values) else None,
        origin_displacement_m_p50_p95_max=np.percentile(values, [50, 95, 100]).tolist() if len(values) else None,
        pairs_exceeding_5cm=int((values > .05).sum()), pairs_exceeding_10cm=int((values > .10).sum()))


def audit_origin_motion(record, data_root, output):
    """Diagnose the shared end-of-scan ray-origin approximation, not beam truth."""
    target = output/'origin_motion_report.json'
    if target.exists():
        raise ValueError('Origin motion report already exists')
    report = json.loads((output/'report.json').read_text())
    frontend, backend = data_root/record['frontend'], data_root/record['backend']
    for name, key in [('optimized_3cm.pcd', 'map_sha256'), ('trajectory.csv', 'trajectory_sha256')]:
        digest = hashlib.sha256((backend/name).read_bytes()).hexdigest()
        if digest != record[key] or digest != report[key]:
            raise ValueError('Origin audit source hash mismatch: '+name)
    params_path = frontend/'ros_parameters.yaml'
    if hashlib.sha256(params_path.read_bytes()).hexdigest() != report['metadata_sha256']['ros_parameters.yaml']:
        raise ValueError('Origin audit parameters changed')
    params = yaml.safe_load(params_path.read_text())
    extrinsic = np.asarray(params['mapping']['extrinsic_T'], dtype=float)
    if params['mapping']['extrinsic_est_en'] or extrinsic.shape != (3,) or not np.isfinite(extrinsic).all():
        raise ValueError('Expected fixed finite extrinsic translation')
    poses, rotations = load_trajectory(backend/'trajectory.csv')
    result = dict(id=record['id'], **origin_motion_summary(poses, rotations, extrinsic),
        map_sha256=report['map_sha256'], trajectory_sha256=report['trajectory_sha256'],
        parameters_sha256=report['metadata_sha256']['ros_parameters.yaml'],
        dense_publish_en=params['publish']['dense_publish_en'],
        scope='Adjacent approximately 0.1s optimized end-of-scan LiDAR origins including the translation extrinsic. '
              'Current evidence rays share the frame-end origin although endpoints have been deskewed. '
              'These displacements quantify the scale of that approximation, not actual per-point beam origins, '
              'true intra-scan maximum displacement, pose error or confirmed false deletion. No PCD modified.')
    target.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    return result


def compare_structure(record, data_root, evidence_root, output):
    """Compare fixed geometry using completed footprint evidence, without replay."""
    from audit_dynamic_filter import planar_sample_ids
    from sweep_dynamic_filter_params import static_patch_counts

    target = output/'structural_report.json'
    if target.exists():
        raise ValueError('Use a fresh structural report path')
    backend = data_root/record['backend']
    evidence = evidence_root/record['id']
    report = json.loads((output/'report.json').read_text())
    for name, key in [('optimized_3cm.pcd', 'map_sha256'), ('trajectory.csv', 'trajectory_sha256')]:
        digest = hashlib.sha256((backend/name).read_bytes()).hexdigest()
        if digest != record[key] or digest != report[key]:
            raise ValueError('Structural source hash mismatch: '+name)
    if hashlib.sha256((evidence/'point_evidence.npz').read_bytes()).hexdigest() != report['parent_evidence_sha256']:
        raise ValueError('Parent evidence changed')
    points = read_map(backend/'optimized_3cm.pcd')
    with np.load(evidence/'point_evidence.npz') as archive:
        parent_removed = archive['removed'].copy()
        parent_hits, parent_free = archive['hit_bins'].copy(), archive['free_bins'].copy()
    with np.load(output/'point_audit.npz') as archive:
        ids, masks, gaps = archive['baseline_indices'], archive['deletion_masks'], archive['max_ray_gap_m']
        bins, first, last, hits = archive['free_bins'], archive['first_free'], archive['last_free'], archive['hit_bins']
        if (ids.dtype.kind not in 'iu' or len(np.unique(ids)) != len(ids) or np.any(ids >= len(points))
                or np.any(ids < 0) or masks.dtype != np.bool_ or masks.shape != (len(gaps)+1, len(ids))
                or bins.shape != masks.shape or first.shape != masks.shape or last.shape != masks.shape
                or hits.shape != (len(ids),) or not np.array_equal(gaps, [r['max_ray_gap_m'] for r in report['comparisons']])):
            raise ValueError('Invalid footprint arrays')
        settings = report['settings']
        for row, mask in enumerate(masks):
            expected = removal_mask(hits, bins[row], first[row], last[row], settings['min_free_bins'],
                settings['free_ratio'], settings['min_span'], settings['max_hit_bins'])
            if not np.array_equal(mask, expected) or np.any(mask & ~masks[0]):
                raise ValueError('Footprint classification mismatch')
        if (not np.array_equal(masks[0], parent_removed[ids]) or masks[0].sum() != parent_removed.sum()
                or not np.array_equal(hits, parent_hits[ids]) or not np.array_equal(bins[0], parent_free[ids])):
            raise ValueError('Footprint audit does not cover the entire parent deletion set')
    planar_ids = planar_sample_ids(points)
    patches = backend.parent/'coverage_geometry_v5_validation/surface_patches/patches.json'
    original_patches = static_patch_counts(points, parent_removed, patches)
    original_planar_removed = int(parent_removed[planar_ids].sum())
    comparisons = []
    for gap, mask in zip(gaps, masks[1:]):
        removed = parent_removed.copy()
        removed[ids] = mask
        patch_counts = static_patch_counts(points, removed, patches)
        comparisons.append(dict(max_ray_gap_m=float(gap), removed_points=int(removed.sum()),
            restored_points=int((parent_removed & ~removed).sum()),
            planar_sample_removed=int(removed[planar_ids].sum()),
            planar_sample_restored=original_planar_removed-int(removed[planar_ids].sum()),
            fixed_patch_removed=sum(item['removed'] for item in patch_counts), fixed_patches=patch_counts))
    result = dict(id=record['id'], baseline_points=len(points), original_removed=int(parent_removed.sum()),
        planar_sample_points=len(planar_ids), original_planar_sample_removed=original_planar_removed,
        original_fixed_patch_removed=sum(item['removed'] for item in original_patches), original_fixed_patches=original_patches,
        map_sha256=report['map_sha256'], trajectory_sha256=report['trajectory_sha256'],
        point_audit_sha256=hashlib.sha256((output/'point_audit.npz').read_bytes()).hexdigest(), comparisons=comparisons,
        scope='Same deterministic local PCA sample and existing fixed baseline-coordinate reference patches. '
              'All original deleted points evaluated; original retained map is preserved by construction. '
              'Geometry candidates and fixed reference patches are not semantic static labels. No PCD modified.')
    target.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(id=record['id'], original_planar_removed=original_planar_removed,
        comparisons=[{key: value for key, value in row.items() if key != 'fixed_patches'} for row in comparisons])), flush=True)
    return result


def audit_record(record, data_root, evidence_root, review_root, output, radii, controls):
    started = time.monotonic()
    frontend, backend = data_root/record['frontend'], data_root/record['backend']
    evidence = evidence_root/record['id']
    prior = json.loads((evidence/'report.json').read_text())
    source = json.loads((review_root/record['id']/'report.json').read_text())
    settings = prior['settings']
    if (not settings.get('require_angular_support') or
            pathlib.Path(prior['frontend']).resolve() != frontend.resolve() or
            pathlib.Path(prior['backend']).resolve() != backend.resolve()):
        raise ValueError('Expected same-run angular-guard parent')
    if pathlib.Path(source['source_capture']).resolve() != (frontend/'capture').resolve():
        raise ValueError('Source audit belongs to a different capture')
    for name, key in [('optimized_3cm.pcd', 'map_sha256'), ('trajectory.csv', 'trajectory_sha256')]:
        digest = hashlib.sha256((backend/name).read_bytes()).hexdigest()
        if any(digest != manifest[key] for manifest in [record, prior, source]):
            raise ValueError('Map/trajectory hash mismatch: '+name)
    for name, digest in source['source_metadata_sha256'].items():
        if hashlib.sha256((frontend/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Source metadata changed: '+name)
    params = yaml.safe_load((frontend/'ros_parameters.yaml').read_text())
    if params['mapping']['extrinsic_est_en']:
        raise ValueError('Per-frame extrinsics required')
    extrinsic = np.asarray(params['mapping']['extrinsic_T'], dtype=float)
    if extrinsic.shape != (3,) or not np.isfinite(extrinsic).all():
        raise ValueError('Invalid extrinsic translation')
    original, old_rotations = load_trajectory(frontend/'capture/trajectory.csv')
    optimized, new_rotations = load_trajectory(backend/'trajectory.csv')
    capture = json.loads((frontend/'capture/capture.json').read_text())
    points = read_map(backend/'optimized_3cm.pcd')
    with np.load(evidence/'point_evidence.npz') as archive:
        saved = {key: archive[key].copy() for key in archive.files}
    if (saved['removed'].dtype != np.bool_ or any(value.shape != (len(points),) for value in saved.values())
            or np.any(saved['removed'] & (saved['hit_bins'] >= 4))):
        raise ValueError('Invalid or unsafe parent evidence')
    if not np.array_equal(read_map(evidence/'filtered_3cm.pcd'), points[~saved['removed']]):
        raise ValueError('Parent map differs from evidence')
    retained = np.flatnonzero(~saved['removed'])
    rng = np.random.default_rng(20261004)
    control_ids = rng.choice(retained, min(controls, len(retained)), replace=False)
    ids = np.r_[np.flatnonzero(saved['removed']), control_ids]
    candidate_count = int(saved['removed'].sum())
    xyz = points[ids, :3].astype(float)
    n = len(ids)
    # Row zero repeats the exact existing evidence; other rows restrict its
    # free votes without changing hit counts or the original time sampling.
    limits = np.r_[np.inf, radii]
    bins = np.zeros((len(limits), n), dtype=np.uint16)
    bin_seen = np.full(bins.shape, -1, dtype=np.int32)
    first = np.full(bins.shape, np.inf)
    last = np.full(bins.shape, -np.inf)
    hits, hit_seen = np.zeros(n, np.uint16), np.full(n, -1, np.int32)
    hit_first, hit_last = np.full(n, np.inf), np.full(n, -np.inf)
    frame_count, selected, last_selected = 0, 0, -np.inf
    source_hashes = {item['file']: item['sha256'] for item in source['source_archives']}
    if set(source_hashes) != set(capture['temporal_raw']):
        raise ValueError('Capture archive inventory mismatch')
    footprint_sum, footprint_max, free_pairs = 0., 0., 0
    for filename in capture['temporal_raw']:
        path = frontend/'capture'/filename
        if hashlib.sha256(path.read_bytes()).hexdigest() != source_hashes[filename]:
            raise ValueError('Source archive changed: '+filename)
        with np.load(path) as archive:
            cuts = np.r_[0, np.cumsum(archive['lengths'])]
            scans = archive['points']
            for j, stamp in enumerate(archive['stamps']):
                frame_count += 1
                a, b = pose_index(original, stamp), pose_index(optimized, stamp)
                if stamp-last_selected < settings['frame_step']:
                    continue
                last_selected = stamp
                selected += 1
                scan, origin = correct_scan(scans[cuts[j]:cuts[j+1]], original[a, 1:4], old_rotations[a],
                    optimized[b, 1:4], new_rotations[b], extrinsic)
                ranges = np.linalg.norm(xyz-origin, axis=1)
                eligible = np.flatnonzero((ranges > .35) & (ranges <= settings['max_range']))
                hit, free = ray_evidence(xyz[eligible], scan, origin, settings['angle_deg'],
                    settings['hit_distance'], settings['free_margin'], require_angular_support=True)
                seconds = float(stamp-optimized[0, 0])
                group = int(seconds/settings['evidence_bin'])
                count_bins(eligible[hit], group, seconds, hits, hit_seen, hit_first, hit_last)
                free_ids = eligible[free]
                radius = ray_footprint_radius(xyz[free_ids], scan, origin, settings['angle_deg'])
                if not np.isfinite(radius).all():
                    raise ValueError('Free evidence lacks three forward rays')
                removed_radius = radius[free_ids < candidate_count]
                if len(removed_radius):
                    free_pairs += len(removed_radius)
                    footprint_sum += removed_radius.sum()
                    footprint_max = max(footprint_max, float(removed_radius.max()))
                for row, limit in enumerate(limits):
                    chosen = free_ids[radius <= limit]
                    count_bins(chosen, group, seconds, bins[row], bin_seen[row], first[row], last[row])
        print('{} footprint audit: {} / {} frames'.format(record['id'], frame_count, record['frames']), flush=True)
    if frame_count != record['frames'] or frame_count != prior['input_frames'] or selected != prior['evidence_frames']:
        raise ValueError('Incomplete source or evidence sampling mismatch')
    for key, actual in [('hit_bins', hits), ('free_bins', bins[0]), ('first_free', first[0]), ('last_free', last[0])]:
        if not np.array_equal(saved[key][ids], actual):
            raise ValueError('Original evidence did not reproduce: '+key)
    masks = np.asarray([removal_mask(hits, row_bins, row_first, row_last, settings['min_free_bins'],
        settings['free_ratio'], settings['min_span'], settings['max_hit_bins'])
        for row_bins, row_first, row_last in zip(bins, first, last)])
    if not np.array_equal(masks[0], saved['removed'][ids]) or np.any(masks & ~masks[0]):
        raise ValueError('Deletion partition or subset invariant failed')
    comparisons = [dict(max_ray_gap_m=float(limit), remaining_deletion_candidates=int(mask[:candidate_count].sum()),
        no_longer_supported_candidates=int((~mask[:candidate_count]).sum()),
        retained_controls_newly_deleted=int(mask[candidate_count:].sum())) for limit, mask in zip(radii, masks[1:])]
    output.mkdir(parents=True)
    np.savez_compressed(output/'point_audit.npz', baseline_indices=ids, max_ray_gap_m=radii,
        original_removed=saved['removed'][ids], hit_bins=hits, free_bins=bins, first_free=first,
        last_free=last, deletion_masks=masks)
    result = dict(id=record['id'], status='physical_ray_spacing_audit_requires_scene_review',
        input_frames=frame_count, evidence_frames=selected, candidate_points=candidate_count,
        retained_control_points=len(control_ids), map_sha256=prior['map_sha256'],
        trajectory_sha256=prior['trajectory_sha256'], source_archives_sha256=source_hashes,
        metadata_sha256=source['source_metadata_sha256'],
        parent_evidence_sha256=hashlib.sha256((evidence/'point_evidence.npz').read_bytes()).hexdigest(),
        parent_evidence_reproduced=True, settings=settings, comparisons=comparisons,
        candidate_free_frame_point_pairs=free_pairs,
        mean_support_radius_m=float(footprint_sum/free_pairs) if free_pairs else None,
        max_support_radius_m=footprint_max if free_pairs else None,
        scope='All original deletion candidates and fixed-seed retained controls. All source pose associations checked; '
              'original evidence time samples replayed. Bounds apply to all three supporting rays, not just the nearest.',
        limitations='Restricted free-space votes diagnose physical interpolation scale; they do not prove static '
                     'false deletions or semantic motion. Registration error, beam divergence and thin structures '
                     'remain uncertain. Deskewed endpoints use a shared frame-end origin rather than individual '
                     'emission-time origins, so these are approximate evidence rays. No PCD or trajectory modified.',
        wall_seconds=time.monotonic()-started, peak_rss_mb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)
    (output/'report.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(id=record['id'], comparisons=comparisons)), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=pathlib.Path, required=True)
    parser.add_argument('--evidence-root', type=pathlib.Path, required=True)
    parser.add_argument('--review-root', type=pathlib.Path, required=True)
    parser.add_argument('--output-root', type=pathlib.Path, required=True)
    parser.add_argument('--id', choices=['215247', '162102', '162342', '162744'], action='append')
    parser.add_argument('--max-ray-gap', type=float, nargs='+', default=[.02, .03, .05, .10, .15])
    parser.add_argument('--retained-controls', type=int, default=10000)
    followup = parser.add_mutually_exclusive_group()
    followup.add_argument('--structure-only', action='store_true',
                          help='Compare fixed geometry using completed output-root evidence, without rescanning')
    followup.add_argument('--origin-motion-only', action='store_true',
                          help='Audit shared end-of-scan ray origin approximation using completed output-root')
    args = parser.parse_args()
    radii = np.asarray(sorted(set(args.max_ray_gap)))
    if not np.isfinite(radii).all() or np.any((radii < .005) | (radii > .2)) or args.retained_controls < 0:
        parser.error('Ray gap .005--.2 m and nonnegative control count required')
    inventory = json.loads((pathlib.Path(__file__).resolve().parents[1]/'config/dynamic_filtering_recordings.json').read_text())
    records = [record for record in inventory['recordings'] if not args.id or record['id'] in args.id]
    if args.origin_motion_only:
        if any((args.output_root/record['id']/'origin_motion_report.json').exists() for record in records):
            parser.error('Origin motion report already exists')
        for record in records:
            audit_origin_motion(record, args.data_root, args.output_root/record['id'])
        return
    if args.structure_only:
        if any((args.output_root/record['id']/'structural_report.json').exists() for record in records):
            parser.error('Structural report already exists')
        for record in records:
            compare_structure(record, args.data_root, args.evidence_root, args.output_root/record['id'])
        return
    if any((args.output_root/record['id']).exists() for record in records):
        parser.error('Use fresh output directories')
    for record in records:
        audit_record(record, args.data_root, args.evidence_root, args.review_root,
                     args.output_root/record['id'], radii, args.retained_controls)


if __name__ == '__main__':
    main()
