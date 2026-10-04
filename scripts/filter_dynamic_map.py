#!/usr/bin/env python3
"""Experimental offline visibility filter for a fixed Faster-LIO baseline map.

Uses SciPy's cKDTree for measured-ray neighbourhoods and Open3D's PCD parser.
This is a custom evidence filter, not an implementation of Removert/Dynablox.
"""
import argparse
import csv
import hashlib
import json
import pathlib
import resource
import time

import numpy as np
import open3d as o3d
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from evaluate_independent_run import write_pcd


def read_map(path):
    cloud = o3d.t.io.read_point_cloud(str(path))
    if 'intensity' not in cloud.point:
        raise ValueError('Expected the XYZI baseline PCD, including intensity')
    points = np.column_stack((cloud.point.positions.numpy(),
                              cloud.point.intensity.numpy().reshape(-1)))
    if not len(points) or not np.isfinite(points).all():
        raise ValueError('Map is empty or contains nonfinite values')
    return points.astype(np.float32)


def load_trajectory(path):
    poses = np.atleast_2d(np.loadtxt(path, delimiter=',', skiprows=1))
    if poses.shape[1] != 8 or not np.isfinite(poses).all():
        raise ValueError('Expected finite timestamp/position/quaternion trajectory')
    if np.any(np.diff(poses[:, 0]) <= 0):
        raise ValueError('Trajectory timestamps must be strictly increasing')
    if np.max(np.abs(np.linalg.norm(poses[:, 4:8], axis=1) - 1)) > 1e-3:
        raise ValueError('Nonunit trajectory quaternions')
    return poses, Rotation.from_quat(poses[:, 4:8]).as_matrix()


def pose_index(trajectory, stamp):
    index = int(np.searchsorted(trajectory[:, 0], stamp))
    options = [i for i in (index - 1, index) if 0 <= i < len(trajectory)]
    index = min(options, key=lambda i: abs(trajectory[i, 0] - stamp))
    if abs(trajectory[index, 0] - stamp) > 1e-5:
        raise ValueError('No same-timestamp pose for an output scan')
    return index


def correct_scan(points, old_position, old_rotation, new_position, new_rotation, extrinsic_t):
    rotation = new_rotation @ old_rotation.T
    translation = new_position - rotation @ old_position
    corrected = points[:, :3] @ rotation.T + translation
    origin = new_position + new_rotation @ extrinsic_t
    return corrected, origin


def angular_support(query, neighbours, tolerance=1e-14):
    """Three nearby rays must surround the query in its local tangent plane.

    Collinear neighbours only support an exactly measured ray. Positive
    projection onto the query keeps the local spherical triangle unambiguous.
    """
    projection = np.einsum('nki,ni->nk', neighbours, query)
    tangent = neighbours - projection[:, :, None] * query[:, None, :]
    sides = np.einsum('nki,ni->nk', np.cross(tangent, np.roll(tangent, -1, axis=1)), query)
    same_side = (sides >= -tolerance).all(axis=1) | (sides <= tolerance).all(axis=1)
    area = np.abs(sides.sum(axis=1))
    direct = np.linalg.norm(neighbours - query[:, None, :], axis=2).min(axis=1) <= 1e-8
    return (projection > 0).all(axis=1) & ((same_side & (area > tolerance)) | direct)


def ray_evidence(candidate_points, measured_points, origin, angle_deg=.4,
                 hit_distance=.10, free_margin=.20, range_margin=.02, neighbours=3,
                 require_angular_support=False):
    """Missing rays and occlusion are unknown, never free-space evidence.

    A free vote requires a tight measured-ray cone whose neighbouring returns
    all lie beyond the map point, with no large depth discontinuity. It is an
    approximate visibility test; angular interpolation is not ground truth.
    """
    scan = measured_points - origin
    ranges = np.linalg.norm(scan, axis=1)
    valid = np.isfinite(scan).all(axis=1) & (ranges > .35)
    scan, ranges = scan[valid], ranges[valid]
    hit = np.zeros(len(candidate_points), dtype=bool)
    free = hit.copy()
    if len(scan) < neighbours:
        return hit, free
    directions = scan / ranges[:, None]
    tree = cKDTree(directions)
    delta = candidate_points - origin
    target_range = np.linalg.norm(delta, axis=1)
    query = delta / np.maximum(target_range[:, None], 1e-12)
    cone = 2 * np.sin(np.deg2rad(angle_deg) / 2)
    distance, index = tree.query(query, k=neighbours, distance_upper_bound=cone, workers=2)
    available = index < len(scan)
    safe_index = np.minimum(index, len(scan) - 1)
    returns = ranges[safe_index]
    # Chord distance between unit rays gives the exact endpoint separation.
    endpoint_squared = ((returns - target_range[:, None])**2 +
                        returns * target_range[:, None] * distance**2)
    hit = np.any(available & (endpoint_squared <= hit_distance**2), axis=1)
    margin = np.maximum(free_margin, range_margin * target_range)
    contiguous_depth = np.ptp(returns, axis=1) < (.25 + .01 * target_range)
    free = (available.all(axis=1) & contiguous_depth &
            (returns.min(axis=1) > target_range + margin) & ~hit)
    if require_angular_support:
        if neighbours != 3:
            raise ValueError('Angular triangle support requires exactly three neighbours')
        candidates = np.flatnonzero(free)
        free[candidates] &= angular_support(query[candidates], directions[safe_index[candidates]])
    return hit, free


def grouped_ray_evidence(candidate_points, measured_points, groups, angle_deg=.4,
                         hit_distance=.10, free_margin=.20, require_angular_support=False):
    """Combine ray evidence from short groups with their own interpolated origins."""
    hit = np.zeros(len(candidate_points), dtype=bool)
    free = np.zeros(len(candidate_points), dtype=bool)
    if not groups:
        return hit, free
    for indices, origin, _ in groups:
        if len(indices) < 3:
            continue
        group_hit, group_free = ray_evidence(candidate_points, measured_points[indices], origin,
            angle_deg, hit_distance, free_margin, require_angular_support=require_angular_support)
        hit |= group_hit
        free |= group_free
        free &= ~hit
    return hit, free


def removal_mask(hit_bins, free_bins, first_free, last_free, min_free_bins=4,
                 free_ratio=.8, min_span=6, max_hit_bins=3):
    total = hit_bins.astype(np.float64) + free_bins
    ratio = np.divide(free_bins, total, out=np.zeros_like(total), where=total > 0)
    return ((free_bins >= min_free_bins) & (hit_bins <= max_hit_bins) & (ratio >= free_ratio) &
            (last_free - first_free >= min_span))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frontend', type=pathlib.Path)
    parser.add_argument('backend', type=pathlib.Path)
    parser.add_argument('--output-dir', type=pathlib.Path, required=True)
    parser.add_argument('--frame-step', type=float, default=.75)
    parser.add_argument('--evidence-bin', type=float, default=2)
    parser.add_argument('--angle-deg', type=float, default=.4)
    parser.add_argument('--hit-distance', type=float, default=.10)
    parser.add_argument('--free-margin', type=float, default=.20)
    parser.add_argument('--max-range', type=float, default=20)
    parser.add_argument('--min-free-bins', type=int, default=4)
    parser.add_argument('--free-ratio', type=float, default=.8)
    parser.add_argument('--min-span', type=float, default=6)
    parser.add_argument('--max-hit-bins', type=int, default=3,
                        help='Preserve points repeatedly supported in more independent time bins')
    parser.add_argument('--point-time-groups-ms', type=float, default=0.,
                        help='Use captured per-point times and interpolated origins in short groups; 0 keeps frame-end origin')
    parser.add_argument('--point-time-origin-mode', choices=['interpolated', 'frame-end-control'],
                        default='interpolated',
                        help='Experimental ablation: preserve time partitions but use the shared frame-end origin')
    parser.add_argument('--require-angular-support', action='store_true',
                        help='Reject free votes extrapolated outside the three measured ray directions')
    parser.add_argument('--reuse-evidence', type=pathlib.Path,
                        help='Reclassify identical measured evidence without rescanning; source and settings checked')
    args = parser.parse_args()
    started = time.monotonic()
    if not .2 <= args.frame_step <= 2 or not 1 <= args.evidence_bin <= 10:
        parser.error('Supported frame step .2--2s, evidence bin 1--10s')
    if not .1 <= args.angle_deg <= 1 or not .03 <= args.hit_distance <= .2:
        parser.error('Supported angle .1--1deg, hit distance .03--.2m')
    if not .1 <= args.free_margin <= .5 or not 5 <= args.max_range <= 40:
        parser.error('Supported free margin .1--.5m, max range 5--40m')
    if args.min_free_bins < 2 or not .5 <= args.free_ratio <= 1 or args.min_span < 2 or args.max_hit_bins < 0:
        parser.error('At least two bins, ratio .5--1 and span >=2s required')
    if args.point_time_groups_ms and not .5 <= args.point_time_groups_ms <= 20:
        parser.error('Point-time origin groups must be .5--20ms')
    if args.point_time_origin_mode != 'interpolated' and not args.point_time_groups_ms:
        parser.error('Point-time origin control requires --point-time-groups-ms')
    frontend, backend = args.frontend.resolve(), args.backend.resolve()
    output = args.output_dir.resolve()
    if output.exists():
        parser.error('Output directory already exists; use a fresh directory')
    report = json.loads((backend / 'report.json').read_text())
    if pathlib.Path(report['source_run']).resolve() != frontend:
        raise ValueError('Backend was not generated from this frontend')
    if json.loads((frontend / 'evaluation.json').read_text()).get('failures'):
        raise ValueError('Incomplete or divergent frontend cannot be filtered')
    params = yaml.safe_load((frontend / 'ros_parameters.yaml').read_text())
    if params['mapping']['extrinsic_est_en']:
        raise ValueError('Estimated changing extrinsics require per-frame extrinsic records')
    extrinsic_t = np.asarray(params['mapping']['extrinsic_T'], dtype=float)
    if extrinsic_t.shape != (3,) or not np.isfinite(extrinsic_t).all():
        raise ValueError('Invalid LiDAR-to-IMU translation')
    original, original_rotation = load_trajectory(frontend / 'capture/trajectory.csv')
    optimized, optimized_rotation = load_trajectory(backend / 'trajectory.csv')
    cap = json.loads((frontend / 'capture/capture.json').read_text())
    if report['input_frames'] != cap['counts']['output_cloud']:
        raise ValueError('Backend/capture frame count mismatch')
    if (optimized[-1,0]-optimized[0,0])/args.evidence_bin >= np.iinfo(np.uint16).max:
        raise ValueError('Recording exceeds the supported evidence counter duration')
    points = read_map(backend / 'optimized_3cm.pcd')
    map_hash = hashlib.sha256((backend / 'optimized_3cm.pcd').read_bytes()).hexdigest()
    trajectory_hash = hashlib.sha256((backend / 'trajectory.csv').read_bytes()).hexdigest()
    if args.reuse_evidence:
        previous = json.loads((args.reuse_evidence / 'report.json').read_text())
        if previous['map_sha256'] != map_hash or pathlib.Path(previous['frontend']).resolve() != frontend or pathlib.Path(previous['backend']).resolve() != backend:
            raise ValueError('Evidence source mismatch')
        if previous.get('trajectory_sha256') != trajectory_hash:
            raise ValueError('Evidence trajectory mismatch')
        if previous['settings'].get('point_time_origin_mode', 'interpolated') != args.point_time_origin_mode:
            raise ValueError('Evidence acquisition setting mismatch: point_time_origin_mode')
        if previous['input_frames'] != report['input_frames']:
            raise ValueError('Evidence frame count mismatch')
        for key in ['frame_step','evidence_bin','angle_deg','hit_distance','free_margin','max_range',
                    'point_time_groups_ms']:
            if previous['settings'].get(key, 0.) != getattr(args,key):
                raise ValueError('Evidence acquisition setting mismatch: '+key)
        if previous['settings'].get('require_angular_support', False) != args.require_angular_support:
            raise ValueError('Evidence acquisition setting mismatch: require_angular_support')
        with np.load(args.reuse_evidence / 'point_evidence.npz') as evidence:
            arrays = {key:evidence[key].copy() for key in evidence.files}
        if any(len(array) != len(points) for array in arrays.values()):
            raise ValueError('Evidence length mismatch')
        removed = removal_mask(arrays['hit_bins'],arrays['free_bins'],arrays['first_free'],arrays['last_free'],
            args.min_free_bins,args.free_ratio,args.min_span,args.max_hit_bins)
        output.mkdir(parents=True)
        write_pcd(output/'filtered_3cm.pcd',points[~removed])
        write_pcd(output/'removed_3cm.pcd',points[removed])
        arrays['removed'] = removed
        np.savez_compressed(output/'point_evidence.npz',**arrays)
        (output/'frame_evidence.csv').write_bytes((args.reuse_evidence/'frame_evidence.csv').read_bytes())
        settings={key:value for key,value in vars(args).items() if key not in ('frontend','backend','output_dir','reuse_evidence')}
        previous.update(filtered_points=int((~removed).sum()),removed_points=int(removed.sum()),
            removed_fraction=float(removed.mean()),settings=settings,reused_evidence=str(args.reuse_evidence.resolve()),
            trajectory_sha256=trajectory_hash,
            supported_points_removed=int(((arrays['hit_bins']>=4)&removed).sum()))
        previous['evidence_acquisition_wall_seconds'] = previous.get('evidence_acquisition_wall_seconds',previous['wall_seconds'])
        previous['evidence_source_peak_rss_mb'] = previous.get('evidence_source_peak_rss_mb',previous['peak_rss_mb'])
        previous['wall_seconds'] = time.monotonic()-started
        previous['peak_rss_mb'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
        previous['output_bytes'] = sum(p.stat().st_size for p in output.iterdir() if p.is_file())
        (output/'report.json').write_text(json.dumps(previous,indent=2)+'\n')
        print(json.dumps(previous,indent=2))
        return
    if args.point_time_groups_ms:
        for filename in cap['temporal_raw']:
            with np.load(frontend / 'capture' / filename) as archive:
                if 'point_time_ms' not in archive.files:
                    raise ValueError('Point-time grouping requested but capture has no point_time_ms')
                if len(archive['point_time_ms']) != int(np.sum(archive['lengths'])):
                    raise ValueError('Point-time array length mismatch')
    n = len(points)
    output.mkdir(parents=True)
    hit_bins, free_bins = np.zeros(n, np.uint16), np.zeros(n, np.uint16)
    last_hit_bin, last_free_bin = np.full(n, -1, np.int32), np.full(n, -1, np.int32)
    first_free, last_free = np.full(n, np.inf), np.full(n, -np.inf)
    first_hit, last_hit = first_free.copy(), last_free.copy()
    start, selected, total_frames, last_selected = started, 0, 0, -np.inf
    rows = []
    point_time_skipped = 0
    reference = float(optimized[0, 0])
    map_xyz = points[:, :3].astype(np.float64)
    for filename in cap['temporal_raw']:
        with np.load(frontend / 'capture' / filename) as chunk:
            cloud = chunk['points']
            cuts = np.r_[0, np.cumsum(chunk['lengths'])]
            for j, stamp in enumerate(chunk['stamps']):
                total_frames += 1
                # Check every frame's association, even when voting is sampled.
                old = pose_index(original, stamp)
                new = pose_index(optimized, stamp)
                if stamp - last_selected < args.frame_step:
                    continue
                last_selected = stamp
                frame = cloud[cuts[j]:cuts[j + 1]]
                group = int((stamp - reference) / args.evidence_bin)
                point_times = None
                groups = []
                if args.point_time_groups_ms:
                    if 'point_time_ms' not in chunk.files:
                        raise ValueError('Point-time grouping requested but capture has no point_time_ms')
                    point_times = chunk['point_time_ms'][cuts[j]:cuts[j + 1]]
                    if len(point_times) != len(frame):
                        raise ValueError('Point-time array length mismatch')
                corrected, origin = correct_scan(frame, original[old, 1:4], original_rotation[old],
                    optimized[new, 1:4], optimized_rotation[new], extrinsic_t)
                if point_times is not None:
                    from deskew_ray_origin import coalesce_origin_groups, grouped_origins
                    try:
                        groups = grouped_origins(stamp, point_times, optimized, optimized_rotation, extrinsic_t,
                                                 args.point_time_groups_ms)
                    except ValueError as error:
                        if 'outside the optimized trajectory' not in str(error):
                            raise
                        # The first estimator output can start after scan-start.
                        # Missing pose support is unknown; do not fall back to a
                        # frame-end ray because that would hide the uncertainty.
                        point_time_skipped += 1
                        selected += 1
                        rows.append(dict(stamp=float(stamp), elapsed_s=float(stamp - reference),
                            candidates=0, hits=0, free=0, bin=group, point_time_skipped=1,
                            point_time_groups=0))
                        continue
                    groups = coalesce_origin_groups(groups, max_groups=4)
                    if args.point_time_origin_mode == 'frame-end-control':
                        groups = [(indices, origin, time) for indices, _, time in groups]
                distances = np.linalg.norm(map_xyz - origin, axis=1)
                candidates = np.flatnonzero((distances > .35) & (distances <= args.max_range))
                hit_count = free_count = 0
                for offset in range(0, len(candidates), 150000):
                    subset = candidates[offset:offset + 150000]
                    if point_times is None:
                        hits, frees = ray_evidence(map_xyz[subset], corrected, origin, args.angle_deg,
                                                   args.hit_distance, args.free_margin,
                                                   require_angular_support=args.require_angular_support)
                    else:
                        hits, frees = grouped_ray_evidence(map_xyz[subset], corrected, groups, args.angle_deg,
                                                           args.hit_distance, args.free_margin,
                                                           require_angular_support=args.require_angular_support)
                    hi, fi = subset[hits], subset[frees]
                    fresh_hit = hi[last_hit_bin[hi] != group]
                    fresh_free = fi[last_free_bin[fi] != group]
                    hit_bins[fresh_hit] += 1
                    free_bins[fresh_free] += 1
                    last_hit_bin[hi], last_free_bin[fi] = group, group
                    relative = stamp - reference
                    first_hit[hi] = np.minimum(first_hit[hi], relative)
                    last_hit[hi] = relative
                    first_free[fi] = np.minimum(first_free[fi], relative)
                    last_free[fi] = relative
                    hit_count += len(hi)
                    free_count += len(fi)
                selected += 1
                rows.append(dict(stamp=float(stamp), elapsed_s=float(stamp - reference),
                    candidates=len(candidates), hits=hit_count, free=free_count, bin=group,
                    point_time_skipped=0, point_time_groups=len(groups)))
                if selected % 20 == 0:
                    print('evidence frames {}, elapsed {:.1f}s'.format(selected, time.monotonic()-start), flush=True)
        print('finished {}, scanned {} frames'.format(filename, total_frames), flush=True)
    if total_frames != report['input_frames'] or total_frames != cap['counts']['output_cloud']:
        raise ValueError('Captured cloud frame count does not match baseline reconstruction')
    acquisition_wall_seconds = time.monotonic()-start
    removed = removal_mask(hit_bins, free_bins, first_free, last_free,
                           args.min_free_bins, args.free_ratio, args.min_span, args.max_hit_bins)
    write_pcd(output / 'filtered_3cm.pcd', points[~removed])
    write_pcd(output / 'removed_3cm.pcd', points[removed])
    np.savez_compressed(output / 'point_evidence.npz', removed=removed, hit_bins=hit_bins,
        free_bins=free_bins, first_hit=first_hit, last_hit=last_hit, first_free=first_free, last_free=last_free)
    with (output / 'frame_evidence.csv').open('w') as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    result = dict(status='experimental_requires_dynamic_and_static_region_review',
        frontend=str(frontend), backend=str(backend), trajectory_unchanged=True,
        map_sha256=map_hash, trajectory_sha256=trajectory_hash,
        baseline_points=n, filtered_points=int((~removed).sum()), removed_points=int(removed.sum()),
        removed_fraction=float(removed.mean()), input_frames=total_frames, evidence_frames=selected,
        time_bin_count=int((optimized[-1, 0]-reference)/args.evidence_bin)+1,
        unobserved_points=int(((hit_bins==0)&(free_bins==0)).sum()),
        supported_points=int((hit_bins>=4).sum()), supported_points_removed=int(((hit_bins>=4)&removed).sum()),
        hit_bin_quantiles=np.percentile(hit_bins,[0,25,50,75,90,99,100]).tolist(),
        free_bin_quantiles=np.percentile(free_bins,[0,25,50,75,90,99,100]).tolist(),
        point_time_skipped_frames=point_time_skipped,
        settings={key:value for key,value in vars(args).items() if key not in ('frontend','backend','output_dir','reuse_evidence')},
        wall_seconds=time.monotonic()-start, peak_rss_mb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
        evidence_acquisition_wall_seconds=acquisition_wall_seconds,
        filter_mode='offline subset of fixed baseline map; no pose changes, spatial crop or extra downsampling',
        detector='custom measured-ray visibility evidence using SciPy cKDTree; not Removert or Dynablox',
        limitations='Temporal inconsistency is not a semantic motion label. Glass, pose errors, grouped point-time '
                    'origin approximation and angular interpolation can yield false removals. No ground truth '
                    'precision/recall measured.')
    result['output_bytes'] = sum(p.stat().st_size for p in output.iterdir() if p.is_file())
    (output / 'report.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
