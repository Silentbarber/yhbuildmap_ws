#!/usr/bin/env python3
"""Compare fixed-world and fitted rigid motion in exported temporal regions.

This is a geometric audit of sampled estimator outputs, not semantic labels.
It never changes a trajectory, filter mask or PCD. Partial views and deformable
objects can remain inconclusive even when the visibility filter removes them.
"""
import argparse
import hashlib
import io
import json
import pathlib
import resource
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from audit_dynamic_regions import point_classes, roi_mask
from filter_dynamic_map import correct_scan, load_trajectory, pose_index, read_map


ROOT = pathlib.Path(__file__).resolve().parents[1]
REGISTRATION = o3d.pipelines.registration
MIN_POINTS = 64


def cloud(xyz):
    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(np.asarray(xyz, dtype=float))
    return result


def reduced(xyz):
    return np.asarray(cloud(xyz).voxel_down_sample(.03).points).copy()


def training_mask(xyz):
    cells = np.floor(np.asarray(xyz) / .08).astype(np.int64)
    return (cells @ np.array([73856093, 19349663, 83492791])) % 3 != 0


def apply_transform(xyz, matrix):
    return xyz @ matrix[:3, :3].T + matrix[:3, 3]


def score(source, target, matrix, source_test=None, target_test=None):
    moved = apply_transform(source, matrix)
    a = moved if source_test is None else moved[source_test]
    b = target if target_test is None else target[target_test]
    forward = cKDTree(target).query(a, workers=2)[0]
    reverse = cKDTree(moved).query(b, workers=2)[0]
    distance = np.r_[forward, reverse]
    return dict(capped_rmse_m=float(np.sqrt(np.mean(np.minimum(distance, .25)**2))),
                median_m=float(np.median(distance)), p90_m=float(np.percentile(distance, 90)),
                forward_overlap=float(np.mean(forward < .06)),
                reverse_overlap=float(np.mean(reverse < .06)))


def centroid_initial(source, target):
    matrix = np.eye(4)
    matrix[:3, 3] = np.median(target, axis=0) - np.median(source, axis=0)
    return matrix


def rigid_fit(source, target):
    source_cloud, target_cloud = cloud(source), cloud(target)
    results = []
    for initial in [np.eye(4), centroid_initial(source, target)]:
        fitted = REGISTRATION.registration_icp(source_cloud, target_cloud, .35, initial,
            REGISTRATION.TransformationEstimationPointToPoint(),
            REGISTRATION.ICPConvergenceCriteria(max_iteration=60))
        results.append((score(source, target, fitted.transformation)['capped_rmse_m'],
                        fitted.transformation.copy()))
    return min(results, key=lambda item: item[0])[1]


def plane_information_ratio(source, target, matrix):
    target_cloud = cloud(target)
    target_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.15, max_nn=30))
    transformed = apply_transform(source, matrix)
    distance, nearest = cKDTree(target).query(transformed, workers=2)
    keep = distance < .06
    if keep.sum() < 32:
        return 0.0
    normals = np.asarray(target_cloud.normals)[nearest[keep]]
    centered = transformed[keep] - np.mean(transformed[keep], axis=0)
    radius = max(float(np.sqrt(np.mean(np.sum(centered**2, axis=1)))), .03)
    jacobian = np.column_stack((np.cross(centered, normals) / radius, normals))
    eigenvalues = np.linalg.eigvalsh(jacobian.T @ jacobian)
    return float(max(eigenvalues[0], 0) / max(eigenvalues[-1], 1e-12))


def shape_volume_ratio(xyz):
    # Noisy local normals can manufacture information on a planar scan pattern.
    centered = xyz - np.mean(xyz, axis=0)
    eigenvalues = np.linalg.eigvalsh(centered.T @ centered / len(centered))
    return float(max(eigenvalues[0], 0) / max(eigenvalues[1], 1e-12))


def compare_pair(source, target):
    source, target = reduced(source), reduced(target)
    result = dict(source_voxels=len(source), target_voxels=len(target),
                  status='insufficient_geometry', motion_supported=False)
    if min(len(source), len(target)) < MIN_POINTS:
        return result
    source_train, target_train = training_mask(source), training_mask(target)
    if min(source_train.sum(), target_train.sum()) < 32 or min((~source_train).sum(), (~target_train).sum()) < 12:
        return result
    fixed = score(source, target, np.eye(4), ~source_train, ~target_train)
    fitted = rigid_fit(source[source_train], target[target_train])
    fitted_score = score(source, target, fitted, ~source_train, ~target_train)
    reverse = rigid_fit(target[target_train], source[source_train])
    cycle = reverse @ fitted
    center = np.mean(source, axis=0)
    displacement = float(np.linalg.norm(apply_transform(center[None], fitted)[0] - center))
    cycle_displacement = float(np.linalg.norm(apply_transform(center[None], cycle)[0] - center))
    cycle_angle = float(np.rad2deg(Rotation.from_matrix(cycle[:3, :3].copy()).magnitude()))
    information_ratio = plane_information_ratio(source[source_train], target[target_train], fitted)
    volume_ratio = min(shape_volume_ratio(source), shape_volume_ratio(target))
    improvement = fixed['capped_rmse_m'] - fitted_score['capped_rmse_m']
    gates = dict(displacement=displacement >= .08,
        heldout_improvement=improvement >= .025 and improvement >= .25 * fixed['capped_rmse_m'],
        bulk_improvement=fixed['median_m']-fitted_score['median_m'] >= .01,
        bidirectional_overlap=min(fitted_score['forward_overlap'], fitted_score['reverse_overlap']) >= .60,
        heldout_residual=fitted_score['p90_m'] <= .10,
        full_geometry=information_ratio >= 5e-4,
        nonplanar_shape=volume_ratio >= .01,
        reverse_cycle=cycle_displacement <= .04 and cycle_angle <= 3)
    supported = all(gates.values())
    if supported:
        status = 'rigid_motion_candidate'
    elif fixed['capped_rmse_m'] <= .04 and min(fixed['forward_overlap'], fixed['reverse_overlap']) >= .65:
        status = 'fixed_world_compatible'
    else:
        status = 'inconclusive'
    result.update(status=status, motion_supported=supported, gates=gates,
        fixed_world=fixed, fitted_motion=fitted_score, heldout_improvement_m=improvement,
        displacement_at_source_center_m=displacement,
        fitted_rotation_deg=float(np.rad2deg(Rotation.from_matrix(fitted[:3, :3].copy()).magnitude())),
        plane_information_eigenvalue_ratio=information_ratio,
        shape_volume_ratio=volume_ratio,
        reverse_cycle_displacement_m=cycle_displacement, reverse_cycle_rotation_deg=cycle_angle,
        transformation=fitted.tolist(), source_test_voxels=int((~source_train).sum()),
        target_test_voxels=int((~target_train).sum()))
    return result


def sensor_attachment_score(source, target, before, after, fixed_score):
    source, target = reduced(source), reduced(target)
    relative = np.asarray(after['imu_rotation']) @ np.asarray(before['imu_rotation']).T
    matrix = np.eye(4)
    matrix[:3, :3] = relative
    matrix[:3, 3] = np.asarray(after['lidar_origin']) - relative @ np.asarray(before['lidar_origin'])
    measured = score(source, target, matrix, ~training_mask(source), ~training_mask(target))
    compatible = (fixed_score['capped_rmse_m']-measured['capped_rmse_m'] >= .025
        and fixed_score['median_m']-measured['median_m'] >= .01
        and min(measured['forward_overlap'], measured['reverse_overlap']) >= .60
        and measured['p90_m'] <= .10)
    return dict(score=measured, compatible=compatible, transformation=matrix.tolist(),
        interpretation='Known rigid sensor motion, no ICP fit. Compatibility suggests sensor-attached returns '
                       'but does not identify a hand/body or provide a semantic dynamic label.')


def frame_points(points, classes, frame, region):
    a, b = frame['offset'], frame['offset'] + frame['count']
    xyz = points[a:b, :3]
    if region['kind'].startswith('persistent'):
        keep = np.isin(classes[a:b], [1, 3])
        if 'normal' in region:
            keep &= np.abs((xyz - region['center']) @ np.asarray(region['normal'])) < .08
    else:
        keep = np.isin(classes[a:b], [0, 2, 3])
    return xyz[keep]


def plot_pair(path, source, target, pair):
    fig, axes = plt.subplots(2, 3, figsize=(12, 7), constrained_layout=True)
    fitted = np.asarray(pair.get('transformation', np.eye(4)))
    for row, transformed in enumerate([source, apply_transform(source, fitted)]):
        for col, (a, b) in enumerate([(0, 1), (0, 2), (1, 2)]):
            axes[row, col].scatter(target[:, a], target[:, b], s=3, color='#db8951', label='Later scan')
            axes[row, col].scatter(transformed[:, a], transformed[:, b], s=2, color='#27958b', label='Earlier scan')
            extent = np.vstack((source, target))
            lo, hi = extent.min(axis=0) - .05, extent.max(axis=0) + .05
            axes[row, col].set(xlim=(lo[a], hi[a]), ylim=(lo[b], hi[b]),
                xlabel='XYZ'[a]+' (m)', ylabel='XYZ'[b]+' (m)',
                title='Fixed world' if row == 0 else 'Fitted rigid motion')
            axes[row, col].set_aspect('equal')
    axes[0, 0].legend(fontsize=8)
    fig.suptitle('{} / {:.2f} to {:.2f} s / {}'.format(pair['region'], pair['source_seconds'],
        pair['target_seconds'], pair['status']))
    fig.savefig(path, dpi=130)
    plt.close(fig)


def full_source_regions(data_root, recording, report, peaks, half_window):
    frontend, backend = data_root/recording['frontend'], data_root/recording['backend']
    hashes = {}

    def checked_bytes(path, expected=None):
        data = path.read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        if expected is not None and digest != expected:
            raise ValueError('Source provenance changed: '+str(path))
        hashes[str(path)] = digest
        return data

    for name, digest in report['source_metadata_sha256'].items():
        checked_bytes(frontend/name, digest)
    for name, key in [('optimized_3cm.pcd', 'map_sha256'), ('trajectory.csv', 'trajectory_sha256')]:
        checked_bytes(backend/name, recording[key])
    baseline = read_map(backend/'optimized_3cm.pcd')
    tree = cKDTree(baseline[:, :3])
    candidate_root = data_root/'results/dynamic_filtering'
    masks = []
    for name in ['angular_guard', 'delivery']:
        candidate = candidate_root/name/recording['id']
        candidate_report = json.loads(checked_bytes(candidate/'report.json'))
        if any(candidate_report[key] != recording[key] for key in ['map_sha256', 'trajectory_sha256']):
            raise ValueError('Candidate provenance mismatch')
        if name == 'angular_guard' and not (candidate_report['settings']['require_angular_support']
                and candidate_report['settings']['max_hit_bins'] == 3):
            raise ValueError('Expected angular-support max-hit=3 classes')
        with np.load(io.BytesIO(checked_bytes(candidate/'point_evidence.npz'))) as evidence:
            masks.append(evidence['removed'].copy())
    removed, previous = masks
    if removed.shape != (len(baseline),) or previous.shape != removed.shape or np.any(removed & ~previous):
        raise ValueError('Invalid candidate masks')
    old, old_rotations = load_trajectory(frontend/'capture/trajectory.csv')
    new, new_rotations = load_trajectory(backend/'trajectory.csv')
    params = yaml.safe_load((frontend/'ros_parameters.yaml').read_text())
    if params['mapping']['extrinsic_est_en']:
        raise ValueError('Changing extrinsics require per-frame records')
    extrinsic = np.asarray(params['mapping']['extrinsic_T'])
    samples = {region['key']: [] for region in report['regions']}
    frame_index = 0
    for archive in report['source_archives']:
        data = checked_bytes(frontend/'capture'/archive['file'], archive['sha256'])
        with np.load(io.BytesIO(data)) as chunk:
            stamps = chunk['stamps']
            seconds = stamps-new[0, 0]
            needed = np.zeros(len(stamps), dtype=bool)
            for peak in peaks.values():
                needed |= np.abs(seconds-peak) <= half_window
            if needed.any():
                cuts = np.r_[0, np.cumsum(chunk['lengths'])]
                points = chunk['points']
                for j in np.flatnonzero(needed):
                    stamp = float(stamps[j])
                    a, b = pose_index(old, stamp), pose_index(new, stamp)
                    scan = points[cuts[j]:cuts[j+1]]
                    xyz, origin = correct_scan(scan, old[a, 1:4], old_rotations[a],
                        new[b, 1:4], new_rotations[b], extrinsic)
                    for region in report['regions']:
                        key = region['key']
                        if abs(seconds[j]-peaks[key]) > half_window:
                            continue
                        ids = np.flatnonzero(roi_mask(xyz, region))
                        distance, nearest = tree.query(xyz[ids], workers=2)
                        classes = point_classes(distance, nearest, removed, previous)
                        packed = np.column_stack((xyz[ids], scan[ids, 3]))
                        frame = dict(frame_index=frame_index+int(j), seconds=float(seconds[j]), stamp=stamp,
                            offset=0, count=len(ids), lidar_origin=origin.tolist(), chunk=archive['file'],
                            chunk_frame=int(j), source_scan_points=len(scan), imu_rotation=new_rotations[b].tolist())
                        samples[key].append(dict(frame=frame, points=packed, classes=classes))
                        samples[key][-1].update(source_indices=ids.astype('<u4'),
                            baseline_indices=nearest.astype('<u4'), distances=distance.astype('<f4'))
            frame_index += len(stamps)
    if frame_index != recording['frames']:
        raise ValueError('Incomplete captured source inventory')
    return samples, hashes


def export_full_review(output, review_folder, report, samples):
    if output.exists():
        raise ValueError('Use a fresh full-frame review directory')
    output.mkdir(parents=True)
    regions = []
    for region in report['regions']:
        key = region['key']
        destination = output/key
        destination.mkdir()
        for name in ['baseline.bin', 'baseline_classes.bin', 'baseline_indices.bin']:
            (destination/name).write_bytes((review_folder/key/name).read_bytes())
        frames = []
        offset = 0
        names = ['scan', 'scan_classes', 'source_point_indices', 'associated_baseline_indices', 'association_distances']
        files = {name: (destination/(name+'.bin')).open('wb') for name in names}
        try:
            for sample in samples[key]:
                frame = dict(sample['frame'], offset=offset)
                frames.append(frame)
                offset += frame['count']
                for name, values in [('scan', sample['points'].astype('<f4')), ('scan_classes', sample['classes']),
                        ('source_point_indices', sample['source_indices']),
                        ('associated_baseline_indices', sample['baseline_indices']), ('association_distances', sample['distances'])]:
                    files[name].write(values.tobytes())
        finally:
            for file in files.values():
                file.close()
        (destination/'frames.json').write_text(json.dumps(frames, separators=(',', ':'))+'\n')
        regions.append(dict(region, display_frames=len(frames), display_points=offset,
            window_start_s=frames[0]['seconds'], window_end_s=frames[-1]['seconds']))
    export = dict(report, regions=regions, display_step=None, sampling='all captured frames in selected peak windows',
        validation_scope='Complete captured estimator output frames within each selected local time window. '
            'Baseline context and filter classes are unchanged. This is not full-bag region frame coverage '
            'or unprocessed bag input; no frames.csv is exported here.', status='full_window_scene_review_required')
    (output/'report.json').write_text(json.dumps(export, indent=2)+'\n')
    return {str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in output.rglob('*') if path.is_file()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--review-root', type=pathlib.Path, required=True)
    parser.add_argument('--output-root', type=pathlib.Path, required=True)
    parser.add_argument('--id', choices=['215247', '162102', '162342', '162744'], required=True)
    parser.add_argument('--half-window', type=float, default=4.)
    parser.add_argument('--full-source', action='store_true', help='Read complete captured output frames in selected windows')
    parser.add_argument('--data-root', type=pathlib.Path, help='Existing four-bag dataset root; required with --full-source')
    parser.add_argument('--export-review-root', type=pathlib.Path,
                        help='Optional full-window data for temporal.html; only with --full-source')
    args = parser.parse_args()
    if not np.isfinite(args.half_window) or not 1 <= args.half_window <= 10:
        parser.error('half-window must be finite and in [1,10] seconds')
    if args.full_source and args.data_root is None:
        parser.error('--full-source requires --data-root')
    if args.export_review_root is not None and not args.full_source:
        parser.error('--export-review-root requires --full-source')
    output = args.output_root/args.id
    if output.exists():
        parser.error('Use a fresh output directory')
    if args.export_review_root is not None and (args.export_review_root/args.id).exists():
        parser.error('Use a fresh full-frame review directory')
    started = time.monotonic()
    folder = args.review_root/args.id
    report_path = folder/'report.json'
    report = json.loads(report_path.read_text())
    inventory = json.loads((ROOT/'config/dynamic_filtering_recordings.json').read_text())
    recording = next(item for item in inventory['recordings'] if item['id'] == args.id)
    if report['id'] != args.id or report['input_frames'] != recording['frames'] or any(
            report[key] != recording[key] for key in ['map_sha256', 'trajectory_sha256']):
        raise ValueError('Temporal review provenance mismatch')
    regions = []
    hashes = {str(report_path): hashlib.sha256(report_path.read_bytes()).hexdigest()}
    plots = []
    full_samples = None
    if args.full_source:
        peaks = {}
        for region in report['regions']:
            frames_path = folder/region['key']/'frames.json'
            data = frames_path.read_bytes()
            hashes[str(frames_path)] = hashlib.sha256(data).hexdigest()
            frames = json.loads(data)
            peak = region['peak_visible_frame'] if region['kind'].startswith('persistent') else region['peak_removed_frame']
            peaks[region['key']] = min(frames, key=lambda frame: abs(frame['frame_index']-peak))['seconds']
        full_samples, source_hashes = full_source_regions(args.data_root, recording, report, peaks, args.half_window)
        hashes.update(source_hashes)
    for region in report['regions']:
        directory = folder/region['key']
        if full_samples is None:
            files = {name: (directory/name).read_bytes() for name in ['frames.json', 'scan.bin', 'scan_classes.bin']}
            hashes.update({str(directory/name): hashlib.sha256(data).hexdigest() for name, data in files.items()})
            points = np.frombuffer(files['scan.bin'], dtype='<f4').reshape(-1, 4)
            classes = np.frombuffer(files['scan_classes.bin'], dtype=np.uint8)
            frames = json.loads(files['frames.json'])
            expected = sum(frame['count'] for frame in frames)
            if not frames or len(points) != expected or len(classes) != expected or not np.isfinite(points).all():
                raise ValueError('Invalid packed region data')
            offset = 0
            for frame in frames:
                if frame['offset'] != offset or frame['count'] < 0:
                    raise ValueError('Invalid packed region frame offsets')
                offset += frame['count']

            def selected_points(frame):
                return frame_points(points, classes, frame, region)
        else:
            by_index = {sample['frame']['frame_index']: sample for sample in full_samples[region['key']]}
            frames = [sample['frame'] for sample in full_samples[region['key']]]
            if not frames:
                raise ValueError('Selected window has no captured frames')

            def selected_points(frame):
                sample = by_index[frame['frame_index']]
                return frame_points(sample['points'], sample['classes'], frame, region)
        if any(b['seconds'] <= a['seconds'] for a, b in zip(frames, frames[1:])):
            raise ValueError('Region timestamps must increase')
        peak = region['peak_visible_frame'] if region['kind'].startswith('persistent') else region['peak_removed_frame']
        closest = min(frames, key=lambda frame: abs(frame['frame_index'] - peak))
        chosen = frames if args.full_source else [frame for frame in frames
            if abs(frame['seconds'] - closest['seconds']) <= args.half_window]
        pairs = []
        for before, after in zip(chosen, chosen[1:]):
            dt = after['seconds'] - before['seconds']
            if not 0 < dt <= 1.2:
                continue
            source, target = [selected_points(frame) for frame in [before, after]]
            pair = compare_pair(source, target)
            if args.full_source and 'fixed_world' in pair:
                pair['sensor_attachment'] = sensor_attachment_score(source, target, before, after, pair['fixed_world'])
            pair.update(region=region['key'], source_frame=before['frame_index'], target_frame=after['frame_index'],
                source_seconds=before['seconds'], target_seconds=after['seconds'], dt_seconds=dt,
                source_returns=len(source), target_returns=len(target),
                lidar_origin_displacement_m=float(np.linalg.norm(np.subtract(after['lidar_origin'], before['lidar_origin']))))
            pairs.append(pair)
        counts = {key: sum(pair['status'] == key for pair in pairs) for key in
                  ['rigid_motion_candidate', 'fixed_world_compatible', 'inconclusive', 'insufficient_geometry']}
        regions.append(dict(key=region['key'], kind=region['kind'], peak_seconds=closest['seconds'],
            window_center_seconds=peaks[region['key']] if args.full_source else closest['seconds'],
            source_frames_checked=len(chosen),
            sensor_attachment_pairs=sum(pair.get('sensor_attachment', {}).get('compatible', False) for pair in pairs),
            selection='classes 1/3 near control plane' if region['kind'].startswith('persistent') else 'classes 0/2/3',
            status_counts=counts, pairs=pairs))
        usable = [pair for pair in pairs if 'heldout_improvement_m' in pair]
        if usable:
            best = max(usable, key=lambda pair: (pair['motion_supported'], pair['heldout_improvement_m']))
            source = selected_points(next(f for f in frames if f['frame_index'] == best['source_frame']))
            target = selected_points(next(f for f in frames if f['frame_index'] == best['target_frame']))
            plots.append((region['key']+'.png', source.copy(), target.copy(), best))
        print(args.id, region['key'], counts, flush=True)
    output.mkdir(parents=True)
    for name, source, target, pair in plots:
        plot_pair(output/name, source, target, pair)
    result = dict(id=args.id, bag=recording['bag'], source_sha256=hashes,
        map_sha256=recording['map_sha256'], trajectory_sha256=recording['trajectory_sha256'],
        regions=regions, half_window_s=args.half_window, full_source=args.full_source,
        source_sampling='all captured frames in selected windows' if args.full_source else 'exported display samples',
        status='geometric_motion_audit_only',
        changes_filter_or_map=False, no_semantic_ground_truth=True,
        scope='Adjacent source frames near selected peaks at the declared sampling; not exhaustive full-bag motion detection. '
              'Motion fits use Open3D ICP with unused source/target spatial blocks, bulk median improvement, bidirectional overlap, '
              'full point-to-plane geometry, nonplanar shape and independently fitted reverse-cycle checks. '
              'Rigid motion candidates still require object identity/scene review; occlusion and deformation '
              'can be inconclusive. Selected points depend on existing filter classes, so these are not '
              'independent precision/recall labels. Fixed-world compatibility is local and short-term.',
        wall_seconds=time.monotonic()-started, peak_rss_mb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)
    if args.export_review_root is not None:
        review_output = args.export_review_root/args.id
        result['full_review_directory'] = str(review_output)
        result['full_review_sha256'] = export_full_review(review_output, folder, report, full_samples)
        result['full_review_bytes'] = sum(path.stat().st_size for path in review_output.rglob('*') if path.is_file())
        result['wall_seconds'] = time.monotonic()-started
    (output/'report.json').write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
