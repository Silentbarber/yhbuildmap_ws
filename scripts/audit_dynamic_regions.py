#!/usr/bin/env python3
"""Export traceable local temporal evidence without changing any delivered map."""
import argparse
import csv
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
import yaml
from scipy.spatial import cKDTree

from filter_dynamic_map import correct_scan, load_trajectory, pose_index, read_map


ROOT = pathlib.Path(__file__).resolve().parents[1]
COLORS = ['#8b979f', '#49bbae', '#e86169', '#e7b455']
CLASSES = ['unassociated', 'retained', 'removed', 'restored_from_first_candidate']


def point_classes(distance, index, removed, previous, threshold=.05):
    classes = np.ones(len(distance), dtype=np.uint8)
    classes[removed[index]] = 2
    classes[previous[index] & ~removed[index]] = 3
    classes[distance > threshold] = 0
    return classes


def roi_mask(points, region):
    return ((points[:, :3] >= region['lower']) & (points[:, :3] <= region['upper'])).all(axis=1)


def select_regions(points, removed, hit_bins, patches=None):
    regions = []

    def add(key, kind, center, radius, normal=None):
        center = np.asarray(center, dtype=float)
        region = dict(key=key, kind=kind, center=center.tolist(),
                      lower=(center-radius).tolist(), upper=(center+radius).tolist())
        if normal is not None:
            region['normal'] = np.asarray(normal).tolist()
        regions.append(region)

    deleted = points[removed, :3]
    if len(deleted):
        tree = cKDTree(deleted)
        candidates = deleted[::max(1, len(deleted)//1500)]
        counts = tree.query_ball_point(candidates, .45, return_length=True, workers=2)
        chosen = []
        for i in np.argsort(-counts, kind='stable'):
            anchor = candidates[i]
            if counts[i] < 20 or any(np.linalg.norm(anchor-center) < 1.8 for center in chosen):
                continue
            neighborhood = deleted[tree.query_ball_point(anchor, .45)]
            center = np.median(neighborhood, axis=0)
            chosen.append(center)
            add('removal_{:02d}'.format(len(chosen)-1), 'removal_candidate', center, 1.1)
            if len(chosen) == 3:
                break
    tree = cKDTree(points[:, :3])
    ids = np.flatnonzero((hit_bins >= 8) & ~removed)
    ids = ids[::max(1, len(ids)//3000)]
    if len(ids):
        distance, neighbors = tree.query(points[ids, :3], k=32, workers=2)
        xyz = points[neighbors, :3].astype(float)
        delta = xyz - xyz.mean(axis=1, keepdims=True)
        covariance = np.einsum('nki,nkj->nij', delta, delta)/32
        eigenvalue, eigenvector = np.linalg.eigh(covariance)
        planar = ((distance[:, -1] < .25) & (eigenvalue[:, 0]/np.maximum(eigenvalue.sum(axis=1), 1e-12) < .02)
                  & (eigenvalue[:, 1] > .0003))
        score = hit_bins[ids]/np.maximum(distance[:, -1], .03)
        normals = eigenvector[:, :, 0]
        # Geometric controls are selected without calling them verified walls.
        for key, orientation in [('planar_vertical', np.abs(normals[:, 2]) < .3),
                                 ('planar_horizontal', np.abs(normals[:, 2]) > .8)]:
            eligible = np.flatnonzero(planar & orientation)
            if len(eligible):
                i = eligible[np.argmax(score[eligible])]
                add(key, 'persistent_planar_candidate', points[ids[i], :3], .65, normals[i])
        linear = ((distance[:, -1] < .25) & (eigenvalue[:, 1]/np.maximum(eigenvalue[:, 2], 1e-12) < .15)
                  & (eigenvalue[:, 2] > .001))
        eligible = np.flatnonzero(linear)
        if len(eligible):
            i = eligible[np.argmax(score[eligible])]
            add('linear_geometry', 'persistent_linear_candidate', points[ids[i], :3], .65)
    if patches:
        for patch in patches:
            if patch['key'] == 'reference_11_plane_1_patch_14':
                add('uncertain_patch_14', 'unconfirmed_short_visible_patch', patch['anchor'], .65, patch['normal'])
    return regions


def plot_region(folder, baseline, classes, rows, region):
    fig, axes = plt.subplots(2, 3, figsize=(13, 7), constrained_layout=True)
    subsets = [np.ones(len(baseline), bool), classes != 2, classes == 2]
    for row, (a, b) in enumerate([(0, 1), (0, 2)]):
        for col, (mask, title) in enumerate(zip(subsets, ['Baseline', 'Filtered', 'Removed candidates'])):
            for label in [1, 3, 2]:
                selected = mask & (classes == label)
                axes[row, col].scatter(baseline[selected, a], baseline[selected, b], s=1.5, color=COLORS[label])
            axes[row, col].set(xlim=(region['lower'][a], region['upper'][a]),
                               ylim=(region['lower'][b], region['upper'][b]),
                               xlabel='XYZ'[a]+' (m)', ylabel='XYZ'[b]+' (m)', title=title)
            axes[row, col].set_aspect('equal')
    fig.suptitle(region['key']+' / '+region['kind']+' (not semantic ground truth)')
    fig.savefig(folder/'comparison.png', dpi=130)
    plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True, constrained_layout=True)
    stamps = np.array([r['seconds'] for r in rows])
    for key, color in [('retained', COLORS[1]), ('removed', COLORS[2]), ('restored_from_first_candidate', COLORS[3])]:
        axes[0].plot(stamps, [r[key] for r in rows], color=color, lw=.8, label=key)
    axes[0].set(ylabel='Associated scan return count', title=region['key']+' / all captured output frames')
    axes[0].legend(fontsize=8)
    axes[1].plot(stamps, [r['plane_p95_abs_m'] if r['plane_p95_abs_m'] is not None else np.nan for r in rows],
                 color='#556b91', lw=.8)
    axes[1].set(xlabel='Seconds from first optimized output pose', ylabel='Local plane p95 residual (m)')
    fig.savefig(folder/'time_profile.png', dpi=130)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=pathlib.Path, required=True)
    parser.add_argument('--candidate-root', type=pathlib.Path, required=True)
    parser.add_argument('--first-candidate-root', type=pathlib.Path, required=True)
    parser.add_argument('--output-root', type=pathlib.Path, required=True)
    parser.add_argument('--id', choices=['215247', '162102', '162342', '162744'], required=True)
    parser.add_argument('--display-step', type=float, default=.5)
    args = parser.parse_args()
    if args.display_step < .2 or args.display_step > 2:
        parser.error('Display sampling step must be .2--2 seconds')
    started = time.monotonic()
    recording = next(r for r in json.loads((ROOT/'config/dynamic_filtering_recordings.json').read_text())['recordings']
                     if r['id'] == args.id)
    frontend, backend = args.data_root/recording['frontend'], args.data_root/recording['backend']
    candidate, first = args.candidate_root/args.id, args.first_candidate_root/args.id
    output = args.output_root/args.id
    if output.exists():
        parser.error('Use a fresh output directory')
    report = json.loads((candidate/'report.json').read_text())
    first_report = json.loads((first/'report.json').read_text())
    hashes = {}
    for filename, key in [('optimized_3cm.pcd', 'map_sha256'), ('trajectory.csv', 'trajectory_sha256')]:
        hashes[key] = hashlib.sha256((backend/filename).read_bytes()).hexdigest()
        if any(hashes[key] != source[key] for source in [recording, report, first_report]):
            raise ValueError('Baseline/candidate provenance mismatch: '+filename)
    points = read_map(backend/'optimized_3cm.pcd')
    with np.load(candidate/'point_evidence.npz') as evidence:
        removed, hits = evidence['removed'].copy(), evidence['hit_bins'].copy()
    with np.load(first/'point_evidence.npz') as evidence:
        previous = evidence['removed'].copy()
    if removed.shape != (len(points),) or previous.shape != removed.shape or np.any(removed & ~previous):
        raise ValueError('Candidate partition mismatch')
    if not np.array_equal(read_map(candidate/'filtered_3cm.pcd'), points[~removed]):
        raise ValueError('Filtered map differs from evidence partition')
    params = yaml.safe_load((frontend/'ros_parameters.yaml').read_text())
    if params['mapping']['extrinsic_est_en']:
        raise ValueError('Changing extrinsics need per-frame records')
    extrinsic = np.asarray(params['mapping']['extrinsic_T'])
    original, old_rotations = load_trajectory(frontend/'capture/trajectory.csv')
    optimized, new_rotations = load_trajectory(backend/'trajectory.csv')
    capture = json.loads((frontend/'capture/capture.json').read_text())
    if capture['counts']['output_cloud'] != recording['frames']:
        raise ValueError('Unexpected captured frame count')
    patch_file = backend.parent/'coverage_geometry_v5_validation/surface_patches/patches.json'
    patches = json.loads(patch_file.read_text()) if patch_file.exists() else None
    regions = select_regions(points, removed, hits, patches)
    if not regions:
        raise ValueError('No regions available')
    output.mkdir(parents=True)
    tree = cKDTree(points[:, :3])
    baseline_classes = point_classes(np.zeros(len(points)), np.arange(len(points)), removed, previous)
    states = []
    for region in regions:
        folder = output/region['key']
        folder.mkdir()
        ids = np.flatnonzero(roi_mask(points, region))
        points[ids].astype('<f4').tofile(folder/'baseline.bin')
        baseline_classes[ids].tofile(folder/'baseline_classes.bin')
        ids.astype('<u4').tofile(folder/'baseline_indices.bin')
        region.update(baseline_points=len(ids), filtered_points=int((~removed[ids]).sum()),
                      removed_points=int(removed[ids].sum()),
                      restored_points=int((previous[ids] & ~removed[ids]).sum()),
                      first_supported_frame=None, last_supported_frame=None)
        if 'normal' in region:
            normal = np.asarray(region['normal'])
            center = np.asarray(region['center'])
            delta = points[ids, :3]-center
            signed = delta@normal
            tangent = delta-signed[:, None]*normal
            patch_mask = (np.linalg.norm(tangent, axis=1) < .35) & (np.abs(signed) < .03)
            region['plane_patch'] = dict(points=int(patch_mask.sum()),
                                        removed=int((patch_mask & removed[ids]).sum()))
        states.append(dict(region=region, folder=folder, rows=[], frames=[], offset=0,
            files={name:(folder/(name+'.bin')).open('wb') for name in
                   ['scan', 'scan_classes', 'source_point_indices', 'associated_baseline_indices', 'association_distances']}))
    frame_index, last_display = 0, -np.inf
    sources = []
    try:
        for filename in capture['temporal_raw']:
            path = frontend/'capture'/filename
            archive = path.read_bytes()
            sources.append(dict(file=filename, bytes=len(archive), sha256=hashlib.sha256(archive).hexdigest()))
            with np.load(io.BytesIO(archive)) as chunk:
                cloud = chunk['points']
                cuts = np.r_[0, np.cumsum(chunk['lengths'])]
                for j, stamp in enumerate(chunk['stamps']):
                    old, new = pose_index(original, stamp), pose_index(optimized, stamp)
                    frame = cloud[cuts[j]:cuts[j+1]]
                    corrected, origin = correct_scan(frame, original[old, 1:4], old_rotations[old],
                                                     optimized[new, 1:4], new_rotations[new], extrinsic)
                    display = stamp-last_display >= args.display_step
                    if display:
                        last_display = stamp
                    seconds = float(stamp-optimized[0, 0])
                    for state in states:
                        region = state['region']
                        ids = np.flatnonzero(roi_mask(corrected, region))
                        xyz = corrected[ids]
                        distance, index = tree.query(xyz, workers=2)
                        classes = point_classes(distance, index, removed, previous)
                        row = dict(frame_index=frame_index, stamp=float(stamp), seconds=seconds,
                                   chunk=filename, chunk_frame=j, region_returns=len(ids))
                        row.update({name:int((classes == label).sum()) for label, name in enumerate(CLASSES)})
                        row['plane_p95_abs_m'] = None
                        row['plane_points'] = 0
                        if 'normal' in region:
                            delta = xyz-np.asarray(region['center'])
                            normal = np.asarray(region['normal'])
                            signed = delta@normal
                            tangent = delta-signed[:, None]*normal
                            patch = (np.linalg.norm(tangent, axis=1)<.35) & (np.abs(signed)<.15)
                            row['plane_points'] = int(patch.sum())
                            if row['plane_points'] >= 12:
                                row['plane_p95_abs_m'] = float(np.percentile(np.abs(signed[patch]), 95))
                        state['rows'].append(row)
                        if len(ids):
                            if region['first_supported_frame'] is None:
                                region['first_supported_frame'] = frame_index
                            region['last_supported_frame'] = frame_index
                        if display:
                            packed = np.column_stack((xyz, frame[ids, 3])).astype('<f4')
                            for name, data in [('scan', packed), ('scan_classes', classes),
                                ('source_point_indices', ids.astype('<u4')),
                                ('associated_baseline_indices', index.astype('<u4')),
                                ('association_distances', distance.astype('<f4'))]:
                                state['files'][name].write(data.tobytes())
                            state['frames'].append(dict(frame_index=frame_index, seconds=seconds,
                                stamp=float(stamp), chunk=filename, chunk_frame=j, source_scan_points=len(frame),
                                offset=state['offset'], count=len(ids), lidar_origin=origin.tolist()))
                            state['offset'] += len(ids)
                    frame_index += 1
            print('{}: audited {} / {} frames'.format(args.id, frame_index, recording['frames']), flush=True)
    finally:
        for state in states:
            for file in state['files'].values():
                file.close()
    if frame_index != recording['frames']:
        raise ValueError('Incomplete frame audit')
    for state in states:
        region, folder, rows = state['region'], state['folder'], state['rows']
        region['display_frames'] = len(state['frames'])
        region['display_points'] = state['offset']
        region['peak_removed_frame'] = max(rows, key=lambda r:r['removed'])['frame_index']
        region['peak_visible_frame'] = max(rows, key=lambda r:r['region_returns'])['frame_index']
        region['observed_frames'] = sum(r['region_returns']>0 for r in rows)
        with (folder/'frames.csv').open('w') as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        (folder/'frames.json').write_text(json.dumps(state['frames'], separators=(',', ':'))+'\n')
        mask = roi_mask(points, region)
        plot_region(folder, points[mask], baseline_classes[mask], rows, region)
    result = dict(id=args.id, bag=recording['bag'], baseline_commit='9a11f2e', input_frames=frame_index,
        source_capture=str(frontend/'capture'), source_archives=sources,
        source_metadata_sha256={name:hashlib.sha256((frontend/name).read_bytes()).hexdigest()
            for name in ['capture/trajectory.csv', 'capture/capture.json', 'ros_parameters.yaml']},
        **hashes, regions=regions, display_step=args.display_step,
        classes=CLASSES, binary_format='little-endian float32 XYZI, uint8 classes, uint32 source and baseline indices',
        association_threshold_m=.05,
        validation_scope='All captured estimator output frames in selected fixed-coordinate regions. '
            'Display samples frames only; full frame counts are in CSV. Local boxes are inspection views, never map crops. '
            'Baseline association is nearest-centroid within 5cm, not exact voxel lineage or semantic motion ground truth. '
            'Source chunk/frame/point indices refer to Faster-LIO captured outputs, not unprocessed input bag messages.',
        status='temporal_scene_review_required', wall_seconds=time.monotonic()-started,
        peak_rss_mb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)
    result['output_bytes'] = sum(p.stat().st_size for p in output.rglob('*') if p.is_file())
    (output/'report.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({key:result[key] for key in ['id', 'input_frames', 'wall_seconds', 'output_bytes']}, indent=2))


if __name__ == '__main__':
    main()
