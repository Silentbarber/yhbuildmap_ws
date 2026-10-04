#!/usr/bin/env python3
"""Compare identical point-time partitions while changing only ray origins."""
import argparse
import csv
import hashlib
import json
import pathlib

import numpy as np

from audit_dynamic_filter import planar_sample_ids
from evaluate_independent_run import write_pcd
from filter_dynamic_map import read_map, removal_mask


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def selected_frames(directory):
    with (directory/'frame_evidence.csv').open() as file:
        rows=list(csv.DictReader(file))
    # The first real experiment used Boolean CSV flags; later captures use 0/1.
    return [(float(row['stamp']),float(row['elapsed_s']),int(row['candidates']),
             int(row['bin']),int(row['point_time_groups']),
             str(row.get('point_time_skipped','0')).lower() in ('1','true')) for row in rows]


def checked_result(directory, expected_mode):
    report=json.loads((directory/'report.json').read_text())
    if report['settings'].get('point_time_origin_mode','interpolated')!=expected_mode:
        raise ValueError('Unexpected point-time origin mode: '+str(directory))
    if report['settings']['point_time_groups_ms']<=0 or not report['settings']['require_angular_support']:
        raise ValueError('Expected grouped angular-support evidence')
    backend=pathlib.Path(report['backend'])
    for filename,key in [('optimized_3cm.pcd','map_sha256'),('trajectory.csv','trajectory_sha256')]:
        if digest(backend/filename)!=report[key]:
            raise ValueError('Source hash mismatch: '+filename)
    points=read_map(backend/'optimized_3cm.pcd')
    with np.load(directory/'point_evidence.npz') as archive:
        evidence={key:archive[key].copy() for key in archive.files}
    for key in ('removed','hit_bins','free_bins','first_free','last_free'):
        if evidence[key].shape!=(len(points),):
            raise ValueError('Evidence shape mismatch: '+key)
    removed=evidence['removed']
    settings=report['settings']
    recalculated=removal_mask(evidence['hit_bins'],evidence['free_bins'],evidence['first_free'],
        evidence['last_free'],settings['min_free_bins'],settings['free_ratio'],
        settings['min_span'],settings['max_hit_bins'])
    if removed.dtype!=np.bool_ or not np.array_equal(removed,recalculated):
        raise ValueError('Classification does not match evidence and settings')
    if np.any(removed & (evidence['hit_bins']>=4)):
        raise ValueError('Repeatedly supported geometry was removed')
    if not np.array_equal(read_map(directory/'filtered_3cm.pcd'),points[~removed]):
        raise ValueError('Retained point coordinates or intensity changed')
    if removed.any() and not np.array_equal(read_map(directory/'removed_3cm.pcd'),points[removed]):
        raise ValueError('Removed point partition mismatch')
    if report['baseline_points']!=len(points) or report['removed_points']!=int(removed.sum()):
        raise ValueError('Report count mismatch')
    return report,points,evidence


def compare(control_dir, interpolated_dir, output):
    if output.exists():
        raise ValueError('Use a fresh comparison output directory')
    control,points,old=checked_result(control_dir,'frame-end-control')
    interpolated,new_points,new=checked_result(interpolated_dir,'interpolated')
    for key in ('frontend','backend'):
        if pathlib.Path(control[key]).resolve()!=pathlib.Path(interpolated[key]).resolve():
            raise ValueError('Ablation source directory differs: '+key)
    for key in ('map_sha256','trajectory_sha256','input_frames','evidence_frames',
                'time_bin_count','point_time_skipped_frames'):
        if control.get(key)!=interpolated.get(key):
            raise ValueError('Ablation input/selection mismatch: '+key)
    if not np.array_equal(points,new_points):
        raise ValueError('Ablation baseline point order differs')
    old_settings={k:v for k,v in control['settings'].items() if k!='point_time_origin_mode'}
    new_settings={k:v for k,v in interpolated['settings'].items() if k!='point_time_origin_mode'}
    if old_settings!=new_settings:
        raise ValueError('Ablation settings differ beyond ray origin mode')
    frames=selected_frames(control_dir)
    if frames!=selected_frames(interpolated_dir) or len(frames)!=control['evidence_frames']:
        raise ValueError('Ablation frame selection or point-time grouping differs')
    restored=old['removed'] & ~new['removed']
    added=~old['removed'] & new['removed']
    planar=planar_sample_ids(points)
    result=dict(status='origin_ablation_requires_scene_review',
        changed_variable='ray origin within identical point-time partitions',
        same_source_map_trajectory_settings_and_selected_frames=True,
        control_directory=str(control_dir.resolve()),interpolated_directory=str(interpolated_dir.resolve()),
        map_sha256=control['map_sha256'],trajectory_sha256=control['trajectory_sha256'],
        control_report_sha256=digest(control_dir/'report.json'),
        interpolated_report_sha256=digest(interpolated_dir/'report.json'),
        selected_frames=len(frames),skipped_frames=control['point_time_skipped_frames'],
        baseline_points=len(points),control_removed=int(old['removed'].sum()),
        interpolated_removed=int(new['removed'].sum()),common_removed=int((old['removed'] & new['removed']).sum()),
        restored_by_origin_change=int(restored.sum()),newly_removed_by_origin_change=int(added.sum()),
        planar_sample_points=len(planar),control_planar_removed=int(old['removed'][planar].sum()),
        interpolated_planar_removed=int(new['removed'][planar].sum()),
        restored_planar_samples=int(restored[planar].sum()),newly_removed_planar_samples=int(added[planar].sum()),
        control_wall_seconds=control['wall_seconds'],interpolated_wall_seconds=interpolated['wall_seconds'],
        controlled_settings=old_settings,
        limitation='Restored/newly removed and geometric planar samples are not semantic labels or accuracy. '
                   'Origin interpolation remains an approximation. This control isolates origin changes; '
                   'comparison to ungrouped maps also changes ray sampling and possibly frame selection.')
    for key in ('hit_bins','free_bins'):
        delta=new[key].astype(np.int32)-old[key].astype(np.int32)
        result[key+'_change']=dict(fewer=int((delta<0).sum()),more=int((delta>0).sum()),
            unchanged=int((delta==0).sum()),minimum=int(delta.min()),maximum=int(delta.max()))
    output.mkdir(parents=True)
    np.savez_compressed(output/'changed_points.npz',restored=restored,newly_removed=added)
    for label,mask in [('restored_by_origin',restored),('newly_removed_by_origin',added)]:
        if mask.any():
            write_pcd(output/(label+'.pcd'),points[mask])
    (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('control',type=pathlib.Path)
    parser.add_argument('interpolated',type=pathlib.Path)
    parser.add_argument('--output-dir',type=pathlib.Path,required=True)
    args=parser.parse_args()
    print(json.dumps(compare(args.control,args.interpolated,args.output_dir),indent=2))


if __name__=='__main__':
    main()
