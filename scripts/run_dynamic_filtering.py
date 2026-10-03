#!/usr/bin/env python3
"""Filter the four recorded baselines with identical settings and audit them."""
import argparse
import hashlib
import json
import pathlib
import subprocess
import sys


ROOT=pathlib.Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root',type=pathlib.Path,required=True,
                        help='Existing experiment root matching the checked baseline inventory')
    parser.add_argument('--output-root',type=pathlib.Path,required=True)
    parser.add_argument('--reuse-root',type=pathlib.Path,
                        help='Prior evidence root: <root>/<id>/; legacy nested visibility_v2 also accepted')
    parser.add_argument('--id',choices=['215247','162102','162342','162744'],action='append')
    angular=parser.add_mutually_exclusive_group()
    angular.add_argument('--require-angular-support',action='store_true',
                        help='Use measured-ray triangle support; already enabled by default')
    angular.add_argument('--allow-angular-extrapolation',action='store_true',
                        help='Experimental legacy comparison: disable triangle support')
    parser.add_argument('--point-time-groups-ms',type=float,default=0.,
                        help='Experimental: use captured per-point times and interpolated ray origins')
    args=parser.parse_args()
    if args.point_time_groups_ms and not .5 <= args.point_time_groups_ms <= 20:
        parser.error('point-time groups must be .5--20ms')
    args.require_angular_support=not args.allow_angular_extrapolation
    records=json.loads((ROOT/'config/dynamic_filtering_recordings.json').read_text())['recordings']
    selected=[r for r in records if not args.id or r['id'] in args.id]
    for record in selected:
        backend=args.data_root/record['backend']
        for filename,key in [('optimized_3cm.pcd','map_sha256'),('trajectory.csv','trajectory_sha256')]:
            if hashlib.sha256((backend/filename).read_bytes()).hexdigest()!=record[key]:
                raise ValueError('Baseline hash mismatch: '+record['id']+'/'+filename)
        if (args.output_root/record['id']).exists():
            raise ValueError('Use a fresh output root: '+record['id'])
    published=[]
    for record in selected:
        frontend=args.data_root/record['frontend']
        backend=args.data_root/record['backend']
        output=args.output_root/record['id']
        command=[sys.executable,str(ROOT/'scripts/filter_dynamic_map.py'),str(frontend),str(backend),
            '--output-dir',str(output),'--frame-step','.3','--angle-deg','.7','--max-hit-bins','3',
            '--point-time-groups-ms',str(args.point_time_groups_ms)]
        if args.require_angular_support:
            command+=['--require-angular-support']
        if args.reuse_root:
            reuse=args.reuse_root/record['id']
            if not (reuse/'report.json').exists():
                reuse=reuse/'visibility_v2'
            command+=['--reuse-evidence',str(reuse)]
        subprocess.run(command,check=True)
        audit=[sys.executable,str(ROOT/'scripts/audit_dynamic_filter.py'),
               str(backend/'optimized_3cm.pcd'),str(output)]
        patches=backend.parent/'coverage_geometry_v5_validation/surface_patches/patches.json'
        if patches.exists():
            audit+=['--patches',str(patches)]
        subprocess.run(audit,check=True)
        published.append(dict(id=record['id'],bag=record['bag'],baseline=str(backend/'optimized_3cm.pcd'),
            filtered=str(output/'filtered_3cm.pcd'),removed=str(output/'removed_3cm.pcd'),
            report=json.loads((output/'report.json').read_text()),audit=json.loads((output/'audit/report.json').read_text())))
    (args.output_root/'summary.json').write_text(json.dumps(dict(
        status='experimental_requires_user_scene_review',baseline_commit='9a11f2e',recordings=published),indent=2)+'\n')


if __name__=='__main__':
    main()
