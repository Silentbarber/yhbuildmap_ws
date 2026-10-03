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
                        help='Optional prior evidence root: <root>/<id>/visibility_v2')
    parser.add_argument('--id',choices=['215247','162102','162342','162744'],action='append')
    args=parser.parse_args()
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
            '--output-dir',str(output),'--frame-step','.3','--angle-deg','.7','--max-hit-bins','3']
        if args.reuse_root:
            command+=['--reuse-evidence',str(args.reuse_root/record['id']/'visibility_v2')]
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
