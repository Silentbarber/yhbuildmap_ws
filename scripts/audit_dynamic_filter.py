#!/usr/bin/env python3
"""Audit subset preservation and plot fixed-coordinate removal comparisons."""
import argparse
import json
import pathlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from filter_dynamic_map import read_map


def panels(baseline, removed, path, limits=None, title='Fixed-coordinate comparison'):
    fig, axes=plt.subplots(3,3,figsize=(15,12))
    keep=~removed
    subsets=[baseline,baseline[keep],baseline[removed]]
    names=['Baseline','Retained','Removed (temporal inconsistency candidates)']
    for row,(a,b) in enumerate([(0,1),(0,2),(1,2)]):
        low,high=(np.percentile(baseline[:,:3],[0,100],axis=0) if limits is None else limits)
        for col,points in enumerate(subsets):
            axes[row,col].scatter(points[:,a],points[:,b],s=.15 if col<2 else 2,
                                  color='#687b83' if col<2 else '#bf3346',rasterized=True)
            axes[row,col].set_xlim(low[a],high[a])
            axes[row,col].set_ylim(low[b],high[b])
            axes[row,col].set_aspect('equal',adjustable='box')
            axes[row,col].set_xlabel('XYZ'[a]+' (m)')
            axes[row,col].set_ylabel('XYZ'[b]+' (m)')
            axes[row,col].set_title(names[col]+' / '+str(len(points)))
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path,dpi=140)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline',type=pathlib.Path)
    parser.add_argument('filtered',type=pathlib.Path,help='Directory containing report/evidence/maps')
    parser.add_argument('--patches',type=pathlib.Path)
    args=parser.parse_args()
    source=read_map(args.baseline)
    kept=read_map(args.filtered/'filtered_3cm.pcd')
    with np.load(args.filtered/'point_evidence.npz') as evidence:
        removed=evidence['removed']
        hits=evidence['hit_bins']
        frees=evidence['free_bins']
    removed_points=(read_map(args.filtered/'removed_3cm.pcd') if removed.any() else np.empty((0,4),np.float32))
    assert np.array_equal(kept,source[~removed]),'Retained map coordinates/intensity changed'
    assert np.array_equal(removed_points,source[removed]),'Removed point partition mismatch'
    output=args.filtered/'audit'
    output.mkdir(exist_ok=True)
    # Local PCA yields geometric planar candidates, not verified static labels.
    tree=cKDTree(source[:,:3])
    planar_counts=[]
    sample=np.arange(0,len(source),4)
    for offset in range(0,len(sample),25000):
        ids=sample[offset:offset+25000]
        distances,neighbours=tree.query(source[ids,:3],k=16,workers=2)
        xyz=source[neighbours,:3].astype(float)
        centered=xyz-xyz.mean(axis=1,keepdims=True)
        cov=np.einsum('nki,nkj->nij',centered,centered)/16
        eig=np.linalg.eigvalsh(cov)
        planar=(distances[:,-1]<.20)&(eig[:,0]/np.maximum(eig.sum(axis=1),1e-12)<.02)&(eig[:,1]>.0001)
        planar_counts.append((int(planar.sum()),int((planar&removed[ids]).sum())))
    patches=[]
    if args.patches:
        for patch in json.loads(args.patches.read_text()):
            anchor=np.asarray(patch['anchor'])
            normal=np.asarray(patch['normal'])
            tangent=np.asarray(patch['tangent'])
            projected=(source[:,:3]-anchor)@tangent
            residual=source[:,:3]@normal+patch['offset']
            mask=(np.linalg.norm(projected,axis=1)<.35)&(np.abs(residual)<.03)
            count=int(mask.sum())
            deleted=int((mask&removed).sum())
            patches.append(dict(key=patch['key'],points=count,removed=deleted,
                retained_fraction=1-deleted/count if count else None,
                before_p95_m=float(np.percentile(np.abs(residual[mask]),95)) if count else None,
                after_p95_m=float(np.percentile(np.abs(residual[mask&~removed]),95)) if np.any(mask&~removed) else None))
    clusters=[]
    if len(removed_points):
        cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(removed_points[:,:3]))
        labels=np.asarray(cloud.cluster_dbscan(eps=.35,min_points=10,print_progress=False))
        counts=[(int((labels==label).sum()),label) for label in np.unique(labels) if label>=0]
        for rank,(count,label) in enumerate(sorted(counts,reverse=True)[:8]):
            points=removed_points[labels==label,:3]
            low,high=points.min(axis=0)-.5,points.max(axis=0)+.5
            center=(low+high)/2
            extent=np.maximum(high-low,1.5)
            low,high=center-extent/2,center+extent/2
            local_mask=((source[:,:3]>=low)&(source[:,:3]<=high)).all(axis=1)
            panels(source[local_mask],removed[local_mask],output/('region_{:02d}.png'.format(rank)),
                (low,high),'Removal cluster {} (not a motion label)'.format(rank))
            clusters.append(dict(rank=rank,removed_points=count,lower=low.tolist(),upper=high.tolist(),
                baseline_region_points=int(local_mask.sum()),region_removed=int(removed[local_mask].sum())))
    panels(source,removed,output/'whole_map.png')
    records=dict(exact_subset_partition_passed=True,baseline_points=len(source),filtered_points=len(kept),
        removed_points=len(removed_points),static_patch_checks=patches,
        planar_sample_points=sum(x[0] for x in planar_counts),planar_sample_removed=sum(x[1] for x in planar_counts),
        removal_clusters=clusters,removed_hit_quantiles=np.percentile(hits[removed],[0,50,90,100]).tolist() if removed.any() else [],
        removed_free_quantiles=np.percentile(frees[removed],[0,50,90,100]).tolist() if removed.any() else [],
        metric_scope='Fixed baseline-coordinate geometric candidates and existing fixed surface patches. Not semantic dynamic precision/recall; planar moving objects may exist.')
    (output/'report.json').write_text(json.dumps(records,indent=2)+'\n')
    print(json.dumps(records,indent=2))


if __name__=='__main__':
    main()
