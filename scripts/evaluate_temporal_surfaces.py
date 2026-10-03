#!/usr/bin/env python3
"""Measure cross-time plane residuals in native estimator coordinates."""
import argparse
import csv
import json
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("run",type=pathlib.Path)
    parser.add_argument("--maps",type=pathlib.Path)
    parser.add_argument("--output-dir",type=pathlib.Path)
    args=parser.parse_args()
    run=args.run.resolve()
    map_directory=args.maps.resolve() if args.maps else run/"maps"
    output=args.output_dir.resolve() if args.output_dir else run
    output.mkdir(parents=True, exist_ok=True)
    paths=sorted(map_directory.glob("window_*_3cm.pcd"))
    if not paths:
        raise RuntimeError("Evaluate the complete run before surface checks")
    o3d.utility.random.seed(7)
    clouds=[o3d.io.read_point_cloud(str(p)) for p in paths]
    refs=sorted(set([0,len(clouds)//2,len(clouds)-1]))
    records,planes,histograms=[],[],[]
    for ref in refs:
        remainder=clouds[ref]
        for plane_id in range(3):
            if len(remainder.points)<500:
                break
            model,indices=remainder.segment_plane(distance_threshold=.015,ransac_n=3,num_iterations=1000)
            if len(indices)<500:
                break
            patch=np.asarray(remainder.select_by_index(indices).points)
            model=np.array(model,dtype=float)
            model/=np.linalg.norm(model[:3])
            anchor=patch.mean(axis=0)
            # Query only around the observed patch. Unobserved area is not an error.
            tree=cKDTree(patch)
            ref_sigma=float(np.std(patch@model[:3]+model[3]))
            plane_key=f"reference_{ref:02d}_plane_{plane_id}"
            planes.append(dict(key=plane_key,reference_window=ref,model=model.tolist(),anchor=anchor.tolist(),
                               inliers=len(patch),reference_sigma_m=ref_sigma))
            hist=[]
            for j,cloud in enumerate(clouds):
                points=np.asarray(cloud.points)
                if not len(points):
                    continue
                distances=tree.query(points,workers=2)[0]
                supported=points[distances<.30]
                residual=supported@model[:3]+model[3]
                if len(residual)<100:
                    continue
                absolute=np.abs(residual)
                records.append(dict(plane=plane_key,reference_window=ref,measurement_window=j,
                                    points_near_patch=len(residual),signed_median_m=float(np.median(residual)),
                                    median_abs_m=float(np.median(absolute)),p95_abs_m=float(np.percentile(absolute,95)),
                                    fraction_above_3cm=float((absolute>.03).mean()),fraction_above_6cm=float((absolute>.06).mean())))
                density,_=np.histogram(residual,bins=np.linspace(-.3,.3,121))
                hist.append((j,density/max(len(residual),1)))
            histograms.append((plane_key,hist))
            remainder=remainder.select_by_index(indices,invert=True)
    with (output/"surface_consistency.csv").open("w") as out:
        if records:
            writer=csv.DictWriter(out,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
    (output/"surface_planes.json").write_text(json.dumps(planes,indent=2)+"\n")
    different=[r for r in records if abs(r["reference_window"]-r["measurement_window"])>=2]
    summary=dict(status="measured_requires_scene_review",reference_planes=len(planes),cross_time_patch_observations=len(different),
                 source_map_directory=str(map_directory),
                 no_pose_adjustment=True,guaranteed_zero_ghosting=False,
                 scope="RANSAC plane patches from individual time windows; later raw estimator outputs queried within 30cm of observed patches. Objects near a plane and changing visibility can increase residuals; this is a screening signal, not surveyed truth.",
                 median_cross_time_p95_abs_m=float(np.median([r["p95_abs_m"] for r in different])) if different else None,
                 max_cross_time_p95_abs_m=max((r["p95_abs_m"] for r in different),default=None))
    (output/"surface_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    if histograms:
        fig,axes=plt.subplots(len(histograms),1,figsize=(10,2.5*len(histograms)),squeeze=False,constrained_layout=True)
        for ax,(name,hist) in zip(axes[:,0],histograms):
            matrix=np.zeros((len(clouds),120))
            for j,counts in hist:
                matrix[j]=counts
            ax.imshow(matrix,origin="lower",aspect="auto",extent=(-.3,.3,0,len(clouds)),cmap="magma")
            ax.axvline(0,color="cyan",lw=.7)
            ax.set(title=name,xlabel="Signed point-to-plane distance [m]",ylabel="Time window (20 s)")
        fig.savefig(output/"surface_residuals.png",dpi=150)
        plt.close(fig)
    print(json.dumps(summary,indent=2))


if __name__=="__main__":
    main()
