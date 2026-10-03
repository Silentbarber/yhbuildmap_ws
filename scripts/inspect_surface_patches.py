#!/usr/bin/env python3
"""Inspect isolated surface patches across time, without aligning trajectories."""
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


def basis(normal):
    axis = np.eye(3)[np.argmin(np.abs(normal))]
    first = np.cross(normal, axis)
    first /= np.linalg.norm(first)
    return np.column_stack((first, np.cross(normal, first)))


def select_patches(cloud, plane, radius, max_patches):
    normal = np.array(plane["model"][:3])
    offset = plane["model"][3]
    tangent = basis(normal)
    residual = cloud @ normal + offset
    slab = cloud[np.abs(residual) < .30]
    inliers = cloud[np.abs(residual) < .015]
    if len(inliers) < 200:
        return []
    tree = cKDTree(slab @ tangent)
    candidates = inliers[::max(1, len(inliers)//200)]
    proposals = []
    for anchor in candidates:
        nearby = slab[tree.query_ball_point(anchor @ tangent, radius)]
        if len(nearby) < 150:
            continue
        values = nearby @ normal + offset
        thin_fraction = float((np.abs(values) < .03).mean())
        if thin_fraction < .85:
            continue
        proposals.append((len(nearby) * thin_fraction, anchor, nearby, thin_fraction))
    proposals.sort(key=lambda item: item[0], reverse=True)
    patches = []
    for _, anchor, nearby, fraction in proposals:
        if any(np.linalg.norm((anchor-p["anchor"]) @ tangent) < 2*radius for p in patches):
            continue
        patches.append(dict(anchor=anchor, reference_points=nearby,
                            baseline_thin_fraction=fraction, tangent=tangent,
                            normal=normal, offset=offset, plane=plane["key"]))
        if len(patches) == max_patches:
            break
    return patches


def normal_mask(tree, points, normal):
    if len(tree.data) < 12:
        return np.zeros(len(points), dtype=bool)
    distance, neighbors = tree.query(points, k=12, workers=2)
    near = distance[:, -1] < .15
    valid = np.flatnonzero(near)
    result = np.zeros(len(points), dtype=bool)
    if not len(valid):
        return result
    local = tree.data[neighbors[valid]]
    centered = local-local.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered)/12
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    curvature = eigenvalues[:, 0]/np.maximum(eigenvalues.sum(axis=1), 1e-12)
    aligned = np.abs(eigenvectors[:, :, 0] @ normal) > np.cos(np.deg2rad(15))
    result[valid] = aligned & (curvature < .03)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=pathlib.Path)
    parser.add_argument("--maps", type=pathlib.Path)
    parser.add_argument("--output-dir", type=pathlib.Path)
    parser.add_argument("--patches-from", type=pathlib.Path,
                        help="Reuse fixed native reference patches for an experimental corrected map")
    parser.add_argument("--radius", type=float, default=.35)
    parser.add_argument("--max-patches-per-plane", type=int, default=2)
    args = parser.parse_args()
    if not .15 <= args.radius <= 1 or not 1 <= args.max_patches_per_plane <= 4:
        parser.error("radius must be .15--1 m and patch count 1--4")
    run = args.run.resolve()
    maps = args.maps.resolve() if args.maps else run/"maps"
    paths = sorted(maps.glob("window_*_3cm.pcd"))
    if not paths:
        raise RuntimeError("No temporal maps available for surface inspection")
    clouds = [np.asarray(o3d.io.read_point_cloud(str(p)).points).copy() for p in paths]
    patches = []
    if args.patches_from:
        patches = json.loads(args.patches_from.read_text())
        for patch in patches:
            for key in ("anchor", "tangent", "normal"):
                patch[key] = np.asarray(patch[key], dtype=float)
            if not 0 <= patch["reference_window"] < len(clouds):
                raise ValueError("Reference patch window absent from measured temporal maps")
    else:
        planes = json.loads((run/"surface_planes.json").read_text())
        for plane in planes:
            for patch in select_patches(clouds[plane["reference_window"]], plane, args.radius, args.max_patches_per_plane):
                patch["reference_window"] = plane["reference_window"]
                patch["key"] = plane["key"]+"_patch_"+str(len(patches))
                patches.append(patch)
    out = args.output_dir.resolve() if args.output_dir else run/"surface_patches"
    out.mkdir(parents=True, exist_ok=True)
    serialized = [{k:(v.tolist() if isinstance(v,np.ndarray) else v)
                   for k,v in p.items() if k != "reference_points"} for p in patches]
    (out/"patches.json").write_text(json.dumps(serialized,indent=2)+"\n")
    bins = np.linspace(-.30,.30,121)
    raw_hist = np.zeros((len(patches),len(clouds),120))
    normal_hist = np.zeros_like(raw_hist)
    records = []
    for window, points in enumerate(clouds):
        tree = cKDTree(points)
        for index, patch in enumerate(patches):
            projected = (points-patch["anchor"]) @ patch["tangent"]
            signed = points @ patch["normal"]+patch["offset"]
            footprint = (np.linalg.norm(projected,axis=1)<args.radius) & (np.abs(signed)<.30)
            sampled = points[footprint]
            residual = signed[footprint]
            if len(residual)<30:
                continue
            aligned = normal_mask(tree, sampled, patch["normal"])
            planar = residual[aligned]
            raw_hist[index,window] = np.histogram(residual,bins=bins)[0]/len(residual)
            if len(planar):
                normal_hist[index,window] = np.histogram(planar,bins=bins)[0]/len(planar)
            row = dict(patch=patch["key"], reference_window=patch["reference_window"], measurement_window=window,
                       raw_points=len(residual), normal_aligned_points=len(planar),
                       normal_retained_fraction=float(aligned.mean()),
                       raw_signed_median_m=float(np.median(residual)),
                       raw_p95_abs_m=float(np.percentile(np.abs(residual),95)),
                       planar_signed_median_m=float(np.median(planar)) if len(planar)>=30 else None,
                       planar_p95_abs_m=float(np.percentile(np.abs(planar),95)) if len(planar)>=30 else None,
                       planar_p95_minus_p05_m=float(np.percentile(planar,95)-np.percentile(planar,5)) if len(planar)>=30 else None)
            records.append(row)
        print(f"surface patch window {window+1}/{len(clouds)}",flush=True)
    with (out/"measurements.csv").open("w") as file:
        if records:
            writer = csv.DictWriter(file,fieldnames=list(records[0])); writer.writeheader(); writer.writerows(records)
    cross = [r for r in records if abs(r["reference_window"]-r["measurement_window"])>=2 and r["planar_p95_abs_m"] is not None]
    summary = dict(status="diagnostic_only_requires_visual_scene_review", patches=len(patches),
                   cross_time_observations=len(cross), alignment="none applied by this measurement; input map coordinates",
                   source_map_directory=str(maps),
                   reference_patches=str(args.patches_from.resolve()) if args.patches_from else "selected from measured native temporal maps",
                   median_cross_time_planar_p95_abs_m=float(np.median([r["planar_p95_abs_m"] for r in cross])) if cross else None,
                   max_cross_time_planar_p95_abs_m=max((r["planar_p95_abs_m"] for r in cross),default=None),
                   guaranteed_zero_ghosting=False,
                   scope="Reference patches must have >=85% of baseline points within 3cm in a 30cm slab. Raw residuals and retained normal-aligned, locally planar residuals are both recorded. Normal filtering can reject blurred surfaces; reduced retention is not proof of accuracy. Parallel objects and changing visibility still require scene review.")
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    for begin in range(0,len(patches),6):
        group = patches[begin:begin+6]
        fig,axes = plt.subplots(len(group),2,figsize=(12,2.3*len(group)),squeeze=False,constrained_layout=True)
        for i,patch in enumerate(group):
            for j,data in enumerate((raw_hist,normal_hist)):
                axes[i,j].imshow(data[begin+i],origin="lower",aspect="auto",extent=(-.3,.3,0,len(clouds)),cmap="magma")
                axes[i,j].axvline(0,color="cyan",lw=.7)
                axes[i,j].set(title=patch["key"]+(" raw" if j==0 else " local normals"),xlabel="Signed plane residual [m]",ylabel="20 s window")
        fig.savefig(out/f"residuals_{begin//6:02d}.png",dpi=130); plt.close(fig)
    print(json.dumps(summary,indent=2))


if __name__ == "__main__":
    main()
