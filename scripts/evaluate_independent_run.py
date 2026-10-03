#!/usr/bin/env python3
"""Evaluate full-run continuity and temporal geometry without clipping errors."""
import argparse
import csv
import json
import pathlib
import re

import numpy as np
from scipy.spatial import cKDTree
import open3d as o3d


def write_pcd(path, points):
    data = np.ascontiguousarray(points, dtype="<f4")
    header = f"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(data)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(data)}\nDATA binary\n"
    with path.open("wb") as out:
        out.write(header.encode("ascii"))
        out.write(data.tobytes())


def voxel(points, size):
    xyz = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points[:, :3].astype(np.float64)))
    intensity = np.nan_to_num(points[:, 3], nan=0, posinf=0, neginf=0)
    xyz.colors = o3d.utility.Vector3dVector(np.repeat((intensity / 255)[:, None], 3, axis=1))
    sampled = xyz.voxel_down_sample(size)
    return np.column_stack((np.asarray(sampled.points), np.asarray(sampled.colors)[:, 0] * 255)).astype(np.float32)


def evaluate(run):
    meta = json.loads((run / "run.json").read_text())
    cap = json.loads((run / "capture/capture.json").read_text())
    trajectory = np.atleast_2d(np.loadtxt(run / "capture/trajectory.csv", delimiter=",", skiprows=1))
    if trajectory.size == 0:
        trajectory = np.empty((0, 8))
    if trajectory.shape[1] != 8:
        raise ValueError("Missing trajectory")
    start=cap["first_stamps"].get("input_lidar",meta["bag_info"]["start"])
    end=cap["last_stamps"].get("input_imu",meta["bag_info"]["end"])
    time_mask=(trajectory[:,0]>=start-.25)&(trajectory[:,0]<=end+.25)
    startup_ignored=int((~time_mask).sum())
    trajectory=trajectory[time_mask]
    if len(trajectory)<2:
        report = dict(run_status=meta["status"], counts=cap["counts"], trajectory_rows=len(trajectory),
                      trajectory_duration_s=0.0, cloud_frames=cap["cloud_frames"],
                      raw_input_lidar_missing=meta["expected_messages"]["/livox/lidar"]-cap["counts"]["input_lidar"],
                      raw_input_imu_missing=meta["expected_messages"]["/livox/imu"]-cap["counts"]["input_imu"],
                      startup_unstamped_odom_ignored=startup_ignored,
                      status="failed_not_eligible", failures=["insufficient_estimated_trajectory"],
                      absolute_accuracy="unavailable_without_external_ground_truth", guaranteed_zero_ghosting=False)
        (run / "evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        return report
    repeated=(np.diff(trajectory[:,0])==0)&(np.linalg.norm(np.diff(trajectory[:,1:4],axis=0),axis=1)<1e-9)&(np.linalg.norm(np.diff(trajectory[:,4:8],axis=0),axis=1)<1e-9)
    identical_ignored=int(repeated.sum())
    trajectory=trajectory[np.r_[True,~repeated]]
    pos = trajectory[:, 1:4]
    dt = np.diff(trajectory[:, 0])
    steps = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    speed = np.divide(steps, dt, out=np.full_like(steps, np.inf), where=dt > 0)
    q = trajectory[:, 4:8]
    qnorm = np.linalg.norm(q, axis=1)
    unit_q = q / np.maximum(qnorm[:, None], 1e-15)
    rotations = 2 * np.arccos(np.clip(np.abs(np.sum(unit_q[1:] * unit_q[:-1], axis=1)), 0, 1))
    expected = meta["expected_messages"]
    lost_lidar = expected["/livox/lidar"] - cap["counts"]["input_lidar"]
    lost_imu = expected["/livox/imu"] - cap["counts"]["input_imu"]
    report = {"run_status":meta["status"], "raw_input_lidar_missing":lost_lidar, "raw_input_imu_missing":lost_imu,
              "counts":cap["counts"], "trajectory_rows":len(trajectory), "trajectory_finite":bool(np.isfinite(trajectory).all()),
              "max_position_norm_m":float(np.linalg.norm(pos, axis=1).max()), "max_step_m":float(steps.max()),
              "p95_step_m":float(np.percentile(steps,95)), "max_speed_m_s":float(speed[dt>0].max()) if (dt>0).any() else None,
              "startup_unstamped_odom_ignored":startup_ignored,"identical_duplicate_odom_ignored":identical_ignored,
              "max_rotation_step_deg":float(np.rad2deg(rotations.max())), "nonpositive_dt":int((dt<=0).sum()),
              "quaternion_norm_max_error":float(np.abs(qnorm-1).max()),
              "trajectory_duration_s":float(trajectory[-1,0]-trajectory[0,0]), "cloud_frames":cap["cloud_frames"],
              "absolute_accuracy":"unavailable_without_external_ground_truth", "guaranteed_zero_ghosting":False}
    failures = []
    integration = meta.get("integration", {})
    state_time = integration.get("state_time", "scan_start")
    target_key = "input_lidar_scan_end" if state_time == "scan_end" else "input_lidar"
    target = cap["last_stamps"].get(target_key)
    tolerance = integration.get("end_output_tolerance_s", 1.0)
    report["endpoint_state_time"] = state_time
    report["endpoint_target_stamp"] = target
    if target is not None:
        report["endpoint_output_lag_s"] = float(target - min(cap["last_stamps"].get("output_cloud", 0), trajectory[-1, 0]))
        if report["endpoint_output_lag_s"] > tolerance:
            failures.append("incomplete_estimator_endpoint_coverage")
    elif state_time == "scan_end":
        failures.append("missing_raw_scan_end_evidence")
    launch_log = run / "mapping.log"
    process_failures = re.findall(r"process has died.*exit code (-?\d+)", launch_log.read_text(errors="replace")) if launch_log.exists() else []
    if any(int(code) != 0 for code in process_failures):
        failures.append("estimator_launch_child_process_failed")
    if launch_log.exists() and "terminate called after throwing" in launch_log.read_text(errors="replace"):
        failures.append("uncaught_estimator_exception")
    if cap["counts"]["output_cloud"] == 0 or not cap["temporal_raw"]:
        failures.append("no_estimated_point_clouds")
    if lost_lidar or lost_imu or cap["errors"]:
        failures.append("capture_message_loss_or_errors")
    if not report["trajectory_finite"] or report["max_position_norm_m"]>30 or (report["max_speed_m_s"] is not None and report["max_speed_m_s"]>5) or report["nonpositive_dt"]:
        failures.append("trajectory_divergence_or_implausible_handheld_motion")
    if report["trajectory_duration_s"] < meta["bag_info"]["duration"] - 5:
        failures.append("incomplete_trajectory_coverage")
    if meta["status"] != "full_bag_replay_finished_quality_pending":
        failures.append("runner_did_not_complete")
    report["failures"] = failures
    report["status"] = "failed_not_eligible" if failures else "bounded_full_run_geometry_screening_pending"
    evaluation = run / "evaluation.json"
    evaluation.write_text(json.dumps(report, indent=2) + "\n")
    if failures:
        print(json.dumps(report, indent=2))
        return report
    maps = run / "maps"
    maps.mkdir(exist_ok=True)
    temporal = []
    raw_count, finite_count = 0, 0
    for name in cap["temporal_raw"]:
        with np.load(run / "capture" / name) as chunk:
            points = chunk["points"]
            raw_count += len(points)
            mask = np.isfinite(points[:, :3]).all(axis=1)
            finite_count += int(mask.sum())
            sampled = voxel(points[mask], 0.03)
            temporal.append(sampled)
            write_pcd(maps / (pathlib.Path(name).stem + "_3cm.pcd"), sampled)
    merged = voxel(np.concatenate(temporal), 0.03)
    capture_map = maps / "full_capture_3cm.pcd"
    write_pcd(capture_map, merged)
    primary = run / meta.get("integration", {}).get("primary_map_relative", "upstream_global_keyframes.pcd")
    if not primary.exists():
        primary = capture_map
    preview_cloud = o3d.io.read_point_cloud(str(primary)).voxel_down_sample(0.06)
    preview = maps / "preview_6cm.pcd"
    o3d.io.write_point_cloud(str(preview), preview_cloud, write_ascii=False, compressed=False)
    geometry_rows = []
    for i in range(len(temporal)):
        target = temporal[i][:, :3]
        if not len(target):
            continue
        tree = cKDTree(target)
        for j in range(i+2, len(temporal)):
            source = temporal[j][::max(1,len(temporal[j])//25000),:3]
            distances = tree.query(source, workers=2)[0]
            matched = distances[distances<0.3]
            geometry_rows.append(dict(reference_window=i, later_window=j, source_samples=len(source),
                                      overlap_at_30cm=float((distances<0.3).mean()),
                                      median_nn_m=float(np.median(matched)) if len(matched) else None,
                                      p95_nn_m=float(np.percentile(matched,95)) if len(matched) else None,
                                      fraction_over_10cm=float((distances>0.1).mean())))
    with (run / "temporal_consistency.csv").open("w") as out:
        if geometry_rows:
            writer=csv.DictWriter(out,fieldnames=list(geometry_rows[0]))
            writer.writeheader()
            writer.writerows(geometry_rows)
    report.update(raw_cloud_points=raw_count, finite_cloud_points=finite_count, capture_map_points=len(merged),
                  capture_map=str(capture_map), primary_map=str(primary), preview=str(preview),
                  preview_points=len(preview_cloud.points), temporal_pairs=len(geometry_rows),
                  status="screened_requires_surface_and_visual_validation",
                  temporal_metric_scope="Nearest-neighbor comparisons in estimator coordinates; no per-window ICP. Overlap/scene changes affect these metrics.",
                  fusion="3cm voxel means for capture map; raw temporal outputs retained; no coordinate clipping or trajectory substitution")
    evaluation.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return report


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("run",type=pathlib.Path)
    evaluate(parser.parse_args().run.resolve())
