#!/usr/bin/env python3
"""Optimize one complete estimator run using only its own scans and trajectory."""
import argparse
import csv
import json
import pathlib
import time

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

from evaluate_independent_run import voxel, write_pcd


def transform(pose):
    matrix=np.eye(4)
    matrix[:3,:3]=Rotation.from_quat(pose[4:8]).as_matrix()
    matrix[:3,3]=pose[1:4]
    return matrix


def validate_loop(source, target, training, result, minimum_overlap=.55):
    """Check unused spatial blocks, actual plane geometry and reverse convergence."""
    registration = o3d.pipelines.registration
    test = source.select_by_index(np.flatnonzero(~training))
    heldout = registration.evaluate_registration(test, target, .10, result.transformation)
    xyz = np.asarray(test.points)
    transformed = xyz @ result.transformation[:3, :3].T + result.transformation[:3, 3]
    distances, indices = cKDTree(np.asarray(target.points)).query(transformed, workers=2)
    mask = distances < .10
    eigenvalue_ratio = 0.0
    if mask.sum() >= 100:
        points = transformed[mask]
        normals = np.asarray(target.normals)[indices[mask]]
        radius = max(float(np.sqrt(np.mean(np.sum(points * points, axis=1)))), 1e-6)
        jacobian = np.column_stack((np.cross(points, normals) / radius, normals))
        eigenvalues = np.linalg.eigvalsh(jacobian.T @ jacobian)
        eigenvalue_ratio = float(eigenvalues[0] / max(eigenvalues[-1], 1e-12))
    reverse = registration.registration_generalized_icp(target, source, .15,
        np.linalg.inv(result.transformation), registration.TransformationEstimationForGeneralizedICP(),
        registration.ICPConvergenceCriteria(max_iteration=25))
    cycle = reverse.transformation @ result.transformation
    cycle_translation = float(np.linalg.norm(cycle[:3, 3]))
    cycle_angle = float(np.rad2deg(Rotation.from_matrix(cycle[:3, :3].copy()).magnitude()))
    return dict(heldout_overlap=float(heldout.fitness), heldout_rmse_m=float(heldout.inlier_rmse),
                plane_information_eigenvalue_ratio=eigenvalue_ratio,
                reverse_cycle_translation_m=cycle_translation, reverse_cycle_rotation_deg=cycle_angle,
                validation_passed=bool(heldout.fitness > minimum_overlap and heldout.inlier_rmse < .04 and
                                       eigenvalue_ratio > 1e-4 and cycle_translation < .05 and cycle_angle < .5))


def plane_information(source, target, transformation, sigma=.02, effective_samples=32):
    """Preserve planar null directions; cap confidence from correlated scan points."""
    xyz = np.asarray(source.points)
    points = xyz @ transformation[:3, :3].T + transformation[:3, 3]
    distances, indices = cKDTree(np.asarray(target.points)).query(points, workers=2)
    keep = distances < .08
    if keep.sum() < 100:
        return np.eye(6) * 1e-6
    normals = np.asarray(target.normals)[indices[keep]]
    jacobian = np.column_stack((np.cross(points[keep], normals), normals))
    information = jacobian.T @ jacobian / keep.sum() * effective_samples / sigma**2
    return information + np.eye(6) * 1e-6


def local_registration(source, target, initial):
    """Independent forward/reverse fits screen short-range geometric constraints."""
    registration = o3d.pipelines.registration
    result = registration.registration_generalized_icp(source, target, .20, initial,
        registration.TransformationEstimationForGeneralizedICP(),
        registration.ICPConvergenceCriteria(max_iteration=25))
    reverse_fit = registration.registration_generalized_icp(target, source, .20, np.linalg.inv(initial),
        registration.TransformationEstimationForGeneralizedICP(),
        registration.ICPConvergenceCriteria(max_iteration=25))
    forward = registration.evaluate_registration(source, target, .08, result.transformation)
    reverse = registration.evaluate_registration(target, source, .08, reverse_fit.transformation)
    correction = result.transformation @ np.linalg.inv(initial)
    cycle = reverse_fit.transformation @ result.transformation
    translation = float(np.linalg.norm(correction[:3, 3]))
    angle = float(np.rad2deg(Rotation.from_matrix(correction[:3, :3].copy()).magnitude()))
    cycle_translation = float(np.linalg.norm(cycle[:3, 3]))
    cycle_angle = float(np.rad2deg(Rotation.from_matrix(cycle[:3, :3].copy()).magnitude()))
    passed = (min(forward.fitness, reverse.fitness) > .55 and
              max(forward.inlier_rmse, reverse.inlier_rmse) < .035 and
              translation < .15 and angle < 3 and cycle_translation < .015 and cycle_angle < .25)
    return result.transformation, dict(forward_overlap=float(forward.fitness),
        reverse_overlap=float(reverse.fitness), rmse_m=float(forward.inlier_rmse),
        correction_translation_m=translation, correction_rotation_deg=angle,
        cycle_translation_m=cycle_translation, cycle_rotation_deg=cycle_angle, accepted=bool(passed))


def observation_odometry_information(rows, start, end, target_rotation):
    """Heuristic integrated noise: weak scans permit correction, rich scans resist it."""
    selected = [row for row in rows if start < float(row['stamp']) <= end]
    yaw_variance = 0.0
    position_variance = 0.0
    for row in selected:
        features = max(float(row['features']), 0)
        radius = max(float(row['matched_range_rms_m']), 0)
        yaw_strength = features * radius**2 * max(float(row['normalized_yaw_information']), 0)
        spatial_strength = features * radius**2
        yaw_variance += .0002**2 * (1 + 1000/max(yaw_strength, .01))
        position_variance += .001**2 * (1 + 1000/max(spatial_strength, .1))
    # Sub-millisecond timestamp jitter is not a missing 10 Hz scan.
    missing = max(0, round((end-start)/.1)-len(selected)) * .1
    yaw_variance += missing * .20**2
    position_variance += missing * .20**2
    gravity = np.array([float(rows[0][key]) for key in ['gx', 'gy', 'gz']])
    gravity /= np.linalg.norm(gravity)
    axis = target_rotation.T @ gravity
    projector = np.outer(axis, axis)
    tilt_variance = .002**2 * max(end-start, .1)
    information = np.zeros((6, 6))
    information[:3, :3] = (np.eye(3)-projector)/tilt_variance + projector/max(yaw_variance, 1e-10)
    information[3:, 3:] = np.eye(3)/max(position_variance, 1e-8)
    return information, dict(yaw_sigma_rad=float(np.sqrt(yaw_variance)),
                            position_sigma_m=float(np.sqrt(position_variance)), missing_update_duration_s=missing)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("run",type=pathlib.Path)
    parser.add_argument("--submap-half-window",type=float,default=0.0)
    parser.add_argument("--max-candidates",type=int,default=4)
    parser.add_argument("--validate-loops", action="store_true",
                        help="Fit alternating spatial blocks; check heldout geometry and reverse convergence")
    parser.add_argument("--output-dir", type=pathlib.Path)
    parser.add_argument("--keyframe-step", type=float, default=2.0)
    parser.add_argument("--local-geometric-edges", action="store_true",
                        help="Add screened short-range scan constraints; preserve planar null directions")
    parser.add_argument("--plane-information", action="store_true",
                        help="Use bounded point-to-plane information for loops instead of point-to-point information")
    parser.add_argument("--observation-weighted-odometry", action="store_true",
                        help="Use recorded scan observation diagnostics to weight odometry; heuristic, not measured covariance")
    parser.add_argument("--reuse-geometric-constraints", type=pathlib.Path,
                        help="Reuse audited registration constraints from the identical run and graph settings")
    parser.add_argument("--loop-search-radius", type=float, default=1.5)
    parser.add_argument("--loop-overlap", type=float, default=.60)
    parser.add_argument("--heldout-overlap", type=float, default=.55)
    args=parser.parse_args()
    if not 0 <= args.submap_half_window <= 2:
        parser.error("submap half-window must be in [0, 2] seconds")
    if not 1 <= args.max_candidates <= 20:
        parser.error("candidate count must be in [1, 20]")
    if not .25 <= args.keyframe_step <= 2:
        parser.error("keyframe step must be in [.25, 2] seconds")
    if not 1 <= args.loop_search_radius <= 6 or not .30 <= args.loop_overlap <= .8 or not .25 <= args.heldout_overlap <= .8:
        parser.error("Loop radius must be 1--6m and overlap thresholds must be in supported bounds")
    run=args.run.resolve()
    evaluation=json.loads((run/"evaluation.json").read_text())
    if evaluation.get("failures"):
        raise RuntimeError("Do not optimize a divergent or incomplete run into an accepted map")
    trace=[]
    if args.observation_weighted_odometry:
        with (run/"state_trace.csv").open() as file:
            reader=csv.DictReader(file)
            required={'matched_range_rms_m','normalized_yaw_information','features','gx','gy','gz'}
            if not required.issubset(reader.fieldnames or []):
                raise ValueError('Observation weighting requires an instrumented replay with range and yaw diagnostics; rerun the ROS1 frontend with trace_path enabled')
            trace=[row for row in reader if row['stage']=='update']
        if not trace:
            raise RuntimeError("Observation weighting requires recorded update diagnostics")
    output=args.output_dir.resolve() if args.output_dir else run/("pose_graph_"+time.strftime("%Y%m%d_%H%M%S"))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir()
    trajectory=np.loadtxt(run/"capture/trajectory.csv",delimiter=",",skiprows=1)
    cap=json.loads((run/"capture/capture.json").read_text())
    stamps,poses,frames=[],[],[]
    for filename in cap["temporal_raw"]:
        with np.load(run/"capture"/filename) as chunk:
            cuts=np.r_[0,np.cumsum(chunk["lengths"])]
            chunk_points=chunk["points"]
            for j,stamp in enumerate(chunk["stamps"]):
                index=int(np.argmin(np.abs(trajectory[:,0]-stamp)))
                if abs(trajectory[index,0]-stamp)>1e-5:
                    raise RuntimeError("Missing estimator pose for a cloud; cannot borrow or interpolate another trajectory")
                frames.append(chunk_points[cuts[j]:cuts[j+1]].copy())
                stamps.append(stamp)
                poses.append(transform(trajectory[index]))
    poses=np.array(poses)
    stamps=np.array(stamps)
    key_indices=[]
    for i,frame in enumerate(frames):
        if len(frame)<100:
            continue
        if not key_indices or stamps[i]-stamps[key_indices[-1]]>=args.keyframe_step:
            key_indices.append(i)
    if len(key_indices)<3:
        raise RuntimeError("Insufficient keyframes")
    reused_graph=None
    if args.reuse_geometric_constraints:
        previous=args.reuse_geometric_constraints.resolve()
        previous_report=json.loads((previous/"report.json").read_text())
        if pathlib.Path(previous_report['source_run']).resolve()!=run or any([
            previous_report['keyframes']!=len(key_indices),
            previous_report['submap_half_window_s']!=args.submap_half_window,
            previous_report['max_loop_candidates_per_keyframe']!=args.max_candidates,
            previous_report.get('keyframe_step_s',2)!=args.keyframe_step,
            bool(previous_report.get('local_geometric_candidates',0))!=args.local_geometric_edges,
            previous_report['loop_validation'].startswith('heldout')!=args.validate_loops,
            previous_report.get('loop_information','').startswith('Point-to-plane')!=args.plane_information]):
            raise ValueError('Constraint reuse requires identical source, keyframes and registration settings')
        if any(previous_report.get(key,default)!=value for key,default,value in [
            ('loop_search_radius_m',1.5,args.loop_search_radius),('loop_overlap_threshold',.60,args.loop_overlap),
            ('heldout_overlap_threshold',.55,args.heldout_overlap)]):
            raise ValueError('Cannot reuse constraints with different search or validation thresholds')
        reused_graph=o3d.io.read_pose_graph(str(previous/"graph_before_optimization.json"))
        if len(reused_graph.nodes)!=len(key_indices) or any(not np.allclose(node.pose,poses[index],atol=1e-10,rtol=0)
                                                          for node,index in zip(reused_graph.nodes,key_indices)):
            raise ValueError('Reusable graph nodes do not match the actual native trajectory')
    clouds=[]
    for index in (key_indices if reused_graph is None else []):
        nearby=np.flatnonzero(np.abs(stamps-stamps[index])<=args.submap_half_window)
        points=np.concatenate([frames[j][:,:3] for j in nearby])
        points=points[np.isfinite(points).all(axis=1)]
        body=(points-poses[index,:3,3])@poses[index,:3,:3]
        cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(body))
        cloud=cloud.voxel_down_sample(.06)
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.25,max_nn=25))
        clouds.append(cloud)
    registration=o3d.pipelines.registration
    graph=registration.PoseGraph()
    for index in key_indices:
        graph.nodes.append(registration.PoseGraphNode(poses[index]))
    edge_records=[]
    # Odometry constraints come from this estimator, with explicit heuristic noise.
    odom_information=np.diag([1/.03**2]*3+[1/.05**2]*3)
    odom_rows=[]
    for i in range(len(key_indices)-1):
        relative=np.linalg.inv(poses[key_indices[i+1]])@poses[key_indices[i]]
        information=odom_information
        if args.observation_weighted_odometry:
            information, record=observation_odometry_information(trace,stamps[key_indices[i]],stamps[key_indices[i+1]],
                                                                 poses[key_indices[i+1],:3,:3])
            odom_rows.append(dict(source=i,target=i+1,**record))
        graph.edges.append(registration.PoseGraphEdge(i,i+1,relative,information,uncertain=False))
    if odom_rows:
        with (output/"odometry_weights.csv").open("w") as file:
            writer=csv.DictWriter(file,fieldnames=list(odom_rows[0]));writer.writeheader();writer.writerows(odom_rows)
    local_records=[]
    if reused_graph is not None:
        for name,destination in [('loop_candidates.csv',edge_records),('local_candidates.csv',local_records)]:
            if not (previous/name).exists():
                continue
            with (previous/name).open() as file:
                for row in csv.DictReader(file):
                    row['accepted']=row['accepted']=='True'
                    row['source']=int(row['source']);row['target']=int(row['target'])
                    destination.append(row)
        for edge in reused_graph.edges:
            if edge.uncertain:
                graph.edges.append(edge)
        print('Reused independently checked registration constraints; recomputing odometry weights',flush=True)
    if args.local_geometric_edges and reused_graph is None:
        for i, index in enumerate(key_indices):
            for span in sorted(set([1, max(2, round(3/args.keyframe_step)), max(3, round(8/args.keyframe_step))])):
                j=i-span
                if j < 0 or np.linalg.norm(poses[index,:3,3]-poses[key_indices[j],:3,3]) > 3:
                    continue
                initial=np.linalg.inv(poses[key_indices[j]])@poses[index]
                relative, record=local_registration(clouds[i],clouds[j],initial)
                local_records.append(dict(source=i,target=j,elapsed_s=float(stamps[index]-stamps[key_indices[j]]),**record))
                if record['accepted']:
                    info=plane_information(clouds[i],clouds[j],relative)
                    graph.edges.append(registration.PoseGraphEdge(i,j,relative,info,uncertain=True))
            if i%20==0:
                print(f"local geometry {i}/{len(key_indices)}, accepted {sum(r['accepted'] for r in local_records)}",flush=True)
    if local_records:
        with (output/"local_candidates.csv").open("w") as file:
            writer=csv.DictWriter(file,fieldnames=list(local_records[0]));writer.writeheader();writer.writerows(local_records)
    accepted=sum(row['accepted'] for row in edge_records)
    for i,index in (enumerate(key_indices) if reused_graph is None else []):
        candidates=[j for j in range(i) if stamps[index]-stamps[key_indices[j]]>30
                    and np.linalg.norm(poses[index,:3,3]-poses[key_indices[j],:3,3])<args.loop_search_radius]
        candidates.sort(key=lambda j:np.linalg.norm(poses[index,:3,3]-poses[key_indices[j],:3,3]))
        # Nearby recent keyframes should not crowd out earlier visits.
        diverse=[]
        epochs=set()
        for j in candidates:
            epoch=int((stamps[key_indices[j]]-stamps[0])/20)
            if epoch not in epochs:
                epochs.add(epoch)
                diverse.append(j)
        candidates=diverse+[j for j in candidates if j not in diverse]
        accepted_here=0
        for j in candidates[:args.max_candidates]:
            initial=np.linalg.inv(poses[key_indices[j]])@poses[index]
            fitting_cloud=clouds[i]
            training=None
            if args.validate_loops:
                cells=np.floor(np.asarray(clouds[i].points)/.4).astype(np.int64)
                training=(cells.sum(axis=1)%2)==0
                if training.sum()<100 or (~training).sum()<100:
                    continue
                fitting_cloud=clouds[i].select_by_index(np.flatnonzero(training))
            result=registration.registration_generalized_icp(fitting_cloud,clouds[j],.30,initial,
                         registration.TransformationEstimationForGeneralizedICP(),
                         registration.ICPConvergenceCriteria(max_iteration=35))
            reverse=registration.evaluate_registration(clouds[j],clouds[i],.10,np.linalg.inv(result.transformation))
            forward=registration.evaluate_registration(clouds[i],clouds[j],.10,result.transformation)
            correction=result.transformation@np.linalg.inv(initial)
            correction_translation=float(np.linalg.norm(correction[:3,3]))
            correction_angle=float(np.rad2deg(Rotation.from_matrix(correction[:3,:3]).magnitude()))
            information=registration.get_information_matrix_from_point_clouds(clouds[i],clouds[j],.10,result.transformation)
            eigenvalues=np.linalg.eigvalsh(information)
            condition=float(eigenvalues[0]/max(eigenvalues[-1],1e-12))
            eligible=(forward.fitness>args.loop_overlap and reverse.fitness>args.loop_overlap and forward.inlier_rmse<.04
                      and correction_translation<.65 and correction_angle<15 and condition>1e-4)
            validation={}
            if args.validate_loops:
                validation=dict(heldout_overlap=None, heldout_rmse_m=None,
                                plane_information_eigenvalue_ratio=None,
                                reverse_cycle_translation_m=None, reverse_cycle_rotation_deg=None,
                                validation_passed=False)
                if eligible:
                    validation=validate_loop(clouds[i],clouds[j],training,result,args.heldout_overlap)
                eligible=eligible and validation['validation_passed']
            edge_records.append(dict(source=i,target=j,elapsed_s=float(stamps[index]-stamps[key_indices[j]]),
                                      forward_overlap=forward.fitness,reverse_overlap=reverse.fitness,rmse_m=forward.inlier_rmse,
                                      correction_translation_m=correction_translation,correction_rotation_deg=correction_angle,
                                      information_eigenvalue_ratio=condition,accepted=bool(eligible),**validation))
            if eligible:
                if args.plane_information:
                    information=plane_information(clouds[i],clouds[j],result.transformation)
                graph.edges.append(registration.PoseGraphEdge(i,j,result.transformation,information,uncertain=True))
                accepted+=1
                accepted_here+=1
            if accepted_here>=2:
                break
        if i%20==0:
            print(f"keyframe {i}/{len(key_indices)}, accepted loop constraints {accepted}",flush=True)
    with (output/"loop_candidates.csv").open("w") as out:
        if edge_records:
            writer=csv.DictWriter(out,fieldnames=list(edge_records[0]));writer.writeheader();writer.writerows(edge_records)
    if not accepted:
        (output/"report.json").write_text(json.dumps(dict(status="no_validated_loop_constraints",keyframes=len(key_indices),submap_half_window_s=args.submap_half_window),indent=2)+"\n")
        raise RuntimeError("No loop constraints passed registration gates")
    o3d.io.write_pose_graph(str(output/"graph_before_optimization.json"),graph)
    registration.global_optimization(graph,registration.GlobalOptimizationLevenbergMarquardt(),
                                      registration.GlobalOptimizationConvergenceCriteria(),
                                      registration.GlobalOptimizationOption(max_correspondence_distance=.10,edge_prune_threshold=.25,
                                                                              preference_loop_closure=1.0,reference_node=0))
    o3d.io.write_pose_graph(str(output/"optimized_graph.json"),graph)
    offsets=np.array([graph.nodes[j].pose@np.linalg.inv(poses[index]) for j,index in enumerate(key_indices)])
    key_stamps=stamps[key_indices]
    clipped=np.clip(stamps,key_stamps[0],key_stamps[-1])
    rotations=Slerp(key_stamps,Rotation.from_matrix(offsets[:,:3,:3]))(clipped).as_matrix()
    translations=np.column_stack([np.interp(clipped,key_stamps,offsets[:,axis,3]) for axis in range(3)])
    corrected=poses.copy()
    corrected[:,:3,:3]=rotations@poses[:,:3,:3]
    corrected[:,:3,3]=np.einsum("nij,nj->ni",rotations,poses[:,:3,3])+translations
    with (output/"trajectory.csv").open("w") as out:
        writer=csv.writer(out);writer.writerow(["stamp","x","y","z","qx","qy","qz","qw"])
        quaternions=Rotation.from_matrix(corrected[:,:3,:3]).as_quat()
        writer.writerows(np.column_stack((stamps,corrected[:,:3,3],quaternions)))
    temporal=[]
    for window,filename in enumerate(cap["temporal_raw"]):
        with np.load(run/"capture"/filename) as chunk:
            cuts=np.r_[0,np.cumsum(chunk["lengths"])]
            chunk_points=chunk["points"]
            transformed=[]
            for j,stamp in enumerate(chunk["stamps"]):
                frame_index=int(np.searchsorted(stamps,stamp))
                points=chunk_points[cuts[j]:cuts[j+1]].copy()
                points[:,:3]=points[:,:3]@rotations[frame_index].T+translations[frame_index]
                transformed.append(points)
            points=np.concatenate(transformed)
            points=points[np.isfinite(points[:,:3]).all(axis=1)]
            sampled=voxel(points,.03)
            temporal.append(sampled)
            write_pcd(output/f"window_{window:03d}_3cm.pcd",sampled)
    merged=voxel(np.concatenate(temporal),.03)
    write_pcd(output/"optimized_3cm.pcd",merged)
    local_pairs={(r['source'],r['target']) for r in local_records if r['accepted']}
    report=dict(status="optimized_quality_not_verified",source_run=str(run),keyframes=len(key_indices),
                keyframe_step_s=args.keyframe_step,
                loop_search_radius_m=args.loop_search_radius,
                loop_overlap_threshold=args.loop_overlap,
                heldout_overlap_threshold=args.heldout_overlap,
                local_geometric_candidates=len(local_records),
                accepted_local_geometric_candidates=sum(r['accepted'] for r in local_records),
                loop_information="Point-to-plane, effective samples 32, sigma .02m; heuristic confidence floor" if args.plane_information else "Open3D point-to-point information",
                reused_registration_constraints=str(previous) if reused_graph is not None else None,
                submap_half_window_s=args.submap_half_window,
                max_loop_candidates_per_keyframe=args.max_candidates,
                loop_validation="heldout spatial blocks, point-to-plane information and reverse convergence" if args.validate_loops else "bidirectional overlap and heuristic registration gates",
                input_frames=len(frames),accepted_loop_candidates=accepted,points=len(merged),
                retained_loop_constraints=sum(edge.uncertain and (edge.source_node_id,edge.target_node_id) not in local_pairs for edge in graph.edges),
                retained_local_constraints=sum(edge.uncertain and (edge.source_node_id,edge.target_node_id) in local_pairs for edge in graph.edges),
                map=str(output/"optimized_3cm.pcd"),
                independent_algorithm_count_change=0,no_foreign_trajectory=True,
                odometry_information=("Heuristic noise integrated from matched count, range, yaw information and missing updates; not measured covariance" if args.observation_weighted_odometry else "Heuristic: rotation sigma .03rad and translation sigma .05m per keyframe; not measured covariance"),
                correction_interpolation="SLERP of global rotation correction and linear translation correction between optimized keyframes; retained as a separate experimental output",
                limits="Registration gates are screening tests. Rejected/pruned edges and surface consistency must be audited before acceptance.")
    (output/"report.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))


if __name__=="__main__":
    main()
