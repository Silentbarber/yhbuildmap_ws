#!/usr/bin/env python3
"""Replay a complete bag into one isolated ROS1 estimator and record evidence."""
import argparse
import datetime
import hashlib
import json
import os
import pathlib
import shlex
import signal
import socket
import subprocess
import time
import xmlrpc.client

ROOT = pathlib.Path(__file__).resolve().parents[1]


def environment(workspace):
    command = "source /opt/ros/noetic/setup.bash\nsource " + shlex.quote(str(workspace / "devel/setup.bash")) + "\nenv -0"
    result = subprocess.check_output(["bash", "-c", command])
    env = dict(item.decode().split("=", 1) for item in result.split(b"\0") if b"=" in item)
    existing = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = str(workspace / "devel/lib") + ":" + existing
    return env


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=15)


def output_covered(progress, expected, bag_end, tolerance, state_time="scan_start"):
    last = progress.get("last_stamps", {})
    counts = progress.get("counts", {})
    if state_time == "scan_end":
        target = last.get("input_lidar_scan_end")
        if target is None:
            return False
    else:
        target = last.get("input_lidar", bag_end)
    return (last.get("output_cloud", 0) >= target - tolerance and
            last.get("odometry", 0) >= target - tolerance and
            counts.get("input_lidar") == expected.get("/livox/lidar") and
            counts.get("input_imu") == expected.get("/livox/imu") and
            counts.get("output_cloud", 0) > 0 and counts.get("odometry", 0) > 0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("id")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--launch-arg", action="append", default=[])
    parser.add_argument("--drain-timeout", type=float, default=120.0)
    parser.add_argument("--bag", type=pathlib.Path, default=ROOT / "bag/test_1301_2026-10-01-21-52-47.bag")
    parser.add_argument("--output-dir", type=pathlib.Path, help="Fresh result directory for a separate dataset run")
    args = parser.parse_args()
    args.bag = args.bag.resolve()
    if not 0 < args.rate <= 2:
        parser.error("replay rate must be in (0, 2]")
    if not 12 <= args.drain_timeout <= 600:
        parser.error("drain timeout must be in [12, 600] wall seconds")
    integration = json.loads((ROOT / "config/ros1_integrations.json").read_text())[args.id]
    slug = integration["repo"].replace("/", "__")
    workspace = ROOT / "algorithms" / slug
    env = environment(workspace)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output_dir.resolve() if args.output_dir else ROOT / "results/independent_runs" / f"{args.id}_{slug}" / stamp
    try:
        output.relative_to(ROOT)
    except ValueError:
        parser.error("output directory must be inside this workspace")
    output.mkdir(parents=True)
    (output / "Log").mkdir()
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    env.update(ROS_MASTER_URI=f"http://127.0.0.1:{port}", ROS_HOSTNAME="127.0.0.1", ROS_HOME=str(output / "ros_home"))
    # rosbag YAML contains ROS scalar types; read it with its native YAML parser.
    import yaml
    bag_info = yaml.safe_load(subprocess.check_output(["rosbag", "info", "--yaml", str(args.bag)], env=env))
    expected = {t["topic"]: t["messages"] for t in bag_info["topics"]}
    record = {"id":args.id, "repo":integration["repo"], "upstream":json.loads((workspace / "project.json").read_text()),
              "bag":str(args.bag.resolve()), "bag_size":args.bag.stat().st_size, "bag_info":bag_info,
              "expected_messages":expected, "rate":args.rate, "master_uri":env["ROS_MASTER_URI"],
              "output":str(output.relative_to(ROOT)), "working_directory":str(output), "integration":integration, "launch_arguments":args.launch_arg,
              "status":"starting", "quality":"not_verified"}
    code_hash = hashlib.sha256()
    for path in sorted((workspace / "src").rglob("*")):
        if path.is_file() and not path.is_symlink() and path.suffix in (".cpp", ".cc", ".c", ".h", ".hpp", ".xml", ".txt", ".yaml", ".launch", ".json", ".cmake", ".msg", ".srv"):
            code_hash.update(str(path.relative_to(workspace)).encode())
            code_hash.update(path.read_bytes())
    record["adaptation_sha256"] = code_hash.hexdigest()
    record["adaptation_hash_scope"] = "workspace/src regular code/config files, including ROS1 wrapper; shared adapter hashed separately"
    record["shared_adapter_sha256"] = {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted((ROOT / "src/livox_pointcloud2_to_custommsg").rglob("*"))
        if path.is_file() and path.suffix in (".cpp", ".h", ".xml", ".txt")
    }
    metadata = output / "run.json"
    metadata.write_text(json.dumps(record, indent=2) + "\n")
    processes, logs = [], []

    def start(name, command):
        log = (output / f"{name}.log").open("w")
        logs.append(log)
        process = subprocess.Popen(command, env=env, cwd=output, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    started = time.monotonic()
    try:
        master = start("roscore", ["roscore", "-p", str(port)])
        server = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
        deadline = time.monotonic() + 20
        while True:
            try:
                if server.getPid("/independent_runner")[0] == 1:
                    break
            except OSError:
                pass
            if time.monotonic() > deadline:
                raise RuntimeError("ROS master startup timed out")
            time.sleep(0.2)
        launch = start("mapping", ["roslaunch", integration["package"], integration["launch"], *args.launch_arg])
        capture = start("capture", ["python3", str(ROOT / "scripts/capture_lio_run.py"), "--output", str(output / "capture"),
                                    "--cloud-topic", integration["cloud_topic"], "--odom-topic", integration["odom_topic"]])
        deadline = time.monotonic() + 30
        while True:
            published = dict(server.getPublishedTopics("/independent_runner", "")[2])
            if (output / "capture/capture_ready.json").exists() and all(t in published for t in (integration["cloud_topic"], integration["odom_topic"])):
                break
            if capture.poll() is not None or launch.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("Mapping/capture readiness failed; inspect logs")
            time.sleep(0.2)
        time.sleep(1)
        subprocess.run(["rosparam", "dump", str(output / "ros_parameters.yaml")], env=env, check=True, timeout=10)
        record["status"] = "replaying"
        metadata.write_text(json.dumps(record, indent=2) + "\n")
        print(f"replaying {args.id} {slug} on port {port}: {output}", flush=True)
        play = start("bag_play", ["rosbag", "play", "--clock", "--quiet", "--delay=1", "--rate", str(args.rate), str(args.bag)])
        play_deadline = time.monotonic() + bag_info["duration"] / args.rate + 120
        while play.poll() is None:
            if launch.poll() is not None:
                raise RuntimeError("Required estimator launch stopped during playback; inspect mapping.log")
            if capture.poll() is not None:
                raise RuntimeError("Capture stopped during playback; inspect capture.log")
            if time.monotonic() > play_deadline:
                raise RuntimeError("Bag playback timed out")
            time.sleep(0.2)
        record["play_exit_code"] = play.returncode
        if play.returncode:
            raise RuntimeError("rosbag play failed")
        # Continue clock only to drain callbacks. No synthetic sensor data is added.
        clock_code = "import rospy,time; from rosgraph_msgs.msg import Clock; rospy.init_node('drain_clock'); p=rospy.Publisher('/clock',Clock,queue_size=20); base=" + str(bag_info["end"]) + "; start=time.monotonic(); time.sleep(.3);\nwhile not rospy.is_shutdown() and time.monotonic()-start<" + str(args.drain_timeout+1) + ":\n p.publish(Clock(rospy.Time.from_sec(base+time.monotonic()-start))); time.sleep(.01)"
        drain = start("drain_clock", ["python3", "-c", clock_code])
        drain_started = time.monotonic()
        caught_up = False
        tolerance = integration.get("end_output_tolerance_s", 1.0)
        while time.monotonic() - drain_started < args.drain_timeout:
            progress_path = output / "capture/progress.json"
            if progress_path.exists():
                progress = json.loads(progress_path.read_text())
                caught_up = output_covered(progress, expected, bag_info["end"], tolerance,
                                           integration.get("state_time", "scan_start"))
            if (caught_up and time.monotonic()-drain_started >= 12) or launch.poll() is not None:
                break
            time.sleep(0.5)
        record["drain_wall_seconds"] = time.monotonic()-drain_started
        record["output_caught_up_before_save"] = caught_up
        if integration.get("save_service"):
            saved_map = output / integration.get("primary_map_relative", "upstream_global_keyframes.pcd")
            request = f"{{save_path: '{saved_map}', resolution: 0.0}}"
            if integration.get("save_request_type") == "lio_sam_save_map":
                save_target = output / integration.get("save_directory_relative", integration.get("primary_map_relative", "map"))
                request = f"{{destination: '{save_target}', resolution: 0.0}}"
            if "save_service_request" in integration:
                request = yaml.safe_dump(integration["save_service_request"], default_flow_style=True).strip()
            if integration.get("save_service_type") == "trigger":
                save_directory = output / integration["save_directory_relative"]
                subprocess.run(["rosparam", "set", integration["save_path_parameter"], str(save_directory)], env=env, check=True, timeout=10)
                request = "{}"
            service = subprocess.run(["rosservice", "call", integration["save_service"], request], env=env,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=integration.get("save_timeout_s", 30), text=True)
            (output / "save_map.log").write_text(service.stdout)
            record["save_service_exit_code"] = service.returncode
            response = yaml.safe_load(service.stdout) if service.returncode == 0 else {}
            if service.returncode or (isinstance(response, dict) and response.get("success") is False) or not saved_map.exists():
                raise RuntimeError("Upstream map-save service failed")
        stop(capture)
        record["capture_exit_code"] = capture.returncode
        if not (output / "capture/capture.json").exists():
            raise RuntimeError("Capture did not flush evidence")
        final_capture = json.loads((output / "capture/capture.json").read_text())
        caught_up = output_covered(final_capture, expected, bag_info["end"], tolerance,
                                   integration.get("state_time", "scan_start"))
        record["output_caught_up_before_shutdown"] = caught_up
        published = dict(server.getPublishedTopics("/independent_runner", "")[2])
        record["mapping_launch_exit_code_before_shutdown"] = launch.poll()
        record["estimator_output_topics_present_at_end"] = all(
            t in published for t in (integration["cloud_topic"], integration["odom_topic"]))
        if launch.poll() is not None or not record["estimator_output_topics_present_at_end"]:
            raise RuntimeError("Estimator stopped during playback; complete bag playback is not a successful estimator run")
        if not caught_up:
            raise RuntimeError("Estimator output did not reach the end of the bag before drain timeout; inspect capture/progress.json")
        record["status"] = "full_bag_replay_finished_quality_pending"
        print(f"complete replay {args.id}; geometry validation pending: {output}", flush=True)
    except KeyboardInterrupt:
        record.update(status="run_failed", error="Replay runner interrupted before completion")
        print(f"interrupted {args.id}: {output}", flush=True)
    except Exception as error:
        record.update(status="run_failed", error=str(error))
        print(f"failed {args.id}: {error}; {output}", flush=True)
    finally:
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()
        record["wall_seconds"] = time.monotonic() - started
        metadata.write_text(json.dumps(record, indent=2, default=str) + "\n")
    if record["status"] == "run_failed":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
