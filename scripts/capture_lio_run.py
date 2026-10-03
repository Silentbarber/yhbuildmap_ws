#!/usr/bin/env python3
"""Capture one estimator's own output and preserve temporal geometry evidence."""
import argparse
import csv
import json
import pathlib
import threading
import time
from types import SimpleNamespace

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header

TYPES = {PointField.INT8: "i1", PointField.UINT8: "u1", PointField.INT16: "i2",
         PointField.UINT16: "u2", PointField.INT32: "i4", PointField.UINT32: "u4",
         PointField.FLOAT32: "f4", PointField.FLOAT64: "f8"}


def cloud_fields(msg):
    endian = ">" if msg.is_bigendian else "<"
    dtype = np.dtype({"names": [f.name for f in msg.fields],
                      "formats": [(endian + TYPES[f.datatype], (f.count,)) if f.count > 1
                                  else endian + TYPES[f.datatype] for f in msg.fields],
                      "offsets": [f.offset for f in msg.fields], "itemsize": msg.point_step})
    if msg.row_step == msg.width * msg.point_step:
        data = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    else:
        data = np.concatenate([np.frombuffer(msg.data, dtype=dtype, count=msg.width,
                                             offset=i * msg.row_step) for i in range(msg.height)])
    return data


def cloud_array(msg):
    data = cloud_fields(msg)
    points = np.zeros((len(data), 4), dtype=np.float32)
    for i, name in enumerate(("x", "y", "z", "intensity")):
        if name in data.dtype.names:
            points[:, i] = data[name]
    return points


def cloud_point_time_ms(msg):
    """Return the estimator's per-point time field when the output preserves it."""
    data = cloud_fields(msg)
    for name in ("curvature", "time", "timestamp"):
        if name in data.dtype.names:
            values = np.asarray(data[name], dtype=np.float32).reshape(-1)
            if len(values) == len(data):
                return values, name
    return None, None


class Capture:
    def __init__(self, args):
        self.args = args
        self.root = args.output
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / "temporal_raw").mkdir()
        self.lock = threading.RLock()
        self.counts = dict(input_lidar=0, input_imu=0, converted_lidar=0, si_imu=0, output_cloud=0, odometry=0)
        self.first = {}
        self.last = {}
        self.parts = []
        self.part_stamps = []
        self.part_lengths = []
        self.part_times = []
        self.point_time_fields = set()
        self.window_start = None
        self.window = 0
        self.frames = set()
        self.files = []
        self.errors = []
        self.finished = False
        self.last_progress_wall = 0.0
        self.trajectory = (self.root / "trajectory.csv").open("w")
        self.traj_writer = csv.writer(self.trajectory)
        self.traj_writer.writerow(("stamp", "x", "y", "z", "qx", "qy", "qz", "qw"))
        self.frame_file = (self.root / "frames.csv").open("w")
        self.frame_writer = csv.writer(self.frame_file)
        self.frame_writer.writerow(("stamp", "frame_id", "points", "finite_points", "max_abs_coordinate"))
        self.subs = [rospy.Subscriber("/livox/lidar", PointCloud2, lambda m: self.count("input_lidar", m), queue_size=500, buff_size=2**25),
                     rospy.Subscriber("/livox/imu", Imu, lambda m: self.count("input_imu", m), queue_size=50000),
                     rospy.Subscriber("/livox/lidar_custom", rospy.AnyMsg, self.converted, queue_size=500, buff_size=2**25),
                     rospy.Subscriber("/mid360/imu_si", Imu, lambda m: self.count("si_imu", m), queue_size=50000),
                     rospy.Subscriber(args.cloud_topic, PointCloud2, self.cloud, queue_size=500, buff_size=2**26),
                     rospy.Subscriber(args.odom_topic, Odometry, self.odom, queue_size=10000)]
        rospy.on_shutdown(self.finish)
        # Keep the final callback counts visible while the estimator drains.
        self.progress_timer = rospy.Timer(rospy.Duration(0.5), lambda event: self.write_progress(), reset=True)
        (self.root / "capture_ready.json").write_text(json.dumps({"cloud": args.cloud_topic, "odom": args.odom_topic}))

    def count(self, name, msg):
        with self.lock:
            stamp = msg.header.stamp.to_sec()
            self.counts[name] += 1
            self.first.setdefault(name, stamp)
            self.last[name] = stamp
            if name == "input_lidar":
                data = cloud_fields(msg)
                if "timestamp" in data.dtype.names:
                    delta = (data["timestamp"] - float(msg.header.stamp.to_nsec())) * 1e-9
                    valid = np.isfinite(delta) & (delta >= 0) & (delta <= .2)
                    if valid.any():
                        scan_end = stamp + float(delta[valid].max())
                        self.first.setdefault("input_lidar_scan_end", scan_end)
                        self.last["input_lidar_scan_end"] = scan_end
            wall = time.monotonic()
            if not self.finished and wall - self.last_progress_wall > 0.5:
                self.last_progress_wall = wall
                self.write_progress()

    def converted(self, msg):
        # CustomMsg starts with Header; telemetry does not need thousands of point objects.
        header = Header()
        header.deserialize(msg._buff)
        self.count("converted_lidar", SimpleNamespace(header=header))

    def write_progress(self):
        with self.lock:
            if self.finished:
                return
            temporary = self.root / "progress.tmp"
            temporary.write_text(json.dumps(dict(counts=self.counts, first_stamps=self.first,
                                                 last_stamps=self.last)))
            temporary.replace(self.root / "progress.json")

    def cloud(self, msg):
        try:
            points = cloud_array(msg)
            point_times, point_time_field = cloud_point_time_ms(msg)
            with self.lock:
                if self.finished:
                    return
                self.count("output_cloud", msg)
                stamp = msg.header.stamp.to_sec()
                self.frames.add(msg.header.frame_id)
                if self.window_start is None:
                    self.window_start = stamp
                if stamp - self.window_start >= 20.0:
                    self.flush()
                    self.window_start = stamp
                finite = np.isfinite(points[:, :3]).all(axis=1)
                maximum = float(np.abs(points[finite, :3]).max()) if finite.any() else None
                self.frame_writer.writerow((stamp, msg.header.frame_id, len(points), int(finite.sum()), maximum))
                self.frame_file.flush()
                self.parts.append(points)
                self.part_stamps.append(stamp)
                self.part_lengths.append(len(points))
                self.part_times.append(point_times)
                if point_times is not None:
                    self.point_time_fields.add(point_time_field)
                elif self.point_time_fields:
                    self.errors.append("point-time field disappeared after it was present")
        except Exception as error:
            with self.lock:
                self.errors.append(str(error))
            rospy.logerr("cloud capture failed: %s", error)

    def odom(self, msg):
        with self.lock:
            if self.finished:
                return
            self.count("odometry", msg)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.traj_writer.writerow((msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            self.trajectory.flush()

    def flush(self):
        if not self.parts:
            return
        path = self.root / "temporal_raw" / f"window_{self.window:03d}.npz"
        payload = dict(points=np.concatenate(self.parts), stamps=np.array(self.part_stamps),
                       lengths=np.array(self.part_lengths, dtype=np.int64))
        if (self.part_times and len(self.part_times) == len(self.parts)
                and all(values is not None for values in self.part_times)):
            payload["point_time_ms"] = np.concatenate(self.part_times).astype(np.float32)
        np.savez(path, **payload)
        self.files.append(str(path.relative_to(self.root)))
        self.parts.clear()
        self.part_stamps.clear()
        self.part_lengths.clear()
        self.part_times.clear()
        self.window += 1

    def finish(self):
        with self.lock:
            if self.finished:
                return
            self.write_progress()
            self.finished = True
            self.progress_timer.shutdown()
            self.flush()
            self.trajectory.close()
            self.frame_file.close()
            report = dict(counts=self.counts, first_stamps=self.first, last_stamps=self.last,
                          cloud_frames=sorted(self.frames), temporal_raw=self.files,
                          point_time_fields=sorted(self.point_time_fields),
                          point_time_preserved=bool(self.point_time_fields),
                          errors=self.errors, quality="pending_geometry_validation")
            (self.root / "capture.json").write_text(json.dumps(report, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--cloud-topic", required=True)
    parser.add_argument("--odom-topic", required=True)
    args = parser.parse_args()
    rospy.init_node("independent_capture", disable_signals=False)
    capture = Capture(args)
    rospy.spin()
    capture.finish()


if __name__ == "__main__":
    main()
