#!/usr/bin/env python3
"""Audit absolute per-return timing in a Livox MID360 PointCloud2 bag."""
import argparse
import json
import pathlib
import struct
import time

import numpy as np
import rosbag


def field_layout(msg):
    fields = {field.name: field for field in msg.fields}
    if "timestamp" not in fields:
        raise ValueError("PointCloud2 has no timestamp field")
    field = fields["timestamp"]
    if field.datatype != 8 or field.count != 1:
        raise ValueError("Expected one FLOAT64 timestamp field")
    return field.offset, msg.point_step


def read_timestamps(msg, offset, step):
    return np.fromiter((struct.unpack_from("<d", msg.data, index + offset)[0]
                        for index in range(0, msg.width * msg.height * step, step)),
                       dtype=np.float64, count=msg.width * msg.height)


def audit_bag(path, topic):
    started = time.monotonic()
    frame_count = point_count = 0
    fields = None
    frame_durations, point_offsets, invalid = [], [], 0
    first_header = last_header = None
    with rosbag.Bag(str(path)) as bag:
        for _, message, bag_time in bag.read_messages(topics=[topic]):
            if fields is None:
                offset, step = field_layout(message)
                fields = [(field.name, field.offset, field.datatype, field.count) for field in message.fields]
            elif (offset, step) != field_layout(message):
                raise ValueError("PointCloud2 layout changed during bag")
            stamp = message.header.stamp.to_sec()
            values = read_timestamps(message, offset, step)
            delta = values * 1e-9 - stamp
            valid = np.isfinite(delta) & (delta >= -.001) & (delta <= .25)
            invalid += int((~valid).sum())
            if valid.any():
                point_offsets.extend(delta[valid].tolist())
                frame_durations.append(float(delta[valid].max()-delta[valid].min()))
            frame_count += 1
            point_count += len(values)
            first_header = stamp if first_header is None else first_header
            last_header = stamp
    if not frame_count or not point_offsets:
        raise ValueError("No valid timestamped point cloud frames")
    offsets = np.asarray(point_offsets)
    durations = np.asarray(frame_durations)
    result = dict(bag=str(path.resolve()), topic=topic, frames=frame_count, points=point_count,
        fields=fields, invalid_timestamps=invalid, first_header_stamp=first_header,
        last_header_stamp=last_header, point_offset_s_quantiles=np.percentile(offsets, [0, 1, 50, 99, 100]).tolist(),
        scan_duration_s_quantiles=np.percentile(durations, [0, 1, 50, 99, 100]).tolist(),
        header_interval_s= float((last_header-first_header)/(frame_count-1)) if frame_count > 1 else None,
        wall_seconds=time.monotonic()-started,
        interpretation='Timestamp values are absolute nanoseconds from the Livox PointCloud2 field. '
                       'Offsets are relative to each message header. This proves source timing availability, '
                       'not estimator pose accuracy or dynamic-object labels.')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=pathlib.Path)
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--output", type=pathlib.Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a fresh output path")
    result = audit_bag(args.bag, args.topic)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
