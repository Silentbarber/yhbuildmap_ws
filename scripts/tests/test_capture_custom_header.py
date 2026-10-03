#!/usr/bin/env python3
"""Converted-cloud telemetry must read Header without decoding all points."""
import io
import pathlib
import struct
import sys
import unittest
from types import SimpleNamespace

import rospy
from livox_ros_driver.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from capture_lio_run import Capture, cloud_fields, cloud_point_time_ms


class CaptureHeaderTest(unittest.TestCase):
    def test_custom_payload_header_matches_typed_message(self):
        header = Header(seq=17, stamp=rospy.Time(1790929422, 254159200), frame_id='livox_frame')
        message = CustomMsg(header=header, point_num=2, points=[CustomPoint(), CustomPoint()])
        stream = io.BytesIO()
        message.serialize(stream)
        capture = Capture.__new__(Capture)
        recorded = []
        capture.count = lambda name, msg: recorded.append((name, msg.header))
        capture.converted(SimpleNamespace(_buff=stream.getvalue()))
        self.assertEqual(recorded, [('converted_lidar', header)])

    def test_point_time_reader_accepts_pcl_curvature_field(self):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='curvature', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        data = b''.join(struct.pack('<fffff', 1., 2., 3., 4., t) for t in (0., 1.25, 2.5))
        message = SimpleNamespace(fields=fields, point_step=20, row_step=60, width=3, height=1,
                                  is_bigendian=False, data=data)
        parsed = cloud_fields(message)
        values, field = cloud_point_time_ms(message)
        self.assertEqual(parsed.dtype.names, ('x', 'y', 'z', 'intensity', 'curvature'))
        self.assertEqual(field, 'curvature')
        self.assertEqual(values.tolist(), [0., 1.25, 2.5])

    def test_point_time_reader_returns_none_for_xyzi_only(self):
        fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                  for name, offset in [('x', 0), ('y', 4), ('z', 8), ('intensity', 12)]]
        message = SimpleNamespace(fields=fields, point_step=16, row_step=32, width=2, height=1,
                                  is_bigendian=False, data=b'\0'*32)
        values, field = cloud_point_time_ms(message)
        self.assertIsNone(values)
        self.assertIsNone(field)


if __name__ == '__main__':
    unittest.main()
