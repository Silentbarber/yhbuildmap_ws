#!/usr/bin/env python3
"""Converted-cloud telemetry must read Header without decoding all points."""
import io
import pathlib
import sys
import unittest
from types import SimpleNamespace

import rospy
from livox_ros_driver.msg import CustomMsg, CustomPoint
from std_msgs.msg import Header

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from capture_lio_run import Capture


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


if __name__ == '__main__':
    unittest.main()
