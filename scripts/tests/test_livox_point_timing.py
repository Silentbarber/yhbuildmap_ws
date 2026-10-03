#!/usr/bin/env python3
import pathlib
import struct
import sys
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from audit_livox_point_timing import field_layout, read_timestamps


class LivoxPointTimingTest(unittest.TestCase):
    def message(self, values):
        fields = [SimpleNamespace(name='timestamp', offset=18, datatype=8, count=1)]
        step = 26
        data = b''.join(bytes(18)+struct.pack('<d', value)+bytes(0) for value in values)
        return SimpleNamespace(fields=fields, point_step=step, width=len(values), height=1, data=data)

    def test_absolute_nanoseconds_are_read_without_rounding(self):
        message = self.message([1.7908627678479677e18, 1.7908627678479726e18])
        offset, step = field_layout(message)
        parsed = read_timestamps(message, offset, step)
        np.testing.assert_allclose(parsed, [1.7908627678479677e18, 1.7908627678479726e18], rtol=0, atol=1)

    def test_layout_requires_float64_timestamp(self):
        message = self.message([1.])
        message.fields[0].datatype = 7
        with self.assertRaisesRegex(ValueError, 'FLOAT64'):
            field_layout(message)

    def test_missing_timestamp_is_rejected(self):
        message = self.message([1.])
        message.fields[0].name = 'time'
        with self.assertRaisesRegex(ValueError, 'no timestamp'):
            field_layout(message)


if __name__ == '__main__':
    unittest.main()
