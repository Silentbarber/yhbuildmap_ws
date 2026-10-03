#!/usr/bin/env python3
"""CLI integration only; external ROS/backend/filter processes are mocked."""
import contextlib
import io
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import run_faster_robust_mapping as pipeline


class PipelineFlagsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temp.name)
        self.bag = self.root/'synthetic.bag'
        self.bag.write_bytes(b'CLI fixture only')
        self.output = self.root/'output'
        self.commands = []

    def tearDown(self):
        self.temp.cleanup()

    def run_process(self, command, **kwargs):
        self.commands.append(command)
        name = pathlib.Path(command[1]).name
        if name == 'run_independent_lio.py':
            (self.output/'frontend').mkdir()
        elif name == 'evaluate_independent_run.py':
            (self.output/'frontend/evaluation.json').write_text(json.dumps({'failures': []}))
        elif name == 'refine_independent_pose_graph.py':
            (self.output/'backend').mkdir()
            (self.output/'backend/report.json').write_text(json.dumps({'status': 'fixture_completed'}))
        elif name == 'filter_dynamic_map.py':
            (self.output/'dynamic').mkdir()
            (self.output/'dynamic/report.json').write_text(json.dumps({'status': 'fixture_only'}))
        elif name == 'audit_dynamic_filter.py':
            (self.output/'dynamic/audit').mkdir()
            (self.output/'dynamic/audit/report.json').write_text(json.dumps({'fixture_only': True}))
        return subprocess.CompletedProcess(command, 0)

    def invoke(self, *extra):
        command = ['pipeline', '--bag', str(self.bag), '--calibration', 'raw', '--output-dir', str(self.output), *extra]
        with patch.object(pipeline, 'ROOT', self.root), patch.object(sys, 'argv', command), \
                patch.object(pipeline.subprocess, 'run', side_effect=self.run_process), \
                contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            pipeline.main()

    def test_baseline_does_not_run_filter(self):
        self.invoke()
        self.assertFalse(any(pathlib.Path(command[1]).name == 'filter_dynamic_map.py' for command in self.commands))
        report = json.loads((self.output/'pipeline.json').read_text())
        self.assertIsNone(report['dynamic_filter'])
        self.assertEqual(report['map'], report['unfiltered_map'])

    def test_angular_guard_and_conservative_threshold_reach_filter(self):
        self.invoke('--dynamic-filter', '--dynamic-angular-support', '--dynamic-max-hit-bins', '1')
        command = next(command for command in self.commands if pathlib.Path(command[1]).name == 'filter_dynamic_map.py')
        self.assertIn('--require-angular-support', command)
        self.assertEqual(command[command.index('--max-hit-bins')+1], '1')
        report = json.loads((self.output/'pipeline.json').read_text())
        self.assertEqual(report['map'], str(self.output/'dynamic/filtered_3cm.pcd'))
        self.assertEqual(report['unfiltered_map'], str(self.output/'backend/optimized_3cm.pcd'))

    def test_dynamic_filter_uses_guard_without_extra_flag(self):
        self.invoke('--dynamic-filter')
        command = next(command for command in self.commands if pathlib.Path(command[1]).name == 'filter_dynamic_map.py')
        self.assertIn('--require-angular-support', command)

    def test_legacy_extrapolation_requires_explicit_request(self):
        self.invoke('--dynamic-filter', '--dynamic-allow-angular-extrapolation')
        command = next(command for command in self.commands if pathlib.Path(command[1]).name == 'filter_dynamic_map.py')
        self.assertNotIn('--require-angular-support', command)

    def test_point_time_groups_reach_filter(self):
        self.invoke('--dynamic-filter', '--dynamic-point-time-groups-ms', '5')
        command = next(command for command in self.commands if pathlib.Path(command[1]).name == 'filter_dynamic_map.py')
        self.assertEqual(command[command.index('--point-time-groups-ms')+1], '5.0')

    def test_dynamic_setting_without_filter_fails_before_output(self):
        for arguments in [('--dynamic-angular-support',), ('--dynamic-max-hit-bins', '3'),
                          ('--dynamic-allow-angular-extrapolation',),
                          ('--dynamic-point-time-groups-ms', '5')]:
            with self.subTest(arguments=arguments), self.assertRaises(SystemExit):
                self.invoke(*arguments)
        self.assertFalse(self.output.exists())
        self.assertFalse(self.commands)

    def test_repeatedly_supported_point_threshold_cannot_be_requested(self):
        with self.assertRaises(SystemExit):
            self.invoke('--dynamic-filter', '--dynamic-max-hit-bins', '4')
        self.assertFalse(self.output.exists())


if __name__ == '__main__':
    unittest.main()
