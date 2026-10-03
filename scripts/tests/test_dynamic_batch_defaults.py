#!/usr/bin/env python3
"""Batch CLI plumbing only; filter/audit subprocesses are mocked."""
import hashlib
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import run_dynamic_filtering as batch


class BatchDefaultsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temp.name)
        (self.root/'config').mkdir()
        backend = self.root/'backend'
        backend.mkdir()
        record = dict(id='215247', bag='synthetic.bag', frontend='frontend', backend='backend')
        for name, key in [('optimized_3cm.pcd', 'map_sha256'), ('trajectory.csv', 'trajectory_sha256')]:
            (backend/name).write_bytes(b'CLI fixture only')
            record[key] = hashlib.sha256((backend/name).read_bytes()).hexdigest()
        (self.root/'config/dynamic_filtering_recordings.json').write_text(json.dumps({'recordings': [record]}))
        (self.root/'cache/215247').mkdir(parents=True)
        (self.root/'cache/215247/report.json').write_text('{}')
        self.commands = []

    def tearDown(self):
        self.temp.cleanup()

    def process(self, command, **kwargs):
        self.commands.append(command)
        output = self.root/'output/215247'
        if pathlib.Path(command[1]).name == 'filter_dynamic_map.py':
            output.mkdir(parents=True)
            (output/'report.json').write_text('{"fixture_only": true}')
        else:
            (output/'audit').mkdir()
            (output/'audit/report.json').write_text('{"fixture_only": true}')
        return subprocess.CompletedProcess(command, 0)

    def invoke(self, *extra):
        arguments = ['batch', '--data-root', str(self.root), '--output-root', str(self.root/'output'),
                     '--reuse-root', str(self.root/'cache'), *extra]
        with patch.object(batch, 'ROOT', self.root), patch.object(sys, 'argv', arguments), \
                patch.object(batch.subprocess, 'run', side_effect=self.process):
            batch.main()
        return self.commands[0]

    def test_default_guard_and_direct_cache_layout(self):
        command = self.invoke()
        self.assertIn('--require-angular-support', command)
        self.assertEqual(command[command.index('--reuse-evidence')+1], str(self.root/'cache/215247'))

    def test_legacy_mode_and_nested_cache_layout(self):
        (self.root/'cache/215247/report.json').unlink()
        (self.root/'cache/215247/visibility_v2').mkdir()
        command = self.invoke('--allow-angular-extrapolation')
        self.assertNotIn('--require-angular-support', command)
        self.assertEqual(command[command.index('--reuse-evidence')+1], str(self.root/'cache/215247/visibility_v2'))

    def test_point_time_groups_reach_filter(self):
        command = self.invoke('--point-time-groups-ms', '5')
        self.assertEqual(command[command.index('--point-time-groups-ms')+1], '5.0')


if __name__ == '__main__':
    unittest.main()
