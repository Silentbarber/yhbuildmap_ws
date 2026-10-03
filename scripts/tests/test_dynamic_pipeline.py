#!/usr/bin/env python3
"""Synthetic CLI checks, separate from real-recording quality validation."""
import csv
import hashlib
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from evaluate_independent_run import write_pcd
from filter_dynamic_map import read_map


SCRIPT=pathlib.Path(__file__).resolve().parents[1]/'filter_dynamic_map.py'


class DynamicPipelineTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=pathlib.Path(self.temp.name)
        self.frontend=self.root/'frontend'
        self.backend=self.root/'backend'
        (self.frontend/'capture').mkdir(parents=True)
        self.backend.mkdir()
        self.stamps=np.array([10.,12,14,16,18])
        for path in [self.frontend/'capture/trajectory.csv',self.backend/'trajectory.csv']:
            with path.open('w') as file:
                writer=csv.writer(file)
                writer.writerow(['stamp','x','y','z','qx','qy','qz','qw'])
                writer.writerows([[stamp,0,0,0,0,0,0,1] for stamp in self.stamps])
        (self.frontend/'evaluation.json').write_text(json.dumps({'failures':[]}))
        (self.frontend/'ros_parameters.yaml').write_text('mapping:\n  extrinsic_est_en: false\n  extrinsic_T: [0,0,0]\n')
        (self.backend/'report.json').write_text(json.dumps(dict(source_run=str(self.frontend),input_frames=5)))
        directions=np.array([[1.,0,0],[1,.001,0],[1,0,.001]])
        directions/=np.linalg.norm(directions,axis=1)[:,None]
        wall=np.column_stack((directions*4,np.array([10.,20,30])))
        ghost=np.array([[2.,0,0,40.]])
        unknown=np.array([[0.,5,0,50.]])
        self.points=np.vstack((ghost,wall,unknown)).astype(np.float32)
        write_pcd(self.backend/'optimized_3cm.pcd',self.points)
        frames=[np.vstack((ghost,wall))]+[wall]*4
        np.savez_compressed(self.frontend/'capture/window.npz',points=np.concatenate(frames),
            stamps=self.stamps,lengths=np.array([len(frame) for frame in frames]))
        (self.frontend/'capture/capture.json').write_text(json.dumps(dict(
            counts={'output_cloud':5},temporal_raw=['window.npz'])))

    def tearDown(self):
        self.temp.cleanup()

    def run_filter(self,output,*extra):
        return subprocess.run([sys.executable,str(SCRIPT),str(self.frontend),str(self.backend),
            '--output-dir',str(output),*extra],text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)

    def test_full_cli_keeps_static_and_unknown_and_removes_transient(self):
        output=self.root/'fresh'
        process=self.run_filter(output)
        self.assertEqual(process.returncode,0,process.stderr)
        np.testing.assert_array_equal(read_map(output/'filtered_3cm.pcd'),self.points[1:])
        np.testing.assert_array_equal(read_map(output/'removed_3cm.pcd'),self.points[:1])
        report=json.loads((output/'report.json').read_text())
        self.assertEqual(report['input_frames'],5)
        self.assertEqual(report['evidence_frames'],5)
        self.assertEqual(report['supported_points_removed'],0)
        cached=self.root/'cached'
        process=self.run_filter(cached,'--reuse-evidence',str(output))
        self.assertEqual(process.returncode,0,process.stderr)
        self.assertEqual((cached/'filtered_3cm.pcd').read_bytes(),(output/'filtered_3cm.pcd').read_bytes())

    def test_foreign_trajectory_cache_is_rejected_before_outputs(self):
        evidence=self.root/'evidence'
        evidence.mkdir()
        meta=dict(frontend=str(self.frontend),backend=str(self.backend),input_frames=5,
            map_sha256=hashlib.sha256((self.backend/'optimized_3cm.pcd').read_bytes()).hexdigest(),
            trajectory_sha256='not-this-trajectory')
        (evidence/'report.json').write_text(json.dumps(meta))
        output=self.root/'rejected'
        process=self.run_filter(output,'--reuse-evidence',str(evidence))
        self.assertNotEqual(process.returncode,0)
        self.assertIn('Evidence trajectory mismatch',process.stderr)
        self.assertFalse(output.exists())

    def test_mismatched_baseline_frame_count_is_rejected(self):
        (self.backend/'report.json').write_text(json.dumps(dict(source_run=str(self.frontend),input_frames=4)))
        output=self.root/'rejected'
        process=self.run_filter(output)
        self.assertNotEqual(process.returncode,0)
        self.assertIn('Backend/capture frame count mismatch',process.stderr)
        self.assertFalse(output.exists())


if __name__=='__main__':
    unittest.main()
