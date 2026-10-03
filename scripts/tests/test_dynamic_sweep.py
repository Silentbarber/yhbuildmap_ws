#!/usr/bin/env python3
import pathlib
import sys
import hashlib
import json
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from filter_dynamic_map import removal_mask
from evaluate_independent_run import write_pcd
import sweep_dynamic_filter_params as sweep


class SweepTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=pathlib.Path(self.temp.name)
        (self.root/'config').mkdir()
        backend=self.root/'backend'
        backend.mkdir()
        evidence=self.root/'evidence'/'sample'
        evidence.mkdir(parents=True)
        points=np.column_stack((np.arange(5),np.zeros((5,2)),np.ones(5))).astype(np.float32)
        write_pcd(backend/'optimized_3cm.pcd',points)
        (backend/'trajectory.csv').write_text('stamp,x,y,z,qx,qy,qz,qw\n0,0,0,0,0,0,0,1\n10,0,0,0,0,0,0,1\n')
        hashes={key:hashlib.sha256((backend/name).read_bytes()).hexdigest()
                for name,key in [('optimized_3cm.pcd','map_sha256'),('trajectory.csv','trajectory_sha256')]}
        record=dict(id='sample',bag='synthetic',backend='backend',frames=2,**hashes)
        (self.root/'config/dynamic_filtering_recordings.json').write_text(json.dumps({'recordings':[record]}))
        (evidence/'report.json').write_text(json.dumps(dict(input_frames=2,settings={'require_angular_support':True},**hashes)))
        np.savez_compressed(evidence/'point_evidence.npz',hit_bins=np.array([0,1,3,4,0]),
                            free_bins=np.array([12,8,5,20,0]),first_free=np.zeros(5),last_free=np.full(5,20.))

    def tearDown(self):
        self.temp.cleanup()

    def invoke(self):
        command=['sweep','--data-root',str(self.root),'--evidence-root',str(self.root/'evidence'),
                 '--output',str(self.root/'output.json'),'--min-free','4','--ratio','.8','--span','6','--max-hit','3']
        with patch.object(sweep,'ROOT',self.root),patch.object(sys,'argv',command):
            sweep.main()

    def test_command_reports_counts_without_writing_maps(self):
        self.invoke()
        candidate=json.loads((self.root/'output.json').read_text())['recordings'][0]['candidates'][0]
        self.assertEqual(candidate['removed_points'],2)
        self.assertEqual(candidate['supported_points_removed'],0)
        self.assertEqual(candidate['planar_sample_points'],0)
        self.assertEqual(len(list(self.root.rglob('*.pcd'))),1)

    def test_changed_trajectory_is_rejected(self):
        (self.root/'backend/trajectory.csv').write_text('changed')
        with self.assertRaisesRegex(ValueError,'Trajectory provenance mismatch'):
            self.invoke()
        self.assertFalse((self.root/'output.json').exists())

    def test_stricter_threshold_is_subset(self):
        hit = np.array([0, 1, 3, 4], dtype=np.uint16)
        free = np.array([12, 8, 5, 20], dtype=np.uint16)
        first = np.zeros(4)
        last = np.array([20, 20, 20, 20.])
        loose = removal_mask(hit, free, first, last, 4, .8, 6, 3)
        strict = removal_mask(hit, free, first, last, 8, .9, 10, 1)
        self.assertTrue(np.all(~strict | loose))


if __name__ == '__main__':
    unittest.main()
