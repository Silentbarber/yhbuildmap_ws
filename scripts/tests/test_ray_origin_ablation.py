#!/usr/bin/env python3
import csv
import hashlib
import json
import pathlib
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from compare_ray_origin_ablation import compare
from evaluate_independent_run import write_pcd
from filter_dynamic_map import read_map


class OriginAblationTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=pathlib.Path(self.temp.name)
        backend=self.root/'backend'
        backend.mkdir()
        self.points=np.array([[1.,0,0,10],[2,0,0,20],[3,0,0,30],[4,0,0,40]],np.float32)
        write_pcd(backend/'optimized_3cm.pcd',self.points)
        (backend/'trajectory.csv').write_text('stamp,x,y,z,qx,qy,qz,qw\n10,0,0,0,0,0,0,1\n12,0,0,0,0,0,0,1\n')
        self.control=self.root/'control'
        self.interpolated=self.root/'interpolated'
        for directory,mode,free in [(self.control,'frame-end-control',[4,0,8,0]),
                                    (self.interpolated,'interpolated',[0,4,8,0])]:
            directory.mkdir()
            removed=np.array(free)==4
            np.savez_compressed(directory/'point_evidence.npz',removed=removed,
                hit_bins=np.array([0,0,4,0],np.uint16),free_bins=np.array(free,np.uint16),
                first_free=np.zeros(4),last_free=np.full(4,8.))
            write_pcd(directory/'filtered_3cm.pcd',self.points[~removed])
            write_pcd(directory/'removed_3cm.pcd',self.points[removed])
            report=dict(frontend=str(self.root/'frontend'),backend=str(backend),
                baseline_points=4,removed_points=1,input_frames=2,evidence_frames=2,
                time_bin_count=2,point_time_skipped_frames=0,wall_seconds=1.,
                settings=dict(point_time_origin_mode=mode,point_time_groups_ms=5.,
                    require_angular_support=True,min_free_bins=4,free_ratio=.8,min_span=6,max_hit_bins=3))
            for filename,key in [('optimized_3cm.pcd','map_sha256'),('trajectory.csv','trajectory_sha256')]:
                report[key]=hashlib.sha256((backend/filename).read_bytes()).hexdigest()
            (directory/'report.json').write_text(json.dumps(report))
            with (directory/'frame_evidence.csv').open('w') as file:
                writer=csv.writer(file)
                writer.writerow(['stamp','elapsed_s','candidates','bin','point_time_groups','point_time_skipped'])
                writer.writerows([[10,0,4,0,1,'False'],[12,2,4,1,1,0]])
        self.output=self.root/'comparison'

    def tearDown(self):
        self.temp.cleanup()

    def test_comparison_preserves_baseline_indices_and_changed_coordinates(self):
        result=compare(self.control,self.interpolated,self.output)
        self.assertEqual(result['restored_by_origin_change'],1)
        self.assertEqual(result['newly_removed_by_origin_change'],1)
        np.testing.assert_array_equal(read_map(self.output/'restored_by_origin.pcd'),self.points[:1])
        np.testing.assert_array_equal(read_map(self.output/'newly_removed_by_origin.pcd'),self.points[1:2])

    def test_changed_group_selection_is_rejected_before_output(self):
        path=self.interpolated/'frame_evidence.csv'
        path.write_text(path.read_text().replace('12,2,4,1,1,0','12,2,4,1,2,0'))
        with self.assertRaisesRegex(ValueError,'frame selection or point-time grouping differs'):
            compare(self.control,self.interpolated,self.output)
        self.assertFalse(self.output.exists())

    def test_changed_trajectory_is_rejected_before_output(self):
        (self.root/'backend/trajectory.csv').write_text('changed')
        with self.assertRaisesRegex(ValueError,'Source hash mismatch'):
            compare(self.control,self.interpolated,self.output)
        self.assertFalse(self.output.exists())


if __name__=='__main__':
    unittest.main()
