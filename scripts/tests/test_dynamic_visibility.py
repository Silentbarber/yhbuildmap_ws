#!/usr/bin/env python3
import pathlib
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from filter_dynamic_map import correct_scan, grouped_ray_evidence, ray_evidence, removal_mask


def returns(distance, angle=.001):
    directions = np.array([[1,0,0],[1,angle,0],[1,0,angle]],dtype=float)
    directions /= np.linalg.norm(directions,axis=1)[:,None]
    return directions * distance


class VisibilityTest(unittest.TestCase):
    def test_measured_returns_beyond_point_are_free_evidence(self):
        hit,free=ray_evidence(np.array([[2.,0,0]]),returns(4),np.zeros(3))
        self.assertFalse(hit[0])
        self.assertTrue(free[0])

    def test_static_wall_is_supported_and_not_free(self):
        hit,free=ray_evidence(np.array([[4.,0,0]]),returns(4),np.zeros(3))
        self.assertTrue(hit[0])
        self.assertFalse(free[0])

    def test_occluded_wall_is_unknown(self):
        hit,free=ray_evidence(np.array([[4.,0,0]]),returns(2),np.zeros(3))
        self.assertFalse(hit[0])
        self.assertFalse(free[0])

    def test_missing_directions_are_unknown(self):
        hit,free=ray_evidence(np.array([[2.,0,0]]),returns(4,angle=.2),np.zeros(3))
        self.assertFalse(hit[0])
        self.assertFalse(free[0])

    def test_thin_surface_near_depth_edge_is_not_removed(self):
        scan=returns(4)
        scan[1]=returns(2)[1]
        hit,free=ray_evidence(np.array([[2.,0,0]]),scan,np.zeros(3))
        self.assertTrue(hit[0])
        self.assertFalse(free[0])

    def test_depth_discontinuity_does_not_prove_empty_space(self):
        scan=returns(4)
        scan[2]=returns(8)[2]
        _,free=ray_evidence(np.array([[2.,0,0]]),scan,np.zeros(3))
        self.assertFalse(free[0])

    def test_small_depth_residual_is_not_dynamic(self):
        _,free=ray_evidence(np.array([[3.85,0,0]]),returns(4),np.zeros(3))
        self.assertFalse(free[0])

    def test_guard_rejects_one_sided_returns_beyond_thin_structure(self):
        scan=np.array([[4.,.004,0],[4,.008,.004],[4,.008,-.004]])
        point=np.array([[2.,0,0]])
        _,unguarded=ray_evidence(point,scan,np.zeros(3))
        hit,guarded=ray_evidence(point,scan,np.zeros(3),require_angular_support=True)
        self.assertTrue(unguarded[0])
        self.assertFalse(hit[0])
        self.assertFalse(guarded[0])

    def test_guard_accepts_surrounded_free_space(self):
        scan=np.array([[4.,.004,0],[4,-.004,.004],[4,-.004,-.004]])
        _,free=ray_evidence(np.array([[2.,0,0]]),scan,np.zeros(3),require_angular_support=True)
        self.assertTrue(free[0])

    def test_guard_does_not_change_endpoint_hits(self):
        point=np.array([[4.,0,0]])
        base_hit,_=ray_evidence(point,returns(4),np.zeros(3))
        guarded_hit,_=ray_evidence(point,returns(4),np.zeros(3),require_angular_support=True)
        np.testing.assert_array_equal(base_hit,guarded_hit)

    def test_grouped_origins_keep_hit_precedence_over_free_vote(self):
        point=np.array([[2.,0,0]])
        groups=[(np.arange(3), np.zeros(3), 0.),
                (np.arange(3,6), np.zeros(3), .01)]
        measured=np.vstack([returns(4), returns(2)])
        hit,free=grouped_ray_evidence(point,measured,groups)
        self.assertTrue(hit[0])
        self.assertFalse(free[0])

    def test_group_with_too_few_returns_is_unknown(self):
        point=np.array([[2.,0,0]])
        measured=returns(4)
        hit,free=grouped_ray_evidence(point,measured,[(np.arange(2),np.zeros(3),0.)])
        self.assertFalse(hit[0])
        self.assertFalse(free[0])

    def test_group_origin_changes_ray_visibility_at_identical_partition(self):
        point=np.array([[2.,0,0]])
        measured=returns(4)
        _,shared_free=grouped_ray_evidence(point,measured,[(np.arange(3),np.zeros(3),0.)])
        hit,moving_free=grouped_ray_evidence(point,measured,[(np.arange(3),np.array([0.,1.,0.]),0.)])
        self.assertTrue(shared_free[0])
        self.assertFalse(hit[0])
        self.assertFalse(moving_free[0])

    def test_votes_need_independent_bins_span_and_low_support(self):
        hits=np.array([0,0,8,0,1],dtype=np.uint16)
        frees=np.array([4,3,4,4,8],dtype=np.uint16)
        mask=removal_mask(hits,frees,np.zeros(5),np.array([8,8,8,1,8]))
        np.testing.assert_array_equal(mask,[True,False,False,False,True])

    def test_uncertain_no_observation_is_retained(self):
        mask=removal_mask(np.array([0]),np.array([0]),np.array([np.inf]),np.array([-np.inf]))
        self.assertFalse(mask[0])

    def test_repeatedly_supported_geometry_is_retained(self):
        mask=removal_mask(np.array([4]),np.array([30]),np.array([0]),np.array([20]))
        self.assertFalse(mask[0])

    def test_global_correction_and_lidar_origin(self):
        old_r=Rotation.from_euler('z',30,degrees=True).as_matrix()
        new_r=Rotation.from_euler('z',-20,degrees=True).as_matrix()
        old_t,new_t=np.array([1,2,3]),np.array([3,1,0])
        local=np.array([[2.,1,0]])
        extrinsic=np.array([-.01,-.02,.04])
        corrected,origin=correct_scan(local@old_r.T+old_t,old_t,old_r,new_t,new_r,extrinsic)
        np.testing.assert_allclose(corrected,local@new_r.T+new_t,atol=1e-12)
        np.testing.assert_allclose(origin,new_t+new_r@extrinsic,atol=1e-12)


if __name__=='__main__':
    unittest.main()
