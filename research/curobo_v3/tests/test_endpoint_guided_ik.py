import sys
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from v3_endpoint_guided_ik import endpoint_references, biased_seeds


def test_reference_keeps_bounded_joint_coordinates_without_angle_wrapping():
    start=np.zeros(15);end=np.zeros(15)
    start[5]=np.deg2rad(170);end[5]=np.deg2rad(-170)
    values=endpoint_references(start,end,3)
    assert np.allclose(values[0],start)
    assert np.allclose(values[-1],end)
    assert np.isclose(values[1,5],0)


def test_seed_corridors_are_shared_across_points_and_respect_limits():
    reference=endpoint_references(np.zeros(15),np.full(15,.1))
    lower=np.full(15,-1.);upper=np.full(15,1.)
    seeds=biased_seeds(reference,lower,upper)
    assert seeds.shape==(128,16,15)
    assert np.allclose(seeds[:,0],reference)
    assert np.allclose(seeds[0,1:12]-reference[0],seeds[-1,1:12]-reference[-1],atol=1e-6)
    assert np.all(seeds>=lower) and np.all(seeds<=upper)
    assert not np.allclose(seeds[0,12:],seeds[-1,12:])
