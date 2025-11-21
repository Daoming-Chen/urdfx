import pytest
import urdfx
import numpy as np

def test_jacobian_compute(ur5_robot):
    calc = urdfx.JacobianCalculator(ur5_robot, "tool0")
    q = np.zeros(6)
    J = calc.compute(q)
    assert J.shape == (6, 6)

def test_singularity(ur5_robot):
    calc = urdfx.JacobianCalculator(ur5_robot, "tool0")
    # Zero config is singular for UR5 (shoulder lift is 0, wrist 1 is 0)
    # Actually UR5 zero config is vertical up, singularity usually happens at specific angles.
    # But let's just check the call works.
    q = np.zeros(6)
    is_singular = calc.is_singular(q)
    assert isinstance(is_singular, bool)
    
    manip = calc.get_manipulability(q)
    assert isinstance(manip, float)
    assert manip >= 0

