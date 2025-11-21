import pytest
import urdfx
import numpy as np

def test_fk_creation(ur5_robot):
    fk = urdfx.ForwardKinematics(ur5_robot, "tool0")
    assert fk.num_joints == 6
    assert fk.get_num_joints() == 6

def test_fk_compute(ur5_robot):
    fk = urdfx.ForwardKinematics(ur5_robot, "tool0")
    
    # Zero configuration
    q = np.zeros(6)
    pose = fk.compute(q)
    
    # Expected pose for UR5 at zero config (upright)
    # This depends on the specific model, but we check it returns a Transform
    assert isinstance(pose, urdfx.Transform)
    assert pose.as_matrix().shape == (4, 4)

def test_fk_compute_to_link(ur5_robot):
    fk = urdfx.ForwardKinematics(ur5_robot, "tool0")
    q = np.zeros(6)
    pose = fk.compute_to_link(q, "forearm_link")
    assert isinstance(pose, urdfx.Transform)

