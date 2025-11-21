import pytest
import urdfx
import numpy as np

def test_transform_identity():
    t = urdfx.Transform()
    assert np.allclose(t.translation(), np.zeros(3))
    assert np.allclose(t.rotation(), np.eye(3))
    assert np.allclose(t.as_matrix(), np.eye(4))

def test_transform_from_pos_quat():
    pos = np.array([1.0, 2.0, 3.0])
    # Nanobind maps Eigen::Quaternion to [x, y, z, w]
    quat = np.array([0.0, 0.0, 0.70710678, 0.70710678]) # x, y, z, w -> 90 deg around Z
    
    t = urdfx.Transform.from_position_quaternion(pos, quat)
    assert np.allclose(t.translation(), pos)
    
    # Check rotation
    # Rot(z, 90) * [1,0,0] = [0,1,0]
    v = np.array([1.0, 0.0, 0.0])
    v_rot = t.rotation() @ v
    assert np.allclose(v_rot, np.array([0.0, 1.0, 0.0]), atol=1e-5)

def test_transform_rpy():
    pos = np.array([0.0, 0.0, 0.0])
    rpy = np.array([0.0, 0.0, np.pi/2]) # 90 deg yaw
    t = urdfx.Transform.from_position_rpy(pos, rpy)
    
    v = np.array([1.0, 0.0, 0.0])
    v_rot = t.rotation() @ v
    assert np.allclose(v_rot, np.array([0.0, 1.0, 0.0]), atol=1e-5)

def test_inverse():
    t = urdfx.Transform.from_position_rpy(np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))
    inv = t.inverse()
    assert np.allclose(inv.translation(), np.array([-1.0, 0.0, 0.0]))

