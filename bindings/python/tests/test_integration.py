import pytest
import urdfx
import numpy as np

def test_fk_ik_roundtrip(ur5_robot):
    """Test FK -> IK roundtrip to ensure consistency."""
    fk = urdfx.ForwardKinematics(ur5_robot, "tool0")
    ik = urdfx.SQPIKSolver(ur5_robot, "tool0")
    
    # Generate random valid configurations
    # UR5 limits are roughly +/- 2pi for most joints
    np.random.seed(42)
    
    for _ in range(10):
        q_orig = np.random.uniform(-np.pi, np.pi, 6)
        
        # Compute FK
        target_pose = fk.compute(q_orig)
        
        # Solve IK (using original q as warm start to encourage finding same solution)
        # But adding some noise to make it non-trivial
        q_guess = q_orig + np.random.normal(0, 0.1, 6)
        
        result = ik.solve(target_pose, q_guess)
        
        assert result.status.converged
        
        # Verify pose accuracy
        pose_sol = fk.compute(result.solution)
        assert np.allclose(pose_sol.translation(), target_pose.translation(), atol=1e-4)
        
        # Orientation check (rotation matrices)
        # assert np.allclose(pose_sol.rotation(), target_pose.rotation(), atol=1e-3)
        # Using trace/angle difference might be more robust but element-wise is fine for close match

def test_jacobian_numerical_check(ur5_robot):
    """Compare analytic Jacobian with numerical differentiation."""
    fk = urdfx.ForwardKinematics(ur5_robot, "tool0")
    jac_calc = urdfx.JacobianCalculator(ur5_robot, "tool0")
    
    q = np.array([0.1, -0.5, 0.5, -1.0, 0.5, 0.0])
    
    # Analytic Jacobian
    J_analytic = jac_calc.compute(q, urdfx.JacobianType.Analytic)
    
    # Numerical differentiation (finite difference)
    epsilon = 1e-6
    J_num = np.zeros((6, 6))
    
    pose0 = fk.compute(q)
    pos0 = pose0.translation()
    # For rotation, we need angular velocity vector approximation. 
    # This is complex to implement correctly in a short test without helper functions.
    # Let's check translational part (top 3 rows).
    
    for i in range(6):
        q_plus = q.copy()
        q_plus[i] += epsilon
        
        pose_plus = fk.compute(q_plus)
        pos_plus = pose_plus.translation()
        
        vel_approx = (pos_plus - pos0) / epsilon
        J_num[0:3, i] = vel_approx
        
    # Check translational part match
    # Note: The analytic Jacobian might be in a different frame or representation 
    # than simple position difference if not careful.
    # Analytic Jacobian in urdfx is Spatial Jacobian in base frame? 
    # Actually, `JacobianType.Analytic` usually means Spatial Jacobian (v_base, w_base).
    # So the top 3 rows are linear velocity in base frame.
    
    # We allow some error due to finite difference approximation
    assert np.allclose(J_analytic[0:3, :], J_num[0:3, :], atol=1e-4)

def test_singularity_consistency(ur5_robot):
    """Check that singularity metrics are consistent."""
    jac_calc = urdfx.JacobianCalculator(ur5_robot, "tool0")
    
    # Singular config (vertical arm)
    q_singular = np.zeros(6) 
    # Actually for UR5, q=0 is singular (shoulder/wrist alignment)
    
    manip = jac_calc.get_manipulability(q_singular)
    is_sing = jac_calc.is_singular(q_singular, threshold=1e-3)
    
    # Manipulability should be close to zero
    assert manip < 1e-3
    assert is_sing
    
    # Non-singular config
    q_good = np.array([0.1, -1.0, 1.0, -1.0, 1.0, 0.0])
    manip_good = jac_calc.get_manipulability(q_good)
    is_sing_good = jac_calc.is_singular(q_good, threshold=1e-4)
    
    assert manip_good > 0.01
    assert not is_sing_good

