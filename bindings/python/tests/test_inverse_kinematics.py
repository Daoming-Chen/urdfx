import pytest
import urdfx
import numpy as np

def test_ik_solve(ur5_robot):
    solver = urdfx.SQPIKSolver(ur5_robot, "tool0")
    
    # Target: Forward kinematics of a known good config
    fk = urdfx.ForwardKinematics(ur5_robot, "tool0")
    target_q = np.array([0.1, -1.5, 1.5, -1.5, 1.5, 0.0])
    target_pose = fk.compute(target_q)
    
    initial_guess = np.zeros(6)
    result = solver.solve(target_pose, initial_guess)
    
    assert result.status.converged
    # We can't guarantee finding the EXACT same joint angles (redundancy/local minima),
    # but the pose should match.
    
    # Verify solution with FK
    pose_sol = fk.compute(result.solution)
    assert np.allclose(pose_sol.translation(), target_pose.translation(), atol=1e-3)
    # Orientation check involves quaternions or rotation matrices
    assert np.allclose(pose_sol.rotation(), target_pose.rotation(), atol=1e-3)

def test_solver_config(ur5_robot):
    solver = urdfx.SQPIKSolver(ur5_robot, "tool0")
    config = solver.get_solver_config()
    config.max_iterations = 100
    solver.set_solver_config(config)
    assert solver.get_solver_config().max_iterations == 100

