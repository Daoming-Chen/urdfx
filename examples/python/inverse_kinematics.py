import urdfx
import numpy as np
import os
import sys

def main():
    # Path to URDF
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/ur5/ur5e.urdf")
    
    if not os.path.exists(urdf_path):
        urdf_path = "examples/models/ur5/ur5e.urdf"
        
    if not os.path.exists(urdf_path):
        print(f"URDF file not found: {urdf_path}")
        sys.exit(1)

    # Load robot
    robot = urdfx.Robot.from_urdf_file(urdf_path)
    
    # Create IK solver (use wrist_3_link as end effector)
    ik = urdfx.SQPIKSolver(robot, "wrist_3_link")
    fk = urdfx.ForwardKinematics(robot, "wrist_3_link")
    
    # Define target
    # Let's use FK to generate a reachable target
    target_q = np.array([0.5, -1.0, 1.5, -1.0, 0.5, 0.0])
    target_pose = fk.compute(target_q)
    
    print("Target configuration (joint angles):")
    print(target_q)
    print("\nTarget Pose:")
    print(f"Translation: {target_pose.translation()}")
    
    # Solve IK
    initial_guess = np.zeros(6)
    print("\nSolving IK...")
    result = ik.solve(target_pose, initial_guess)
    
    if result.status.converged:
        print("IK Converged!")
        print("Solution (joint angles):")
        print(result.solution)
        print(f"Iterations: {result.status.iterations}")
        print(f"Error: {result.status.final_error_norm}")
        
        # Verify
        pose_sol = fk.compute(result.solution)
        diff_pos = np.linalg.norm(pose_sol.translation() - target_pose.translation())
        print(f"\nPosition Error: {diff_pos:.6f}")
    else:
        print("IK Failed to converge")
        print(result.status.message)

if __name__ == "__main__":
    main()

