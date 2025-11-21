import time
import urdfx
import numpy as np
import os

def benchmark_ik():
    # Load robot
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_model_path = os.path.join(script_dir, "../../../examples/models/ur5/ur5e.urdf")
    model_path = os.environ.get("URDFX_MODEL_PATH", default_model_path)
    
    if not os.path.exists(model_path):
        print(f"Model not found at {model_path}. Set URDFX_MODEL_PATH.")
        return

    robot = urdfx.Robot.from_urdf_file(model_path)
    ik_solver = urdfx.SQPIKSolver(robot, "wrist_3_link")
    fk = urdfx.ForwardKinematics(robot, "wrist_3_link")
    
    # Prepare a target
    target_q = np.array([0.5, -1.0, 1.0, -1.0, 0.5, 0.0])
    target_pose = fk.compute(target_q)
    
    initial_guess = np.zeros(6)
    iterations = 1000
    
    print(f"Running IK benchmark ({iterations} iterations)...")
    start = time.time()
    success_count = 0
    for _ in range(iterations):
        # Use warm start from previous? No, let's test cold start or consistent start
        result = ik_solver.solve(target_pose, initial_guess)
        if result.status.converged:
            success_count += 1
            
    end = time.time()
    
    duration = end - start
    print(f"Total time: {duration:.4f}s")
    print(f"Average time per call: {duration/iterations*1e6:.2f} us")
    print(f"Calls per second: {iterations/duration:.2f}")
    print(f"Success rate: {success_count/iterations*100:.1f}%")

if __name__ == "__main__":
    benchmark_ik()

