import time
import urdfx
import numpy as np
import os

def benchmark_fk():
    # Load robot
    # Try to find model relative to script location if env var not set
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_model_path = os.path.join(script_dir, "../../../examples/models/ur5/ur5e.urdf")
    
    model_path = os.environ.get("URDFX_MODEL_PATH", default_model_path)
    
    if not os.path.exists(model_path):
        print(f"Model not found at {model_path}. Set URDFX_MODEL_PATH.")
        return

    robot = urdfx.Robot.from_urdf_file(model_path)
    fk = urdfx.ForwardKinematics(robot, "wrist_3_link")
    
    q = np.zeros(6)
    iterations = 100000
    
    print(f"Running FK benchmark ({iterations} iterations)...")
    start = time.time()
    for _ in range(iterations):
        fk.compute(q)
    end = time.time()
    
    duration = end - start
    print(f"Total time: {duration:.4f}s")
    print(f"Average time per call: {duration/iterations*1e6:.2f} us")
    print(f"Calls per second: {iterations/duration:.2f}")

if __name__ == "__main__":
    benchmark_fk()

