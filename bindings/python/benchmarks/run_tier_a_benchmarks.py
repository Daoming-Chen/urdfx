#!/usr/bin/env python3
"""
Run benchmarks on Tier A robots (UR5e configurations with 6-10 DOF).
Tests IK solver with real-time generated test cases.
"""

import os
import sys
import time
import json
import numpy as np
from pathlib import Path

# Add current directory to path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)
    
# Add bindings path
bindings_path = os.path.abspath(os.path.join(current_dir, ".."))
if bindings_path not in sys.path:
    sys.path.insert(0, bindings_path)

try:
    import urdfx
except ImportError as e:
    print(f"Error: urdfx module not found. Please build Python bindings first.")
    print(f"Error details: {e}")
    sys.exit(1)

from oracle import FKOracle, JointSampler

def run_benchmark(robot_name, urdf_path, output_dir, num_samples=1000, seed=42):
    """Run IK benchmark with real-time generated test cases."""
    print(f"\n{'='*70}")
    print(f"Benchmarking {robot_name}")
    print(f"{'='*70}")
    
    # Load robot
    print(f"Loading robot from {urdf_path}...")
    robot = urdfx.Robot.from_urdf_file(urdf_path)
    dof = robot.dof
    print(f"  DOF: {dof}")
    print(f"  Test samples: {num_samples}")
    
    # Setup FK oracle and joint sampler
    oracle = FKOracle(robot)
    sampler = JointSampler(robot, rng=np.random.RandomState(seed))
    end_link = oracle.end_link
    
    print(f"  End-effector: {end_link}")
    
    # Create IK solver
    print("Creating IK solver...")
    solver = urdfx.SQPIKSolver(robot, end_link)
    
    # Configure solver
    config = urdfx.SolverConfig()
    config.tolerance = 1e-4
    config.max_iterations = 100
    config.enable_warm_start = True
    solver.set_solver_config(config)
    
    # Generate test samples (sample random joint configs once)
    print("\n  Generating test cases...")
    q_samples = sampler.sample(num_samples)
    
    # Run benchmarks with 3 initialization strategies
    results = {
        "robot_name": robot_name,
        "dof": dof,
        "total_samples": num_samples,
        "seed": seed,
        "cold_start_zero": {},
        "cold_start_random": {},
        "warm_start": {}
    }
    
    for case_type in ["cold_start_zero", "cold_start_random", "warm_start"]:
        print(f"\n  Testing {case_type}...")
        
        successes = 0
        failures = 0
        total_time = 0.0
        total_iterations = 0
        pos_errors = []
        rot_errors = []
        
        # Generate random initial guesses for cold_start_random (reuse across samples)
        if case_type == "cold_start_random":
            random_inits = sampler.sample(num_samples)
        
        for i, q_target in enumerate(q_samples):
            # 1. Compute target pose via FK (very fast)
            target_pos, target_rot = oracle.compute_pose(q_target)
            
            # Convert to Transform
            from scipy.spatial.transform import Rotation as R
            rpy = R.from_matrix(target_rot).as_euler('xyz', degrees=False)
            target_transform = urdfx.Transform.from_position_rpy(target_pos, rpy)
            
            # 2. Choose initial guess based on strategy
            if case_type == "cold_start_zero":
                q_init = np.zeros(dof)
            elif case_type == "cold_start_random":
                q_init = random_inits[i]
            else:  # warm_start
                # Add small noise to ground truth
                noise = np.random.normal(0, 0.1, size=dof)
                q_init = q_target + noise
            
            # 3. Solve IK (timed)
            start_time = time.perf_counter()
            result = solver.solve(target_transform, q_init)
            solve_time = time.perf_counter() - start_time
            
            total_time += solve_time
            
            # 4. Verify solution by comparing poses (not joint angles!)
            if result.status.converged:
                successes += 1
                total_iterations += result.status.iterations
                
                # Compute achieved pose
                achieved_pos, achieved_rot = oracle.compute_pose(result.solution)
                
                # Position error
                pos_error = np.linalg.norm(achieved_pos - target_pos)
                pos_errors.append(pos_error)
                
                # Rotation error (Frobenius norm of rotation matrix difference)
                rot_error = np.linalg.norm(achieved_rot - target_rot, 'fro')
                rot_errors.append(rot_error)
            else:
                failures += 1
        
        # Compute statistics
        success_rate = successes / num_samples * 100
        avg_time = total_time / num_samples * 1_000_000  # Convert to µs
        avg_iterations = total_iterations / successes if successes > 0 else 0
        avg_pos_error = np.mean(pos_errors) * 1000 if pos_errors else 0  # Convert to mm
        avg_rot_error = np.mean(rot_errors) if rot_errors else 0
        
        results[case_type] = {
            "cases": num_samples,
            "successes": successes,
            "failures": failures,
            "success_rate": success_rate,
            "avg_time_us": avg_time,
            "avg_iterations": avg_iterations,
            "avg_pos_error_mm": avg_pos_error,
            "avg_rot_error": avg_rot_error
        }
        
        print(f"    Cases: {num_samples}")
        print(f"    Success rate: {success_rate:.1f}%")
        print(f"    Avg time: {avg_time:.1f} µs")
        print(f"    Avg iterations: {avg_iterations:.1f}")
        print(f"    Avg position error: {avg_pos_error:.6f} mm")
        print(f"    Avg rotation error: {avg_rot_error:.6f}")
    
    # Save results
    output_path = os.path.join(output_dir, f"{robot_name}_results.json")
    with open(output_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\n  Results saved to {output_path}")
    
    return results

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Tier A IK Benchmark Runner")
    parser.add_argument("--samples", "-n", type=int, default=1000,
                       help="Number of test samples per robot (default: 1000)")
    parser.add_argument("--seed", "-s", type=int, default=42,
                       help="Random seed for reproducibility (default: 42)")
    parser.add_argument("--output", "-o", type=str, default=None,
                       help="Output directory for results")
    args = parser.parse_args()
    
    # Navigate from benchmarks directory to project root
    project_root = os.path.abspath(os.path.join(current_dir, "../../.."))
    
    models_dir = os.path.join(project_root, "examples/models/ur5")
    output_dir = args.output or os.path.join(project_root, "benchmarks/results")
    
    os.makedirs(output_dir, exist_ok=True)
    
    print("MixKinBench Tier A Benchmark Runner")
    print(f"Project root: {project_root}")
    print(f"Models directory: {models_dir}")
    print(f"Output directory: {output_dir}")
    print(f"Samples per robot: {args.samples}")
    print(f"Random seed: {args.seed}")
    
    # Configs: (robot_name, urdf_filename)
    configs = [
        ("ur5e", "ur5e.urdf"),
        ("ur5e+x", "ur5e+x.urdf"),
        ("ur5e+xy", "ur5e+xy.urdf"),
        ("ur5e+xyz", "ur5e+xyz.urdf"),
    ]
    
    all_results = []
    
    for robot_name, urdf_file in configs:
        urdf_path = os.path.join(models_dir, urdf_file)
        
        if not os.path.exists(urdf_path):
            print(f"\nWARNING: URDF not found: {urdf_path}")
            continue
        
        try:
            result = run_benchmark(robot_name, urdf_path, output_dir, 
                                 num_samples=args.samples, seed=args.seed)
            all_results.append(result)
        except Exception as e:
            print(f"\nERROR benchmarking {robot_name}: {e}")
            import traceback
            traceback.print_exc()
    
    # Generate summary report
    print(f"\n{'='*70}")
    print("BENCHMARK SUMMARY")
    print(f"{'='*70}")
    print(f"\n{'Robot':<15} {'DOF':<5} {'Cold(0)':<12} {'Cold(R)':<12} {'Warm':<12}")
    print("-" * 70)
    
    for result in all_results:
        robot = result['robot_name']
        dof = result['dof']
        cold_zero_sr = result['cold_start_zero'].get('success_rate', 0)
        cold_rand_sr = result['cold_start_random'].get('success_rate', 0)
        warm_sr = result['warm_start'].get('success_rate', 0)
        
        print(f"{robot:<15} {dof:<5} {cold_zero_sr:>5.1f}% {cold_rand_sr:>10.1f}% {warm_sr:>10.1f}%")
    
    print("\n" + "="*70)
    print("Benchmark complete!")
    print("="*70)

if __name__ == "__main__":
    main()
