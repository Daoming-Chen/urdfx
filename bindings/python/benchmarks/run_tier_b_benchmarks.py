#!/usr/bin/env python3
"""
Tier B IK Benchmark Runner for synthetic mixed-chain robots (8-20 DOF).
Generates test cases in real-time without pre-generated datasets.

Usage:
    # Run complete benchmark suite
    python run_benchmarks.py
    
    # Custom DOF range and sample count
    python run_benchmarks.py --dof-min 8 --dof-max 12 --samples 500
    
    # Specify output directory
    python run_benchmarks.py --output benchmarks/results
"""

import os
import sys
import argparse
import json
import time
from pathlib import Path
from typing import List, Dict, Optional
import numpy as np

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
except ImportError:
    print("Error: urdfx module not found. Please build Python bindings first.")
    print(f"Tried paths: {bindings_path}")
    sys.exit(1)

from urdf_generator import MixedChainGenerator
from oracle import FKOracle, JointSampler


class BenchmarkRunner:
    """Manages IK benchmark execution with real-time test generation."""
    
    def __init__(self, verbose: bool = True):
        self.verbose = verbose
        
    def log(self, message: str):
        if self.verbose:
            print(message)
    
    def run_benchmark(self, 
                     dof: int,
                     num_samples: int,
                     robot_seed: int = 42,
                     sample_seed: int = 42,
                     solver_config: Optional[urdfx.SolverConfig] = None) -> Dict:
        """Run IK benchmark for a specific DOF configuration."""
        
        self.log(f"\n{'='*70}")
        self.log(f"Benchmarking {dof}-DOF Mixed-Chain Robot")
        self.log(f"{'='*70}")
        
        # 1. Generate synthetic robot
        self.log(f"Generating robot with seed={robot_seed}...")
        gen = MixedChainGenerator(dof=dof, prismatic_prob=0.3, seed=robot_seed)
        urdf_str = gen.to_urdf_string()
        
        stats = gen.get_statistics()
        self.log(f"  Robot: {stats['num_revolute']} revolute, {stats['num_prismatic']} prismatic joints")
        
        # 2. Load robot
        robot = urdfx.Robot.from_urdf_string(urdf_str)
        
        # 3. Setup oracle and sampler
        oracle = FKOracle(robot)
        sampler = JointSampler(robot, rng=np.random.RandomState(sample_seed))
        
        self.log(f"  End-effector: {oracle.end_link}")
        self.log(f"  Samples: {num_samples}")
        
        # 4. Create IK solver
        solver = urdfx.SQPIKSolver(robot, oracle.end_link)
        
        if solver_config:
            solver.set_solver_config(solver_config)
        else:
            config = urdfx.SolverConfig()
            config.tolerance = 1e-6
            config.max_iterations = 500
            solver.set_solver_config(config)
        
        # 5. Generate test samples once
        self.log("\n  Generating test cases...")
        q_samples = sampler.sample(num_samples)
        
        # 6. Run benchmarks with 3 initialization strategies
        results = {
            "dof": dof,
            "robot_stats": stats,
            "num_samples": num_samples,
            "robot_seed": robot_seed,
            "sample_seed": sample_seed,
            "cold_start_zero": {},
            "cold_start_random": {},
            "warm_start": {}
        }
        
        for case_type in ["cold_start_zero", "cold_start_random", "warm_start"]:
            self.log(f"\n  Testing {case_type}...")
            
            case_results = self._run_case_type(
                robot, oracle, sampler, solver, 
                q_samples, case_type, dof
            )
            
            results[case_type] = case_results
            
            self.log(f"    Success rate: {case_results['success_rate']:.1f}%")
            self.log(f"    Avg time: {case_results['avg_time_us']:.1f} µs")
            self.log(f"    Avg iterations: {case_results['avg_iterations']:.1f}")
            self.log(f"    Avg position error: {case_results['avg_pos_error_mm']:.6f} mm")
            self.log(f"    Avg rotation error: {case_results['avg_rot_error']:.6f}")
        
        return results
    
    def _run_case_type(self,
                      robot: urdfx.Robot,
                      oracle: FKOracle,
                      sampler: JointSampler,
                      solver: urdfx.SQPIKSolver,
                      q_samples: np.ndarray,
                      case_type: str,
                      dof: int) -> Dict:
        """Run benchmark for a specific initialization strategy."""
        
        num_samples = len(q_samples)
        successes = 0
        total_time = 0.0
        total_iterations = 0
        pos_errors = []
        rot_errors = []
        
        # Pre-generate random initial guesses if needed
        if case_type == "cold_start_random":
            random_inits = sampler.sample(num_samples)
        
        for i, q_target in enumerate(q_samples):
            # 1. Compute target pose via FK (very fast, <1ms)
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
                noise = np.random.normal(0, 0.1, size=dof)
                q_init = q_target + noise
            
            # 3. Solve IK (timed)
            start_time = time.perf_counter()
            result = solver.solve(target_transform, q_init)
            elapsed_s = time.perf_counter() - start_time
            
            total_time += elapsed_s
            
            # 4. Verify solution by comparing poses (not joint angles!)
            if result.status.converged:
                successes += 1
                total_iterations += result.status.iterations
                
                # Compute achieved pose
                achieved_pos, achieved_rot = oracle.compute_pose(result.solution)
                
                # Position error
                pos_error = np.linalg.norm(achieved_pos - target_pos)
                pos_errors.append(pos_error)
                
                # Rotation error (Frobenius norm)
                rot_error = np.linalg.norm(achieved_rot - target_rot, 'fro')
                rot_errors.append(rot_error)
        
        # Compute statistics
        success_rate = successes / num_samples * 100
        avg_time_us = total_time / num_samples * 1_000_000  # Convert to µs
        avg_iterations = total_iterations / successes if successes > 0 else 0
        avg_pos_error_mm = np.mean(pos_errors) * 1000 if pos_errors else 0
        avg_rot_error = np.mean(rot_errors) if rot_errors else 0
        
        return {
            "cases": num_samples,
            "successes": successes,
            "failures": num_samples - successes,
            "success_rate": success_rate,
            "avg_time_us": avg_time_us,
            "avg_iterations": avg_iterations,
            "avg_pos_error_mm": avg_pos_error_mm,
            "avg_rot_error": avg_rot_error
        }


def main():
    parser = argparse.ArgumentParser(
        description="Tier B IK Benchmark Runner (Synthetic Mixed-Chain Robots)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    # DOF range configuration
    parser.add_argument('--dof-min', type=int, default=8,
                       help='Minimum DOF for generated robots (default: 8)')
    parser.add_argument('--dof-max', type=int, default=20,
                       help='Maximum DOF for generated robots (default: 20)')
    
    # Sample configuration
    parser.add_argument('--samples', '-n', type=int, default=1000,
                       help='Number of test samples per DOF (default: 1000)')
    
    # Solver configuration
    parser.add_argument('--max-iterations', type=int, default=500,
                       help='Maximum IK solver iterations (default: 500)')
    parser.add_argument('--tolerance', type=float, default=1e-6,
                       help='IK solver tolerance (default: 1e-6)')
    
    # Seeds for reproducibility
    parser.add_argument('--robot-seed', type=int, default=42,
                       help='Seed for robot generation (default: 42)')
    parser.add_argument('--sample-seed', type=int, default=42,
                       help='Seed for sample generation (default: 42)')
    
    # Output configuration
    parser.add_argument('--output', '-o', type=str, default='benchmarks/results',
                       help='Output directory for results')
    
    # Visualization
    parser.add_argument('--visualize', action='store_true',
                       help='Generate visualization after benchmarking')
    
    args = parser.parse_args()
    
    # Setup paths
    project_root = os.path.abspath(os.path.join(current_dir, "../../.."))
    output_dir = Path(args.output) if os.path.isabs(args.output) else Path(project_root) / args.output
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print("urdfx Tier B IK Benchmark Runner")
    print("=" * 70)
    print(f"DOF range: {args.dof_min}-{args.dof_max}")
    print(f"Samples per DOF: {args.samples}")
    print(f"Max iterations: {args.max_iterations}")
    print(f"Tolerance: {args.tolerance}")
    print(f"Robot seed: {args.robot_seed}")
    print(f"Sample seed: {args.sample_seed}")
    print(f"Output directory: {output_dir}")
    
    # Configure solver
    solver_config = urdfx.SolverConfig()
    solver_config.max_iterations = args.max_iterations
    solver_config.tolerance = args.tolerance
    
    # Run benchmarks
    runner = BenchmarkRunner(verbose=True)
    all_results = []
    
    dof_range = range(args.dof_min, args.dof_max + 1)
    total_dofs = len(dof_range)
    
    start_total = time.time()
    
    for idx, dof in enumerate(dof_range):
        print(f"\n[{idx+1}/{total_dofs}] Processing {dof}-DOF configuration...")
        
        start_time = time.time()
        result = runner.run_benchmark(
            dof=dof,
            num_samples=args.samples,
            robot_seed=args.robot_seed + dof,  # Different seed per DOF
            sample_seed=args.sample_seed,
            solver_config=solver_config
        )
        elapsed = time.time() - start_time
        
        result['benchmark_time_s'] = elapsed
        all_results.append(result)
        
        print(f"\n  ✓ Completed in {elapsed:.2f}s")
    
    total_elapsed = time.time() - start_total
    
    # Save results
    results_file = output_dir / "tier_b_benchmark_results.json"
    with open(results_file, 'w') as f:
        json.dump({
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'config': {
                'dof_range': [args.dof_min, args.dof_max],
                'samples_per_dof': args.samples,
                'max_iterations': args.max_iterations,
                'tolerance': args.tolerance,
                'robot_seed': args.robot_seed,
                'sample_seed': args.sample_seed,
            },
            'total_time_s': total_elapsed,
            'benchmarks': all_results
        }, f, indent=2)
    
    print("\n" + "=" * 70)
    print(f"✓ All benchmarks completed in {total_elapsed:.2f}s")
    print(f"✓ Results saved to: {results_file}")
    print("=" * 70)
    
    # Generate visualization if requested
    if args.visualize:
        print("\nGenerating visualizations...")
        try:
            from visualize import visualize_tier_b_results
            visualize_tier_b_results(str(results_file), str(output_dir))
            print(f"✓ Visualizations saved to: {output_dir}")
        except ImportError:
            print("Warning: Could not import visualization module")
        except Exception as e:
            print(f"Error during visualization: {e}")


if __name__ == "__main__":
    main()
