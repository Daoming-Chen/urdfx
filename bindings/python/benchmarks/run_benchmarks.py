#!/usr/bin/env python3
"""
Complete benchmark runner script for urdfx IK benchmarks.

This script provides a one-stop solution for:
1. Generating benchmark datasets with mixed-chain robots
2. Running IK benchmarks on the datasets
3. Generating visualization and summary reports

Usage:
    # Run complete benchmark suite (generate + run + visualize)
    python run_benchmarks.py --all
    
    # Only generate datasets
    python run_benchmarks.py --generate --dof-range 8 12
    
    # Only run benchmarks on existing datasets
    python run_benchmarks.py --run --dataset-dir benchmarks/datasets
    
    # Only visualize existing results
    python run_benchmarks.py --visualize --results benchmarks/results/benchmark_results.json
    
    # Custom configuration
    python run_benchmarks.py --all --dof-range 8 20 --samples 500 --output benchmarks/results
"""

import os
import sys
import argparse
import json
import time
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import numpy as np

# Add current directory to path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

# Add bindings build path (try multiple common locations)
possible_build_paths = [
    os.path.abspath(os.path.join(current_dir, "..")),  # bindings/python directory (contains urdfx package)
    os.path.abspath(os.path.join(current_dir, "../../../build/bindings/python")),
    os.path.abspath(os.path.join(current_dir, "../../../build-wasm/bindings/python")),
    os.path.abspath(os.path.join(current_dir, "../../build/lib")),
]

for build_path in possible_build_paths:
    if os.path.exists(build_path):
        if build_path not in sys.path:
            sys.path.append(build_path)
        break

try:
    import urdfx
except ImportError:
    print("Error: urdfx module not found. Please build Python bindings first.")
    print("Tried paths:")
    for p in possible_build_paths:
        print(f"  - {p}")
    sys.exit(1)

from generator import MixedChainGenerator
from oracle import FKOracle, JointSampler
from dataset import BenchmarkDataset, BenchmarkCase


class BenchmarkRunner:
    """Manages IK benchmark execution on datasets."""
    
    def __init__(self, verbose: bool = True):
        self.verbose = verbose
        self.results = []
        
    def log(self, message: str):
        if self.verbose:
            print(message)
    
    def run_single_case(self, 
                       robot: urdfx.Robot, 
                       case: BenchmarkCase,
                       solver_config: Optional[urdfx.SolverConfig] = None) -> Dict:
        """Run IK on a single benchmark case."""
        end_link = self._get_end_link(robot)
        solver = urdfx.SQPIKSolver(robot, end_link)
        
        if solver_config:
            solver.set_solver_config(solver_config)
        
        # Create target transform from rotation matrix and translation
        # Convert rotation matrix to RPY (roll-pitch-yaw) angles
        rot_mat = np.array(case.target_rot).reshape(3, 3)
        
        # Extract RPY from rotation matrix
        sy = np.sqrt(rot_mat[0, 0]**2 + rot_mat[1, 0]**2)
        singular = sy < 1e-6
        
        if not singular:
            roll = np.arctan2(rot_mat[2, 1], rot_mat[2, 2])
            pitch = np.arctan2(-rot_mat[2, 0], sy)
            yaw = np.arctan2(rot_mat[1, 0], rot_mat[0, 0])
        else:
            roll = np.arctan2(-rot_mat[1, 2], rot_mat[1, 1])
            pitch = np.arctan2(-rot_mat[2, 0], sy)
            yaw = 0
        
        rpy = np.array([roll, pitch, yaw])
        target_transform = urdfx.Transform.from_position_rpy(
            np.array(case.target_pos), rpy
        )
        
        # Solve IK
        start_time = time.perf_counter()
        result = solver.solve(target_transform, np.array(case.initial_guess))
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        
        # Compute errors if converged
        pos_error_mm = 0.0
        rot_error_deg = 0.0
        converged = result.status.converged
        
        if converged:
            fk = urdfx.ForwardKinematics(robot, end_link)
            actual_pose = fk.compute(result.solution)
            
            # Position error
            pos_diff = np.array(actual_pose.translation()) - np.array(case.target_pos)
            pos_error_mm = np.linalg.norm(pos_diff) * 1000.0  # Convert to mm
            
            # Rotation error (Frobenius norm of difference)
            rot_diff = np.array(actual_pose.rotation()) - np.array(case.target_rot)
            rot_error_deg = np.linalg.norm(rot_diff) * 180.0 / np.pi  # Rough approximation
        
        return {
            'converged': converged,
            'iterations': result.status.iterations,
            'time_ms': elapsed_ms,
            'pos_error_mm': pos_error_mm,
            'rot_error_deg': rot_error_deg,
            'final_error_norm': result.status.final_error_norm,
            'metadata': case.metadata
        }
    
    def run_dataset(self, 
                   dataset: BenchmarkDataset,
                   solver_config: Optional[urdfx.SolverConfig] = None) -> Dict:
        """Run IK benchmark on entire dataset."""
        if not dataset.cases:
            return {}
        
        # Load robot from first case
        case0 = dataset.cases[0]
        self.log(f"Loading robot: {case0.robot_name} ({case0.dof} DOF)")
        
        # Need to reconstruct robot - for synthetic robots, we need the URDF
        # This assumes datasets were generated with URDFs saved alongside
        # For now, regenerate the robot
        robot = self._load_robot_for_dataset(dataset)
        
        results = []
        total_cases = len(dataset.cases)
        
        self.log(f"Running {total_cases} benchmark cases...")
        for i, case in enumerate(dataset.cases):
            if (i + 1) % 100 == 0:
                self.log(f"  Progress: {i+1}/{total_cases}")
            
            case_result = self.run_single_case(robot, case, solver_config)
            results.append(case_result)
        
        # Compute aggregate statistics
        stats = self._compute_statistics(results, dataset.name)
        return stats
    
    def _compute_statistics(self, results: List[Dict], dataset_name: str) -> Dict:
        """Compute aggregate statistics from individual results."""
        converged_results = [r for r in results if r['converged']]
        total = len(results)
        success_count = len(converged_results)
        success_rate = (success_count / total * 100) if total > 0 else 0.0
        
        # Compute averages for successful cases
        avg_time_ms = np.mean([r['time_ms'] for r in results]) if results else 0.0
        avg_iter = np.mean([r['iterations'] for r in results]) if results else 0.0
        avg_pos_err_mm = np.mean([r['pos_error_mm'] for r in converged_results]) if converged_results else 0.0
        avg_rot_err_deg = np.mean([r['rot_error_deg'] for r in converged_results]) if converged_results else 0.0
        
        # Separate by metadata type if available
        type_stats = {}
        for r in results:
            meta_type = r['metadata'].get('type', 'unknown')
            if meta_type not in type_stats:
                type_stats[meta_type] = []
            type_stats[meta_type].append(r)
        
        type_breakdown = {}
        for meta_type, type_results in type_stats.items():
            type_converged = [r for r in type_results if r['converged']]
            type_breakdown[meta_type] = {
                'count': len(type_results),
                'success_rate': (len(type_converged) / len(type_results) * 100) if type_results else 0.0,
                'avg_time_ms': np.mean([r['time_ms'] for r in type_results]) if type_results else 0.0,
                'avg_iter': np.mean([r['iterations'] for r in type_results]) if type_results else 0.0,
            }
        
        return {
            'name': dataset_name,
            'total_cases': total,
            'success_count': success_count,
            'success_rate': success_rate,
            'avg_time_ms': avg_time_ms,
            'avg_iter': avg_iter,
            'avg_pos_err_mm': avg_pos_err_mm,
            'avg_rot_err_deg': avg_rot_err_deg,
            'type_breakdown': type_breakdown,
            'individual_results': results
        }
    
    def _get_end_link(self, robot: urdfx.Robot) -> str:
        """Find the end effector link (leaf node)."""
        links = robot.get_links()
        for link in links:
            name = link.get_name()
            children = robot.get_child_joints(name)
            if not children:
                return name
        return links[-1].get_name()
    
    def _load_robot_for_dataset(self, dataset: BenchmarkDataset) -> urdfx.Robot:
        """Load robot for the dataset (regenerate for synthetic robots)."""
        # For synthetic robots, we need to regenerate
        # Extract DOF from dataset
        if not dataset.cases:
            raise ValueError("Empty dataset")
        
        case0 = dataset.cases[0]
        dof = case0.dof
        
        # Parse dataset name to get DOF and regenerate
        # Expected format: "tier_b_8dof" etc.
        if "tier_b" in dataset.name or "mixed" in case0.robot_name:
            # Regenerate synthetic robot with same seed
            seed = 42 + dof  # Same seed used in generation
            gen = MixedChainGenerator(dof=dof, prismatic_prob=0.3, seed=seed)
            urdf_str = gen.to_urdf_string()
            return urdfx.Robot.from_urdf_string(urdf_str)
        else:
            # Try to load from file
            raise NotImplementedError(f"Loading non-synthetic robot not implemented: {case0.robot_name}")
    
    def save_results(self, output_path: str):
        """Save benchmark results to JSON file."""
        with open(output_path, 'w') as f:
            json.dump(self.results, f, indent=2)
        self.log(f"Results saved to: {output_path}")


def generate_datasets(args):
    """Generate benchmark datasets."""
    print("=" * 60)
    print("STEP 1: Generating Benchmark Datasets")
    print("=" * 60)
    
    output_dir = Path(args.output) / "datasets"
    output_dir.mkdir(parents=True, exist_ok=True)
    
    dof_range = range(args.dof_min, args.dof_max + 1)
    
    for dof in dof_range:
        print(f"\n[{dof - args.dof_min + 1}/{args.dof_max - args.dof_min + 1}] Generating {dof}-DOF dataset...")
        
        # Generate robot
        gen = MixedChainGenerator(dof=dof, prismatic_prob=0.3, seed=42+dof)
        urdf_str = gen.to_urdf_string()
        
        # Save URDF
        urdf_path = output_dir / f"mixed_{dof}dof.urdf"
        gen.save_urdf(str(urdf_path))
        print(f"  ✓ Robot URDF saved: {urdf_path.name}")
        
        # Print robot statistics
        stats = gen.get_statistics()
        print(f"  ✓ Robot stats: {stats['num_revolute']} revolute, {stats['num_prismatic']} prismatic joints")
        
        # Load robot
        robot = urdfx.Robot.from_urdf_string(urdf_str)
        
        # Setup oracle and sampler
        oracle = FKOracle(robot)
        sampler = JointSampler(robot)
        
        # Generate dataset
        dataset_name = f"tier_b_{dof}dof"
        dataset = BenchmarkDataset(dataset_name)
        
        if args.variations:
            dataset.generate_variations(oracle, sampler, count=args.samples)
            print(f"  ✓ Generated {len(dataset.cases)} cases (with variations)")
        else:
            dataset.generate(oracle, sampler, count=args.samples)
            print(f"  ✓ Generated {len(dataset.cases)} cases")
        
        # Save dataset
        dataset_path = output_dir / f"{dataset_name}.npz"
        dataset.save_npz(str(dataset_path))
        print(f"  ✓ Dataset saved: {dataset_path.name}")
        
        if args.save_binary:
            bin_path = output_dir / f"{dataset_name}.bin"
            dataset.save_binary(str(bin_path))
            print(f"  ✓ Binary format saved: {bin_path.name}")
    
    print(f"\n✓ All datasets generated successfully in: {output_dir}")


def run_benchmarks(args):
    """Run benchmarks on generated datasets."""
    print("\n" + "=" * 60)
    print("STEP 2: Running Benchmarks")
    print("=" * 60)
    
    dataset_dir = Path(args.dataset_dir) if args.dataset_dir else Path(args.output) / "datasets"
    results_dir = Path(args.output) / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    
    # Find all dataset files
    dataset_files = sorted(dataset_dir.glob("tier_b_*.npz"))
    
    if not dataset_files:
        print(f"No datasets found in {dataset_dir}")
        return None
    
    print(f"Found {len(dataset_files)} datasets\n")
    
    # Configure solver
    solver_config = urdfx.SolverConfig()
    solver_config.max_iterations = args.max_iterations
    solver_config.tolerance = args.tolerance
    
    # Run benchmarks
    runner = BenchmarkRunner(verbose=True)
    all_results = []
    
    for i, dataset_file in enumerate(dataset_files):
        print(f"[{i+1}/{len(dataset_files)}] Processing: {dataset_file.name}")
        
        # Load dataset
        dataset = BenchmarkDataset.load_npz(str(dataset_file))
        
        # Run benchmark
        start_time = time.time()
        result = runner.run_dataset(dataset, solver_config)
        elapsed = time.time() - start_time
        
        result['dataset_file'] = dataset_file.name
        result['elapsed_time_s'] = elapsed
        all_results.append(result)
        
        # Print summary
        print(f"  ✓ Completed in {elapsed:.2f}s")
        print(f"  ✓ Success rate: {result['success_rate']:.1f}%")
        print(f"  ✓ Avg time: {result['avg_time_ms']:.3f} ms")
        print(f"  ✓ Avg iterations: {result['avg_iter']:.1f}")
        
        if result['type_breakdown']:
            print("  ✓ Breakdown by type:")
            for meta_type, stats in result['type_breakdown'].items():
                print(f"      {meta_type}: {stats['success_rate']:.1f}% success, {stats['avg_time_ms']:.3f} ms")
        print()
    
    # Save results
    results_file = results_dir / "benchmark_results.json"
    with open(results_file, 'w') as f:
        json.dump({
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'config': {
                'max_iterations': args.max_iterations,
                'tolerance': args.tolerance,
            },
            'benchmarks': all_results
        }, f, indent=2)
    
    print(f"✓ Benchmark results saved to: {results_file}")
    return results_file


def visualize_results(args, results_file=None):
    """Generate visualization from benchmark results."""
    print("\n" + "=" * 60)
    print("STEP 3: Generating Visualizations")
    print("=" * 60)
    
    if results_file is None:
        results_file = args.results if args.results else Path(args.output) / "results" / "benchmark_results.json"
    
    results_file = Path(results_file)
    
    if not results_file.exists():
        print(f"Results file not found: {results_file}")
        return
    
    print(f"Loading results from: {results_file}")
    
    # Use existing visualization module
    try:
        from visualize import load_benchmark_results, extract_metrics, plot_results, generate_summary_table
        
        # Convert our format to expected format
        converted_benchmarks = []
        with open(results_file, 'r') as f:
            data = json.load(f)
        
        for bm in data.get('benchmarks', []):
            # Extract DOF from dataset name
            dof_str = bm['name'].split('_')[-1].replace('dof', '')
            dof = int(dof_str)
            
            # Create entries for each type
            if bm.get('type_breakdown'):
                for meta_type, stats in bm['type_breakdown'].items():
                    # Map metadata types to benchmark names
                    type_map = {
                        'cold_start_zero': 'BM_MixedChainIK_ColdStart',
                        'cold_start_random': 'BM_MixedChainIK_ColdStart',
                        'warm_start': 'BM_MixedChainIK_WarmStart'
                    }
                    
                    bench_type = type_map.get(meta_type, 'BM_MixedChainIK')
                    bench_name = f"{bench_type}/{meta_type}/{dof}"
                    
                    converted_benchmarks.append({
                        'name': bench_name,
                        'real_time': stats['avg_time_ms'],
                        'success_rate': stats['success_rate'],
                        'avg_iter': stats['avg_iter'],
                        'avg_pos_err_mm': bm.get('avg_pos_err_mm', 0),
                        'avg_rot_err_deg': bm.get('avg_rot_err_deg', 0),
                        'avg_rev_err_deg': 0,
                        'avg_pris_err_mm': 0,
                    })
            
            # Overall entry (mixed cold+warm)
            bench_name = f"BM_MixedChainIK/overall/{dof}"
            converted_benchmarks.append({
                'name': bench_name,
                'real_time': bm['avg_time_ms'],
                'success_rate': bm['success_rate'],
                'avg_iter': bm['avg_iter'],
                'avg_pos_err_mm': bm.get('avg_pos_err_mm', 0),
                'avg_rot_err_deg': bm.get('avg_rot_err_deg', 0),
                'avg_rev_err_deg': 0,
                'avg_pris_err_mm': 0,
            })
        
        results = extract_metrics(converted_benchmarks)
        output_dir = results_file.parent
        
        plot_results(results, str(output_dir))
        generate_summary_table(results, str(output_dir))
        
        print(f"✓ Visualizations generated in: {output_dir}")
        
    except ImportError as e:
        print(f"Warning: Could not import visualization module: {e}")
        print("Skipping visualization generation.")
    except Exception as e:
        print(f"Error during visualization: {e}")
        import traceback
        traceback.print_exc()


def main():
    parser = argparse.ArgumentParser(
        description="Complete benchmark runner for urdfx IK",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    # Main operation modes
    parser.add_argument('--all', action='store_true', 
                       help='Run complete pipeline (generate + run + visualize)')
    parser.add_argument('--generate', action='store_true',
                       help='Only generate datasets')
    parser.add_argument('--run', action='store_true',
                       help='Only run benchmarks')
    parser.add_argument('--visualize', action='store_true',
                       help='Only generate visualizations')
    
    # Dataset generation options
    parser.add_argument('--dof-min', type=int, default=8,
                       help='Minimum DOF for generated robots (default: 8)')
    parser.add_argument('--dof-max', type=int, default=20,
                       help='Maximum DOF for generated robots (default: 20)')
    parser.add_argument('--samples', type=int, default=1000,
                       help='Number of samples per dataset (default: 1000)')
    parser.add_argument('--variations', action='store_true', default=True,
                       help='Generate with variations (cold/warm starts)')
    parser.add_argument('--no-variations', action='store_false', dest='variations',
                       help='Generate without variations')
    parser.add_argument('--save-binary', action='store_true',
                       help='Also save datasets in binary format for C++')
    
    # Benchmark options
    parser.add_argument('--dataset-dir', type=str,
                       help='Directory containing datasets (default: OUTPUT/datasets)')
    parser.add_argument('--max-iterations', type=int, default=500,
                       help='Maximum IK solver iterations (default: 500)')
    parser.add_argument('--tolerance', type=float, default=1e-6,
                       help='IK solver tolerance (default: 1e-6)')
    
    # Visualization options
    parser.add_argument('--results', type=str,
                       help='Path to benchmark results JSON file')
    
    # General options
    parser.add_argument('--output', '-o', type=str, default='.',
                       help='Output directory for all results')
    
    args = parser.parse_args()
    
    # Default to --all if no mode specified
    if not any([args.all, args.generate, args.run, args.visualize]):
        args.all = True
    
    # Print configuration
    print("urdfx IK Benchmark Runner")
    print("=" * 60)
    print(f"Output directory: {args.output}")
    print(f"Mode: {'Complete Pipeline' if args.all else 'Partial'}")
    
    results_file = None
    
    try:
        # Execute pipeline
        if args.all or args.generate:
            generate_datasets(args)
        
        if args.all or args.run:
            results_file = run_benchmarks(args)
        
        if args.all or args.visualize:
            visualize_results(args, results_file)
        
        print("\n" + "=" * 60)
        print("✓ All operations completed successfully!")
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n\nOperation cancelled by user.")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
