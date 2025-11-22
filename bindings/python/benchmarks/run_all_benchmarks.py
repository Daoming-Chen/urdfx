#!/usr/bin/env python3
"""
Master benchmark runner - runs ALL urdfx benchmarks (Python + C++).

This script:
1. Runs Python Tier A benchmarks (real-world robots)
2. Runs Python Tier B benchmarks (synthetic robots)
3. Runs C++ IK benchmarks
4. Runs C++ Jacobian benchmarks
5. Generates all visualizations

Usage:
    python run_all_benchmarks.py
    python run_all_benchmarks.py --samples 500  # Fewer samples for quick test
    python run_all_benchmarks.py --skip-cpp     # Skip C++ benchmarks
"""

import os
import sys
import subprocess
import argparse
import time
from pathlib import Path

# Add current directory to path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

# Determine project root
project_root = os.path.abspath(os.path.join(current_dir, "../../.."))
results_dir = os.path.join(project_root, "benchmarks/results")
cpp_build_dir = os.path.join(project_root, "build/benchmarks")


def run_python_benchmarks(args):
    """Run all Python benchmarks."""
    print("\n" + "="*70)
    print("RUNNING PYTHON BENCHMARKS")
    print("="*70)
    
    cmd = [sys.executable, "run_benchmarks.py", "--all"]
    
    if args.samples:
        cmd.extend(["--samples", str(args.samples)])
    if args.visualize:
        cmd.append("--visualize")
    
    result = subprocess.run(cmd, cwd=current_dir)
    return result.returncode == 0


def run_cpp_benchmarks():
    """Run all C++ benchmarks."""
    print("\n" + "="*70)
    print("RUNNING C++ BENCHMARKS")
    print("="*70)
    
    if not os.path.exists(cpp_build_dir):
        print(f"Warning: C++ build directory not found: {cpp_build_dir}")
        print("Skipping C++ benchmarks. Build the project first with CMake.")
        return True
    
    success = True
    
    # Run IK benchmarks
    ik_executable = os.path.join(cpp_build_dir, "ik_benchmarks")
    if os.path.exists(ik_executable):
        print("\nRunning C++ IK benchmarks...")
        output_file = os.path.join(results_dir, "ik_benchmarks_latest.json")
        result = subprocess.run([
            ik_executable,
            "--benchmark_format=json",
            f"--benchmark_out={output_file}"
        ], cwd=cpp_build_dir)
        
        if result.returncode == 0:
            print("✓ C++ IK benchmarks completed")
        else:
            print("✗ C++ IK benchmarks failed")
            success = False
    else:
        print(f"Warning: IK benchmark executable not found: {ik_executable}")
    
    # Run Jacobian benchmarks
    jacobian_executable = os.path.join(cpp_build_dir, "jacobian_benchmarks")
    if os.path.exists(jacobian_executable):
        print("\nRunning C++ Jacobian benchmarks...")
        output_file = os.path.join(results_dir, "jacobian_benchmarks_latest.json")
        result = subprocess.run([
            jacobian_executable,
            "--benchmark_format=json",
            f"--benchmark_out={output_file}"
        ], cwd=cpp_build_dir)
        
        if result.returncode == 0:
            print("✓ C++ Jacobian benchmarks completed")
        else:
            print("✗ C++ Jacobian benchmarks failed")
            success = False
    else:
        print(f"Warning: Jacobian benchmark executable not found: {jacobian_executable}")
    
    return success


def run_visualizations():
    """Generate all visualizations."""
    print("\n" + "="*70)
    print("GENERATING VISUALIZATIONS")
    print("="*70)
    
    success = True
    
    # Python benchmarks visualization
    print("\nGenerating Python benchmark visualizations...")
    result = subprocess.run([
        sys.executable, "visualize_benchmarks.py",
        "--results-dir", results_dir
    ], cwd=current_dir)
    
    if result.returncode == 0:
        print("✓ Python benchmark visualizations completed")
    else:
        print("✗ Python benchmark visualizations failed")
        success = False
    
    # C++ benchmarks visualization
    print("\nGenerating C++ benchmark visualizations...")
    result = subprocess.run([
        sys.executable, "visualize_cpp_benchmarks.py",
        "--results-dir", results_dir
    ], cwd=current_dir)
    
    if result.returncode == 0:
        print("✓ C++ benchmark visualizations completed")
    else:
        print("✗ C++ benchmark visualizations failed")
        success = False
    
    return success


def main():
    parser = argparse.ArgumentParser(
        description="Master benchmark runner for urdfx (Python + C++)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    parser.add_argument('--samples', '-n', type=int,
                       help='Number of test samples per configuration (default: 1000)')
    parser.add_argument('--skip-python', action='store_true',
                       help='Skip Python benchmarks')
    parser.add_argument('--skip-cpp', action='store_true',
                       help='Skip C++ benchmarks')
    parser.add_argument('--skip-viz', action='store_true',
                       help='Skip visualization generation')
    parser.add_argument('--visualize', action='store_true', default=True,
                       help='Generate visualizations (default: True)')
    parser.add_argument('--no-visualize', dest='visualize', action='store_false',
                       help='Do not generate visualizations')
    
    args = parser.parse_args()
    
    # Print header
    print("\n" + "="*70)
    print("urdfx MASTER BENCHMARK RUNNER")
    print("="*70)
    print(f"Project root: {project_root}")
    print(f"Results directory: {results_dir}")
    print(f"Python benchmarks: {'SKIP' if args.skip_python else 'RUN'}")
    print(f"C++ benchmarks: {'SKIP' if args.skip_cpp else 'RUN'}")
    print(f"Visualizations: {'SKIP' if args.skip_viz else 'GENERATE'}")
    print("="*70)
    
    # Ensure results directory exists
    os.makedirs(results_dir, exist_ok=True)
    
    overall_start = time.time()
    success = True
    
    # Run Python benchmarks
    if not args.skip_python:
        if not run_python_benchmarks(args):
            success = False
            print("\n✗ Python benchmarks failed")
    
    # Run C++ benchmarks
    if not args.skip_cpp:
        if not run_cpp_benchmarks():
            success = False
            print("\n✗ C++ benchmarks failed")
    
    # Generate visualizations
    if not args.skip_viz and args.visualize:
        if not run_visualizations():
            success = False
            print("\n✗ Visualization generation failed")
    
    overall_elapsed = time.time() - overall_start
    
    # Final summary
    print("\n" + "="*70)
    if success:
        print("✓✓✓ ALL BENCHMARKS AND VISUALIZATIONS COMPLETED SUCCESSFULLY ✓✓✓")
    else:
        print("✗✗✗ SOME BENCHMARKS OR VISUALIZATIONS FAILED ✗✗✗")
    print(f"Total time: {overall_elapsed:.2f}s")
    print("="*70)
    print(f"\nResults available in: {results_dir}")
    print("\nGenerated files:")
    print("  - benchmark_summary.md (Python benchmarks)")
    print("  - cpp_benchmark_summary.md (C++ benchmarks)")
    print("  - tier_a_visualization.png/pdf (Python Tier A)")
    print("  - tier_b_visualization.png/pdf (Python Tier B)")
    print("  - cpp_ik_benchmarks.png/pdf (C++ IK)")
    print("  - cpp_jacobian_benchmarks.png/pdf (C++ Jacobian)")
    print("="*70 + "\n")
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
