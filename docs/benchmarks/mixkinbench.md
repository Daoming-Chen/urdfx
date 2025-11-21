# Mixed-Chain Benchmark Suite (MixKinBench)

MixKinBench is a comprehensive benchmark suite for evaluating numerical Inverse Kinematics (IK) solvers on high-degree-of-freedom (DOF) serial chains with mixed joint types (revolute and prismatic).

## Overview

Unlike standard benchmarks that focus on 6-7 DOF industrial robots, MixKinBench evaluates performance on:
- **High DOF Chains**: 10, 20, 50, and up to 100 DOF.
- **Mixed Joint Types**: Randomly generated combinations of Revolute and Prismatic joints.
- **Complex Topologies**: Randomized link lengths and joint axes.

## Components

1.  **Synthetic Robot Generator**: Generates random URDF models with specified DOF and joint type probabilities.
2.  **Dataset Generator**: Creates benchmark datasets with reachable targets, ground truth, and initial guesses.
3.  **C++ Benchmark Runner**: Executes the IK solver against the datasets using Google Benchmark.
4.  **Reporting Tools**: Analyzes results and generates Markdown reports.

## Usage

### 1. Generate Datasets

Use the Python script to generate synthetic robots and benchmark datasets.

```bash
# Generate Tier B datasets (Synthetic Mixed Chains)
python3 bindings/python/benchmarks/mixkinbench/generate_dataset.py --output benchmarks/datasets
```

This will create URDF files and `.bin` dataset files in `benchmarks/datasets/`.

### 2. Run Benchmarks

Compile and run the C++ benchmarks.

```bash
# Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make mixed_ik_benchmarks

# Run and save output to JSON
./benchmarks/mixed_ik_benchmarks --benchmark_format=json > results.json
```

### 3. Generate Report

Convert the JSON results into a readable Markdown report.

```bash
python3 bindings/python/benchmarks/mixkinbench/report.py results.json report.md
```

## Dataset Format

The benchmark uses a custom binary format (`.bin`) for efficiency.
- **Magic**: `MKBN`
- **Header**: Version, NumCases, DOF
- **Data**: Target Position (3d), Target Rotation (9d), Initial Guess (Nd), Ground Truth (Nd)

## Extending

To add new robot types or scenarios, modify `bindings/python/benchmarks/mixkinbench/generator.py` or `dataset.py`.
