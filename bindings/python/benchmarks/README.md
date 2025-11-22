# urdfx Python Benchmarks

Comprehensive benchmarking suite for evaluating urdfx inverse kinematics (IK) solvers across diverse robot configurations.

## Overview

This package provides tools and scripts to benchmark IK solvers on:
- **Tier A**: Real-world robots (UR5e and variants, 6-10 DOF)
- **Tier B**: Synthetic mixed-chain robots (8-20 DOF, varied joint types)


## Quick Start

### Run All Benchmarks

```bash
# Run everything (Python + C++ benchmarks)
python run_all_benchmarks.py

# Quick test with fewer samples
python run_all_benchmarks.py --samples 500

# Skip C++ benchmarks
python run_all_benchmarks.py --skip-cpp
```

### Run Python Benchmarks Only

```bash
# Run both Tier A and Tier B
python run_benchmarks.py --all

# Run only Tier A (real-world robots)
python run_benchmarks.py --tier-a

# Run only Tier B (synthetic robots)
python run_benchmarks.py --tier-b

# With visualization
python run_benchmarks.py --all --visualize
```

### Run Specific Tiers

```bash
# Tier A: Real-world robots
python run_tier_a_benchmarks.py --samples 2000

# Tier B: Synthetic robots with custom DOF range
python run_tier_b_benchmarks.py --dof-min 10 --dof-max 15 --robots-per-dof 5
```

## Core Components

### 1. `MixedChainGenerator` (urdf_generator.py)

Generates synthetic serial kinematic chains with mixed joint types (revolute/prismatic).

```python
from urdfx.benchmarks import MixedChainGenerator

# Generate 12-DOF robot with 20% prismatic joints
generator = MixedChainGenerator(dof=12, prismatic_prob=0.2, seed=42)

# Export to URDF
generator.save_urdf("robot_12dof.urdf")

# Get robot statistics
stats = generator.get_statistics()
print(f"Revolute: {stats['num_revolute']}, Prismatic: {stats['num_prismatic']}")
```

### 2. `FKOracle` (oracle.py)

Forward kinematics wrapper for benchmarking.

```python
from urdfx.benchmarks import FKOracle
import urdfx

robot = urdfx.Robot.from_urdf_file("ur5e.urdf")
oracle = FKOracle(robot, end_link="tool0")

# Compute FK
position, rotation = oracle.compute_pose(q)
```

### 3. `JointSampler` (oracle.py)

Samples joint configurations within robot limits.

```python
from urdfx.benchmarks import JointSampler
import urdfx

robot = urdfx.Robot.from_urdf_file("ur5e.urdf")
sampler = JointSampler(robot, rng=np.random.RandomState(42))

# Uniform sampling
q_uniform = sampler.sample(n_samples=1000)

# Gaussian sampling (centered at range midpoint)
q_gaussian = sampler.sample_gaussian(n_samples=1000, sigma=0.5)
```

## Benchmark Tiers

### Tier A: Real-World Robots

Tests on industrial robots with varying DOF:
- **ur5e** (6 DOF): Standard UR5e
- **ur5e+x** (7 DOF): UR5e + 1 prismatic joint
- **ur5e+xy** (8 DOF): UR5e + 2 prismatic joints
- **ur5e+xyz** (10 DOF): UR5e + 3 prismatic joints + 1 revolute

**Metrics:**
- Success rate
- Average solve time
- Position error (mm)
- Orientation error (degrees)

### Tier B: Synthetic Robots

Tests on procedurally generated robots:
- DOF range: 8-20
- Mixed joint types (revolute/prismatic)
- Varied kinematic structures

**Metrics:**
- Success rate vs DOF
- Solve time vs DOF
- Scalability analysis

## Output

Results are saved to `benchmarks/results/`:

```
results/
├── ur5e_results.json              # Tier A individual results
├── ur5e+x_results.json
├── ur5e+xy_results.json
├── ur5e+xyz_results.json
├── tier_b_results.json            # Tier B aggregate results
├── tier_a_summary.png             # Visualizations
├── tier_b_success_rate.png
└── benchmark_summary.txt          # Text report
```

## Command-Line Options

### `run_benchmarks.py`

```
--all                 Run both Tier A and Tier B benchmarks
--tier-a              Run only Tier A benchmarks
--tier-b              Run only Tier B benchmarks
--samples N           Number of test samples (default: 1000)
--seed SEED           Random seed for reproducibility
--dof-min MIN         Tier B: Minimum DOF (default: 8)
--dof-max MAX         Tier B: Maximum DOF (default: 20)
--robots-per-dof N    Tier B: Robots per DOF level (default: 3)
--visualize           Generate plots after benchmarks
```

### `run_all_benchmarks.py`

```
--samples N           Number of test samples (default: 1000)
--skip-cpp            Skip C++ benchmarks
--visualize           Generate all visualizations
```

## Visualization

Generate plots from existing results:

```bash
# Python benchmarks
python visualize_benchmarks.py

# C++ benchmarks
python visualize_cpp_benchmarks.py

# Specify custom results directory
python visualize_benchmarks.py --results-dir /path/to/results
```

## Requirements

- Python 3.8+
- urdfx (with IK solver)
- NumPy
- matplotlib (for visualization)

## Development

### Adding New Robots

Add URDF files to `examples/models/` and update `run_tier_a_benchmarks.py`:

```python
robots = [
    ("ur5e", "examples/models/ur5e.urdf", "tool0"),
    ("my_robot", "examples/models/my_robot.urdf", "end_effector"),
]
```

### Custom Benchmark Metrics

Extend the oracle classes in `oracle.py` to track additional metrics:

```python
class CustomOracle(FKOracle):
    def compute_with_jacobian(self, q):
        # Custom metric computation
        pass
```

## Performance Tips

1. **Quick Testing**: Use `--samples 500` for rapid iteration
2. **Parallel Execution**: Run Tier A and Tier B in separate terminals
3. **Selective Testing**: Use `--dof-min` and `--dof-max` to focus on specific DOF ranges
4. **Reproducibility**: Always set `--seed` for consistent results

## Troubleshooting

### ImportError: No module named 'urdfx'

Ensure urdfx is installed:
```bash
cd bindings/python
pip install -e .
```

### Benchmark fails with "Could not determine end link"

Specify the end effector link explicitly in the benchmark script.

### Visualization generates empty plots

Check that JSON results exist in `benchmarks/results/`. Run benchmarks first.
