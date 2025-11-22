## ADDED Requirements

### Requirement: Benchmark Dataset Generation

The benchmark infrastructure SHALL provide tools to generate ground-truth datasets for IK evaluation.

#### Scenario: Generate reachable target dataset

- **WHEN** user runs dataset generation script with a robot URDF
- **THEN** N random joint configurations are sampled within joint limits
- **AND** forward kinematics is computed for each configuration (ground truth pose)
- **AND** dataset is saved as `{robot_name}_reachable_{N}samples.npz`

#### Scenario: FK oracle validation

- **WHEN** a dataset is generated using urdfx Python bindings as FK oracle
- **THEN** FK results match C++ implementation (position error < 1e-6 m, rotation error < 1e-6 rad)

#### Scenario: Generate initial guess variations

- **WHEN** a target pose is included in the dataset
- **THEN** multiple initial guesses are generated:
  - Cold start: $q_{init} = 0$ or random far from $q_{gt}$
  - Warm start: $q_{init} = q_{gt} + \mathcal{N}(0, \sigma^2)$ with $\sigma = 0.1$
  - Trajectory start: $q_{init}$ from previous waypoint

#### Scenario: Generate unreachable target dataset (optional)

- **WHEN** user requests unreachable targets
- **THEN** poses are sampled outside the workspace (e.g., beyond max reach)
- **AND** dataset is labeled as `unreachable` for testing "closest solution" behavior

### Requirement: Variable-DOF Benchmark Support

The C++ benchmark suite SHALL support robots with arbitrary degrees of freedom (6 to 100+).

#### Scenario: Load robot model dynamically

- **WHEN** a benchmark is run with a generated URDF file path
- **THEN** the robot is parsed and FK/IK solvers are instantiated
- **AND** DOF is detected automatically from the robot model

#### Scenario: Parametric benchmarks for DOF sweep

- **WHEN** user runs `BM_IK_MixedChain`
- **THEN** benchmarks are executed for DOF values {10, 20, 50, 100}
- **AND** results are aggregated by DOF

#### Scenario: Dataset loading from file

- **WHEN** a benchmark fixture loads a `.npz` dataset
- **THEN** target poses, initial guesses, and ground truth are extracted
- **AND** C++ Eigen matrices are populated from NumPy arrays (via intermediate JSON or binary format)

### Requirement: Performance Metrics Collection

The benchmark infrastructure SHALL collect comprehensive metrics for IK solver evaluation.

#### Scenario: Success rate measurement

- **WHEN** IK is solved for N targets
- **THEN** success rate is computed as: $\frac{\text{converged solutions}}{\text{total attempts}} \times 100\%$
- **AND** convergence is defined as: $\|FK(q_{sol}) - T_{target}\|_{pos} < 5 \times 10^{-4}$ m AND $\|FK(q_{sol}) - T_{target}\|_{rot} < 1 \times 10^{-3}$ rad

#### Scenario: Iteration count statistics

- **WHEN** IK solver is benchmarked
- **THEN** average, median, min, max iterations per solve are recorded
- **AND** iteration count is tracked separately for converged and failed attempts

#### Scenario: Execution time measurement

- **WHEN** IK solve is executed
- **THEN** wall-clock time is measured in microseconds
- **AND** time excludes dataset loading and result validation

#### Scenario: Position and rotation error tracking

- **WHEN** IK solution is obtained
- **THEN** position error (mm) is computed as: $1000 \times \|p_{achieved} - p_{target}\|_2$
- **AND** rotation error (deg) is computed as: $\frac{180}{\pi} \times \text{angularDistance}(q_{achieved}, q_{target})$

#### Scenario: Joint type sensitivity analysis

- **WHEN** benchmarking mixed-joint robots
- **THEN** errors are grouped by joint type (revolute vs prismatic)
- **AND** average error per joint type is reported

### Requirement: Cold Start vs Warm Start Comparison

The benchmark infrastructure SHALL evaluate the impact of initial guess quality on solver performance.

#### Scenario: Cold start benchmark

- **WHEN** user runs `BM_IK_ColdStart`
- **THEN** all IK solves use $q_{init} = 0$ or random initial guess
- **AND** warm start is disabled in solver configuration

#### Scenario: Warm start benchmark

- **WHEN** user runs `BM_IK_WarmStart`
- **THEN** IK solves reuse previous solution as initial guess
- **AND** first solve is primed outside measured region

#### Scenario: Initial guess quality impact

- **WHEN** benchmarks compare cold vs warm start
- **THEN** report shows:
  - Iteration count reduction (%)
  - Execution time speedup (×)
  - Success rate difference (%)

### Requirement: Trajectory Tracking Benchmark

The benchmark infrastructure SHALL evaluate IK solver performance on continuous trajectories.

#### Scenario: Generate trajectory dataset

- **WHEN** a trajectory dataset is created
- **THEN** N waypoints are sampled along a smooth path in joint space
- **AND** waypoints are separated by small increments (e.g., 0.08 rad per joint)

#### Scenario: Trajectory following benchmark

- **WHEN** user runs `BM_IK_Trajectory`
- **THEN** IK is solved sequentially for each waypoint
- **AND** previous solution is used as initial guess for next waypoint (warm start)
- **AND** metrics track cumulative error and convergence failure rate

### Requirement: Report Generation and Visualization

The benchmark infrastructure SHALL provide automated report generation with visualizations.

#### Scenario: Markdown report generation

- **WHEN** benchmarks are completed
- **THEN** a Markdown report is generated with:
  - Summary table (DOF vs success rate vs time)
  - Detailed metrics per benchmark
  - Timestamp and system information

#### Scenario: JSON benchmark output parsing

- **WHEN** Google Benchmark produces JSON output
- **THEN** Python script parses the JSON and extracts custom counters
- **AND** metrics are aggregated across multiple runs

#### Scenario: Interactive HTML visualization

- **WHEN** user runs report generation tool
- **THEN** an HTML file with Plotly/Matplotlib charts is created
- **AND** charts include:
  - Success rate vs DOF (line plot)
  - Iteration count distribution (histogram)
  - Execution time scaling (log-log plot)

#### Scenario: CI/CD integration

- **WHEN** benchmarks run in GitHub Actions
- **THEN** results are uploaded as workflow artifacts
- **AND** regression checks compare current results with baseline

### Requirement: Extensibility for Custom Robots

The benchmark infrastructure SHALL allow users to benchmark custom URDF files.

#### Scenario: Benchmark custom URDF

- **WHEN** user provides a custom URDF file path
- **THEN** the system generates dataset for that robot
- **AND** all benchmarks are executed without code modification

#### Scenario: Command-line interface for benchmark execution

- **WHEN** user runs `python -m mixkinbench run --urdf my_robot.urdf --samples 500`
- **THEN** dataset is generated, benchmarks are executed, and report is saved
- **AND** no manual C++ compilation is required for standard benchmarks

### Requirement: Documentation and Examples

The benchmark infrastructure SHALL provide comprehensive documentation and example workflows.

#### Scenario: Quick start guide

- **WHEN** user reads `docs/benchmarks/mixkinbench.md`
- **THEN** they can generate a dataset and run benchmarks within 5 minutes
- **AND** examples include both standard (UR5) and synthetic robots

#### Scenario: Python API reference

- **WHEN** user views Python module docstrings
- **THEN** all classes (`MixedChainGenerator`, `BenchmarkDataset`) have:
  - Class-level documentation
  - Parameter descriptions for all public methods
  - Usage examples

#### Scenario: Example benchmark workflow

- **WHEN** user runs provided example script `examples/python/run_mixkinbench.py`
- **THEN** a complete workflow executes:
  1. Generate a 20-DOF mixed robot
  2. Create dataset with 100 targets
  3. Run C++ benchmarks
  4. Generate HTML report

