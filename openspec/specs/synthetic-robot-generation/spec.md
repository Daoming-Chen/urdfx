# synthetic-robot-generation Specification

## Purpose
TBD - created by archiving change add-mixed-chain-benchmark-suite. Update Purpose after archive.
## Requirements
### Requirement: Mixed-Chain Generator Interface

The benchmark suite SHALL provide a programmatic interface to generate serial kinematic chains with configurable joint type distributions.

#### Scenario: Generate N-DOF robot with default parameters

- **WHEN** user calls `MixedChainGenerator(dof=20)`
- **THEN** a 20-DOF serial chain is generated
- **AND** joint types are randomly distributed (default: ~20-30% prismatic)
- **AND** the output is a valid URDF string

#### Scenario: Generate robot with fixed prismatic ratio

- **WHEN** user calls `MixedChainGenerator(dof=10, prismatic_prob=0.4)`
- **THEN** approximately 40% of joints are prismatic (±10% tolerance)
- **AND** the rest are revolute joints

#### Scenario: Reproducible generation with seed

- **WHEN** user calls `MixedChainGenerator(dof=15, seed=42)`
- **THEN** the same robot is generated every time with the same seed
- **AND** different seeds produce different topologies

### Requirement: Geometric Parameter Configuration

The generator SHALL produce physically plausible robots with bounded geometric parameters.

#### Scenario: Default link lengths

- **WHEN** a robot is generated with default settings
- **THEN** all link lengths are between 0.1 and 0.5 meters
- **AND** links are not degenerate (length > 1e-3 meters)

#### Scenario: Custom link length range

- **WHEN** user specifies `link_length_range=(0.2, 0.8)`
- **THEN** all generated links have lengths within [0.2, 0.8] meters

#### Scenario: Prismatic joint limits

- **WHEN** a prismatic joint is generated
- **THEN** it has bounded limits (default: -0.2 to 0.5 meters)
- **AND** it does NOT have infinite range (to prevent trivial solutions)

#### Scenario: Revolute joint limits

- **WHEN** a revolute joint is generated
- **THEN** it has limits (default: -π to π radians)
- **AND** joint axis is randomized (alternating X/Y/Z to ensure spatial complexity)

### Requirement: URDF Output Format

The generator SHALL produce valid URDF files compatible with urdfx parser.

#### Scenario: URDF structure validation

- **WHEN** a URDF is generated
- **THEN** it contains a root link and N joints
- **AND** it passes urdfx `URDFParser.parseString()` without errors
- **AND** joint names are unique (e.g., `joint_0`, `joint_1`, ...)
- **AND** link names are unique (e.g., `link_0`, `link_1`, ...)

#### Scenario: Joint type specification

- **WHEN** a revolute joint is generated
- **THEN** the URDF contains `<joint type="revolute">`
- **AND** axis is defined as `<axis xyz="..."/>`

#### Scenario: Prismatic joint specification

- **WHEN** a prismatic joint is generated
- **THEN** the URDF contains `<joint type="prismatic">`
- **AND** axis is defined (typically Z-axis for simplicity)

#### Scenario: Joint limits in URDF

- **WHEN** a URDF is generated
- **THEN** all joints have `<limit lower="..." upper="..." effort="..." velocity="..."/>` tags
- **AND** effort and velocity are set to reasonable defaults (10.0 Nm, 1.0 rad/s)

### Requirement: Python API Usability

The generator SHALL provide a convenient Python API for benchmark automation.

#### Scenario: Generate and save URDF to file

- **WHEN** user calls `generator.save_urdf("output/robot.urdf")`
- **THEN** a URDF file is written to the specified path
- **AND** the file is valid and can be parsed by urdfx

#### Scenario: Generate URDF as string

- **WHEN** user calls `generator.to_urdf_string()`
- **THEN** a URDF XML string is returned
- **AND** it does not write to filesystem

#### Scenario: Access generated robot metadata

- **WHEN** a robot is generated
- **THEN** user can query `generator.joint_types` (list of 'revolute'/'prismatic')
- **AND** user can query `generator.joint_limits` (list of (lower, upper) tuples)

### Requirement: Validation and Diagnostics

The generator SHALL provide validation tools to ensure generated robots are suitable for benchmarking.

#### Scenario: Validate generated URDF with urdfx

- **WHEN** a robot is generated
- **THEN** the generator can call `validate()` method
- **AND** it returns True if urdfx can parse and create a `Robot` object
- **AND** it returns False with error messages if parsing fails

#### Scenario: Visualize generated robot (optional)

- **WHEN** user calls `generator.visualize()` (if PyBullet or RViz is available)
- **THEN** a 3D visualization of the robot is displayed
- **AND** link connections are correct

#### Scenario: Export robot statistics

- **WHEN** user calls `generator.get_statistics()`
- **THEN** a dictionary is returned with:
  - `total_dof`: total degrees of freedom
  - `num_revolute`: number of revolute joints
  - `num_prismatic`: number of prismatic joints
  - `total_chain_length`: sum of all link lengths

### Requirement: Batch Generation Support

The generator SHALL support batch generation for dataset creation.

#### Scenario: Generate multiple robots with varying DOF

- **WHEN** user calls `MixedChainGenerator.batch_generate(dof_list=[10, 20, 50], count=3)`
- **THEN** 3 robots are generated for each DOF value (total 9 robots)
- **AND** each robot has a unique seed for reproducibility

#### Scenario: Generate with systematic joint ratio sweep

- **WHEN** user specifies `prismatic_prob_range=[0.1, 0.2, 0.3, 0.4]`
- **THEN** robots are generated with each specified prismatic ratio
- **AND** results are returned as a list of `(generator, metadata)` tuples

