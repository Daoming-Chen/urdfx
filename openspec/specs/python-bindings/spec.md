# python-bindings Specification

## Purpose
TBD - created by archiving change add-python-bindings. Update Purpose after archive.
## Requirements
### Requirement: URDF Parser Binding

The Python bindings SHALL expose URDF parsing functionality through a `Robot` class with factory methods.

#### Scenario: Parse URDF from file

- **WHEN** user calls `Robot.from_urdf("path/to/robot.urdf")`
- **THEN** a Robot instance is returned with parsed robot model
- **AND** all links, joints, and properties are accessible

#### Scenario: Parse URDF from string

- **WHEN** user calls `Robot.from_urdf_string(urdf_xml_string)`
- **THEN** a Robot instance is returned with parsed robot model

#### Scenario: Invalid URDF file

- **WHEN** user provides non-existent file path
- **THEN** a `FileNotFoundError` is raised

#### Scenario: Malformed URDF XML

- **WHEN** user provides invalid URDF XML
- **THEN** a `RuntimeError` is raised with descriptive error message

### Requirement: Robot Model Access

The Python bindings SHALL provide access to robot model properties including links, joints, and metadata.

#### Scenario: Query robot name

- **WHEN** user accesses `robot.name`
- **THEN** the robot name from URDF is returned

#### Scenario: Get joint names

- **WHEN** user calls `robot.get_joint_names()`
- **THEN** a list of all actuated joint names is returned in order

#### Scenario: Get degrees of freedom

- **WHEN** user accesses `robot.dof`
- **THEN** the number of actuated joints is returned

#### Scenario: Get joint limits

- **WHEN** user calls `robot.get_joint_limits()`
- **THEN** a tuple of (lower_limits, upper_limits) NumPy arrays is returned

#### Scenario: Access link by name

- **WHEN** user calls `robot.get_link("link_name")`
- **THEN** a Link object is returned if exists
- **AND** raises `KeyError` if link does not exist

### Requirement: Transform Representation

The Python bindings SHALL provide a `Transform` class for representing 3D transformations.

#### Scenario: Create transform from position and quaternion

- **WHEN** user calls `Transform.from_position_quaternion(pos, quat)`
- **THEN** a Transform instance is created
- **AND** `pos` is a 3-element array-like (x, y, z)
- **AND** `quat` is a 4-element array-like (w, x, y, z)

#### Scenario: Extract transformation matrix

- **WHEN** user accesses `transform.matrix()`
- **THEN** a 4x4 NumPy array representing the homogeneous transformation is returned

#### Scenario: Extract position and quaternion

- **WHEN** user accesses `transform.position` and `transform.quaternion`
- **THEN** position is returned as NumPy array [x, y, z]
- **AND** quaternion is returned as NumPy array [w, x, y, z]

### Requirement: Forward Kinematics Computation

The Python bindings SHALL provide forward kinematics computation through a `ForwardKinematics` class.

#### Scenario: Create FK solver

- **WHEN** user instantiates `ForwardKinematics(robot, end_link)`
- **THEN** a FK solver is created for the kinematic chain to `end_link`

#### Scenario: Compute end-effector pose

- **WHEN** user calls `fk.compute(joint_angles)`
- **THEN** a Transform representing end-effector pose is returned
- **AND** `joint_angles` is a NumPy array matching DOF

#### Scenario: Invalid joint angles size

- **WHEN** user provides joint_angles with incorrect size
- **THEN** a `ValueError` is raised with descriptive message

#### Scenario: Joint bounds checking

- **WHEN** user calls `fk.compute(joint_angles, check_bounds=True)`
- **AND** any joint is out of limits
- **THEN** a `RuntimeError` is raised indicating which joint is out of bounds

### Requirement: Jacobian Computation

The Python bindings SHALL provide Jacobian matrix computation through a `JacobianCalculator` class.

#### Scenario: Create Jacobian calculator

- **WHEN** user instantiates `JacobianCalculator(robot, end_link)`
- **THEN** a Jacobian calculator is created

#### Scenario: Compute Jacobian matrix

- **WHEN** user calls `jacobian.compute(joint_angles)`
- **THEN** a 6×n NumPy array is returned (n = number of joints)
- **AND** first 3 rows represent linear velocity Jacobian
- **AND** last 3 rows represent angular velocity Jacobian

#### Scenario: Check singularity

- **WHEN** user calls `jacobian.is_singular(joint_angles)`
- **THEN** a boolean is returned indicating if configuration is singular

#### Scenario: Compute manipulability

- **WHEN** user calls `jacobian.get_manipulability(joint_angles)`
- **THEN** a scalar manipulability measure is returned

### Requirement: Inverse Kinematics Solving

The Python bindings SHALL provide IK solving through a `SQPIKSolver` class.

#### Scenario: Create IK solver

- **WHEN** user instantiates `SQPIKSolver(robot, end_link)`
- **THEN** an IK solver is created

#### Scenario: Solve IK problem

- **WHEN** user calls `result = ik_solver.solve(target_pose, initial_guess)`
- **THEN** an `IKResult` object is returned
- **AND** `result.converged` indicates success
- **AND** `result.solution` contains joint angles as NumPy array
- **AND** `result.iterations` contains iteration count

#### Scenario: Configure solver parameters

- **WHEN** user sets `ik_solver.tolerance = 1e-5`
- **THEN** solver uses updated tolerance in subsequent solves

#### Scenario: Position-only IK

- **WHEN** user calls `ik_solver.set_position_only(True)`
- **AND** calls `ik_solver.solve(target_pose, initial_guess)`
- **THEN** solver only matches position, ignoring orientation

### Requirement: NumPy Integration

The Python bindings SHALL seamlessly integrate with NumPy for array inputs and outputs.

#### Scenario: Accept NumPy arrays

- **WHEN** user passes NumPy arrays as joint angles
- **THEN** they are accepted without requiring conversion

#### Scenario: Accept Python lists

- **WHEN** user passes Python lists as joint angles
- **THEN** they are automatically converted to appropriate format

#### Scenario: Return NumPy arrays

- **WHEN** any function returns array data
- **THEN** it returns NumPy arrays with appropriate dtype

### Requirement: Type Hints

The Python bindings SHALL provide complete type hints for all public APIs.

#### Scenario: IDE autocompletion

- **WHEN** user types code in IDE with type checking enabled
- **THEN** IDE provides accurate autocompletion and type checking

#### Scenario: mypy validation

- **WHEN** user runs mypy on code using urdfx
- **THEN** mypy validates types correctly without errors

### Requirement: Documentation Strings

The Python bindings SHALL provide comprehensive docstrings for all classes and methods.

#### Scenario: Access help

- **WHEN** user calls `help(urdfx.ForwardKinematics)`
- **THEN** detailed documentation is displayed

#### Scenario: Parameter documentation

- **WHEN** user calls `help(fk.compute)`
- **THEN** all parameters and return values are documented

### Requirement: Error Messages

The Python bindings SHALL provide clear, actionable error messages.

#### Scenario: Descriptive exceptions

- **WHEN** an error occurs
- **THEN** the exception message clearly describes the problem
- **AND** includes relevant context (e.g., which joint, expected vs actual)

### Requirement: Build System Integration

The Python bindings SHALL integrate with standard Python build tools.

#### Scenario: Install from source

- **WHEN** user runs `pip install .` in bindings/python directory
- **THEN** the urdfx package is built and installed

#### Scenario: Development mode installation

- **WHEN** user runs `pip install -e .`
- **THEN** the package is installed in editable mode

#### Scenario: CMake build integration

- **WHEN** CMake is configured with Python bindings enabled
- **THEN** the Python module is built as part of the main build

### Requirement: Test Coverage

The Python bindings SHALL have comprehensive test coverage using pytest.

#### Scenario: Unit tests for all classes

- **WHEN** tests are run with `pytest bindings/python/tests/`
- **THEN** all core classes have unit tests
- **AND** test coverage is > 90%

#### Scenario: Integration tests

- **WHEN** integration tests are run
- **THEN** full FK→IK roundtrip tests pass
- **AND** results match C++ implementation within numerical tolerance

#### Scenario: Numerical accuracy tests

- **WHEN** Python results are compared to C++ results
- **THEN** differences are < 1e-10 for same inputs

### Requirement: Performance Benchmarks

The Python bindings SHALL provide performance benchmarks to measure overhead.

#### Scenario: FK performance measurement

- **WHEN** user runs FK benchmark
- **THEN** time per FK computation is measured
- **AND** overhead vs C++ is reported

#### Scenario: IK performance measurement

- **WHEN** user runs IK benchmark
- **THEN** time per IK solve is measured
- **AND** iterations and convergence rate are reported

#### Scenario: Python overhead within limits

- **WHEN** benchmarks are run
- **THEN** Python binding overhead is < 10% compared to C++

### Requirement: Example Code

The Python bindings SHALL include example code demonstrating common use cases.

#### Scenario: Basic FK example

- **WHEN** user reads examples/python/forward_kinematics.py
- **THEN** a complete FK example is provided

#### Scenario: IK example

- **WHEN** user reads examples/python/inverse_kinematics.py
- **THEN** a complete IK example with visualization hints is provided

#### Scenario: Trajectory generation example

- **WHEN** user reads examples/python/trajectory.py
- **THEN** an example showing multiple IK solves with warm-starting is provided

