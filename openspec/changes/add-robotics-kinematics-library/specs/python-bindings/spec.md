# Capability: Python Bindings

## ADDED Requirements

### Requirement: System SHALL expose C++ API to Python via nanobind
The system SHALL provide Python bindings using nanobind for all core functionality.

#### Scenario: Import urdfx module in Python
**Given** the urdfx library is installed  
**When** the user runs `import urdfx` in Python  
**Then** the module loads without errors  
**And** all core classes are accessible (Robot, ForwardKinematics, JacobianCalculator, IKSolver)

#### Scenario: Load robot from URDF in Python
**Given** a URDF file path "ur5_urdf/ur5e.urdf"  
**When** the user calls `robot = urdfx.Robot.from_urdf("ur5_urdf/ur5e.urdf")`  
**Then** a Robot object is returned  
**And** the robot can be queried for links and joints

### Requirement: System SHALL support NumPy array interoperability
The system SHALL accept and return NumPy arrays for joint angles, poses, and matrices.

#### Scenario: Compute FK with NumPy array input
**Given** a Robot and ForwardKinematics object  
**And** a NumPy array q = np.array([0, -1.57, 0, 0, 0, 0])  
**When** the user calls `pose = fk.compute(q)`  
**Then** the function accepts the NumPy array  
**And** returns a 4×4 NumPy array representing the pose matrix

#### Scenario: Compute Jacobian returns NumPy array
**Given** a JacobianCalculator and joint angles as NumPy array  
**When** the user calls `J = jacobian_calc.compute(q)`  
**Then** the system returns a 6×6 NumPy array  
**And** the array is writable and can be modified

#### Scenario: Zero-copy NumPy integration
**Given** a large NumPy array of joint angles  
**When** passed to C++ functions via nanobind  
**Then** no data copying occurs (zero-copy)  
**And** the C++ code operates directly on NumPy buffer

### Requirement: System SHALL provide Pythonic API with snake_case naming
The system SHALL follow Python naming conventions for better integration.

#### Scenario: Python API uses snake_case
**Given** the C++ API has CamelCase methods  
**When** accessed from Python  
**Then** methods are exposed with snake_case names  
**And** `forwardKinematics.compute()` becomes `forward_kinematics.compute()`

### Requirement: System SHALL provide type stubs for IDE support
The system SHALL include .pyi type stub files for static type checking.

#### Scenario: Type checking with mypy
**Given** Python code using urdfx with type hints  
**When** the user runs `mypy script.py`  
**Then** mypy recognizes urdfx types  
**And** type errors are caught at check time  
**And** IDE autocomplete works correctly

### Requirement: System SHALL handle exceptions across language boundary
The system SHALL translate C++ exceptions to Python exceptions.

#### Scenario: URDF parse error raises Python exception
**Given** an invalid URDF file  
**When** Python code calls `Robot.from_urdf("invalid.urdf")`  
**Then** a Python `URDFParseError` exception is raised  
**And** the exception message contains the C++ error details  
**And** the Python traceback is preserved

### Requirement: System SHALL support Python context managers
The system SHALL support Python's context manager protocol for resource management.

#### Scenario: Use Robot as context manager
**Given** a Robot object  
**When** the user uses `with urdfx.Robot.from_urdf(...) as robot:`  
**Then** the robot is properly initialized  
**And** resources are cleaned up on context exit

### Requirement: System SHALL provide installable package via pip
The system SHALL be installable as a Python package.

#### Scenario: Install via pip
**Given** the urdfx project with setup.py  
**When** the user runs `pip install .`  
**Then** the package is installed to site-packages  
**And** `import urdfx` works from any directory

#### Scenario: Install wheel distribution
**Given** a built wheel file urdfx-1.0.0-cp38-cp38-linux_x86_64.whl  
**When** the user runs `pip install urdfx-1.0.0-*.whl`  
**Then** the binary package installs without compilation  
**And** all dependencies are satisfied

### Requirement: System SHALL support multiple Python versions
The system SHALL support Python 3.8 through 3.12.

#### Scenario: Test on Python 3.8
**Given** Python 3.8 environment  
**When** urdfx is installed and tested  
**Then** all tests pass  
**And** NumPy 1.20+ compatibility is verified

#### Scenario: Test on Python 3.12
**Given** Python 3.12 environment  
**When** urdfx is installed and tested  
**Then** all tests pass  
**And** nanobind is compatible with Python 3.12

### Requirement: System SHALL efficient memory management
The system SHALL minimize memory overhead in Python bindings.

#### Scenario: Robot object memory footprint
**Given** a Robot loaded in Python  
**When** memory usage is measured  
**Then** the Python object overhead is < 1KB  
**And** the C++ object is referenced (not copied)

#### Scenario: No memory leaks across calls
**Given** 10,000 FK computations in Python  
**When** monitored with memory profiler  
**Then** memory usage remains constant  
**And** no memory leaks are detected

### Requirement: System SHALL provide comprehensive documentation
The system SHALL provide documentation for Python API.

#### Scenario: Access docstrings
**Given** any urdfx Python class or function  
**When** the user calls `help(urdfx.Robot)`  
**Then** detailed docstrings are displayed  
**And** parameter types and return types are documented  
**And** usage examples are included
