# Capability: Jacobian Computation

## ADDED Requirements

### Requirement: System SHALL compute geometric Jacobian using automatic differentiation
The system SHALL compute the geometric Jacobian matrix using CppAD's automatic differentiation.

#### Scenario: Compute Jacobian for UR5e at zero configuration
**Given** a UR5e robot model at q = [0, 0, 0, 0, 0, 0]  
**When** the user calls `jacobianCalculator.compute(q)`  
**Then** the system returns a 6×6 Jacobian matrix  
**And** the top 3 rows represent linear velocity mapping  
**And** the bottom 3 rows represent angular velocity mapping

#### Scenario: Jacobian matches numerical differentiation
**Given** a robot model and joint angles q  
**When** the system computes the Jacobian using CppAD  
**Then** the result matches numerical differentiation within 1e-6 tolerance  
**And** the computation is faster than numerical differentiation

### Requirement: System SHALL tape FK computation for efficient Jacobian evaluation
The system SHALL create a CppAD tape of the FK computation for reuse across multiple Jacobian evaluations.

#### Scenario: Reuse CppAD tape for multiple Jacobian computations
**Given** a JacobianCalculator initialized with a robot model  
**When** the user calls compute(q) 100 times with different joint angles  
**Then** the CppAD tape is created only once during initialization  
**And** subsequent calls only evaluate the tape (no re-taping)  
**And** average computation time is < 0.5ms per call

### Requirement: System SHALL support both geometric and analytic Jacobians
The system SHALL compute both geometric (body) and analytic (spatial) Jacobians.

#### Scenario: Compute geometric Jacobian
**Given** a robot model  
**When** the user requests a geometric Jacobian  
**Then** the system returns J_geom that maps joint velocities to body twist  
**And** J_geom relates dq/dt to [v_linear, ω_angular] in end-effector frame

#### Scenario: Compute analytic Jacobian
**Given** a robot model  
**When** the user requests an analytic Jacobian  
**Then** the system returns J_analytic that maps joint velocities to spatial twist  
**And** J_analytic relates dq/dt to [v_linear, ω_angular] in base frame

### Requirement: System SHALL compute Jacobian to intermediate links
The system SHALL compute Jacobians for any link in the kinematic chain.

#### Scenario: Jacobian to intermediate link
**Given** a UR5e robot model  
**When** the user requests Jacobian to "forearm_link"  
**Then** the returned Jacobian has dimensions 6×n where n is the number of joints up to that link  
**And** the Jacobian excludes downstream joints

### Requirement: System SHALL handle singular configurations
The system SHALL detect and report singular configurations.

#### Scenario: Detect singularity
**Given** a robot in a singular configuration (e.g., UR5e elbow fully extended)  
**When** the user computes the Jacobian  
**Then** the system computes the Jacobian normally  
**And** provides a method to check singularity via `isSingular()`  
**And** `isSingular()` returns true if smallest singular value < threshold

#### Scenario: Query manipulability measure
**Given** a computed Jacobian matrix J  
**When** the user calls `getManipulability()`  
**Then** the system returns sqrt(det(J*J^T))  
**And** the value is close to 0 near singularities

### Requirement: System SHALL provide Jacobian derivatives (Hessian)
The system SHALL optionally compute second-order derivatives for acceleration analysis.

#### Scenario: Compute Jacobian time derivative
**Given** a robot model, joint angles q, and joint velocities dq  
**When** the user calls `computeJacobianDerivative(q, dq)`  
**Then** the system returns dJ/dt using second-order CppAD tape  
**And** the result is used for acceleration-level IK

### Requirement: System SHALL efficient memory management
The system SHALL minimize dynamic allocation during Jacobian computation.

#### Scenario: Zero allocation after initialization
**Given** a JacobianCalculator initialized once  
**When** compute(q) is called repeatedly  
**Then** no heap allocations occur during compute()  
**And** all matrices are pre-allocated and reused

### Requirement: System SHALL ensure numerical stability
The system SHALL use numerically stable algorithms for Jacobian computation.

#### Scenario: Jacobian for small joint angles
**Given** a robot with very small joint angles (< 1e-10 rad)  
**When** the Jacobian is computed  
**Then** the result has no NaN or Inf values  
**And** the relative error compared to analytical solution is < 1e-6
