# Capability: Jacobian Computation

## MODIFIED Requirements

### Requirement: Jacobian Matrix Computation
**ID**: JAC-001  
**Priority**: P0

The system SHALL compute the Jacobian matrix relating joint velocities to end-effector twist using **analytical geometric methods** based on the kinematic chain structure.

**Change from previous**: Previously used CppAD automatic differentiation. Now uses hand-crafted analytical formulas for improved performance.

#### Scenario: Compute Spatial Jacobian for Revolute Chain
**Given** a robot with 6 revolute joints  
**And** joint configuration q = [0.1, -0.5, 0.3, 1.2, -0.8, 0.6] rad  
**When** computing the spatial Jacobian for the end-effector  
**Then** the result SHALL be a 6×6 matrix J  
**And** each column i SHALL be computed as:
- Linear part: `J[0:3, i] = z_i × (p_ee - p_i)`
- Angular part: `J[3:6, i] = z_i`  

Where:
- `z_i` is the joint axis direction in world frame
- `p_i` is the joint position in world frame
- `p_ee` is the end-effector position in world frame

**And** computation time SHALL be less than 5µs on modern x86-64 CPU (i7-12700K or equivalent)

---

### Requirement: Prismatic Joint Support
**ID**: JAC-002  
**Priority**: P0

The system SHALL correctly compute Jacobian columns for prismatic joints.

#### Scenario: Compute Jacobian Column for Prismatic Joint
**Given** a kinematic chain containing a prismatic joint  
**And** the joint translates along axis z_i in world frame  
**When** computing the Jacobian column for that joint  
**Then** the linear part SHALL be `J[0:3, i] = z_i`  
**And** the angular part SHALL be `J[3:6, i] = [0, 0, 0]`  
**And** the result SHALL match numerical differentiation within 1e-6 tolerance

---

### Requirement: Jacobian Type Conversion
**ID**: JAC-003  
**Priority**: P0

The system SHALL support conversion between Analytic (spatial) and Geometric (body) Jacobian representations.

**Change from previous**: No change to this requirement, but underlying implementation now uses analytical base Jacobian.

#### Scenario: Convert Spatial to Body Jacobian
**Given** a spatial Jacobian J_spatial computed at configuration q  
**And** the end-effector pose T_ee = FK(q)  
**When** requesting Geometric Jacobian type  
**Then** the system SHALL return `J_body = Adjoint^{-1}(T_ee) * J_spatial`  
**And** the conversion SHALL use the existing adjoint matrix computation  
**And** the result SHALL satisfy: `twist_body = J_body * q_dot`

---

### Requirement: Numerical Accuracy
**ID**: JAC-004  
**Priority**: P0

The analytical Jacobian computation SHALL maintain numerical accuracy equivalent to or better than the previous CppAD implementation.

**Change from previous**: New requirement to ensure migration doesn't degrade accuracy.

#### Scenario: Validate Against Finite Difference
**Given** a random joint configuration q  
**And** a numerical Jacobian J_numerical computed via central difference with ε = 1e-7  
**When** computing analytical Jacobian J_analytical  
**Then** the Frobenius norm error SHALL be less than 1e-4  
**And** the test SHALL pass for 10,000 random configurations

#### Scenario: Validate Against CppAD Reference (Migration Only)
**Given** the old CppAD-based implementation is available  
**And** a random joint configuration q  
**When** computing Jacobian with both methods  
**Then** the Frobenius norm error SHALL be less than 1e-6  
**And** the test SHALL pass for 10,000 random configurations  
**And** this test is used only during migration validation period

---

### Requirement: Performance Targets
**ID**: JAC-005  
**Priority**: P1

The analytical Jacobian implementation SHALL achieve significant performance improvements over the CppAD-based implementation.

**Change from previous**: New requirement defining performance expectations.

#### Scenario: Jacobian Computation Performance
**Given** a UR5e robot with 6 DOF  
**And** a representative joint configuration  
**When** computing the Jacobian 10,000 times in a benchmark loop  
**Then** the mean computation time SHALL be less than 5µs per call  
**And** the 95th percentile SHALL be less than 8µs  
**And** the performance SHALL be at least 5× faster than the previous CppAD implementation

#### Scenario: End-to-End IK Performance
**Given** an IK problem with convergence requiring 10-15 iterations  
**And** using the SQP solver with DaQP QP backend  
**When** solving from a cold start  
**Then** the total solve time SHALL be less than 100µs  
**And** the performance SHALL be at least 2× faster than the previous implementation

---

### Requirement: Memory Efficiency
**ID**: JAC-006  
**Priority**: P1

The analytical implementation SHALL reduce memory usage compared to CppAD tape storage.

**Change from previous**: New requirement focused on memory improvements.

#### Scenario: No Tape Storage Overhead
**Given** a JacobianCalculator instance for multiple end-effector links  
**When** the calculator is initialized  
**Then** no CppAD tape SHALL be stored in memory  
**And** the only cached data SHALL be kinematic chain structures  
**And** memory usage SHALL be reduced by at least 80% compared to CppAD implementation

#### Scenario: Minimal Runtime Allocation
**Given** a pre-warmed JacobianCalculator instance  
**When** computing Jacobian for a known end-effector link  
**Then** zero heap allocations SHALL occur during computation  
**And** all workspace SHALL be pre-allocated in frame cache

---

## REMOVED Requirements

### Requirement: CppAD Tape Optimization
**ID**: JAC-REMOVED-001  
**Removed**: This proposal

The system previously required CppAD tape optimization to be called after tape creation. This requirement is obsolete as CppAD is being removed.

**Rationale**: Analytical implementation has no tape concept.

---

### Requirement: Thread-Safe Tape Access
**ID**: JAC-REMOVED-002  
**Removed**: This proposal

The system previously required mutex protection around CppAD tape evaluation due to non-thread-safe tape data structures.

**Rationale**: Analytical implementation can be made fully thread-safe by using thread-local frame caches.

---

## ADDED Requirements

### Requirement: Support for Intermediate Link Jacobians
**ID**: JAC-007  
**Priority**: P0

The system SHALL compute Jacobians for any link in the kinematic chain, not just the end-effector.

**New requirement**: Clarifies existing behavior that must be preserved.

#### Scenario: Compute Jacobian to Mid-Chain Link
**Given** a UR5e robot with links [base, shoulder, upper_arm, forearm, wrist_1, wrist_2, wrist_3, tool0]  
**And** joint configuration q = [q1, q2, q3, q4, q5, q6]  
**When** computing Jacobian for target_link = "wrist_2"  
**Then** the result SHALL be a 6×5 matrix (5 joints contribute)  
**And** the Jacobian SHALL relate joint velocities [q1, q2, q3, q4, q5] to wrist_2 twist  
**And** joints after wrist_2 SHALL not contribute to the Jacobian

---

### Requirement: API Backward Compatibility
**ID**: JAC-008  
**Priority**: P0

The new analytical implementation SHALL maintain 100% API compatibility with the existing JacobianCalculator interface.

**New requirement**: Ensures drop-in replacement for existing code.

#### Scenario: Existing Code Works Without Changes
**Given** existing code using `JacobianCalculator calculator(robot, "tool0")`  
**And** code calling `MatrixXd J = calculator.compute(q, JacobianType::Analytic)`  
**When** the analytical implementation replaces CppAD  
**Then** the code SHALL compile without changes  
**And** the code SHALL run without changes  
**And** the results SHALL be numerically equivalent (within 1e-6 tolerance)

#### Scenario: Python Bindings Unchanged
**Given** Python code using `urdfx.JacobianCalculator(robot, "tool0")`  
**When** the analytical implementation is deployed  
**Then** the Python API SHALL remain identical  
**And** existing Python scripts SHALL work without modification

---

### Requirement: Edge Case Handling
**ID**: JAC-009  
**Priority**: P1

The analytical implementation SHALL handle edge cases robustly.

**New requirement**: Defines behavior for special configurations.

#### Scenario: Zero Joint Angles
**Given** joint configuration q = [0, 0, 0, 0, 0, 0]  
**When** computing Jacobian  
**Then** the result SHALL be well-defined and finite  
**And** the computation SHALL not trigger floating-point exceptions

#### Scenario: Near-Singular Configuration
**Given** a joint configuration where the end-effector is near full extension  
**And** the Jacobian has a singular value less than 1e-6  
**When** computing Jacobian  
**Then** the result SHALL be numerically stable  
**And** the `isSingular()` method SHALL correctly detect the singular configuration

#### Scenario: Joint at Limits
**Given** a joint at its upper or lower limit  
**When** computing Jacobian  
**Then** the result SHALL be valid  
**And** the Jacobian column for that joint SHALL reflect the constraint

---

### Requirement: Cross-Platform Performance
**ID**: JAC-010  
**Priority**: P1

The analytical implementation SHALL perform well across all supported platforms.

**New requirement**: Ensures consistent behavior across build targets.

#### Scenario: Native Platform Performance
**Given** native builds on Windows (MSVC), Linux (GCC), macOS (Clang)  
**When** running Jacobian benchmarks  
**Then** all platforms SHALL achieve <5µs mean computation time  
**And** no platform SHALL show performance regression vs CppAD baseline

#### Scenario: WebAssembly Performance
**Given** a WebAssembly build with Emscripten -O3 optimization  
**When** running Jacobian benchmarks in browser (Chrome or Firefox)  
**Then** computation time SHALL be less than 20µs  
**And** performance SHALL be at least 30% faster than CppAD-based WASM build

---

## Implementation Notes

### Key Changes
1. **Algorithm**: Replaced automatic differentiation with geometric Jacobian formula
2. **Dependencies**: Removed CppAD, now only depends on Eigen
3. **Performance**: Expected 5-10× improvement in Jacobian computation
4. **Memory**: Eliminated tape storage overhead (~10KB per link)

### Migration Approach
- Implement new `AnalyticalJacobianCalculator` in parallel
- Validate against existing CppAD implementation (10,000 random tests)
- Switch IK solver to use analytical implementation
- Remove CppAD after validation complete

### Testing Strategy
- **Accuracy**: Finite difference validation + CppAD cross-check
- **Performance**: Micro-benchmarks (Jacobian only) + End-to-end IK benchmarks
- **Compatibility**: Existing test suite must pass unchanged
- **Edge cases**: Zero angles, limits, singular configurations

### References
- Geometric Jacobian formula: Murray et al., *Mathematical Introduction to Robotic Manipulation*
- Reference implementations: Pinocchio, KDL
- Performance targets: LOIK paper (Low-Overhead Inverse Kinematics)
