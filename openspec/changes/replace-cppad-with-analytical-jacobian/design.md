# Design: Analytical Jacobian Implementation

## Architecture Overview

This document describes the technical design for replacing CppAD-based automatic differentiation with hand-crafted analytical Jacobian computation using geometric methods.

## Mathematical Foundation

### Spatial Jacobian (Analytic)

For a serial manipulator with joint configuration `q ∈ ℝⁿ`, the spatial Jacobian `J ∈ ℝ⁶ˣⁿ` relates joint velocities to end-effector twist:

```
ẋ = J(q) q̇

where ẋ = [v; ω] ∈ ℝ⁶  (linear and angular velocity in world frame)
```

Each column `J_i` represents the contribution of joint `i`:

**Revolute Joint:**
```
J_i = [ z_i × (p_ee - p_i) ]  ∈ ℝ⁶
      [        z_i          ]

where:
  z_i   = R_i * axis_local    (joint axis in world frame)
  p_i   = position of joint i in world frame
  p_ee  = end-effector position in world frame
```

**Prismatic Joint:**
```
J_i = [  z_i  ]  ∈ ℝ⁶
      [  0    ]
```

### Computation Algorithm

**Forward Pass (Single Traversal):**
```
Input: Joint angles q ∈ ℝⁿ
Output: Jacobian J ∈ ℝ⁶ˣⁿ

1. Initialize:
   T_world = Identity  (accumulator for world transform)

2. For each joint i = 0..n-1:
   a. Apply static transform (joint origin from URDF)
   b. Apply joint-dependent transform (rotation/translation by q[i])
   c. Extract and cache:
      - z_i = T_world.rotation() * joint.axis
      - p_i = T_world.translation()
   d. Update T_world for next joint

3. Store p_ee = T_world.translation()

4. For each joint i = 0..n-1:
   a. Compute cross product: z_i × (p_ee - p_i)
   b. Fill column: J.col(i) = [cross_result; z_i]
```

**Complexity:**
- Time: O(n) — single forward pass + O(n) cross products
- Space: O(n) — cache n joint frames (3 vectors each: z_i, p_i, p_ee)

## Data Structure Design

### JointFrameCache

Pre-allocate workspace to avoid heap allocations during computation:

```cpp
struct JointFrameCache {
    std::vector<Eigen::Vector3d> z_world;  // Joint axes in world frame
    std::vector<Eigen::Vector3d> p_world;  // Joint positions in world frame
    Eigen::Vector3d p_ee;                  // End-effector position
    
    void resize(size_t n) {
        z_world.resize(n);
        p_world.resize(n);
    }
};
```

### AnalyticalJacobianCalculator API

```cpp
class URDFX_API AnalyticalJacobianCalculator {
public:
    // Constructor (same as JacobianCalculator)
    AnalyticalJacobianCalculator(
        std::shared_ptr<const Robot> robot,
        const std::string& end_link,
        const std::string& base_link = "");
    
    // Primary API (drop-in replacement)
    Eigen::MatrixXd compute(
        const Eigen::VectorXd& joint_angles,
        JacobianType type = JacobianType::Analytic,
        const std::string& target_link = "") const;
    
    // Utility methods (preserve existing API)
    bool isSingular(const Eigen::VectorXd& joint_angles, 
                    double threshold = 1e-6, 
                    JacobianType type = JacobianType::Analytic,
                    const std::string& target_link = "") const;
    
    double getManipulability(...) const;
    double getConditionNumber(...) const;
    
    static Eigen::MatrixXd convertJacobian(
        const Eigen::MatrixXd& jacobian,
        const Transform& pose,
        JacobianType from,
        JacobianType to);

private:
    std::shared_ptr<const Robot> robot_;
    std::string base_link_;
    std::string default_end_link_;
    
    // Cached kinematic chains per end-effector link
    mutable std::unordered_map<std::string, std::unique_ptr<KinematicChain>> chain_cache_;
    mutable std::unordered_map<std::string, std::unique_ptr<ForwardKinematics>> fk_cache_;
    
    // Pre-allocated workspace (thread-local in multi-threaded context)
    mutable std::unordered_map<std::string, JointFrameCache> frame_cache_;
    
    // Core implementation
    void computeAnalyticalJacobian(
        const Eigen::VectorXd& joint_angles,
        const KinematicChain& chain,
        JointFrameCache& cache,
        Eigen::MatrixXd& J_out) const;
};
```

## Implementation Details

### Forward Kinematics Integration

Reuse `ForwardKinematics::compute()` for end-effector pose but add internal method for joint frame extraction:

```cpp
// New method in ForwardKinematics class
void computeWithIntermediates(
    const Eigen::VectorXd& joint_angles,
    JointFrameCache& cache,
    Transform& end_pose) const;
```

This avoids duplicating FK logic while exposing intermediate frames needed for Jacobian.

### Optimizations

**1. SIMD Vectorization (Eigen Automatic)**
- Cross products: `z_i.cross(p_ee - p_i)` maps to SSE/AVX instructions
- Matrix fills: Column assignment uses vectorized stores

**2. Cache Locality**
- Store joint frames contiguously (std::vector of Vector3d)
- Access pattern: sequential reads (optimal for prefetching)

**3. Reduced Allocations**
- Pre-allocate `JointFrameCache` per kinematic chain
- Reuse across multiple `compute()` calls

**4. Rotation Matrix Optimization**
- For Z-axis revolute joints (common case):
  ```cpp
  // Specialized rotation matrix (no sin/cos lookup needed for axis extraction)
  R_joint << cos(q), -sin(q), 0,
             sin(q),  cos(q), 0,
             0,       0,      1;
  z_world = R_world * Vector3d(0, 0, 1);  // Simplified for Z-axis
  ```

### Joint Type Handling

```cpp
// Pseudo-code for joint-dependent transform
switch (joint->getType()) {
    case JointType::Revolute:
    case JointType::Continuous: {
        // Axis-angle rotation
        Eigen::AngleAxisd rot(q[i], joint->getAxis());
        R_local = rot.toRotationMatrix();
        // No translation
        break;
    }
    case JointType::Prismatic: {
        // Pure translation along axis
        p_local = q[i] * joint->getAxis();
        // No rotation
        break;
    }
    case JointType::Fixed: {
        // Static transform only (already in chain setup)
        break;
    }
}
```

### Jacobian Type Conversion

Preserve existing conversion between Analytic (spatial) and Geometric (body) Jacobians:

```cpp
// From JacobianCalculator::convertJacobian (keep as-is)
if (from == Analytic && to == Geometric) {
    // Body Jacobian = Adjoint^{-1} * Spatial Jacobian
    return adjointInverseMatrix(pose) * jacobian;
}
```

## Testing Strategy

### Unit Tests

**1. Accuracy Validation**
```cpp
TEST(AnalyticalJacobianTest, CompareWithCppAD) {
    auto robot = loadUR5e();
    JacobianCalculator cppad_calc(robot, "tool0");
    AnalyticalJacobianCalculator analytical_calc(robot, "tool0");
    
    for (int trial = 0; trial < 10000; ++trial) {
        VectorXd q = randomConfiguration();
        
        MatrixXd J_cppad = cppad_calc.compute(q);
        MatrixXd J_analytical = analytical_calc.compute(q);
        
        double error = (J_cppad - J_analytical).norm();
        EXPECT_LT(error, 1e-6) << "Trial " << trial;
    }
}
```

**2. Finite Difference Check**
```cpp
TEST(AnalyticalJacobianTest, FiniteDifferenceValidation) {
    auto robot = loadUR5e();
    AnalyticalJacobianCalculator calc(robot, "tool0");
    ForwardKinematics fk(robot, "tool0");
    
    VectorXd q = VectorXd::Random(6);
    MatrixXd J = calc.compute(q);
    
    // Numerical differentiation
    const double eps = 1e-7;
    MatrixXd J_numerical(6, 6);
    for (int i = 0; i < 6; ++i) {
        VectorXd q_plus = q; q_plus[i] += eps;
        VectorXd q_minus = q; q_minus[i] -= eps;
        
        auto T_plus = fk.compute(q_plus);
        auto T_minus = fk.compute(q_minus);
        
        // Central difference for position
        J_numerical.col(i).head<3>() = 
            (T_plus.translation() - T_minus.translation()) / (2*eps);
        
        // Rotation difference (convert to axis-angle)
        // ... (implementation details)
    }
    
    EXPECT_LT((J - J_numerical).norm(), 1e-4);
}
```

**3. Edge Cases**
- Zero joint angles
- Joint limits (near boundaries)
- Singular configurations (extended arm)
- Prismatic joints
- Mixed revolute + prismatic chains

### Integration Tests

**1. IK Solver Compatibility**
```cpp
TEST(SQPIKSolverTest, AnalyticalJacobianProducesSameResults) {
    auto robot = loadUR5e();
    auto target = randomReachablePose();
    
    // Old implementation (CppAD-based)
    SQPIKSolver solver_old(robot, "tool0");
    VectorXd solution_old;
    auto status_old = solver_old.solve(target, VectorXd::Zero(6), solution_old);
    
    // New implementation (analytical)
    SQPIKSolver solver_new(robot, "tool0");  // Internally uses AnalyticalJacobianCalculator
    VectorXd solution_new;
    auto status_new = solver_new.solve(target, VectorXd::Zero(6), solution_new);
    
    EXPECT_EQ(status_old.converged, status_new.converged);
    EXPECT_LT((solution_old - solution_new).norm(), 1e-3);
}
```

### Benchmark Tests

**1. Micro-benchmark (Jacobian Only)**
```cpp
static void BM_AnalyticalJacobian(benchmark::State& state) {
    auto robot = loadUR5e();
    AnalyticalJacobianCalculator calc(robot, "tool0");
    VectorXd q = VectorXd::Random(6);
    
    for (auto _ : state) {
        MatrixXd J = calc.compute(q);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(BM_AnalyticalJacobian)->Unit(benchmark::kMicrosecond);
```

**2. End-to-End IK Benchmark**
- Reuse existing `ik_benchmarks.cpp`
- Compare before/after performance
- Target: 2-3x speedup in total IK time

## Migration Plan

### Phase 1: Parallel Implementation (Week 1)
- ✅ Create `AnalyticalJacobianCalculator` in new files:
  - `core/include/urdfx/analytical_jacobian.h`
  - `core/src/analytical_jacobian.cpp`
- ✅ Implement core algorithm (forward pass + Jacobian fill)
- ✅ Add unit tests for accuracy validation

### Phase 2: API Compatibility (Week 1-2)
- ✅ Port all methods from `JacobianCalculator`:
  - `compute()`, `isSingular()`, `getManipulability()`, etc.
- ✅ Ensure identical results (within numerical tolerance)
- ✅ Add integration tests with `SQPIKSolver`

### Phase 3: Integration (Week 2)
- ✅ Update `SQPIKSolver` to use `AnalyticalJacobianCalculator`
- ✅ Run full test suite (all tests pass)
- ✅ Benchmark and document performance improvements

### Phase 4: Cleanup (Week 2)
- ✅ Remove `JacobianCalculator` class
- ✅ Remove CppAD includes from `kinematics.cpp` and headers
- ✅ Update CMakeLists.txt to remove CppAD dependency
- ✅ Remove `third_party/CppAD` submodule
- ✅ Update documentation and README

### Rollback Strategy

If analytical implementation fails validation:
1. Keep both implementations in codebase
2. Add CMake option: `URDFX_USE_ANALYTICAL_JACOBIAN` (default: ON)
3. Fall back to CppAD if issues found in production

Rollback is low-risk since:
- Old implementation remains functional during Phase 1-2
- Both implementations can coexist temporarily
- CMake flag allows per-build selection

## Performance Targets

### Micro-benchmarks (Jacobian Computation)

| Configuration | Current (CppAD) | Target (Analytical) | Speedup |
|---------------|-----------------|---------------------|---------|
| UR5e (6-DOF)  | 20-30µs         | 3-5µs               | 5-7x    |
| Panda (7-DOF) | 25-35µs         | 4-6µs               | 5-7x    |
| UR10 (6-DOF)  | 20-30µs         | 3-5µs               | 5-7x    |

### End-to-End IK Benchmarks

| Scenario      | Current (CppAD) | Target (Analytical) | Speedup |
|---------------|-----------------|---------------------|---------|
| Cold Start    | 150-250µs       | 60-100µs            | 2-3x    |
| Warm Start    | 100-180µs       | 40-70µs             | 2-3x    |
| Trajectory    | 120-200µs       | 50-80µs             | 2-3x    |

### Memory Usage

| Metric                | Current (CppAD) | Target (Analytical) | Reduction |
|-----------------------|-----------------|---------------------|-----------|
| Tape Storage          | ~10KB per link  | 0 (no tape)         | 100%      |
| Runtime Workspace     | ~5KB            | ~1KB                | 80%       |
| Peak Heap Allocations | ~50KB           | ~10KB               | 80%       |

## Risks & Contingencies

### Risk: Performance Target Miss

**If analytical Jacobian is only 2x faster (not 5x):**
- Still worthwhile (simpler code, fewer dependencies)
- Investigate bottlenecks (profiling, assembly inspection)
- Consider specialized code paths for common joint configurations

### Risk: WASM Performance Regression

**If WASM shows minimal improvement:**
- Investigate Emscripten optimizations (-O3, SIMD flags)
- Consider keeping CppAD for WASM builds only (conditional compilation)
- WASM is secondary target; native performance is priority

### Risk: Numerical Instability

**If near-singular configurations cause issues:**
- Add damping/regularization in cross product computation
- Use higher precision (long double) for critical calculations
- Add runtime checks and fallback to slower but stable method

## References

- **Pinocchio**: `src/algorithm/jacobian.hxx` (reference implementation)
- **KDL**: `src/jacobian.cpp` (alternative approach)
- **Modern Robotics**: Chapter 5 (Jacobian theory)
- **Featherstone**: *Rigid Body Dynamics Algorithms* (spatial notation)
