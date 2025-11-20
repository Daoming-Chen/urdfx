# Proposal: Replace CppAD with Analytical Jacobian Computation

## Overview

This proposal replaces the CppAD-based automatic differentiation Jacobian computation with a hand-crafted analytical implementation using geometric methods. This change eliminates the computational overhead of automatic differentiation (AD tape recording/playback) and targets a **5-10x performance improvement** in inverse kinematics solving.

## Motivation

### Current Performance Bottleneck

The existing `JacobianCalculator` uses CppAD for automatic differentiation:
- **Single IK solve**: 100-300µs on i7-12700K
- **Jacobian computation**: ~20-50µs per iteration (estimated from AD overhead)
- **Overhead sources**: Tape creation, optimization, and forward/reverse mode execution

### Target Performance

Industry-standard robotics libraries (Pinocchio, KDL) and recent research (LOIK paper) achieve:
- **Jacobian computation**: <5µs for 7-DOF manipulators
- **Complete IK solve**: 20-80µs including QP solver

### Root Cause

Automatic differentiation is general-purpose but inefficient for rigid body kinematics:
1. **Tape overhead**: CppAD records computational graph for generic functions
2. **Runtime cost**: Forward/reverse mode sweeps through tape
3. **Memory pressure**: Caching tapes per end-effector link

For robot kinematics, the structure is known a priori—joint types, axis directions, and transformation hierarchy are static. We can exploit this structure to compute Jacobians directly using geometric formulas.

## Proposed Solution

### Geometric Jacobian Method

Replace CppAD with **closed-form geometric Jacobian** computation:

For a revolute joint `i`, the Jacobian column is:
```
J_i = [ z_i × (p_ee - p_i) ]  // Linear velocity contribution
      [        z_i          ]  // Angular velocity contribution
```

Where:
- `z_i`: Joint axis direction in world frame
- `p_i`: Joint position in world frame  
- `p_ee`: End-effector position in world frame
- `×`: Cross product

For prismatic joints:
```
J_i = [  z_i  ]  // Linear velocity along axis
      [  0    ]  // No angular velocity
```

### Implementation Strategy

**Phase 1: New Analytical Implementation**
- Create `AnalyticalJacobianCalculator` class alongside existing `JacobianCalculator`
- Implement geometric method using only Eigen (no CppAD dependency)
- Maintain identical API to ensure drop-in replacement

**Phase 2: Integration & Validation**
- Add finite-difference validation tests comparing analytical vs CppAD results
- Update `SQPIKSolver` to use new calculator
- Run comprehensive accuracy tests on UR5e benchmark

**Phase 3: Performance Verification**
- Benchmark analytical Jacobian computation in isolation
- Measure end-to-end IK solve performance
- Compare against baseline (current CppAD implementation)

**Phase 4: Migration & Cleanup**
- Replace all uses of `JacobianCalculator` with `AnalyticalJacobianCalculator`
- Remove CppAD dependency from CMake and third_party/
- Archive old implementation for historical reference

## Benefits

### Performance
- **Primary**: 5-10x faster Jacobian computation (50µs → 5µs target)
- **Secondary**: 2-3x faster overall IK solve (200µs → 50-80µs target)
- **Tertiary**: Reduced memory footprint (no tape storage)

### Maintainability
- **Simpler codebase**: Direct formula implementation easier to understand
- **Easier debugging**: No AD black box, can inspect intermediate values
- **Better error messages**: Clear geometric interpretation of failures

### Dependencies
- **Lighter build**: Remove CppAD submodule (~5MB, complex build config)
- **Faster compilation**: No template-heavy AD headers
- **Fewer conflicts**: CppAD has MSVC warning suppression requirements

## Risks & Mitigation

### Risk 1: Accuracy Loss
**Concern**: Hand-written derivatives might have numerical errors.

**Mitigation**:
- Comprehensive finite-difference validation in unit tests
- Compare analytical results against CppAD for 10,000 random configurations
- Assert error < 1e-6 for all test cases
- Keep CppAD implementation available during validation phase

### Risk 2: Implementation Complexity
**Concern**: Geometric method requires careful frame transformations.

**Mitigation**:
- Use established formulas from robotics textbooks (Murray, Sciavicco)
- Reference implementations: Pinocchio, KDL, Drake
- Start with simple test case (2-DOF planar arm) before UR5e

### Risk 3: Feature Loss
**Concern**: Current `JacobianCalculator` supports multiple end-effector links via caching.

**Mitigation**:
- Port caching mechanism to new implementation
- Use same `chain_cache_` pattern to store pre-computed link chains
- Maintain full API compatibility (including `target_link` parameter)

### Risk 4: WebAssembly Build
**Concern**: Performance characteristics differ in WASM environment.

**Mitigation**:
- Keep WASM benchmarks in CI to detect regressions
- Emscripten optimizations (-O3, SIMD) apply to both implementations
- Expected improvement may be smaller (~2-3x vs 5-10x native)

## Success Criteria

### Must Have (P0)
1. ✅ Analytical Jacobian accuracy within 1e-6 of CppAD reference
2. ✅ All existing unit tests pass without modification
3. ✅ IK benchmark shows ≥2x speedup on native builds
4. ✅ CppAD dependency fully removed from repository

### Should Have (P1)
1. ✅ Jacobian computation <5µs for 7-DOF arm
2. ✅ Full IK solve <80µs in median case  
3. ✅ WASM build shows measurable improvement (≥30% faster)
4. ✅ Documentation updated with performance numbers

### Nice to Have (P2)
1. 🎯 Python bindings benchmark shows improvement
2. 🎯 Comparison chart added to README (before/after)
3. 🎯 Blog post explaining geometric method

## Alternatives Considered

### Alternative 1: Keep CppAD, Optimize Configuration
**Approach**: Tune CppAD tape optimization flags, use forward mode only.

**Rejected because**: 
- Fundamental overhead remains (tape structure)
- Limited improvement potential (10-20% vs 5-10x)
- Still carries dependency burden

### Alternative 2: Switch to Different AD Library
**Approach**: Use lighter-weight AD (autodiff, CasADi, ADOL-C).

**Rejected because**:
- All AD libraries have similar overhead for this use case
- Doesn't address root problem (generic differentiation for specialized task)
- Adds migration cost without solving core issue

### Alternative 3: Hybrid Approach
**Approach**: Use analytical Jacobian for common cases, CppAD for edge cases.

**Rejected because**:
- Adds complexity (two code paths)
- CppAD dependency still required
- Hard to justify maintaining both implementations

## Dependencies

### Blocking
- None (self-contained change)

### Blocked By This
- None (but future work may benefit):
  - Dynamics computation (mass matrix, Coriolis terms)
  - Acceleration-level IK solvers
  - Model predictive control (MPC) integration

## Timeline Estimate

- **Phase 1 (Implementation)**: 3-5 days
- **Phase 2 (Integration)**: 1-2 days  
- **Phase 3 (Performance)**: 1 day
- **Phase 4 (Migration)**: 1-2 days

**Total**: ~1-2 weeks for complete migration

## References

- Murray, Li, Sastry: *A Mathematical Introduction to Robotic Manipulation* (Chapter 3)
- Siciliano et al.: *Robotics: Modelling, Planning and Control* (Chapter 3)
- Pinocchio library: https://github.com/stack-of-tasks/pinocchio
- LOIK paper: *Low-Overhead Inverse Kinematics* (performance benchmarks)
- KDL library: http://www.orocos.org/kdl (reference implementation)
