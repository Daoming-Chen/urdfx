# Tasks: Replace CppAD with Analytical Jacobian

## Phase 1: Core Implementation (Days 1-3)

### 1.1 Create Analytical Jacobian Calculator Class
- [ ] Create `core/include/urdfx/analytical_jacobian.h`
  - Define `AnalyticalJacobianCalculator` class
  - Match API surface of existing `JacobianCalculator`
  - Add `JointFrameCache` struct for workspace
- [ ] Create `core/src/analytical_jacobian.cpp`
  - Implement constructor with chain setup
  - Add chain/FK caching similar to existing implementation
- [ ] Update `core/src/CMakeLists.txt`
  - Add new source file to build

### 1.2 Implement Core Geometric Jacobian Algorithm
- [x] Implement `computeAnalyticalJacobian()` private method
  - Forward pass: accumulate transforms and cache joint frames
  - Handle revolute joints (axis-angle rotation)
  - Handle prismatic joints (translation along axis)
  - Handle fixed joints (static transform only)
  - Extract `z_i` (axis in world frame) and `p_i` (position in world frame)
- [x] Implement Jacobian column computation
  - Linear part: `z_i × (p_ee - p_i)` using `Eigen::Vector3d::cross()`
  - Angular part: `z_i` directly
  - Fill 6×n output matrix column-by-column
- [x] Add optimization for Z-axis revolute joints (common case)
  - Use simplified rotation matrix for standard DH parameters

### 1.3 Implement Public API Methods
- [x] Implement `compute(joint_angles, type, target_link)`
  - Call core geometric method
  - Apply Jacobian type conversion (Analytic ↔ Geometric)
  - Support target_link parameter (use chain cache)
- [x] Implement `isSingular(joint_angles, threshold, type, target_link)`
  - Reuse existing SVD-based logic
  - Use analytical Jacobian as input
- [x] Implement `getManipulability(joint_angles, type, target_link)`
  - Reuse existing determinant-based calculation
- [x] Implement `getConditionNumber(joint_angles, type, target_link)`
  - Reuse existing SVD condition number calculation
- [x] Implement static `convertJacobian(jacobian, pose, from, to)`
  - Copy existing adjoint matrix conversion logic (no changes needed)

## Phase 2: Validation & Testing (Days 3-5)

### 2.1 Unit Tests for Accuracy
- [ ] Create `core/tests/test_analytical_jacobian.cpp`
  - Add test fixture with UR5e robot model
- [ ] Add `CompareWithCppAD` test
  - Load UR5e model
  - Generate 1,000 random configurations
  - Compute Jacobian with both CppAD and analytical methods
  - Assert error < 1e-6 for each trial
- [ ] Add `FiniteDifferenceValidation` test
  - Compute analytical Jacobian
  - Compute numerical Jacobian via central difference (ε = 1e-7)
  - Assert error < 1e-4
  - Test both position and orientation components
- [ ] Add edge case tests
  - Zero joint angles
  - Joint at upper/lower limits
  - Near-singular configuration (extended arm)
  - All-prismatic joints (if supported)
  - Mixed revolute + prismatic chain

### 2.2 Integration Tests with IK Solver
- [ ] Add `SQPIKSolverWithAnalyticalJacobian` test suite
  - Modify `SQPIKSolver` to accept custom Jacobian calculator (dependency injection)
  - Run existing IK tests with analytical Jacobian
  - Assert solutions match CppAD-based results within 1e-3
- [ ] Add warm start convergence test
  - Verify warm start behavior unchanged
  - Check iteration count consistency
- [ ] Add trajectory following test
  - Sequence of IK solves along a path
  - Verify smooth convergence

### 2.3 Benchmark Tests
- [x] Add micro-benchmark for Jacobian computation
  - Create `benchmarks/jacobian_benchmarks.cpp`
  - Benchmark analytical method in isolation
  - Target: <5µs for 7-DOF arm on i7-12700K
- [ ] Update existing IK benchmarks
  - Add flag to switch between CppAD and analytical
  - Run side-by-side comparison
  - Record baseline results before integration
- [ ] Run benchmarks on multiple platforms
  - Windows (MSVC)
  - Linux (GCC)
  - macOS (Clang)
  - WebAssembly (Emscripten)

## Phase 3: Integration (Days 5-7)

### 3.1 Update IK Solver to Use Analytical Jacobian
- [ ] Modify `SQPIKSolver` class
  - Replace `JacobianCalculator jacobian_` with `AnalyticalJacobianCalculator analytical_jacobian_`
  - Update constructor initialization
  - No changes to `solve()` method (API identical)
- [ ] Run full test suite
  - Execute all existing unit tests
  - Execute all existing integration tests
  - Ensure 100% pass rate
- [ ] Run full benchmark suite
  - Compare performance metrics against baseline
  - Document improvements in benchmark results JSON

### 3.2 Update Python Bindings
- [ ] Update `bindings/python/src/bindings.cpp`
  - Replace `JacobianCalculator` exports with `AnalyticalJacobianCalculator`
  - Maintain backward-compatible class name in Python (alias if needed)
- [ ] Run Python tests
  - Execute `pytest` suite
  - Verify numpy integration works
- [ ] Update Python examples
  - Test `examples/python/inverse_kinematics.py`
  - Verify no API changes required

### 3.3 Update WebAssembly Bindings
- [ ] Update `bindings/wasm/src/bindings.cpp`
  - Replace Jacobian calculator in WASM exports
  - Test in browser environment
- [ ] Run WASM tests
  - Execute Jest test suite
  - Test in Puppeteer browser automation
- [ ] Update JavaScript examples
  - Test `examples/javascript/forward_kinematics.js`
  - Verify visualization app still works

## Phase 4: Cleanup & Documentation (Days 7-10)

### 4.1 Remove CppAD Implementation
- [x] Delete old `JacobianCalculator` class
  - Remove CppAD-specific code from `core/src/kinematics.cpp`
  - Remove `ADForwardKinematics` helper class
  - Remove tape caching and mutex logic
- [ ] Rename `AnalyticalJacobianCalculator` → `JacobianCalculator`
  - Update header: `analytical_jacobian.h` → part of `kinematics.h`
  - Update source file
  - Update all includes in test and example files
- [x] Remove CppAD headers
  - Delete `#include <cppad/cppad.hpp>` from `kinematics.cpp`
  - Remove pragma diagnostic blocks for CppAD warnings
  - Clean up `CMakeLists.txt` include directories

### 4.2 Remove CppAD Dependency
- [ ] Update `CMakeLists.txt` (root)
  - Remove `add_subdirectory(third_party/CppAD)`
  - Remove CppAD from `find_package()` or `FetchContent`
- [x] Update `cmake/Dependencies.cmake`
  - Remove CppAD configuration
- [ ] Remove CppAD submodule
  - Run `git submodule deinit third_party/CppAD`
  - Run `git rm third_party/CppAD`
  - Delete `.gitmodules` entry
- [ ] Update `.gitignore`
  - Remove CppAD-related entries if any

### 4.3 Update Documentation
- [ ] Update `README.md`
  - Remove CppAD from dependency list
  - Update "How It Works" section (remove AD explanation)
  - Add performance comparison table (before/after)
  - Update architecture diagram if applicable
- [ ] Update `docs/api/README.md`
  - Document new Jacobian computation method
  - Add reference to geometric Jacobian formula
- [ ] Update `docs/guides/getting-started.md`
  - Remove CppAD installation steps (if mentioned)
- [ ] Update benchmark results
  - Copy new JSON results to `benchmarks/results/`
  - Update `visualize.html` if needed
  - Add comparison chart showing improvement

### 4.4 Update Build Scripts
- [ ] Update `scripts/setup.sh` (Linux/macOS)
  - Remove CppAD git submodule initialization
- [ ] Update `scripts/setup.ps1` (Windows)
  - Remove CppAD setup steps
- [ ] Update CI configuration (if applicable)
  - `.github/workflows/*.yml` or similar
  - Remove CppAD build steps
  - Update caching keys (dependencies changed)

## Phase 5: Verification & Release (Days 10+)

### 5.1 Final Validation
- [ ] Run complete test suite on all platforms
  - Windows (MSVC 2022)
  - Ubuntu 22.04 (GCC 11)
  - macOS (latest Clang)
  - WebAssembly (Emscripten)
- [ ] Run complete benchmark suite
  - Record final performance numbers
  - Generate comparison charts
- [ ] Manual testing
  - Test examples in `examples/cpp/`
  - Test Python bindings
  - Test WASM visualization app

### 5.2 Performance Documentation
- [ ] Create performance report document
  - Include micro-benchmark results (Jacobian only)
  - Include end-to-end IK benchmark results
  - Add platform comparison table
  - Add memory usage comparison
- [ ] Create before/after visualization
  - Bar chart comparing timing
  - Include in README or docs/
- [ ] Optional: Write blog post
  - Explain geometric Jacobian method
  - Show performance gains
  - Discuss migration process

### 5.3 Code Review & Merge
- [ ] Self-review all changes
  - Check code style consistency
  - Verify no debug code left
  - Ensure all TODOs addressed
- [ ] Create pull request
  - Write detailed PR description
  - Link to proposal and design docs
  - Include benchmark results in PR body
- [ ] Address review comments
- [ ] Merge to main branch

## Success Criteria Checklist

### P0 (Must Have)
- [ ] Analytical Jacobian accuracy within 1e-6 of CppAD reference
- [ ] All existing unit tests pass without modification
- [ ] IK benchmark shows ≥2x speedup on native builds
- [ ] CppAD dependency fully removed from repository

### P1 (Should Have)
- [ ] Jacobian computation <5µs for 7-DOF arm
- [ ] Full IK solve <80µs in median case
- [ ] WASM build shows measurable improvement (≥30% faster)
- [ ] Documentation updated with performance numbers

### P2 (Nice to Have)
- [ ] Python bindings benchmark shows improvement
- [ ] Comparison chart added to README (before/after)
- [ ] Blog post explaining geometric method

## Rollback Plan

If critical issues are discovered:
1. Revert commits in Phase 4 (keep both implementations)
2. Add CMake flag: `URDFX_USE_ANALYTICAL_JACOBIAN` (default: OFF)
3. Restore CppAD submodule temporarily
4. Investigate root cause before re-attempting migration

## Notes

- **Testing Priority**: Accuracy validation (Phase 2.1) is critical before integration
- **Performance Baseline**: Record CppAD performance before making changes
- **Incremental Commits**: Commit after each completed task for easy rollback
- **Documentation**: Update docs as you go, not at the end
