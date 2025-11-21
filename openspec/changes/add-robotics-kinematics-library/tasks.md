# Implementation Tasks

This document outlines the implementation tasks in dependency order. Each task should be completed and tested before moving to the next.

## Phase 1: Project Setup and Build System

### 1. Initialize Repository Structure
- [x] Create directory structure (include/, src/, python/, wasm/, tests/, scripts/)
- [x] Initialize git repository and .gitignore
- [x] Add README.md with project overview
- [x] Create LICENSE file

### 2. Setup Git Submodules
- [x] Add Eigen submodule to third_party/eigen
- [x] Add pugixml submodule to third_party/pugixml
- [x] ~~Add CppAD submodule to third_party/CppAD~~ (Not used - implemented analytical Jacobian instead)
- [x] Add DaQP submodule to third_party/daqp
- [x] Add nanobind submodule to third_party/nanobind
- [x] Add googletest submodule to third_party/googletest
- [x] Add spdlog submodule to third_party/spdlog
- [x] Verify all submodules checkout correctly

### 3. Setup Logging Infrastructure with spdlog
- [x] Add spdlog submodule to third_party/spdlog (if not already added)
- [x] Create include/urdfx/logging.h header with logging macros
- [x] Configure spdlog logger with appropriate sinks (console, file)
- [x] Set default log level (INFO) and allow runtime configuration
- [x] Create logging utility functions (setLogLevel, setLogFile)
- [x] Add spdlog dependency to CMakeLists.txt
- [x] Write unit tests for logging functionality
- [x] Document logging usage in code comments

### 4. Create Root CMakeLists.txt
- [x] Set project name, version, and C++20 standard
- [x] Configure compiler flags (warnings, optimizations)
- [x] Add options for BUILD_TESTING, BUILD_PYTHON_BINDINGS, BUILD_WASM
- [x] Include cmake/Dependencies.cmake for finding dependencies
- [x] Add subdirectories (src, tests, python, wasm)
- [x] Configure installation rules and export targets

### 5. Create Dependencies CMake Module
- [x] Create cmake/Dependencies.cmake
- [x] Add logic to find or build Eigen from submodule
- [x] Add logic to find or build pugixml from submodule
- [x] ~~Add logic to find or build CppAD from submodule~~ (Not needed)
- [x] Add logic to find or build DaQP from submodule
- [x] Add logic to find or build spdlog from submodule
- [x] Handle conditional dependencies (nanobind only if BUILD_PYTHON_BINDINGS)

### 6. Create Setup Scripts
- [x] Write scripts/setup.sh for Linux/macOS
- [x] Install CMake, Python, Node.js if missing
- [x] Install Emscripten SDK to third_party/emsdk
- [x] Make script idempotent (detect existing installations)
- [x] Add usage documentation to script header

## Phase 2: Core C++ Library - URDF Parsing

### 7. Design Robot Model Data Structures
- [x] Define Link class (name, inertial, visual, collision)
- [x] Define Joint class (name, type, axis, limits, origin, parent, child)
- [x] Define Robot class (name, links, joints, root_link)
- [x] Define Transform class (wrapper around Eigen::Isometry3d)
- [x] Implement accessors and utility methods

### 8. Implement URDF Parser
- [x] Create URDFParser class using pugixml
- [x] Implement parseURDFFile(path) method
- [x] Implement parseURDFString(content) method
- [x] Parse <robot> root element
- [x] Parse <link> elements (inertial, visual, collision)
- [x] Parse <joint> elements (type, axis, limits, origin)
- [x] Build kinematic tree structure
- [x] Validate robot model (check for disconnected links, invalid limits)

### 9. Implement Error Handling
- [x] Define URDFParseException class
- [x] Add descriptive error messages with line numbers
- [x] Handle missing required tags gracefully
- [x] Add logging for warnings using spdlog (e.g., disconnected links)

### 10. Write URDF Parsing Tests
- [x] Create test fixture with UR5e URDF
- [x] Test successful parsing of valid URDF
- [x] Test error handling for malformed URDF
- [x] Test kinematic tree structure
- [x] Test joint property extraction
- [x] Test mesh path resolution
- [x] Run tests and verify all pass

## Phase 3: Core C++ Library - Forward Kinematics

### 11. Implement Kinematic Chain Builder
- [x] Create KinematicChain class
- [x] Implement buildChain(robot, end_link) method
- [x] Traverse from end_link to root
- [x] Store ordered list of joints and transformations
- [x] Pre-compute static transformations

### 12. Implement Forward Kinematics
- [x] Create ForwardKinematics class
- [x] Implement compute(joint_angles) → Transform
- [x] Use Eigen for matrix operations
- [x] Support computing to intermediate links
- [x] Implement bounds checking (optional)
- [x] Optimize for repeated calls (pre-allocate matrices)

### 13. Implement Pose Representations
- [x] Implement asMatrix() → Eigen::Matrix4d
- [x] Implement asPositionQuaternion() → (Vector3d, Quaterniond)
- [x] Implement asPositionEuler() → (Vector3d, Vector3d)
- [x] Ensure quaternion normalization

### 14. Write Forward Kinematics Tests
- [x] Test FK at zero configuration
- [x] Test FK at various configurations
- [x] Compare with analytical solutions (UR5e known poses)
- [x] Test FK to intermediate links
- [x] Test bounds checking functionality
- [x] Benchmark FK performance (target < 1ms)
- [x] Test thread-safety with concurrent calls
- [x] Run tests and verify all pass

## Phase 4: Core C++ Library - Jacobian Computation

> **Note**: The project was completed using analytical geometric Jacobian computation instead of CppAD automatic differentiation. See [replace-cppad-with-analytical-jacobian](../replace-cppad-with-analytical-jacobian/) for details on this design decision.

### 15. ~~Integrate CppAD~~ Implement Analytical Jacobian (Completed with analytical approach)
- [x] ~~Create ADForwardKinematics (CppAD-compatible FK)~~ Implemented geometric Jacobian formulas
- [x] ~~Use AD<double> types for joint angles~~ Used Eigen types directly
- [x] ~~Implement FK with CppAD tape recording~~ Implemented closed-form geometric computation
- [x] ~~Verify AD FK matches regular FK~~ Verified analytical Jacobian accuracy

### 16. Implement Jacobian Calculator
- [x] Create JacobianCalculator class
- [x] ~~Create CppAD tape during initialization~~ Cache kinematic chain structure
- [x] Implement compute(joint_angles) → Eigen::MatrixXd
- [x] Support geometric (body) Jacobian
- [x] Support analytic (spatial) Jacobian
- [x] Implement conversion between Jacobian types

### 17. Implement Singularity Detection
- [x] Compute singular values of Jacobian
- [x] Implement isSingular(threshold) method
- [x] Implement getManipulability() method
- [x] Add condition number calculation

### 18. Write Jacobian Tests
- [x] Test Jacobian at zero configuration
- [x] ~~Compare CppAD Jacobian with numerical differentiation~~ Compare analytical Jacobian with finite differences
- [ ] Verify geometric vs analytic Jacobian relationship
- [ ] Test Jacobian to intermediate links
- [ ] Test singularity detection (e.g., UR5e elbow extended)
- [ ] Benchmark Jacobian performance (target < 0.5ms)
- [ ] Test numerical stability with small angles
- [ ] Run tests and verify all pass

## Phase 5: Core C++ Library - Inverse Kinematics

### 19. Integrate DaQP QP Solver
- [x] Study DaQP API and examples
- [x] Create wrapper for DaQP solver initialization
- [x] Implement QP formulation helper functions
- [x] Test basic QP solving with simple problems

### 20. Implement SQP IK Solver
- [x] Create IKSolver abstract base class
- [x] Create SQPIKSolver implementation
- [x] Implement solve(target, initial_guess) method
- [x] Implement SQP iteration loop
- [x] Compute FK and Jacobian in each iteration
- [x] Formulate and solve QP subproblem with DaQP
- [x] Enforce joint limit constraints
- [x] Implement line search for step size
- [x] Check convergence criteria
- [x] Return solution and status

### 21. Implement IK Solver Configuration
- [x] Add SolverConfig struct (tolerances, max_iterations, step_limits)
- [x] Implement setSolverConfig() method
- [x] Add methods for position-only and orientation-only IK
- [x] Implement warm-starting support

### 22. Implement IK Diagnostics
- [x] Create SolverStatus struct (converged, iterations, error, etc.)
- [x] Track convergence history (optional)
- [x] Provide detailed error reporting
- [x] Handle unreachable poses gracefully

### 23. Write Inverse Kinematics Tests
- [x] Test IK for reachable poses
- [x] Test IK with warm start vs cold start
- [x] Test joint limit enforcement
- [x] Test position-only and orientation-only IK
- [x] Test multiple solutions (elbow-up vs elbow-down)
- [x] Test unreachable pose handling
- [x] Test convergence for various target poses
- [x] Benchmark IK performance (target < 10ms)
- [x] Test thread-safety
- [x] Run tests and verify all pass

## Phase 6: Python Bindings

### 24. Setup nanobind Build Configuration
- [x] Create python/CMakeLists.txt
- [x] Configure nanobind with find_package or submodule
- [x] Define Python extension module target
- [x] Link against core urdfx library

### 25. Implement Python Bindings
- [x] Create python/src/bindings.cpp
- [x] Bind Robot class (from_urdf, from_urdf_string methods)
- [x] Bind ForwardKinematics class (compute method)
- [x] Bind JacobianCalculator class (compute method)
- [x] Bind IKSolver class (solve method)
- [x] Configure NumPy array interoperability
- [x] Translate C++ exceptions to Python exceptions
- [x] Use snake_case naming for Python API

### 26. Create Python Package Structure
- [x] Create python/urdfx/__init__.py
- [x] Create setup.py with nanobind integration
- [x] Generate type stubs (.pyi files) for IDE support
- [x] Add pyproject.toml for modern Python packaging

### 27. Write Python Tests
- [x] Create python_tests/ directory
- [x] Setup pytest configuration
- [x] Test Robot loading from URDF
- [x] Test FK computation with NumPy arrays
- [x] Test Jacobian computation
- [x] Test IK solving
- [x] Test exception handling
- [x] Test memory management (no leaks)
- [x] Test NumPy array zero-copy
- [x] Run tests and verify all pass

### 28. Create Python Documentation
- [x] Add docstrings to all bound functions
- [x] Create Python usage examples
- [x] Document installation instructions

## Phase 7: WebAssembly Bindings

### 29. Setup Emscripten Build Configuration
- [x] Create wasm/CMakeLists.txt
- [x] Configure for Emscripten toolchain
- [x] Set Embind compilation flags
- [x] Configure optimization flags (-O3, SIMD)
- [x] Set output format (MODULARIZE=1)

### 30. Implement WASM Bindings with Embind
- [x] Create wasm/src/bindings.cpp
- [x] Bind Robot class (fromURDFString method)
- [x] Bind ForwardKinematics class
- [x] Bind JacobianCalculator class
- [x] Bind IKSolver class
- [x] Convert C++ types to JavaScript-friendly formats
- [x] Implement explicit memory management (delete methods)

### 31. Create TypeScript Definitions
- [x] Create wasm/urdfx.d.ts with type declarations
- [x] Define interfaces for Robot, ForwardKinematics, etc.
- [x] Document method signatures and return types

### 32. Build and Test WASM Module
- [x] Build WASM with Emscripten
- [x] Verify binary size (< 2MB uncompressed)
- [x] Create test HTML page for manual testing
- [x] Test module loading in browser
- [x] Test FK/IK functionality
- [x] Verify no memory leaks
- [x] Measure performance in browser

### 33. Write JavaScript Tests
- [x] Setup Jest for Node.js testing
- [x] Setup Puppeteer for browser testing
- [x] Test WASM module loading
- [x] Test Robot creation from URDF string
- [x] Test FK computation
- [x] Test IK solving
- [x] Test memory cleanup
- [x] Run tests and verify all pass

## Phase 8: Visualization Application

### 34. Setup React + Vite Project
- [ ] Initialize Vite project with React + TypeScript
- [ ] Configure TypeScript with strict mode
- [ ] Setup ESLint and Prettier
- [ ] Install Three.js and @react-three/fiber
- [ ] Install dependencies (e.g., @react-three/drei for helpers)

### 35. Integrate urdfx WASM Module
- [ ] Copy built urdfx.js and urdfx.wasm to visualization/public/
- [ ] Create WASM loader utility
- [ ] Test module loading in React app
- [ ] Handle loading states and errors

### 36. Implement URDF Loader and Parser
- [ ] Create URDFLoader class
- [ ] Parse URDF string and extract geometry
- [ ] Load mesh files (OBJ format)
- [ ] Create Three.js meshes from URDF visual elements
- [ ] Apply materials and colors
- [ ] Build scene graph matching kinematic structure

### 37. Implement Robot Renderer
- [ ] Create RobotRenderer React component
- [ ] Setup Three.js scene, camera, lights
- [ ] Render loaded robot geometry
- [ ] Implement joint transformations
- [ ] Update robot pose based on joint angles

### 38. Implement Joint Control UI
- [ ] Create JointControlPanel component
- [ ] Generate sliders for each joint
- [ ] Respect joint limits in slider ranges
- [ ] Update robot in real-time on slider change
- [ ] Display current joint angle values

### 39. Implement Forward Kinematics Mode
- [ ] Integrate urdfx WASM FK computation
- [ ] Compute and display end-effector pose
- [ ] Render coordinate frame at end-effector
- [ ] Display position and orientation in UI

### 40. Implement Inverse Kinematics Mode
- [ ] Add IK mode toggle in UI
- [ ] Implement end-effector gizmo (TransformControls)
- [ ] Call urdfx WASM IK solver on gizmo drag
- [ ] Update joint angles based on IK solution
- [ ] Handle unreachable poses with visual feedback
- [ ] Display IK solver status

### 41. Implement Kinematic Information Display
- [ ] Compute and display Jacobian condition number
- [ ] Indicate singularities with color coding
- [ ] Optionally render manipulability ellipsoid

### 42. Implement Camera Controls
- [ ] Setup OrbitControls from @react-three/drei
- [ ] Configure zoom, pan, and orbit
- [ ] Set reasonable camera limits

### 43. Implement URDF File Upload
- [ ] Create file upload UI component
- [ ] Handle file reading in browser
- [ ] Parse uploaded URDF
- [ ] Resolve mesh paths (relative to URDF)
- [ ] Render uploaded robot

### 44. Implement Responsive Layout
- [ ] Create responsive CSS layout
- [ ] Adapt UI for desktop and tablet sizes
- [ ] Test on various screen resolutions

### 45. Optimize Performance
- [ ] Profile rendering performance
- [ ] Optimize FK/IK call frequency
- [ ] Implement request animation frame batching
- [ ] Ensure 60 FPS during interaction

### 46. Write Visualization App Tests
- [ ] Create Jest unit tests for components
- [ ] Test URDF loading logic
- [ ] Test joint angle calculations
- [ ] Create Puppeteer integration tests
- [ ] Test user interactions (slider, IK drag)
- [ ] Run tests and verify all pass

### 47. Create Visualization App Documentation
- [ ] Write user guide
- [ ] Document keyboard shortcuts
- [ ] Add in-app help modal

## Phase 9: Documentation and Examples

### 48. Write C++ API Documentation
- [ ] Add Doxygen comments to all public APIs
- [ ] Generate API reference with Doxygen
- [ ] Create usage examples (FK, IK, Jacobian)

### 49. Write Python API Documentation
- [ ] Ensure all docstrings are complete
- [ ] Create Sphinx documentation
- [ ] Add Python usage examples

### 50. Write JavaScript/WASM Documentation
- [ ] Document WASM module API
- [ ] Create JavaScript usage examples
- [ ] Add TypeScript example project

### 51. Create Comprehensive README
- [ ] Write project overview
- [ ] Add installation instructions (C++, Python, WASM)
- [ ] Document build instructions for all platforms
- [ ] Add usage examples
- [ ] Link to detailed documentation
- [ ] Add troubleshooting section

### 52. Create CONTRIBUTING.md
- [ ] Document contribution guidelines
- [ ] Explain code style and conventions
- [ ] Describe PR process
- [ ] Add issue templates

## Phase 10: Testing and CI/CD

### 53. Setup Continuous Integration
- [ ] Create .github/workflows/ci.yml
- [ ] Configure builds for Linux, macOS, Windows
- [ ] Run C++ tests on all platforms
- [ ] Run Python tests on all platforms
- [ ] Build and test WASM module
- [ ] Check code formatting (clang-format)

### 54. Add Code Coverage
- [ ] Configure coverage tools (gcov/lcov)
- [ ] Add coverage reporting to CI
- [ ] Set minimum coverage threshold

### 55. Performance Benchmarking
- [ ] Create benchmark suite with Google Benchmark
- [ ] Benchmark FK computation
- [ ] Benchmark Jacobian computation
- [ ] Benchmark IK solving
- [ ] Track performance over time in CI

### 56. Create Release Process
- [ ] Setup semantic versioning
- [ ] Create release checklist
- [ ] Configure automated releases via GitHub Actions
- [ ] Build distributable packages (wheels, archives)

## Phase 11: Final Integration and Validation

### 57. End-to-End Testing
- [ ] Test complete pipeline: URDF → FK → Jacobian → IK
- [ ] Test Python integration with real robot model
- [ ] Test WASM integration in visualization app
- [ ] Test installation from source
- [ ] Test installation via pip (Python)

### 58. Performance Validation
- [ ] Verify FK < 1ms for 6-DOF robot
- [ ] Verify IK < 10ms for typical poses
- [ ] Verify WASM binary < 2MB
- [ ] Verify visualization app > 30 FPS

### 59. Documentation Review
- [ ] Review all documentation for accuracy
- [ ] Verify all links work
- [ ] Check code examples compile and run
- [ ] Get external review

### 60. Security Audit
- [ ] Review URDF parsing for XML injection vulnerabilities
- [ ] Check bounds on array accesses
- [ ] Validate user inputs in visualization app
- [ ] Run static analysis tools

### 61. Prepare for Release
- [ ] Tag initial release (v1.0.0)
- [ ] Create release notes
- [ ] Update project website/repository
- [ ] Announce release

---

## Validation Criteria

Each task should be considered complete when:
- Code is implemented and follows project conventions
- Unit tests are written and passing
- Code is reviewed (self-review at minimum)
- Documentation is updated
- CI passes

## Dependency Tracking

- Tasks 1-6 must complete before any other work
- Task 3 (spdlog logging) must complete before any C++ code writing
- Tasks 7-10 (URDF) must complete before Tasks 11-14 (FK)
- Tasks 11-14 (FK) must complete before Tasks 15-18 (Jacobian)
- Tasks 15-18 (Jacobian) must complete before Tasks 19-23 (IK)
- Tasks 7-23 (Core Library) must complete before Tasks 24-33 (Bindings)
- Tasks 29-33 (WASM) must complete before Tasks 34-47 (Visualization)
- Most tasks can proceed in parallel after Phase 5 completes
