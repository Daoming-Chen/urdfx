# Project Context

## Purpose
urdfx is a modern C++20 robotics kinematics library providing URDF parsing, forward kinematics, Jacobian computation, and inverse kinematics solving capabilities. The library offers Python and WebAssembly bindings for cross-platform usage, along with a Three.js-based visualization web application.

**Core Goals**:
- Fast and accurate kinematics computations for robot manipulators
- Modern C++20 implementation with minimal dependencies
- Cross-platform support (Linux, macOS, Windows, Web)
- Easy integration into robotics projects via multiple language bindings

## Tech Stack

### Core Library (C++20)
- **Build System**: CMake 3.20+, git submodules for dependencies
- **Linear Algebra**: Eigen 3.4+
- **XML Parsing**: pugixml
- **Automatic Differentiation**: CppAD
- **Optimization**: DaQP (QP solver)
- **Testing**: Google Test

### Language Bindings
- **Python**: nanobind (3.8+ support)
- **WebAssembly**: Emscripten with Embind

### Visualization Application
- **Frontend**: React + TypeScript
- **3D Rendering**: Three.js
- **Build Tool**: Vite
- **Testing**: Jest + Puppeteer

### Testing
- **C++**: GTest with fixtures for UR5e robot
- **Python**: pytest with numpy integration tests
- **JavaScript**: Jest for unit tests, Puppeteer for browser integration

## Project Conventions

### Code Style
- **C++**: 
  - Use clang-format with Google style
  - CamelCase for classes, snake_case for functions/variables
  - Use `const` and `constexpr` extensively
  - Prefer `auto` for complex types
  - Use modern C++20 features (concepts, ranges, modules where applicable)
- **Python**: 
  - Follow PEP 8
  - Use type hints for all public APIs
  - snake_case for all names
- **JavaScript/TypeScript**: 
  - Use ESLint + Prettier
  - camelCase for variables/functions, PascalCase for classes
  - Strict TypeScript mode enabled

### File Organization
```
urdfx/
├── include/urdfx/          # Public C++ headers
├── src/                    # C++ implementation
├── python/                 # nanobind bindings
├── wasm/                   # Emscripten bindings
├── visualization/          # Three.js app
├── tests/                  # C++ tests
├── python_tests/           # Python tests
├── third_party/            # Git submodules
└── scripts/                # Build and setup scripts
```

### Architecture Patterns
- **Separation of Concerns**: Parser, FK, Jacobian, IK are separate modules
- **Dependency Injection**: Solvers accept strategy objects for customization
- **RAII**: Automatic resource management for robot models and solvers
- **Immutability**: Robot model is immutable after parsing
- **Lazy Evaluation**: CppAD tapes created on first Jacobian request

### Testing Strategy
- **Unit Tests**: Each class has dedicated test suite
- **Integration Tests**: Full FK→Jacobian→IK pipeline tests
- **Fixtures**: Reusable UR5e robot model for all tests
- **Property-Based Testing**: Random joint angles validate FK/IK round-trips
- **Performance Benchmarks**: Track solver convergence time and iterations
- **CI/CD**: Automated testing on Linux, macOS, Windows, and WebAssembly

### Git Workflow
- **Branching**: feature/*, bugfix/*, release/*
- **Commits**: Conventional commits (feat:, fix:, docs:, test:, refactor:)
- **Pull Requests**: Required reviews, all tests must pass
- **Releases**: Semantic versioning (MAJOR.MINOR.PATCH)

## Domain Context

### Robotics Kinematics
- **Forward Kinematics (FK)**: Compute end-effector pose from joint angles
- **Inverse Kinematics (IK)**: Compute joint angles for desired end-effector pose
- **Jacobian**: Maps joint velocities to end-effector velocities
- **URDF**: Unified Robot Description Format (XML-based robot model)

### Key Concepts
- **Kinematic Chain**: Sequence of links and joints from base to end-effector
- **Joint Types**: Revolute (rotation), Prismatic (translation), Fixed
- **Joint Limits**: Min/max bounds on joint angles/positions
- **DH Parameters**: Denavit-Hartenberg convention for kinematic modeling
- **SQP**: Sequential Quadratic Programming (iterative IK solving method)

### Coordinate Systems
- Follow standard robotics conventions (right-handed, Z-up for joints)
- Transformations represented as 4×4 homogeneous matrices
- Rotations use quaternions internally (Eigen::Quaterniond)

## Important Constraints

### Performance Requirements
- FK computation: < 1ms for 6-DOF robot
- IK solving: < 10ms for typical poses (6-DOF, 10cm tolerance)
- Python binding overhead: < 10% compared to C++
- WASM binary size: < 2MB (compressed)

### Compatibility
- C++20 standard (GCC 10+, Clang 12+, MSVC 2019+)
- Python 3.8+ (support for NumPy 1.20+)
- Modern browsers with WebAssembly support (Chrome 90+, Firefox 88+)

### Limitations
- No collision detection (future work)
- Single kinematic chain (no parallel robots)
- No dynamics computation (future work)
- Joint limits only (no task-space constraints in IK)

## External Dependencies

### Git Submodules (third_party/)
- **Eigen**: 3.4+ (linear algebra)
- **pugixml**: 1.13+ (XML parsing)
- **CppAD**: 20230000+ (automatic differentiation)
- **DaQP**: Latest (QP solver)
- **nanobind**: Latest (Python bindings)
- **googletest**: 1.14+ (C++ testing)

### System Dependencies
- **Emscripten**: For WebAssembly builds (install via scripts/setup_emscripten.sh)
- **Python**: 3.8+ with pip (for pytest, numpy, mypy)
- **Node.js**: 18+ with npm (for visualization app)

### Build Requirements
- CMake 3.20+
- C++20 compatible compiler
- Git (for submodules)
- Optional: ccache for faster rebuilds

## Development Workflow
1. Clone with submodules: `git clone --recursive`
2. Run setup script: `./scripts/setup.sh` (installs Emscripten if needed)
3. Configure build: `cmake -B build -DCMAKE_BUILD_TYPE=Release`
4. Build: `cmake --build build -j$(nproc)`
5. Test: `ctest --test-dir build --output-on-failure`
6. Install: `cmake --install build --prefix /usr/local`
