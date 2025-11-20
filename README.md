# urdfx

A modern C++20 robotics kinematics library providing URDF parsing, forward kinematics, Jacobian computation, and inverse kinematics solving capabilities with Python and WebAssembly bindings.

## Features

- **URDF Parsing**: Parse Unified Robot Description Format (URDF) files using pugixml
- **Forward Kinematics**: Compute end-effector pose from joint angles using Eigen transformations
- **Jacobian Computation**: Automatic differentiation-based Jacobian calculation using CppAD
- **Inverse Kinematics**: SQP-based IK solver with joint limit constraints using DaQP
- **Python Bindings**: NumPy-compatible Python interface via nanobind
- **WebAssembly Support**: Browser-based robotics applications via Emscripten
- **3D Visualization**: Three.js-based web application for interactive robot visualization

## Quick Start

### C++ Example

```cpp
#include <urdfx/urdf_parser.h>
#include <urdfx/forward_kinematics.h>
#include <urdfx/ik_solver.h>

// Parse URDF file
auto robot = urdfx::URDFParser::parse("ur5e.urdf");

// Compute forward kinematics
urdfx::ForwardKinematics fk(robot);
std::vector<double> joint_angles = {0.0, -1.57, 0.0, 0.0, 0.0, 0.0};
auto pose = fk.compute(joint_angles);

// Solve inverse kinematics
urdfx::SQPIKSolver ik_solver(robot);
Eigen::Isometry3d target_pose;
// ... set target pose ...
auto solution = ik_solver.solve(target_pose);
```

### Python Example

```python
import urdfx
import numpy as np

# Load robot from URDF
robot = urdfx.Robot.from_urdf("ur5e.urdf")

# Compute forward kinematics
fk = urdfx.ForwardKinematics(robot)
joint_angles = np.array([0.0, -1.57, 0.0, 0.0, 0.0, 0.0])
pose = fk.compute(joint_angles)

# Solve inverse kinematics
ik = urdfx.IKSolver(robot)
target_pose = np.eye(4)  # 4x4 transformation matrix
solution = ik.solve(target_pose)
```

### JavaScript/WebAssembly Example

```bash
# Install from npm
npm install urdfx
```

```javascript
const createUrdfxModule = require('urdfx');

// Initialize WASM module
const urdfx = await createUrdfxModule();

// Load robot from URDF string
const urdfXml = `<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Your URDF content -->
</robot>`;

const robot = urdfx.Robot.fromURDFString(urdfXml);

// Compute forward kinematics
const fk = new urdfx.ForwardKinematics(robot);
const pose = fk.compute([0.0, -1.57, 0.0, 0.0, 0.0, 0.0]);

// Solve inverse kinematics
const ik = new urdfx.SQPIKSolver(robot);
const targetPose = {
  position: [0.5, 0.0, 0.5],
  quaternion: [1.0, 0.0, 0.0, 0.0]
};
const solution = ik.solve(targetPose, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

// Clean up
ik.dispose();
fk.dispose();
robot.dispose();
```

## Installation

### npm Package (WebAssembly)

The easiest way to use urdfx in JavaScript/TypeScript projects:

```bash
npm install urdfx
```

Available on npm: https://www.npmjs.com/package/urdfx

### Building from Source

#### Prerequisites

- C++20 compatible compiler (GCC 10+, Clang 12+, MSVC 19.29+ / Visual Studio 2019 16.11+)
- CMake 3.20 or later
- Python 3.8+ (for Python bindings)
- Node.js 18+ (for visualization app)
- Emscripten (for WebAssembly bindings)


**Platform-Specific Requirements:**
- **Linux**: Build tools via `apt`, `yum`, or similar package manager
- **macOS**: Xcode Command Line Tools or Homebrew
- **Windows**:
    - Visual Studio 2019 (16.11+) or Visual Studio 2022 with "Desktop development with C++" workload
    - CMake 3.20+ (recommended to install via [Chocolatey](https://chocolatey.org/packages/cmake) or manually)
    - Python 3.8+ (for Python bindings)
    - Node.js 18+ (for visualization)
    - [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html) (for WASM)
    - All-in-one dependency check and environment setup: `scripts/setup.ps1`

### Building from Source

#### Linux / macOS

```bash
# Clone with submodules
git clone --recursive https://github.com/Daoming-Chen/urdfx.git
cd urdfx

# Run setup script to check dependencies
./scripts/setup.sh

# Build C++ library
mkdir build && cd build
cmake ..
cmake --build .

# Install
sudo cmake --install .
```


#### Windows

```powershell
# Clone with submodules
git clone --recursive https://github.com/Daoming-Chen/urdfx.git
cd urdfx

# Run Windows setup script (checks/install dependencies: CMake, Python, Node.js, Emscripten, Visual Studio)
./scripts/setup.ps1

# Build C++ library (from PowerShell or VS Developer Command Prompt)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release -j

# Install (run as Administrator)
cmake --install build --prefix "C:\Program Files\urdfx"
```

**Windows Build Notes & Troubleshooting:**
- Always use `scripts/setup.ps1` to check/install all required tools and dependencies for Windows/MSVC builds.
- If CMake can't find MSVC, open a "Developer Command Prompt for VS 2022" or "x64 Native Tools Command Prompt" and retry.
- For long path issues, enable long paths in Windows:
    ```powershell
    Set-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name LongPathsEnabled -Value 1
    ```
- If submodules fail to clone, check proxy settings or manually run:
    ```powershell
    git submodule update --init --recursive
    ```
- If you see errors about missing DLL exports or import macros, ensure you are using the latest CMake and Visual Studio versions as required above.
- All dependencies (Eigen, pugixml, CppAD, DaQP, spdlog) are built from source and tested for MSVC compatibility.
- For Emscripten/WebAssembly builds, run `setup.ps1` to install and activate the Emscripten SDK on Windows.

**Tested on:**
- Windows 10 and 11, Visual Studio 2019/2022, CMake 3.20+, Python 3.8+, Node.js 18+, Emscripten 3.1+

For more details, see `openspec/changes/add-windows-build-support/`.

### Building Python Bindings

```bash
cd python
pip install .
```

### Building WebAssembly

```bash
cd wasm
emcmake cmake -B build
cmake --build build
```

## Project Structure

```
urdfx/
├── core/                       # C++ core library
│   ├── include/urdfx/          # Public C++ headers
│   │   ├── urdf_parser.h
│   │   ├── forward_kinematics.h
│   │   ├── jacobian_calculator.h
│   │   └── ik_solver.h
│   ├── src/                    # C++ implementation
│   ├── tests/                  # C++ unit tests (GTest)
│   └── CMakeLists.txt
│
├── bindings/                   # Language bindings
│   ├── python/                 # Python bindings (nanobind)
│   │   ├── src/                # nanobind binding code
│   │   ├── urdfx/              # Python package
│   │   ├── tests/              # Python tests (pytest)
│   │   ├── CMakeLists.txt
│   │   ├── setup.py
│   │   └── pyproject.toml
│   │
│   └── wasm/                   # WebAssembly bindings (Emscripten)
│       ├── src/                # Embind binding code
│       ├── tests/              # WASM tests (Jest)
│       ├── CMakeLists.txt
│       ├── package.json
│       └── urdfx.d.ts
│
├── examples/                   # Multi-language examples
│   ├── cpp/                    # C++ examples
│   ├── python/                 # Python examples
│   ├── javascript/             # JavaScript examples
│   └── models/                 # Robot URDF models
│
├── apps/                       # Complete applications
│   └── visualization/          # Three.js visualization web app
│       ├── src/
│       ├── public/
│       └── package.json
│
├── benchmarks/                 # Performance benchmarks
│   ├── ik_benchmarks.cpp
│   └── results/
│
├── docs/                       # Project documentation
│   ├── api/                    # API reference
│   │   ├── cpp/                # C++ API (Doxygen)
│   │   ├── python/             # Python API (Sphinx)
│   │   └── javascript/         # JavaScript API
│   ├── guides/                 # User guides
│   └── tutorials/              # Tutorials
│
├── third_party/                # Git submodules
│   ├── eigen/
│   ├── pugixml/
│   ├── CppAD/
│   ├── daqp/
│   ├── googletest/
│   ├── nanobind/
│   └── spdlog/
│
├── cmake/                      # CMake modules
├── scripts/                    # Build and setup scripts
│   ├── build-wasm.ps1/sh      # Build WASM bindings
│   ├── publish-npm.ps1/sh     # Publish to npm (Windows/Linux)
│   ├── setup.ps1/sh           # Development environment setup
│   └── PUBLISH_README.md      # npm publishing guide
├── openspec/                   # OpenSpec specifications
└── CMakeLists.txt              # Root CMake configuration
```

## Dependencies

### Core Dependencies (Git Submodules)
- **Eigen 3.4+**: Linear algebra library
- **pugixml**: Lightweight XML parser
- **CppAD**: Automatic differentiation library
- **DaQP**: Quadratic programming solver
- **Google Test**: Unit testing framework

### Binding Dependencies
- **nanobind**: Python bindings (smaller binary size than pybind11)
- **Emscripten**: WebAssembly compiler toolchain

### Visualization Dependencies
- **React**: UI framework
- **Three.js**: 3D rendering library
- **TypeScript**: Type-safe JavaScript
- **Vite**: Build tool

## Testing

### C++ Tests
```bash
cd build
ctest --output-on-failure
```

### Python Tests
```bash
pytest python_tests/
```

### JavaScript Tests
```bash
cd visualization
npm test
```

## Benchmarking

Run the Google Benchmark-based IK suite to measure cold-start, warm-start, and trajectory performance:

```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DBUILD_BENCHMARKS=ON
cmake --build build --target ik_benchmarks -j
./build/benchmarks/ik_benchmarks \
    --benchmark_out=benchmarks/results/ik_benchmarks_<date>.json \
    --benchmark_out_format=json
```

基准套件会依次覆盖三个典型场景：
- **ColdStart**：每次都从全零初值求解，反映最差收敛时间；
- **WarmStart**：先跑一次获得热启动，再用上一帧解作为初值，观察迭代数是否明显下降；
- **Trajectory**：连续 24 个目标姿态，通过热启动评估轨迹场景的稳态速度。

报告中的指标含义：
- `real_time`/`cpu_time` 代表每轮 6 或 24 次求解的平均耗时（微秒），越小越好；
- `avg_iterations` 是单次 IK 的平均迭代次数，反映数值稳定性；
- `solves_per_iteration` 表示一次 `state` 循环里批量了多少个目标；
- `success_rate` 为收敛比率，理想情况下应为 1；
- `iterations` 列出 Google Benchmark 实际重复次数，可用于衡量统计置信度。

基准结果会写入 `benchmarks/results/`，可直接提交或留作历史对比。若想快速查看曲线，可用 `benchmarks/benchmark_visualizer.html`：
1. 在浏览器打开该文件；
2. 上传 `ik_benchmarks_<date>.json`；
3. 页面会展示系统信息、指标卡片以及耗时/迭代的对比柱状图。

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     urdfx Library Core                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ URDF Parser  │→ │   Forward    │→ │   Jacobian   │     │
│  │  (pugixml)   │  │  Kinematics  │  │ Computation  │     │
│  └──────────────┘  │   (Eigen)    │  │   (CppAD)    │     │
│                     └──────────────┘  └──────────────┘     │
│                            ↓                  ↓             │
│                     ┌──────────────────────────────┐       │
│                     │   Inverse Kinematics         │       │
│                     │   (DaQP + SQP)               │       │
│                     └──────────────────────────────┘       │
└─────────────────────────────────────────────────────────────┘
                ↓                              ↓
    ┌──────────────────────┐      ┌──────────────────────┐
    │  Python Bindings     │      │  WASM Bindings       │
    │    (nanobind)        │      │   (Emscripten)       │
    └──────────────────────┘      └──────────────────────┘
                                              ↓
                                  ┌──────────────────────┐
                                  │  Visualization App   │
                                  │     (Three.js)       │
                                  └──────────────────────┘
```

## Key Algorithms

### Forward Kinematics
Computes end-effector pose from joint angles using transformation matrices:
```
T_end = T_base × T_joint1 × T_joint2 × ... × T_jointn
```

### Jacobian Computation
Uses CppAD automatic differentiation to compute the Jacobian matrix:
- Tape the forward kinematics computation once
- Efficiently evaluate Jacobian for different joint configurations

### Inverse Kinematics (SQP-based)
Sequential Quadratic Programming approach:
1. Compute current pose: `FK(q)`
2. Compute Jacobian: `J(q)`
3. Solve QP: minimize `||J·Δq - (target - FK(q))||²`
4. Apply joint limits: `q_min ≤ q + Δq ≤ q_max`
5. Update: `q ← q + α·Δq` (with line search)
6. Repeat until convergence

## Performance

- **Forward Kinematics**: Sub-millisecond computation for typical 6-DOF manipulators
- **Jacobian**: Cached tape evaluation for efficient repeated computations
- **Inverse Kinematics**: Convergence typically within 10-20 iterations
- **Python Overhead**: Minimal overhead due to nanobind's efficient binding
- **WebAssembly**: Near-native performance with SIMD optimizations

## Visualization App

The included Three.js visualization application provides:
- Interactive 3D robot visualization
- Joint angle sliders for FK exploration
- Drag-and-drop end-effector positioning for IK
- Real-time kinematics updates
- URDF mesh loading and rendering

To run the visualization app:
```bash
cd visualization
npm install
npm run dev
```

## Examples

See the `tests/` directory for comprehensive examples including:
- UR5e robot forward kinematics
- 6-DOF manipulator inverse kinematics
- Jacobian-based velocity control
- Trajectory generation with warm-starting

## Contributing

We follow conventional commits and require:
- Code formatted with clang-format (C++) or black (Python)
- All tests passing
- Type hints for Python code
- Documentation for public APIs

## License

[License information to be added]

## Citation

If you use urdfx in your research, please cite:
```
[Citation information to be added]
```

## Roadmap

- ✅ URDF parsing
- ✅ Forward kinematics
- ✅ Jacobian computation with automatic differentiation
- ✅ Inverse kinematics with SQP solver
- ✅ Python bindings
- ✅ WebAssembly support
- ✅ Three.js visualization
- 🚧 Collision detection integration (FCL)
- 🚧 Multi-solution IK solving
- 🚧 ROS2 integration
- 🚧 Dynamics computation

## Support

For questions, issues, or contributions:
- GitHub Issues: https://github.com/Daoming-Chen/urdfx/issues
- Documentation: [To be added]

## Acknowledgments

- Eigen for fast linear algebra
- CppAD for automatic differentiation
- DaQP for efficient QP solving
- The robotics community for URDF standardization
