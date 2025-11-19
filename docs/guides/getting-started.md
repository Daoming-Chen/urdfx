# Getting Started with urdfx

## Overview

urdfx is a modern C++20 robotics kinematics library with multi-language support through WebAssembly and Python bindings.

## Installation

### From Source (C++)

```bash
git clone https://github.com/Daoming-Chen/urdfx.git
cd urdfx
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build
sudo cmake --install build
```

### Using vcpkg

```bash
vcpkg install urdfx
```

### Python (Coming Soon)

```bash
pip install urdfx
```

### JavaScript/WASM

```bash
npm install urdfx-wasm
```

## Quick Start

### C++

```cpp
#include <urdfx/robot_model.h>
#include <urdfx/urdf_parser.h>
#include <urdfx/kinematics.h>

// Parse URDF
auto robot = urdfx::parseURDF("robot.urdf");

// Compute forward kinematics
Eigen::VectorXd joint_angles(6);
joint_angles << 0, M_PI/2, -M_PI/2, 0, 0, 0;

urdfx::ForwardKinematics fk(robot, "base_link", "end_effector");
auto pose = fk.compute(joint_angles);
```

### JavaScript/WASM

```javascript
import urdfx from 'urdfx-wasm';

const robot = await urdfx.parseURDF('robot.urdf');
const pose = robot.forwardKinematics([0, 1.57, -1.57, 0, 0, 0]);
```

## Next Steps

- [URDF Parsing Guide](urdf-parsing.md)
- [Forward Kinematics Tutorial](../tutorials/cpp-tutorial.md)
- [Inverse Kinematics Guide](inverse-kinematics.md)
- [API Reference](../api/)
