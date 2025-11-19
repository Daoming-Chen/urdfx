# C++ Tutorial

## Introduction

This tutorial covers the basic usage of urdfx for C++ developers.

## Prerequisites

- C++20 compatible compiler (GCC 10+, Clang 10+, MSVC 19.29+)
- CMake 3.20+
- Eigen3 library

## Topics

1. **Parsing URDF Files**
2. **Forward Kinematics**
3. **Inverse Kinematics**
4. **Jacobian Computation**
5. **Advanced Topics**

## 1. Parsing URDF Files

```cpp
#include <urdfx/urdf_parser.h>

auto robot = urdfx::parseURDF("path/to/robot.urdf");
std::cout << "Robot name: " << robot.getName() << std::endl;
std::cout << "Number of joints: " << robot.getJoints().size() << std::endl;
```

## 2. Forward Kinematics

```cpp
#include <urdfx/kinematics.h>

urdfx::ForwardKinematics fk(robot, "base_link", "end_effector");

Eigen::VectorXd q(6);
q << 0, M_PI/4, -M_PI/4, 0, M_PI/2, 0;

auto pose = fk.compute(q);
std::cout << "Position: " << pose.translation().transpose() << std::endl;
```

## 3. Inverse Kinematics

```cpp
#include <urdfx/inverse_kinematics.h>

urdfx::InverseKinematics ik(robot, "base_link", "end_effector");

Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
target_pose.translation() << 0.5, 0.0, 0.5;

auto result = ik.solve(target_pose);
if (result.success) {
    std::cout << "Solution: " << result.joint_angles.transpose() << std::endl;
}
```

## More Examples

See `examples/cpp/` for complete working examples.
