# Python Bindings for urdfx

## Status: Coming Soon

Python bindings for the urdfx robotics kinematics library are currently under development.

## Planned Features

- Full access to URDF parsing functionality
- Forward kinematics computation
- Inverse kinematics solvers
- Jacobian computation
- NumPy-based API for easy integration with scientific Python ecosystem

## Installation (Future)

```bash
pip install urdfx
```

## Basic Usage Example (Future)

```python
import urdfx
import numpy as np

# Parse URDF
robot = urdfx.parse_urdf("robot.urdf")

# Compute forward kinematics
joint_angles = np.array([0.0, 1.57, -1.57, 0.0, 0.0, 0.0])
pose = robot.forward_kinematics(joint_angles)

# Compute inverse kinematics
target_pose = np.eye(4)
target_pose[:3, 3] = [0.5, 0.0, 0.5]  # target position
solution = robot.inverse_kinematics(target_pose)
```

## Development

See `openspec/changes/restructure-project-layout/` for the project restructuring plan.
