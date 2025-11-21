# urdfx Python Bindings

Python bindings for the urdfx robotics kinematics library.

## Installation

### From Source

Requirements:
- CMake 3.20+
- C++20 compiler
- Python 3.8+
- NumPy 1.20+

```bash
pip install .
```

## Usage

```python
import urdfx
import numpy as np

# Load robot
robot = urdfx.Robot.from_urdf_file("ur5.urdf")

# Forward Kinematics
fk = urdfx.ForwardKinematics(robot, "tool0")
q = np.zeros(6)
pose = fk.compute(q)
print(f"Position: {pose.translation()}")

# Inverse Kinematics
solver = urdfx.SQPIKSolver(robot, "tool0")
result = solver.solve(pose, np.zeros(6))
if result.status.converged:
    print("IK Solution:", result.solution)
else:
    print("IK Failed:", result.status.message)
```

## Development

To build for development (editable install):

```bash
pip install --no-build-isolation -ve .
```

## Testing

```bash
pytest tests/
```
