# C++ API Documentation

This directory will contain the generated C++ API documentation using Doxygen.

## Generating Documentation

```bash
cd docs/api/cpp
doxygen Doxyfile
```

The generated HTML documentation will be in `docs/api/cpp/html/`.

## Configuration

`Doxyfile` will be added in a future update. It will extract documentation from:

- `core/include/urdfx/*.h` - Public API headers
- `core/src/*.cpp` - Implementation files (for internal docs)

## Planned Documentation Sections

- **Modules**: Robot Model, URDF Parser, Kinematics, Inverse Kinematics
- **Classes**: `RobotModel`, `ForwardKinematics`, `InverseKinematics`
- **Examples**: Code snippets from `examples/cpp/`
