# Design: Python Bindings Implementation

## Context

urdfx 是一个 C++20 机器人运动学库，已经有 WebAssembly bindings（使用 Emscripten）。我们需要添加 Python bindings 以支持 Python 科学计算生态系统。项目已选择 nanobind 作为绑定工具（相比 pybind11 更轻量，生成的二进制更小）。

### Constraints

- 必须使用现有的 nanobind submodule
- 需要支持 Python 3.8+（与项目要求一致）
- 性能开销应 < 10%（根据 project.md）
- 需要与 NumPy 无缝集成
- 应遵循 Python PEP 8 代码风格

### Stakeholders

- Python 机器人研究人员
- 科学计算用户
- 机器学习/深度学习开发者

## Goals / Non-Goals

### Goals

- 提供完整的 Python API，覆盖所有 C++ 核心功能
- NumPy 数组与 Eigen 向量/矩阵的零拷贝或高效转换
- 完整的类型提示和文档字符串
- 全面的测试覆盖（单元测试 + 集成测试）
- 性能基准测试，与 C++ 对比
- 简单的 pip 安装方式

### Non-Goals

- 不创建纯 Python 的算法实现（仅绑定 C++）
- 不支持 Python 2.x
- 不实现 C++ 中不存在的新功能
- 不在此阶段发布到 PyPI（可在后续单独发布）

## Decisions

### Decision 1: 使用 nanobind 而非 pybind11

**Rationale**: 
- nanobind 生成的二进制更小（~50% 减少）
- 编译速度更快
- API 与 pybind11 相似，易于使用
- 已在 project.md 中选定

**Alternatives considered**:
- pybind11: 更成熟但二进制更大
- Boost.Python: 过于重量级
- SWIG: 生成代码质量较低
- Cython: 需要编写更多胶水代码

### Decision 2: 模块结构设计

**Structure**:
```python
urdfx/
  __init__.py          # 主模块入口
  _urdfx.so            # nanobind 编译的二进制
  
# Python API:
import urdfx
robot = urdfx.Robot.from_urdf("robot.urdf")
fk = urdfx.ForwardKinematics(robot, "tool0")
pose = fk.compute(joint_angles)
```

**Rationale**:
- 扁平化命名空间，与 WASM bindings 一致
- 直接暴露核心类，无需子模块
- 简洁的 API，符合 Python 习惯

### Decision 3: NumPy 集成策略

使用 nanobind 的 `nb::ndarray` 实现：
- 输入：接受 NumPy 数组，自动转换为 Eigen 类型
- 输出：返回 NumPy 数组（从 Eigen 转换），共享内存或拷贝（取决于所有权）

**Code pattern**:
```cpp
Eigen::VectorXd to_eigen(nb::ndarray<double> arr);
nb::ndarray<double> from_eigen(const Eigen::VectorXd& vec);
```

### Decision 4: 错误处理

- C++ 异常自动映射到 Python 异常
- 保持异常类型对应：
  - `std::invalid_argument` → `ValueError`
  - `std::runtime_error` → `RuntimeError`
  - `std::out_of_range` → `IndexError`

### Decision 5: 测试策略

**Test categories**:
1. **Unit tests**: 每个类的基本功能
2. **Integration tests**: FK → Jacobian → IK 完整流程
3. **Roundtrip tests**: C++ 测试用例的 Python 版本
4. **Numerical tests**: 与 C++ 结果比较精度
5. **Performance tests**: Benchmark 与 C++ 对比

**Test framework**: pytest + numpy.testing

### Decision 6: 构建系统集成

- 使用 CMake + nanobind 集成
- 提供 `pyproject.toml` 使用 scikit-build-core
- 支持 `pip install .` 和 `pip install -e .`（开发模式）
- 构建输出到 `bindings/python/urdfx/`

## Risks / Trade-offs

### Risk: NumPy 版本兼容性

**Risk**: NumPy C API 在不同版本间可能不兼容

**Mitigation**: 
- 使用 nanobind 的 NumPy 抽象层
- 设置最低版本要求为 NumPy 1.20+
- 在多个 NumPy 版本上测试

### Risk: 性能开销

**Risk**: Python 调用开销可能超过 10% 目标

**Mitigation**:
- 使用 nanobind 的零拷贝特性
- 在 benchmark 中明确测量
- 对批量操作使用向量化

### Trade-off: API 设计

**Trade-off**: Python API 是否应该与 C++ 完全一致？

**Decision**: 
- 核心功能保持一致
- 添加 Pythonic 便利方法（如 `from_urdf` 类方法）
- 使用 NumPy 数组代替 Eigen 类型

## Migration Plan

不适用（这是新功能，无需迁移）

## Implementation Tasks

参见 `tasks.md`

## Open Questions

- [ ] 是否需要在 bindings 中添加可视化辅助函数（如导出到 matplotlib）？
  - **Decision**: 在初始版本中不包含，可在后续 PR 中添加

- [ ] 是否支持 Python 中的多线程 IK 求解？
  - **Decision**: 初始版本不支持，保持与 C++ 单线程一致

