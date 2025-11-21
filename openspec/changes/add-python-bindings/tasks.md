# Implementation Tasks: Python Bindings

## 1. Build System Setup

- [ ] 1.1 创建 `bindings/python/CMakeLists.txt` 配置 nanobind 构建
- [ ] 1.2 添加 nanobind 查找和链接配置
- [ ] 1.3 配置 Python 模块输出路径为 `bindings/python/urdfx/`
- [ ] 1.4 创建 `bindings/python/pyproject.toml` 使用 scikit-build-core
- [ ] 1.5 创建 `bindings/python/setup.py` 作为备用构建方式
- [ ] 1.6 添加 `bindings/python/.gitignore` 忽略构建产物
- [ ] 1.7 在根 CMakeLists.txt 添加 Python bindings 选项（可选构建）

## 2. Core Bindings Implementation

- [ ] 2.1 创建 `bindings/python/src/bindings.cpp` 主绑定文件
- [ ] 2.2 实现 NumPy ↔ Eigen 转换辅助函数
- [ ] 2.3 绑定 `Transform` 类
  - [ ] 2.3.1 构造函数和工厂方法
  - [ ] 2.3.2 `from_position_quaternion` 类方法
  - [ ] 2.3.3 `from_position_rpy` 类方法
  - [ ] 2.3.4 属性：`position`, `quaternion`, `matrix`
  - [ ] 2.3.5 方法：`inverse()`
- [ ] 2.4 绑定枚举类型
  - [ ] 2.4.1 `JointType` 枚举
  - [ ] 2.4.2 `GeometryType` 枚举
  - [ ] 2.4.3 `JacobianType` 枚举
- [ ] 2.5 绑定数据结构
  - [ ] 2.5.1 `JointLimits` 结构体
  - [ ] 2.5.2 `JointDynamics` 结构体
  - [ ] 2.5.3 `Geometry` 结构体
  - [ ] 2.5.4 `Visual` 结构体
  - [ ] 2.5.5 `Collision` 结构体
  - [ ] 2.5.6 `Inertial` 结构体

## 3. Robot Model Bindings

- [ ] 3.1 绑定 `Link` 类
  - [ ] 3.1.1 `get_name()` 方法
  - [ ] 3.1.2 `get_inertial()` 方法
  - [ ] 3.1.3 `get_visuals()` 方法
  - [ ] 3.1.4 `get_collisions()` 方法
- [ ] 3.2 绑定 `Joint` 类
  - [ ] 3.2.1 `get_name()`, `get_type()` 方法
  - [ ] 3.2.2 `get_parent_link()`, `get_child_link()` 方法
  - [ ] 3.2.3 `get_origin()`, `get_axis()` 方法
  - [ ] 3.2.4 `get_limits()`, `get_dynamics()` 方法
- [ ] 3.3 绑定 `Robot` 类
  - [ ] 3.3.1 `get_name()` 方法
  - [ ] 3.3.2 `get_links()`, `get_joints()` 方法
  - [ ] 3.3.3 `get_link(name)`, `get_joint(name)` 方法
  - [ ] 3.3.4 `get_actuated_joints()` 方法
  - [ ] 3.3.5 添加 `dof` 属性（返回自由度数量）
  - [ ] 3.3.6 添加 `get_joint_names()` 便利方法
  - [ ] 3.3.7 添加 `get_joint_limits()` 返回 NumPy 数组
- [ ] 3.4 绑定 `URDFParser` 类
  - [ ] 3.4.1 `parse_file(filename)` 方法
  - [ ] 3.4.2 `parse_string(urdf_xml)` 方法
  - [ ] 3.4.3 添加 `Robot.from_urdf(path)` 类方法
  - [ ] 3.4.4 添加 `Robot.from_urdf_string(xml)` 类方法

## 4. Kinematics Bindings

- [ ] 4.1 绑定 `ForwardKinematics` 类
  - [ ] 4.1.1 构造函数 `(robot, end_link, base_link="")`
  - [ ] 4.1.2 `compute(joint_angles, check_bounds=False)` 方法
  - [ ] 4.1.3 `compute_to_link(joint_angles, target_link, check_bounds=False)` 方法
  - [ ] 4.1.4 `get_num_joints()` 方法
  - [ ] 4.1.5 添加 `num_joints` 属性
- [ ] 4.2 绑定 `JacobianCalculator` 类
  - [ ] 4.2.1 构造函数 `(robot, end_link, base_link="")`
  - [ ] 4.2.2 `compute(joint_angles, type=JacobianType.Analytic)` 方法
  - [ ] 4.2.3 `is_singular(joint_angles, threshold=1e-6)` 方法
  - [ ] 4.2.4 `get_manipulability(joint_angles)` 方法
  - [ ] 4.2.5 `get_condition_number(joint_angles)` 方法

## 5. Inverse Kinematics Bindings

- [ ] 5.1 绑定 `SolverConfig` 结构体
  - [ ] 5.1.1 所有配置参数字段
  - [ ] 5.1.2 提供合理的默认值
- [ ] 5.2 绑定 `SolverStatus` 结构体
  - [ ] 5.2.1 `converged`, `iterations` 字段
  - [ ] 5.2.2 `final_error_norm`, `final_step_norm` 字段
  - [ ] 5.2.3 `message`, `error_history` 字段
- [ ] 5.3 创建 `IKResult` Python 类包装 `SolverStatus`
  - [ ] 5.3.1 添加 `solution` 字段（NumPy 数组）
  - [ ] 5.3.2 添加 `__repr__` 方法便于调试
- [ ] 5.4 绑定 `SQPIKSolver` 类
  - [ ] 5.4.1 构造函数 `(robot, end_link, base_link="")`
  - [ ] 5.4.2 `solve(target_pose, initial_guess)` 方法
  - [ ] 5.4.3 `set_solver_config(config)` 方法
  - [ ] 5.4.4 `get_solver_config()` 方法
  - [ ] 5.4.5 `set_position_only(enable)` 方法
  - [ ] 5.4.6 `set_orientation_only(enable)` 方法
  - [ ] 5.4.7 `set_warm_start(guess)` 方法
  - [ ] 5.4.8 添加属性访问器（`tolerance`, `max_iterations` 等）

## 6. Python Package Structure

- [ ] 6.1 创建 `bindings/python/urdfx/__init__.py`
  - [ ] 6.1.1 导入所有核心类
  - [ ] 6.1.2 定义 `__version__`
  - [ ] 6.1.3 定义 `__all__` 列表
- [ ] 6.2 创建 `bindings/python/urdfx/py.typed` 标记类型提示
- [ ] 6.3 创建 `bindings/python/urdfx/__init__.pyi` 类型存根文件
  - [ ] 6.3.1 所有类的类型签名
  - [ ] 6.3.2 所有方法的参数和返回类型
  - [ ] 6.3.3 使用 `numpy.typing` 类型注解

## 7. Unit Tests

- [ ] 7.1 创建 `bindings/python/tests/conftest.py` pytest 配置
  - [ ] 7.1.1 定义测试数据路径
  - [ ] 7.1.2 创建共享 fixture（UR5 robot）
- [ ] 7.2 创建 `test_robot_model.py`
  - [ ] 7.2.1 测试 URDF 解析（文件和字符串）
  - [ ] 7.2.2 测试 Robot 属性访问
  - [ ] 7.2.3 测试 Link 和 Joint 访问
  - [ ] 7.2.4 测试错误处理（无效文件、不存在的 link）
- [ ] 7.3 创建 `test_transform.py`
  - [ ] 7.3.1 测试 Transform 创建
  - [ ] 7.3.2 测试位置和四元数提取
  - [ ] 7.3.3 测试矩阵转换
  - [ ] 7.3.4 测试 Transform 组合和逆运算
- [ ] 7.4 创建 `test_forward_kinematics.py`
  - [ ] 7.4.1 测试 FK 计算正确性
  - [ ] 7.4.2 测试 NumPy 数组输入
  - [ ] 7.4.3 测试 Python list 输入
  - [ ] 7.4.4 测试边界检查
  - [ ] 7.4.5 测试错误输入（错误的数组大小）
- [ ] 7.5 创建 `test_jacobian.py`
  - [ ] 7.5.1 测试 Jacobian 计算
  - [ ] 7.5.2 测试奇异性检测
  - [ ] 7.5.3 测试可操作性度量
  - [ ] 7.5.4 验证 Jacobian 形状和数值
- [ ] 7.6 创建 `test_inverse_kinematics.py`
  - [ ] 7.6.1 测试 IK 求解收敛
  - [ ] 7.6.2 测试 IK 结果精度
  - [ ] 7.6.3 测试 position-only 模式
  - [ ] 7.6.4 测试求解器配置
  - [ ] 7.6.5 测试 warm start 功能
- [ ] 7.7 创建 `test_integration.py`
  - [ ] 7.7.1 FK → IK 往返测试
  - [ ] 7.7.2 多目标轨迹测试
  - [ ] 7.7.3 与 C++ 结果对比测试

## 8. Numerical Accuracy Tests

- [ ] 8.1 创建 `bindings/python/tests/test_cpp_parity.py`
  - [ ] 8.1.1 加载 C++ 测试用例数据
  - [ ] 8.1.2 对每个测试用例运行 Python 版本
  - [ ] 8.1.3 验证结果与 C++ 在 1e-10 误差内一致
- [ ] 8.2 创建数值稳定性测试
  - [ ] 8.2.1 测试接近奇异点的配置
  - [ ] 8.2.2 测试接近关节限位的配置

## 9. Performance Benchmarks

- [ ] 9.1 创建 `bindings/python/benchmarks/bench_forward_kinematics.py`
  - [ ] 9.1.1 测量单次 FK 计算时间
  - [ ] 9.1.2 测量批量 FK 计算时间
  - [ ] 9.1.3 与 C++ 性能对比
- [ ] 9.2 创建 `bindings/python/benchmarks/bench_jacobian.py`
  - [ ] 9.2.1 测量 Jacobian 计算时间
  - [ ] 9.2.2 与 C++ 性能对比
- [ ] 9.3 创建 `bindings/python/benchmarks/bench_inverse_kinematics.py`
  - [ ] 9.3.1 测量 IK 求解时间（冷启动）
  - [ ] 9.3.2 测量 IK 求解时间（热启动）
  - [ ] 9.3.3 测量轨迹生成性能
  - [ ] 9.3.4 记录收敛率和迭代次数
  - [ ] 9.3.5 与 C++ 性能对比
- [ ] 9.4 创建 `bindings/python/benchmarks/run_benchmarks.py`
  - [ ] 9.4.1 运行所有 benchmark
  - [ ] 9.4.2 生成性能报告
  - [ ] 9.4.3 验证 Python 开销 < 10%

## 10. Examples

- [ ] 10.1 创建 `examples/python/forward_kinematics.py`
  - [ ] 10.1.1 加载 UR5 URDF
  - [ ] 10.1.2 创建 FK solver
  - [ ] 10.1.3 计算多个关节配置的 FK
  - [ ] 10.1.4 打印结果
- [ ] 10.2 创建 `examples/python/inverse_kinematics.py`
  - [ ] 10.2.1 加载机器人模型
  - [ ] 10.2.2 定义目标位姿
  - [ ] 10.2.3 求解 IK
  - [ ] 10.2.4 验证解的正确性
- [ ] 10.3 创建 `examples/python/trajectory_generation.py`
  - [ ] 10.3.1 定义路径点序列
  - [ ] 10.3.2 使用 warm start 求解轨迹
  - [ ] 10.3.3 可视化关节角度轨迹（使用 matplotlib）
- [ ] 10.4 创建 `examples/python/jacobian_analysis.py`
  - [ ] 10.4.1 计算 Jacobian
  - [ ] 10.4.2 分析可操作性
  - [ ] 10.4.3 检测奇异点
- [ ] 10.5 创建 `examples/python/README.md`
  - [ ] 10.5.1 说明如何运行示例
  - [ ] 10.5.2 列出依赖（matplotlib 等）
  - [ ] 10.5.3 提供预期输出

## 11. Documentation

- [ ] 11.1 更新 `bindings/python/README.md`
  - [ ] 11.1.1 安装说明
  - [ ] 11.1.2 快速开始示例
  - [ ] 11.1.3 API 概览
  - [ ] 11.1.4 构建说明
  - [ ] 11.1.5 测试说明
- [ ] 11.2 添加内联文档字符串到所有绑定类
  - [ ] 11.2.1 Robot 相关类
  - [ ] 11.2.2 Transform 类
  - [ ] 11.2.3 ForwardKinematics 类
  - [ ] 11.2.4 JacobianCalculator 类
  - [ ] 11.2.5 SQPIKSolver 类
- [ ] 11.3 创建 `docs/api/python.md`
  - [ ] 11.3.1 完整 API 参考
  - [ ] 11.3.2 代码示例
  - [ ] 11.3.3 性能注意事项

## 12. CI/CD Integration

- [ ] 12.1 添加 Python bindings 到 CI 构建
  - [ ] 12.1.1 在 Linux CI 中构建和测试
  - [ ] 12.1.2 在 macOS CI 中构建和测试
  - [ ] 12.1.3 在 Windows CI 中构建和测试
- [ ] 12.2 添加多个 Python 版本测试
  - [ ] 12.2.1 Python 3.8
  - [ ] 12.2.2 Python 3.9
  - [ ] 12.2.3 Python 3.10
  - [ ] 12.2.4 Python 3.11
  - [ ] 12.2.5 Python 3.12
- [ ] 12.3 添加多个 NumPy 版本测试
  - [ ] 12.3.1 NumPy 1.20.x
  - [ ] 12.3.2 NumPy 1.26.x (latest)

## 13. Validation

- [ ] 13.1 运行所有单元测试并确保通过
- [ ] 13.2 运行所有集成测试并确保通过
- [ ] 13.3 运行 benchmark 并验证性能目标
  - [ ] 13.3.1 Python 开销 < 10%
  - [ ] 13.3.2 生成性能报告
- [ ] 13.4 运行 mypy 类型检查
- [ ] 13.5 运行 pytest 覆盖率检查（目标 > 90%）
- [ ] 13.6 在真实机器人模型上测试（UR5, UR10, Panda 等）
- [ ] 13.7 验证所有示例可以运行

## 14. Final Cleanup

- [ ] 14.1 代码格式化（black 和 isort）
- [ ] 14.2 移除调试代码和打印语句
- [ ] 14.3 确保所有文件有适当的 license header
- [ ] 14.4 更新根目录 README.md 中的 Python bindings 状态
- [ ] 14.5 更新项目 roadmap（标记 Python bindings 为完成）

## Dependencies Between Tasks

- Task 2 (Core Bindings) must be completed before all other binding tasks
- Task 3 (Robot Model) must be completed before Task 4 and 5
- Task 4 (Kinematics) must be completed before Task 5 (IK)
- Task 6 (Package Structure) must be completed before Task 7 (Tests)
- Task 1-6 must be completed before Task 7 (Tests)
- Task 1-6 must be completed before Task 9 (Benchmarks)
- All tasks except 14 must be completed before Task 14 (Cleanup)

## Parallelizable Work

The following tasks can be worked on in parallel after core dependencies are met:

- Tests (Task 7) and Benchmarks (Task 9) can be developed in parallel
- Examples (Task 10) can be developed alongside Tests
- Documentation (Task 11) can be written alongside implementation
- CI Integration (Task 12) can be set up once Task 1 is complete

