# Change: Add Mixed-Chain Benchmark Suite (MixKinBench)

## Why

当前的基准测试主要针对标准的 6-7 自由度工业机器人(如 UR5、Panda)。为了验证数值逆运动学求解器的性能、鲁棒性和可扩展性,需要一个全面的基准测试套件,支持高自由度(10+ 到 100+ DOF)和混合关节类型(旋转 + 移动关节)的串行机械臂。

混合关节拓扑创建了复杂的工作空间流形,并为数值求解器带来不同的梯度缩放行为(弧度 vs 米),这是现有基准测试未充分覆盖的关键场景。

## What Changes

- **新增能力**: `synthetic-robot-generation` - 程序化生成 N-DOF 混合关节串行链的能力
  - 支持配置旋转/移动关节的比例(例如 R-P-R-R-P...)
  - 基于随机种子或配置对象生成运动学链
  - 输出 URDF 格式的机器人描述
  - 定义合理的关节限制(移动关节避免无限工作空间)

- **扩展能力**: `benchmark-infrastructure` - 增强现有基准测试基础设施
  - 支持可变自由度 (N=6 到 N=100) 的基准测试
  - 生成可达目标和不可达目标的数据集
  - 添加冷启动 vs 热启动的初始猜测变化
  - 报告成功率、迭代次数、执行时间、关节类型敏感性等指标

- **分层测试策略**:
  - **Tier A (标准)**: 工业标准 6-7 DOF 机器人 (UR5, Panda) 作为基准
  - **Tier B (合成)**: 程序化生成的 N-DOF 混合链 (10, 20, 50, 100 DOF)

- **Python 工具链**: 使用 Python 进行基准测试数据生成和分析(利用 NumPy/Pinocchio 作为 FK oracle)

## Impact

- **新增规范**: 
  - `specs/synthetic-robot-generation/spec.md` - 定义合成机器人生成器的需求
  - `specs/benchmark-infrastructure/spec.md` - 定义基准测试基础设施的扩展需求

- **影响的代码**:
  - 新增: `bindings/python/benchmarks/mixed_chain_generator.py` - 合成机器人生成器
  - 新增: `bindings/python/benchmarks/benchmark_dataset.py` - 数据集生成工具
  - 新增: `benchmarks/mixed_ik_benchmarks.cpp` - C++ 基准测试入口
  - 扩展: `bindings/python/benchmarks/run_benchmarks.py` - 集成新的基准测试套件
  - 新增: `benchmarks/results/mixed_chain_report.md` - 基准测试报告模板

- **文档**:
  - 新增: `docs/benchmarks/mixkinbench.md` - 使用指南和设计文档

- **依赖项**: 可选依赖 `pinocchio` (仅用于基准测试,不影响核心库)

