# Tasks: Add Mixed-Chain Benchmark Suite (MixKinBench)

## Phase 1: 基础设施与标准模型

- [x] 1.1 创建 Python 基准测试数据生成目录结构
  - `bindings/python/benchmarks/mixkinbench/`
  - 添加 `__init__.py`, `generator.py`, `dataset.py`, `oracle.py`

- [x] 1.2 实现 `MixedChainGenerator` 类
  - 输入: DOF 数量, 移动关节概率, 随机种子
  - 输出: URDF 字符串或文件
  - 支持配置链节长度范围和关节限制

- [x] 1.3 验证生成的 URDF 可被 urdfx 解析
  - 生成 10-DOF 示例机器人
  - 使用 `urdfx.URDFParser` 解析并创建 Robot 对象
  - 验证关节数量、类型、限制正确

- [x] 1.4 添加可视化验证工具(可选)
  - 使用 PyBullet 或 Matplotlib 显示生成的机器人
  - 验证链节连接和几何合理性

- [ ] 1.5 获取标准 6-DOF 和 7-DOF 机器人 URDF
  - UR5e (已有)
  - Franka Panda (新增或引用外部)

## Phase 2: Forward Kinematics Oracle 实现

- [x] 2.1 实现 FK Oracle 接口
  - 优先使用 urdfx Python 绑定的 `ForwardKinematics` 类
  - 备选: 集成 Pinocchio(作为可选依赖)

- [x] 2.2 验证 FK Oracle 准确性
  - 对比 urdfx C++ 和 Python FK 结果
  - 使用 UR5e 机器人进行数值验证(误差 < 1e-6)

- [x] 2.3 实现关节采样器
  - 在关节限制内均匀采样
  - 支持高斯分布采样(集中在工作空间中心)
  - 避免奇异位形(可选)

## Phase 3: 基准测试数据集生成

- [x] 3.1 实现 `BenchmarkDataset` 类
  - 存储: 目标位姿, 初始猜测, ground truth 关节角度
  - 序列化为 JSON 或 NumPy .npz 格式
  - 支持批量加载和迭代

- [ ] 3.2 生成 Tier A 标准机器人数据集
  - UR5e: 1000 个可达目标
  - Panda (如果可用): 1000 个可达目标
  - 保存为 `benchmarks/datasets/tier_a_ur5e.npz`

- [x] 3.3 生成 Tier B 合成机器人数据集
  - 10-DOF 混合链: 1000 个目标 (70% R + 30% P)
  - 20-DOF 混合链: 1000 个目标
  - 50-DOF 混合链: 500 个目标
  - 100-DOF 混合链: 200 个目标
  - 保存为 `benchmarks/datasets/tier_b_*dof.npz`

- [x] 3.4 生成初始猜测变化
  - 冷启动: $q_{init} = 0$ 或随机远离 $q_{gt}$
  - 热启动: $q_{init}$ 在 $q_{gt}$ 附近(高斯噪声 $\sigma = 0.1$)
  - 每个目标生成 3 个初始猜测变体

- [x] 3.5 生成不可达目标数据集(可选)
  - 目标位于工作空间边缘或外部
  - 用于测试"最接近解"行为
  - 保存为 `benchmarks/datasets/unreachable_*.npz`

## Phase 4: C++ 基准测试集成

- [x] 4.1 扩展 `IKBenchmarkFixture` 支持动态加载数据集
  - 从 .npz 或 JSON 文件加载测试用例（实现了二进制格式加载）
  - 支持可变自由度的机器人模型

- [x] 4.2 实现 `BM_IK_MixedChain_NxDOF` 基准测试
  - 参数化 DOF (10, 20, 50, 100)
  - 测量: 成功率, 平均迭代次数, 执行时间, 位置/旋转误差

- [x] 4.3 实现 `BM_IK_JointTypeSensitivity` 基准测试
  - 分别统计旋转关节和移动关节的误差
  - 计算关节类型对收敛性的影响

- [x] 4.4 实现 `BM_IK_ColdVsWarmStart` 基准测试
  - 对比冷启动和热启动的性能差异
  - 测量初始猜测质量对收敛速度的影响

- [x] 4.5 添加基准测试到 CMakeLists.txt
  - `benchmarks/mixed_ik_benchmarks.cpp`
  - 链接必要的库(urdfx, Eigen, Google Benchmark)

## Phase 5: 报告生成与可视化

- [x] 5.1 实现基准测试结果解析器(Python)
  - 读取 Google Benchmark JSON 输出
  - 提取关键指标并统计分析

- [x] 5.2 生成 Markdown 格式报告
  - 包含表格: DOF vs 成功率 vs 时间
  - 包含图表: 收敛曲线, 误差分布
  - 保存为 `benchmarks/results/mixkinbench_report_<timestamp>.md`

- [ ] 5.3 实现交互式可视化(可选)
  - 使用 Plotly 或 Matplotlib 生成 HTML 报告
  - 支持多维度数据对比(DOF, 关节类型, 启动方式)

- [ ] 5.4 添加 CI/CD 集成
  - 在 GitHub Actions 中运行基准测试
  - 自动生成并发布报告到 GitHub Pages 或 Artifacts

## Phase 6: 文档与验证

- [x] 6.1 编写用户指南 `docs/benchmarks/mixkinbench.md`
  - 快速入门: 如何运行基准测试
  - 数据集格式说明
  - 如何生成自定义机器人

- [x] 6.2 添加 Python 脚本的命令行接口
  - `bindings/python/benchmarks/mixkinbench/cli.py`
  - 支持: `generate`, `validate`, `run`, `report` 子命令

- [x] 6.3 编写单元测试
  - 测试 `MixedChainGenerator` 生成的 URDF 格式正确（部分完成：test_generator.py）
  - 测试 FK Oracle 数值精度
  - 测试数据集序列化和反序列化

- [ ] 6.4 端到端验证
  - 生成完整的基准测试数据集
  - 运行所有 C++ 基准测试
  - 验证报告生成正确

- [ ] 6.5 更新主 README.md
  - 添加 MixKinBench 简介
  - 链接到详细文档

## Phase 7: 可选扩展

- [ ] 7.1 支持树形结构机器人(多分支)
- [ ] 7.2 添加任务空间约束测试(如位置固定, 仅姿态变化)
- [ ] 7.3 集成其他 IK 求解器进行对比(如 TRAC-IK, KDL)
- [ ] 7.4 提供预生成的数据集下载链接(减少首次运行时间)

