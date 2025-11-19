# Design Document: Multi-Language Project Restructure

**Change ID**: `restructure-project-layout`  
**Status**: Draft  
**Last Updated**: 2025-11-19

## Overview

本文档详细说明 urdfx 项目重构的架构设计决策，重点解决跨语言项目的代码组织挑战。

## Context and Goals

### Current State

urdfx 是一个 C++20 机器人运动学库，提供：
- 核心 C++ 库（URDF 解析、正向/逆向运动学、Jacobian 计算）
- WebAssembly 绑定（通过 Emscripten + Embind）
- 计划中的 Python 绑定（通过 nanobind）
- Three.js 可视化 Web 应用

当前目录结构扁平化，导致：
- 核心库代码（`src/`, `include/`）与绑定代码（`wasm/`）、应用代码（`visualization/`）在同一层级
- 示例代码混杂，缺少语言分类
- 难以独立构建、测试和发布各语言的绑定

### Design Goals

1. **清晰的分层**：Core → Bindings → Examples → Apps
2. **独立性**：每个语言绑定可以独立开发、测试和发布
3. **可扩展性**：方便未来添加新语言绑定（Rust, Julia, Go 等）
4. **符合惯例**：遵循大型跨语言项目的最佳实践
5. **向后兼容**：尽量减少对现有用户的影响（v1.0.0 前完成）

## Architecture Decisions

### AD-1: 采用 Monorepo 而非 Multi-Repo

**决策**：将所有语言绑定保留在同一个仓库中，而不是拆分为多个独立仓库。

**理由**：
- **版本同步简单**：Core 库的变更可以立即反映到所有绑定中
- **CI/CD 简化**：单一 CI pipeline 可以测试所有语言的集成
- **开发效率高**：跨语言的 bug 修复和 feature 开发可以在一个 PR 中完成
- **代码共享**：测试数据（如 URDF 文件）可以在所有绑定间共享

**权衡**：
- ❌ 仓库体积较大（但对于当前规模可接受）
- ❌ 构建时间较长（可通过 CI 缓存优化）
- ✅ 更容易维护一致性
- ✅ 降低版本管理复杂度

**替代方案考虑**：
- **Multi-Repo**：每个绑定独立仓库（如 `urdfx-python`, `urdfx-wasm`）
  - 优点：每个仓库更轻量，发布流程独立
  - 缺点：版本同步困难，需要 git submodule 或复杂的 CI 编排
  - **结论**：对于当前规模（<10 万行代码）不值得

### AD-2: Core 库独立于 Bindings

**决策**：将 C++ 核心库代码移动到 `core/` 目录，与绑定层完全分离。

**设计**：

```
core/
├── include/urdfx/        # Public API (header-only 优先)
├── src/                  # Implementation
└── tests/                # Unit tests

bindings/
├── python/               # Python 绑定（依赖 core）
└── wasm/                 # WASM 绑定（依赖 core）
```

**理由**：
- **明确依赖方向**：Bindings 依赖 Core，反之则不成立
- **独立测试**：Core 可以在没有任何绑定的情况下完整测试
- **性能优化**：Core 可以使用纯 C++ 优化，不受绑定层限制
- **文档生成**：Doxygen 只需扫描 `core/` 目录

**实现细节**：
- `core/CMakeLists.txt` 定义 `urdfx::core` target
- Bindings 通过 `target_link_libraries(urdfx_python PRIVATE urdfx::core)` 引用
- 安装规则：headers 到 `include/urdfx/`, library 到 `lib/`

### AD-3: 按语言组织 Examples

**决策**：将示例代码按语言分类到 `examples/{cpp,python,javascript}/`。

**当前问题**：
- `examples/` 只有 C++ 示例
- Python 和 JavaScript 用户需要自己摸索 API 用法
- 示例代码缺少构建说明

**新设计**：

```
examples/
├── cpp/
│   ├── forward_kinematics.cpp       # 演示 FK 计算
│   ├── inverse_kinematics.cpp       # 演示 IK 求解
│   ├── jacobian_computation.cpp     # 演示 Jacobian
│   └── CMakeLists.txt               # 独立构建配置
├── python/
│   ├── forward_kinematics.py
│   ├── inverse_kinematics.py
│   ├── visualization.py             # Matplotlib 可视化
│   └── requirements.txt
└── javascript/
    ├── forward_kinematics.js        # Node.js 示例
    ├── inverse_kinematics.html      # 浏览器示例
    └── package.json
```

**理由**：
- **学习曲线降低**：每种语言的开发者只看相关示例
- **独立运行**：每个目录都是一个可运行的最小项目
- **文档友好**：README 可以直接引用对应语言的示例

**实现注意事项**：
- 每个示例都应该是自包含的（包括依赖声明）
- C++ 示例通过 CMake `find_package(urdfx)` 引用已安装的库
- Python 示例通过 `import urdfx` 引用 pip 安装的包

### AD-4: 应用层独立（Apps）

**决策**：将完整应用（如 `visualization/`）移动到 `apps/` 目录。

**区分标准**：
- **Library/Binding**：可以被其他项目导入和使用
- **Application**：独立运行的完整程序，有自己的入口点和 UI

**当前问题**：
- `visualization/` 在根目录，与库代码混在一起
- 不清楚它是示例还是产品级应用

**新设计**：

```
apps/
└── visualization/         # Three.js Web 应用
    ├── src/
    ├── public/
    ├── package.json
    ├── vite.config.ts
    └── README.md          # 应用使用说明
```

**未来扩展可能性**：
```
apps/
├── visualization/         # Web 可视化
├── cli/                   # 命令行工具（计划中）
└── desktop/               # Electron 桌面应用（未来）
```

**理由**：
- **清晰的用途**：用户立即知道这是可运行的应用
- **独立维护**：应用有自己的发布周期和版本号
- **避免混淆**：不会被误认为是库的一部分

### AD-5: 统一文档目录（Docs）

**决策**：创建 `docs/` 目录集中管理所有文档，而不是分散在各个目录的 README。

**结构**：

```
docs/
├── api/                   # API 参考文档（自动生成）
│   ├── cpp/               # Doxygen 输出
│   ├── python/            # Sphinx 输出
│   └── javascript/        # TypeDoc 输出
├── guides/                # 用户指南（手写）
│   ├── getting-started.md
│   ├── urdf-parsing.md
│   ├── kinematics.md
│   └── inverse-kinematics.md
└── tutorials/             # 教程（手写 + 代码）
    ├── cpp-tutorial.md
    ├── python-tutorial.md
    └── web-tutorial.md
```

**理由**：
- **便于查找**：所有文档在一个地方
- **工具友好**：静态网站生成器（如 MkDocs）可以直接使用
- **版本控制**：文档与代码同步更新

**与 README 的关系**：
- 根目录 `README.md`：项目概览、快速开始、链接到详细文档
- `docs/guides/`：深入的概念解释和使用指南
- `examples/`：可运行的代码示例

### AD-6: CMake 构建系统的分层

**决策**：采用分层的 CMake 配置，每个子项目有独立的 CMakeLists.txt。

**结构**：

```
CMakeLists.txt                    # 根配置（版本、选项、子目录）
├── core/CMakeLists.txt           # C++ 核心库
├── bindings/
│   ├── python/CMakeLists.txt     # Python 绑定（条件编译）
│   └── wasm/CMakeLists.txt       # WASM 绑定（条件编译）
├── examples/
│   └── cpp/CMakeLists.txt        # C++ 示例
└── benchmarks/CMakeLists.txt     # 性能测试
```

**关键设计**：

1. **根 CMakeLists.txt**：

```cmake
project(urdfx VERSION 1.0.0)

option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
option(BUILD_WASM "Build WebAssembly module" OFF)
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_TESTING "Build tests" ON)

add_subdirectory(core)

if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(bindings/python)
endif()

if(BUILD_WASM)
    add_subdirectory(bindings/wasm)
endif()

if(BUILD_EXAMPLES)
    add_subdirectory(examples/cpp)
endif()
```

2. **Core CMakeLists.txt**：

```cmake
add_library(urdfx_core
    src/robot_model.cpp
    src/urdf_parser.cpp
    src/kinematics.cpp
    src/inverse_kinematics.cpp
)

target_include_directories(urdfx_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_library(urdfx::core ALIAS urdfx_core)

install(TARGETS urdfx_core EXPORT urdfxTargets)
install(DIRECTORY include/ DESTINATION include)
```

3. **Bindings CMakeLists.txt**：

```cmake
# bindings/wasm/CMakeLists.txt
target_link_libraries(urdfx_wasm PRIVATE urdfx::core)
```

**理由**：
- **选择性构建**：开发者可以只构建需要的部分
- **依赖清晰**：通过 CMake target 明确依赖关系
- **安装支持**：`cmake --install` 可以正确安装头文件和库

### AD-7: 测试策略的分层

**决策**：每个层级有独立的测试，但共享测试数据。

**测试类型**：

1. **Core Tests** (`core/tests/`)：
   - 单元测试（GTest）
   - 性能测试（Google Benchmark）
   - 不依赖任何绑定

2. **Binding Tests**：
   - Python: `bindings/python/tests/` (pytest)
   - WASM: `bindings/wasm/tests/` (Jest)
   - 测试绑定层的正确性和性能

3. **Integration Tests** (`tests/integration/` - 未来)：
   - 测试多个组件的协同工作
   - 例如：URDF → FK → IK → 验证

**共享测试数据**：

```
tests/fixtures/              # 或 examples/models/
└── ur5/
    ├── ur5e.urdf
    └── meshes/
```

- 所有测试引用统一的测试数据
- 避免重复存储相同的 URDF 文件

## Implementation Considerations

### Migration Strategy

**逐步迁移，保持构建始终可用**：

1. **Phase 1**：创建新目录，更新 CMake（不移动文件）
2. **Phase 2**：移动核心文件（`include`, `src`, `tests`）
3. **Phase 3**：移动绑定和应用
4. **Phase 4**：更新文档和清理

每个阶段结束后都要确保：
- ✅ CMake 配置成功
- ✅ 构建成功
- ✅ 所有测试通过

### Git History Preservation

使用 `git mv` 而不是手动删除和创建文件：

```bash
git mv include core/include
git mv src core/src
git mv tests core/tests
```

这样 `git log --follow` 可以追踪文件历史。

对于大规模重构，在 `.git-blame-ignore-revs` 中记录提交：

```
# Restructure project layout for multi-language support
abc123def456...
```

### External Impact

**影响分析**：

1. **Include 路径变化**：
   - 旧：`#include <urdfx/robot_model.h>`（查找 `include/urdfx/`）
   - 新：`#include <urdfx/robot_model.h>`（查找 `core/include/urdfx/`）
   - **缓解**：安装后路径不变（`/usr/local/include/urdfx/`）

2. **CMake Find Package**：
   - 旧：`find_package(urdfx)`
   - 新：`find_package(urdfx)` + `target_link_libraries(... urdfx::core)`
   - **缓解**：在 `urdfxConfig.cmake` 中提供兼容性 alias

3. **Python Import**：
   - 当前无影响（Python 绑定尚未实现）
   - 未来：`import urdfx` 保持不变

4. **WASM 模块**：
   - 当前：从 `build-wasm/` 输出
   - 未来：从 `build/bindings/wasm/` 输出
   - **缓解**：更新 `visualization/` 的加载路径

### Performance Considerations

**构建性能**：
- 分层 CMake 可以提高增量构建速度
- 只修改 Python 绑定时，不需要重新编译 Core

**运行时性能**：
- 无影响（目录结构不影响编译后的二进制）

### Security Considerations

- 无新的安全风险
- 继续使用 submodules 管理依赖（避免供应链攻击）

## Future Extensions

### Adding New Language Bindings

当需要添加新语言（例如 Rust）时：

1. 创建 `bindings/rust/` 目录
2. 添加 `bindings/rust/CMakeLists.txt` 或 `Cargo.toml`
3. 实现绑定层（使用 `cxx` 或 `rust-bindgen`）
4. 添加 `examples/rust/` 示例
5. 更新根 `CMakeLists.txt` 添加 `BUILD_RUST_BINDINGS` 选项

### Separate Documentation Site

未来可以使用 MkDocs/Docusaurus 生成文档网站：

```
docs/
├── mkdocs.yml            # MkDocs 配置
├── index.md              # 主页
├── api/                  # API 引用（链接到生成的文档）
├── guides/               # 手写指南
└── tutorials/            # 手写教程
```

部署到 GitHub Pages：`https://username.github.io/urdfx/`

## Alternatives Revisited

### Why Not Bazel?

Bazel 提供强大的 monorepo 支持，但：
- **学习曲线陡峭**：团队需要学习新的构建系统
- **工具链复杂**：需要配置 C++, Python, WASM 的 Bazel 规则
- **过度设计**：当前项目规模（<50k LOC）不需要 Bazel 的规模

**何时考虑 Bazel**：
- 项目增长到 >100k LOC
- 需要严格的依赖隔离和可重现构建
- 有专门的 DevOps 工程师维护构建系统

### Why Not Conan for Dependencies?

Conan 可以管理 C++ 依赖，但：
- **Submodules 已经工作良好**：Eigen、pugixml 等都是 header-only 或易于编译
- **跨平台挑战**：Conan 在 WASM 支持上不如 Emscripten 原生工具链
- **额外复杂度**：需要维护 `conanfile.py` 和处理版本冲突

**何时考虑 Conan**：
- 依赖项数量 >10
- 需要管理多个版本的同一个库
- 团队已经熟悉 Conan 工作流

## Risks and Mitigation

### Risk 1: 破坏现有用户的构建

**概率**: 中  
**影响**: 高

**缓解措施**：
- 在 v1.0.0 正式发布前完成重构
- 提供详细的 Migration Guide
- 在 CMake 中提供向后兼容的 target alias

### Risk 2: CI/CD Pipeline 失败

**概率**: 高（第一次运行时）  
**影响**: 中

**缓解措施**：
- 在 feature 分支上先测试 CI
- 逐个修复路径问题
- 使用 CI cache 加速重建

### Risk 3: 文档链接失效

**概率**: 高  
**影响**: 低

**缓解措施**：
- 使用 `markdown-link-check` 工具验证链接
- 在 CI 中添加文档验证步骤

### Risk 4: Git Blame 信息丢失

**概率**: 低（如果使用 `git mv`）  
**影响**: 低

**缓解措施**：
- 始终使用 `git mv` 而不是手动移动
- 使用 `.git-blame-ignore-revs` 记录重构提交
- 团队培训：使用 `git log --follow <file>` 查看历史

## Success Criteria

1. ✅ 所有 C++ 测试通过（100% pass rate）
2. ✅ WASM 构建成功且大小 < 2MB
3. ✅ Visualization 应用正常运行
4. ✅ CI/CD pipeline 全部通过
5. ✅ 文档链接无失效（0 broken links）
6. ✅ Git 历史可追踪（`git log --follow` 可用）
7. ✅ 新贡献者能在 < 30 分钟内理解项目结构

## References

- [CMake Best Practices for Multi-Language Projects](https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html)
- [Monorepo.tools](https://monorepo.tools/)
- [Google's Approach to Monorepos](https://research.google/pubs/pub45424/)
- [TensorFlow Repository Structure](https://github.com/tensorflow/tensorflow)
- [PyTorch Repository Structure](https://github.com/pytorch/pytorch)
