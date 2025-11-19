# Proposal: Restructure Project Layout for Multi-Language Support

**Change ID**: `restructure-project-layout`  
**Status**: Draft  
**Created**: 2025-11-19  
**Author**: AI Assistant

## Problem Statement

当前项目是一个跨语言的复杂项目，核心是 C++ 高性能计算库，同时提供 Python 和 JavaScript/WebAssembly 绑定。然而现有的目录结构存在以下问题：

1. **缺乏语言层级的清晰分离**：Python 绑定代码应该存在但目前没有 `python/` 目录，文档中提到的结构与实际不符
2. **示例代码组织混乱**：只有 C++ 示例在 `examples/` 目录，缺少 Python 和 JavaScript 的独立示例
3. **应用与绑定界限不清**：`visualization/` 是一个完整的 React 应用，但它和 `wasm/` 绑定层的关系在目录结构上不够清晰
4. **测试文件缺乏统一规划**：C++ 测试在 `tests/`，但 Python 和 JavaScript 测试的位置不明确
5. **文档分散**：缺少 `docs/` 目录来统一管理 API 文档、教程和指南

这些问题使得：
- 新贡献者难以理解项目结构
- 跨语言开发时需要在多个不相关的目录间切换
- 构建系统配置变得复杂
- 难以维护各语言的独立发布流程

## Proposed Solution

采用**单体仓库（monorepo）风格的多语言项目结构**，清晰分离核心库、语言绑定、示例应用和文档：

```
urdfx/                          # 项目根目录
├── core/                       # C++ 核心库（重命名自根目录的 src/include）
│   ├── include/urdfx/          # 公共 C++ 头文件
│   ├── src/                    # C++ 实现
│   ├── tests/                  # C++ 单元测试
│   └── CMakeLists.txt          # C++ 核心库构建配置
│
├── bindings/                   # 语言绑定层
│   ├── python/                 # Python 绑定（nanobind）
│   │   ├── src/                # nanobind 绑定代码
│   │   ├── urdfx/              # Python 包代码
│   │   ├── tests/              # Python 测试（pytest）
│   │   ├── CMakeLists.txt      # Python 绑定构建配置
│   │   ├── setup.py            # Python 包安装脚本
│   │   └── pyproject.toml      # Python 项目元数据
│   │
│   └── wasm/                   # WebAssembly 绑定（Emscripten + Embind）
│       ├── src/                # Embind 绑定代码
│       ├── tests/              # WASM 测试（Jest）
│       ├── CMakeLists.txt      # WASM 构建配置
│       ├── package.json        # npm 包配置
│       └── urdfx.d.ts          # TypeScript 类型定义
│
├── examples/                   # 多语言示例代码
│   ├── cpp/                    # C++ 示例
│   │   ├── forward_kinematics.cpp
│   │   ├── inverse_kinematics.cpp
│   │   └── CMakeLists.txt
│   │
│   ├── python/                 # Python 示例
│   │   ├── forward_kinematics.py
│   │   ├── inverse_kinematics.py
│   │   └── requirements.txt
│   │
│   └── javascript/             # JavaScript 示例
│       ├── forward_kinematics.js
│       ├── inverse_kinematics.js
│       └── package.json
│
├── apps/                       # 完整应用（非库代码）
│   └── visualization/          # Three.js 可视化 Web 应用
│       ├── src/
│       ├── public/
│       ├── package.json
│       └── vite.config.ts
│
├── benchmarks/                 # 性能基准测试（保持不变）
│   ├── ik_benchmarks.cpp
│   └── results/
│
├── docs/                       # 项目文档
│   ├── api/                    # API 参考文档
│   │   ├── cpp/                # C++ API (Doxygen 生成)
│   │   ├── python/             # Python API (Sphinx 生成)
│   │   └── javascript/         # JavaScript API
│   ├── guides/                 # 用户指南
│   │   ├── getting-started.md
│   │   ├── urdf-parsing.md
│   │   ├── kinematics.md
│   │   └── inverse-kinematics.md
│   └── tutorials/              # 教程
│       ├── cpp-tutorial.md
│       ├── python-tutorial.md
│       └── web-tutorial.md
│
├── third_party/                # 第三方依赖（git submodules）
├── cmake/                      # CMake 模块和辅助脚本
├── scripts/                    # 构建和开发脚本
├── openspec/                   # OpenSpec 规范和提案
├── .github/                    # GitHub workflows 和模板
│
├── CMakeLists.txt              # 根 CMake 配置
├── README.md                   # 项目概览
├── LICENSE
└── .gitignore
```

### 关键改进

1. **`core/` 目录**：集中所有 C++ 核心库代码、头文件和测试，清晰标识项目的核心
2. **`bindings/` 目录**：按语言组织绑定代码，每个子目录是一个独立的可发布包
3. **`examples/` 重组**：按语言分类示例，每个语言的示例都在各自的子目录中，便于学习和参考
4. **`apps/` 目录**：将 `visualization/` 移到这里，明确它是一个独立应用而非库的一部分
5. **新增 `docs/` 目录**：统一管理 API 文档、指南和教程，支持 Doxygen、Sphinx 等工具生成
6. **保持 `benchmarks/`**：性能测试保持独立，便于 CI/CD 集成

### 迁移对应关系

| 当前路径 | 新路径 | 说明 |
|---------|--------|------|
| `include/urdfx/` | `core/include/urdfx/` | C++ 头文件 |
| `src/` | `core/src/` | C++ 实现 |
| `tests/` | `core/tests/` | C++ 测试 |
| `examples/*.cpp` | `examples/cpp/` | C++ 示例 |
| *(新建)* | `bindings/python/` | Python 绑定（待实现） |
| `wasm/` | `bindings/wasm/` | WASM 绑定 |
| `visualization/` | `apps/visualization/` | Web 可视化应用 |
| *(新建)* | `examples/python/` | Python 示例（待创建） |
| *(新建)* | `examples/javascript/` | JavaScript 示例（待创建） |
| *(新建)* | `docs/` | 统一文档 |

## Benefits

1. **清晰的分层架构**：Core → Bindings → Examples → Apps 的层次结构一目了然
2. **独立的发布流程**：每个 binding 可以独立打包和发布（PyPI、npm）
3. **更好的开发体验**：开发者可以专注于特定语言的代码，无需浏览整个项目
4. **简化 CI/CD**：可以为不同语言设置独立的测试和构建流程
5. **符合社区惯例**：类似于 TensorFlow、PyTorch 等大型跨语言项目的组织方式
6. **便于扩展**：未来添加新语言绑定（如 Rust、Julia）时只需在 `bindings/` 下新增目录

## Risks and Mitigations

### Risk 1: 破坏现有构建系统
- **影响**：CMakeLists.txt 中的所有路径都需要更新
- **缓解措施**：逐步迁移，首先更新 CMake 配置，确保构建通过后再移动文件

### Risk 2: 影响外部依赖项目
- **影响**：如果有项目依赖当前的头文件路径，会导致编译失败
- **缓解措施**：在 v1.0.0 之前进行此重构（项目还未正式发布），并在发布说明中明确指出

### Risk 3: Git 历史和 blame 信息丢失
- **影响**：移动文件后 `git blame` 可能无法追踪原始作者
- **缓解措施**：使用 `git mv` 命令保留历史，并在 `.git-blame-ignore-revs` 中记录重构提交

### Risk 4: 开发中的 PR 需要重新调整
- **影响**：正在进行的功能分支需要解决冲突
- **缓解措施**：选择合适的时机（例如当前 sprint 结束后）进行重构，提前通知团队

## Alternatives Considered

### Alternative 1: 保持现状，仅添加缺失目录
- **优点**：改动最小，风险低
- **缺点**：无法解决根本的组织混乱问题，技术债会继续累积

### Alternative 2: 拆分为多个独立仓库
- **优点**：每个语言绑定完全独立
- **缺点**：增加维护负担，版本同步困难，CI/CD 复杂度高

### Alternative 3: 使用 Bazel 等 monorepo 工具
- **优点**：提供更强大的依赖管理和构建优化
- **缺点**：引入新工具的学习成本，对于当前项目规模来说过于复杂

**选择当前方案的原因**：平衡了清晰性、可维护性和迁移成本，符合项目当前阶段的需求。

## Implementation Phases

### Phase 1: 准备和验证（预计 2 小时）
1. 创建新的目录结构
2. 更新根 CMakeLists.txt，添加新的子目录引用
3. 在不移动文件的情况下验证构建系统可以适应新结构

### Phase 2: 核心库迁移（预计 3 小时）
1. 移动 `include/` → `core/include/`
2. 移动 `src/` → `core/src/`
3. 移动 `tests/` → `core/tests/`
4. 更新 `core/CMakeLists.txt`
5. 验证 C++ 库构建和测试通过

### Phase 3: 绑定和示例迁移（预计 3 小时）
1. 移动 `wasm/` → `bindings/wasm/`
2. 移动 `examples/*.cpp` → `examples/cpp/`
3. 移动 `visualization/` → `apps/visualization/`
4. 创建 `examples/python/` 和 `examples/javascript/` 占位目录
5. 验证 WASM 构建和可视化应用正常工作

### Phase 4: 文档和清理（预计 2 小时）
1. 创建 `docs/` 结构
2. 将 README 中的内容拆分到 `docs/guides/`
3. 更新所有文档中的路径引用
4. 更新 `.github/` workflows 中的路径
5. 清理旧的空目录

### Phase 5: 验证和发布（预计 1 小时）
1. 完整的 CI/CD 测试
2. 更新 CONTRIBUTING.md 和其他元文档
3. 创建迁移指南（Migration Guide）
4. 合并到主分支

## Success Criteria

1. ✅ 所有 C++ 测试通过
2. ✅ WASM 构建成功，大小 < 2MB
3. ✅ 可视化应用在浏览器中正常运行
4. ✅ CI/CD pipeline 全部通过
5. ✅ 文档路径引用全部正确
6. ✅ Git 历史保留（使用 `git log --follow` 可以追踪文件）
7. ✅ 新的目录结构在 README 中有清晰说明

## Open Questions

1. 是否需要立即创建 `bindings/python/` 的实际实现，还是先创建占位符？
   - **建议**：先创建占位符目录和 README，标注为 "Coming Soon"
   
2. `ur5_urdf/` 目录应该放在哪里？
   - **建议**：移动到 `examples/models/` 或 `tests/fixtures/`，因为它主要用于测试和示例

3. 是否需要为每个语言创建独立的 LICENSE 文件？
   - **建议**：不需要，根目录的 LICENSE 适用于所有子项目

4. `build/` 和 `build-wasm/` 目录是否需要调整？
   - **建议**：保持在根目录，在 `.gitignore` 中，不需要移动

## Related Changes

- 需要更新 `openspec/project.md` 中的项目结构说明
- 需要在 `CONTRIBUTING.md` 中说明新的目录约定
- 可能需要更新 CI/CD workflow 文件中的路径

## References

- [TensorFlow Repository Structure](https://github.com/tensorflow/tensorflow)
- [PyTorch Repository Structure](https://github.com/pytorch/pytorch)
- [Modern CMake for C++ Multi-Language Projects](https://cliutils.gitlab.io/modern-cmake/)
- [Monorepo Best Practices](https://monorepo.tools/)
