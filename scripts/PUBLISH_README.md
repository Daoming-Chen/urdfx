# npm Publishing Scripts

这个目录包含用于将 urdfx WASM 绑定发布到 npm 的脚本。

## 脚本文件

### Windows
- **`publish-npm.ps1`** - Windows PowerShell 发布脚本

### Linux/macOS
- **`publish-npm.sh`** - Bash 发布脚本

## 使用方法

### Windows (PowerShell)

```powershell
# 基本发布（会提示确认）
.\scripts\publish-npm.ps1

# 指定版本号
.\scripts\publish-npm.ps1 -Version "1.0.1"

# 干运行（不实际发布）
.\scripts\publish-npm.ps1 -DryRun

# 跳过构建步骤（如果已经构建过）
.\scripts\publish-npm.ps1 -SkipBuild

# 组合使用
.\scripts\publish-npm.ps1 -Version "1.0.2" -DryRun
```

### Linux/macOS (Bash)

首先确保脚本有执行权限：
```bash
chmod +x scripts/publish-npm.sh
```

然后运行：
```bash
# 基本发布（会提示确认）
./scripts/publish-npm.sh

# 指定版本号
./scripts/publish-npm.sh --version "1.0.1"

# 干运行（不实际发布）
./scripts/publish-npm.sh --dry-run

# 跳过构建步骤（如果已经构建过）
./scripts/publish-npm.sh --skip-build

# 组合使用
./scripts/publish-npm.sh --version "1.0.2" --dry-run
```

## 脚本功能

这些脚本会自动完成以下步骤：

1. ✅ **检查 npm 认证** - 确认你已登录 npm
2. ✅ **构建 WASM 模块** - 调用 `build-wasm.ps1/sh` 编译 WASM
3. ✅ **验证构建产物** - 检查 urdfx.js 和 urdfx.wasm 是否存在
4. ✅ **准备发布目录** - 创建 dist 目录并复制所有必需文件
5. ✅ **生成 package.json** - 创建或更新包配置文件
6. ✅ **预览包内容** - 运行 `npm pack --dry-run` 查看将发布的内容
7. ✅ **发布到 npm** - 执行 `npm publish`（需要确认）
8. ✅ **清理说明** - 提示清理临时文件

## 命令行选项

### Windows PowerShell

| 选项 | 说明 |
|------|------|
| `-Version <版本号>` | 设置包版本号（如 "1.0.1"） |
| `-DryRun` | 干运行模式，不实际发布 |
| `-SkipBuild` | 跳过 WASM 构建步骤 |
| `-SkipTests` | 跳过测试步骤（保留用于未来） |

### Linux/macOS Bash

| 选项 | 说明 |
|------|------|
| `--version <版本号>` | 设置包版本号（如 "1.0.1"） |
| `--dry-run` | 干运行模式，不实际发布 |
| `--skip-build` | 跳过 WASM 构建步骤 |
| `--skip-tests` | 跳过测试步骤（保留用于未来） |
| `-h, --help` | 显示帮助信息 |

## 发布前检查清单

在运行发布脚本之前，请确保：

- [ ] 已登录 npm：`npm login`
- [ ] 已提交所有代码更改
- [ ] 已更新 CHANGELOG（如果有）
- [ ] 已测试 WASM 构建正常工作
- [ ] 已确认要发布的版本号

## 发布流程示例

### 首次发布

```powershell
# Windows
.\scripts\publish-npm.ps1 -Version "1.0.0"
```

```bash
# Linux/macOS
./scripts/publish-npm.sh --version "1.0.0"
```

### 发布补丁版本

```powershell
# Windows - 先干运行测试
.\scripts\publish-npm.ps1 -Version "1.0.1" -DryRun

# 确认无误后正式发布
.\scripts\publish-npm.ps1 -Version "1.0.1"
```

```bash
# Linux/macOS - 先干运行测试
./scripts/publish-npm.sh --version "1.0.1" --dry-run

# 确认无误后正式发布
./scripts/publish-npm.sh --version "1.0.1"
```

### 快速重新发布（已构建过）

```powershell
# Windows
.\scripts\publish-npm.ps1 -Version "1.0.2" -SkipBuild
```

```bash
# Linux/macOS
./scripts/publish-npm.sh --version "1.0.2" --skip-build
```

## 发布后验证

发布成功后，可以通过以下方式验证：

```bash
# 查看包信息
npm view urdfx

# 在新目录中测试安装
mkdir test-install && cd test-install
npm init -y
npm install urdfx
node -e "require('urdfx').then(m => console.log('✓ Package works!'))"
```

## 故障排除

### 认证失败
```
✗ Not logged in to npm. Please run 'npm login' first.
```
**解决方案**：运行 `npm login` 并按提示登录

### 构建失败
```
✗ WASM build failed
```
**解决方案**：
1. 检查是否安装了 Emscripten
2. 运行 `emsdk activate latest`
3. 检查 CMake 和 C++ 编译器是否正常

### 包名冲突
```
npm ERR! 403 Forbidden - PUT https://registry.npmjs.org/urdfx
```
**解决方案**：包名已被占用，需要更改 package.json 中的包名

### 版本冲突
```
npm ERR! 403 Forbidden - You cannot publish over the previously published version
```
**解决方案**：需要使用更高的版本号

## 目录结构

发布脚本会创建以下目录结构：

```
bindings/wasm/dist/
├── urdfx.js          # WASM 加载器和绑定
├── urdfx.wasm        # WASM 二进制文件
├── urdfx.d.ts        # TypeScript 类型定义
├── README.md         # 包文档
├── LICENSE           # MIT 许可证
├── package.json      # npm 包配置
└── .npmignore        # npm 忽略文件
```

## 清理

发布后，dist 目录可以安全删除：

```powershell
# Windows
Remove-Item -Recurse -Force bindings\wasm\dist
```

```bash
# Linux/macOS
rm -rf bindings/wasm/dist
```

构建产物也可以清理：

```powershell
# Windows
Remove-Item -Recurse -Force build-wasm
```

```bash
# Linux/macOS
rm -rf build-wasm
```

## 版本管理建议

- **补丁版本** (1.0.x)：错误修复、小改进
- **次要版本** (1.x.0)：新功能、向后兼容
- **主要版本** (x.0.0)：破坏性更改

遵循[语义化版本](https://semver.org/lang/zh-CN/)规范。

## 自动化提示

可以将这些脚本集成到 CI/CD 流程中：

### GitHub Actions 示例

```yaml
name: Publish to npm

on:
  release:
    types: [created]

jobs:
  publish:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: '18'
          registry-url: 'https://registry.npmjs.org'
      - name: Install Emscripten
        run: |
          git clone https://github.com/emscripten-core/emsdk.git
          cd emsdk
          ./emsdk install latest
          ./emsdk activate latest
      - name: Publish to npm
        run: ./scripts/publish-npm.sh --version ${{ github.event.release.tag_name }} --skip-tests
        env:
          NODE_AUTH_TOKEN: ${{ secrets.NPM_TOKEN }}
```

## 支持

如有问题，请提交 Issue：https://github.com/Daoming-Chen/urdfx/issues
