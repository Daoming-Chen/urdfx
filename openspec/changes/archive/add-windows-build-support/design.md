# Design: Windows Build Support

## Context
The urdfx project currently supports Linux and macOS through a Bash-based setup script and CMake build system. Windows developers need native support to build the library with MSVC, which is the standard compiler for Windows development. This change adds full Windows platform support while maintaining backward compatibility with existing Linux/macOS workflows.

**Constraints**:
- Must maintain cross-platform CMake configuration
- Must support MSVC 2019+ (version 19.29+) for C++20 features
- Must not break existing Linux/macOS builds
- Should follow Windows conventions (PowerShell scripts, backslash paths where appropriate)

**Stakeholders**:
- Windows developers wanting to use or contribute to urdfx
- CI/CD systems that need to test Windows builds
- Robotics researchers using Windows for simulation and visualization

## Goals / Non-Goals

**Goals**:
- Enable Windows developers to build urdfx with MSVC using a simple setup script
- Fix all MSVC-specific compilation errors and warnings
- Ensure all dependencies work correctly on Windows
- Provide clear documentation for Windows build process
- Support both static and shared library builds on Windows
- Enable Python and WASM bindings to build on Windows

**Non-Goals**:
- Supporting older MSVC versions (< 2019)
- Supporting MinGW or Cygwin (MSVC only for now)
- Changing existing Linux/macOS build process
- Adding Windows-specific features to the library
- Supporting Windows ARM64 (x64 only initially)

## Decisions

### Decision 1: PowerShell over Batch Script
**What**: Use PowerShell 5.1+ for the Windows setup script instead of CMD batch files.

**Why**: 
- PowerShell is the modern scripting environment for Windows
- Better error handling and cross-platform support (PowerShell Core)
- More readable and maintainable than batch files
- Consistent with modern Windows development practices
- Windows 10/11 include PowerShell 5.1+ by default

**Alternatives considered**:
- Batch files: Too limited, poor error handling
- Python script: Adds chicken-egg dependency problem (need Python to install Python)
- WSL/Bash: Defeats purpose of native Windows support

### Decision 2: MSVC as Primary Windows Compiler
**What**: Support MSVC (Visual Studio) as the primary and officially supported compiler on Windows.

**Why**:
- MSVC is the native Windows compiler with best platform integration
- CMake already configured with MSVC version check (19.29+)
- Best debugging experience with Visual Studio
- Standard for professional Windows development

**Alternatives considered**:
- MinGW/GCC: Cross-platform but not native, ABI compatibility issues
- Clang-CL: Possible future addition but MSVC first for simplicity

### Decision 3: Chocolatey for Dependency Management
**What**: Recommend Chocolatey package manager for installing dependencies but also provide manual installation instructions.

**Why**:
- Chocolatey is the de facto package manager for Windows developers
- Simplifies CMake, Python, Node.js installation
- Similar to apt/yum/brew experience on other platforms
- Easy to automate in CI/CD

**Alternatives considered**:
- Manual downloads only: Too error-prone and time-consuming
- Scoop: Less popular, smaller package repository
- vcpkg: More for C++ libraries than system tools

### Decision 4: Shared Library Export Macros
**What**: Add proper `__declspec(dllexport)` and `__declspec(dllimport)` macros for Windows DLL builds.

**Why**:
- Required for shared library builds on Windows
- Standard Windows DLL development practice
- Prevents linker errors when building/using DLLs

**Implementation**:
```cpp
// In a header like urdfx/export.h
#ifdef _MSC_VER
  #ifdef URDFX_BUILD
    #define URDFX_API __declspec(dllexport)
  #else
    #define URDFX_API __declspec(dllimport)
  #endif
#else
  #define URDFX_API
#endif
```

**Alternatives considered**:
- Static library only: Limits flexibility for users
- No export macros: Would break Windows DLL builds

### Decision 5: Path Handling Strategy
**What**: Use CMake's cross-platform path handling and avoid hardcoding path separators in code.

**Why**:
- CMake automatically handles path separators
- C++ standard library filesystem supports both separators
- Minimizes platform-specific code

**Implementation**:
- Use `CMAKE_CURRENT_SOURCE_DIR` and other CMake path variables
- Use `std::filesystem::path` in C++ code (C++17 feature, available in C++20)
- Avoid string concatenation for paths

## Risks / Trade-offs

### Risk 1: MSVC Warning Differences
**Risk**: MSVC may produce different warnings than GCC/Clang, potentially failing with `/W4 /WX`.

**Mitigation**:
- Start with `/W3` instead of `/W4` if needed
- Use `#pragma warning(disable: xxxx)` sparingly for legitimate cases
- Keep warning level consistent with GCC `-Wall -Wextra -Werror`

### Risk 2: Dependency Build Issues
**Risk**: Third-party dependencies may not build cleanly on Windows/MSVC.

**Mitigation**:
- Test each dependency individually
- Update git submodules to latest stable versions if needed
- Add Windows-specific CMake configuration for problematic dependencies
- Document any required patches or workarounds

### Risk 3: C++20 Feature Support
**Risk**: Some C++20 features may have incomplete or different support in MSVC.

**Mitigation**:
- Already require MSVC 19.29+ (VS 2019 16.11) which has good C++20 support
- Test compilation on Windows early
- Use portable C++20 features (avoid compiler-specific extensions)
- Document any MSVC-specific issues in code comments

### Risk 4: Path Length Limitations
**Risk**: Windows has 260-character MAX_PATH limitation (though newer Windows 10+ can disable this).

**Mitigation**:
- Keep build directory names short
- Use build directory at repository root (e.g., `./build`)
- Document Windows long path configuration if needed
- Consider UNC path prefix (`\\?\`) for deeply nested structures

## Migration Plan

### Phase 1: Setup Script (Immediate)
1. Create `scripts/setup.ps1`
2. Test on clean Windows 10/11 installation
3. Document usage in README.md

### Phase 2: Build System (Immediate)
1. Fix CMake configuration for MSVC
2. Test dependency builds
3. Fix compilation errors
4. Validate tests pass

### Phase 3: Documentation (Immediate)
1. Update README.md with Windows instructions
2. Update project.md with Windows platform info
3. Add troubleshooting guide

### Phase 4: CI/CD (Future)
1. Add Windows build to CI pipeline (if applicable)
2. Test matrix: Windows 10/11, Debug/Release, Static/Shared

### Rollback Strategy
- Changes are additive (new files, conditional platform code)
- No changes to existing Linux/macOS functionality
- Can revert new files without affecting other platforms
- Git submodule dependencies remain unchanged

## Open Questions

1. **Q**: Should we support Windows on ARM64?  
   **A**: Not in initial implementation. Focus on x64 first, can add ARM64 later if demand exists.

2. **Q**: Should we support MinGW or Clang-CL as alternative Windows compilers?  
   **A**: Not in initial implementation. MSVC first, can add others later based on community feedback.

3. **Q**: What's the minimum Windows version?  
   **A**: Windows 10 (version 1809 or later) or Windows 11. Covers vast majority of Windows developers.

4. **Q**: Should setup.ps1 automatically install dependencies or just check them?  
   **A**: Check first, then prompt user with installation commands. Avoid automatic installation to respect user control.

5. **Q**: Do we need special handling for vcpkg integration?  
   **A**: Not initially. Use git submodules as primary dependency method, keep vcpkg as optional user choice.
