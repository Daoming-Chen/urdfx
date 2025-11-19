# Change: Add Windows Build Support

## Why
The urdfx project currently only provides a Bash setup script (`setup.sh`) that targets Linux and macOS platforms, making it difficult for Windows developers to build and contribute to the project. Windows is a common development platform, especially for robotics simulation and visualization work, and MSVC is the standard compiler for the Windows ecosystem.

## What Changes
- Create a PowerShell setup script (`setup.ps1`) that mirrors the functionality of `setup.sh` for Windows environments
- Add Windows-specific platform detection and dependency installation logic
- Fix MSVC compiler compatibility issues in CMake configuration and source code
- Ensure all dependencies (Eigen, pugixml, CppAD, DaQP, spdlog) build correctly on Windows with MSVC
- Add Windows-specific compiler flags and definitions where needed
- Update documentation to include Windows build instructions

## Impact
- **Affected specs**: cross-platform
- **Affected code**: 
  - `scripts/setup.ps1` (new)
  - `CMakeLists.txt` (compiler flags, platform detection)
  - `cmake/Dependencies.cmake` (Windows-specific dependency handling)
  - `src/CMakeLists.txt` (potential export definitions for shared libraries)
  - Potentially source files if platform-specific issues are found during compilation
- **Breaking changes**: None - this is purely additive for Windows support
- **Benefits**: 
  - Enables Windows developers to build and use urdfx natively
  - Increases contributor base
  - Improves cross-platform compatibility testing
  - Makes the project accessible to robotics researchers using Windows
