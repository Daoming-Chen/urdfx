# Cross Platform Specification - Delta Changes

## ADDED Requirements

### Requirement: Windows Platform Support
The cross platform SHALL support building urdfx on Windows 10 (version 1809+) and Windows 11 with MSVC compiler version 19.29 or later (Visual Studio 2019 16.11+).

#### Scenario: Windows developer builds library successfully
- **WHEN** a developer runs the Windows setup script on a clean Windows installation
- **THEN** all required dependencies are detected or installation instructions are provided
- **AND** CMake configures the project successfully with MSVC
- **AND** the library compiles without errors in both Debug and Release configurations

#### Scenario: MSVC compiler version check
- **WHEN** CMake configuration is run on Windows
- **THEN** the build system SHALL verify MSVC version is at least 19.29
- **AND** a clear error message is shown if version is too old
- **AND** the error message SHALL specify minimum Visual Studio version required

#### Scenario: Shared library build on Windows
- **WHEN** BUILD_SHARED_LIBS is ON on Windows
- **THEN** the library SHALL build as a DLL with proper export/import declarations
- **AND** dependent executables SHALL link correctly to the DLL
- **AND** the DLL and import library SHALL be installed to correct directories

### Requirement: PowerShell Setup Script
The project SHALL provide a PowerShell setup script (`scripts/setup.ps1`) that checks for and guides installation of required dependencies on Windows.

#### Scenario: Check for CMake installation
- **WHEN** the setup script runs
- **THEN** it SHALL detect if CMake 3.20+ is installed
- **AND** if not found, SHALL provide installation instructions via Chocolatey or manual download
- **AND** if found but version is too old, SHALL warn user and provide upgrade instructions

#### Scenario: Check for Python installation
- **WHEN** the setup script runs
- **THEN** it SHALL detect if Python 3.8+ is installed
- **AND** if not found, SHALL provide installation instructions
- **AND** SHALL verify Python is accessible from PATH

#### Scenario: Check for Node.js installation
- **WHEN** the setup script runs
- **THEN** it SHALL detect if Node.js is installed
- **AND** if not found, SHALL provide installation instructions for LTS version
- **AND** SHALL verify Node.js and npm are accessible from PATH

#### Scenario: Check for Visual Studio/MSVC
- **WHEN** the setup script runs
- **THEN** it SHALL detect if Visual Studio 2019+ or Build Tools are installed
- **AND** SHALL verify MSVC version meets minimum requirements (19.29+)
- **AND** if not found, SHALL provide installation instructions
- **AND** SHALL recommend Visual Studio Community Edition or Build Tools

#### Scenario: Setup Emscripten for Windows
- **WHEN** the setup script runs and user wants WASM support
- **THEN** it SHALL check for Emscripten SDK in third_party/emsdk
- **AND** if not found, SHALL clone the repository
- **AND** SHALL provide instructions to run emsdk install and activate commands
- **AND** SHALL note that emsdk_env.bat should be sourced before WASM builds

#### Scenario: User-friendly output
- **WHEN** the setup script runs
- **THEN** it SHALL display colored output (green for success, yellow for warnings, red for errors)
- **AND** SHALL provide clear step-by-step instructions
- **AND** SHALL show a summary of next steps at the end
- **AND** SHALL have proper error handling for all operations

### Requirement: Windows-Specific CMake Configuration
The CMake build system SHALL apply appropriate MSVC-specific compiler flags and configurations when building on Windows.

#### Scenario: MSVC compiler flags
- **WHEN** configuring with MSVC
- **THEN** the build system SHALL add `/W4` for warning level 4
- **AND** SHALL add `/WX` to treat warnings as errors
- **AND** SHALL add `/bigobj` to handle large object files
- **AND** SHALL add `/EHsc` for exception handling
- **AND** for Release builds, SHALL add `/O2` optimization flag

#### Scenario: Shared library export macros
- **WHEN** building shared libraries on Windows
- **THEN** the build system SHALL define URDFX_BUILD during library compilation
- **AND** SHALL define URDFX_SHARED for consumers when using shared libraries
- **AND** headers SHALL use __declspec(dllexport) when URDFX_BUILD is defined
- **AND** headers SHALL use __declspec(dllimport) when URDFX_BUILD is not defined but URDFX_SHARED is

#### Scenario: Path handling
- **WHEN** CMake scripts reference file paths
- **THEN** they SHALL use CMake path variables (CMAKE_CURRENT_SOURCE_DIR, etc.)
- **AND** SHALL use forward slashes in CMake strings (CMake handles conversion)
- **AND** C++ code SHALL use std::filesystem::path for platform-independent path manipulation

### Requirement: Dependency Build on Windows
All third-party dependencies (Eigen, pugixml, CppAD, DaQP, spdlog, GoogleTest) SHALL build successfully on Windows with MSVC.

#### Scenario: Eigen builds on Windows
- **WHEN** Eigen is built from submodule
- **THEN** it SHALL configure and compile without errors on Windows
- **AND** SHALL provide Eigen3::Eigen target correctly
- **AND** SHALL work with both static and shared urdfx library builds

#### Scenario: pugixml builds on Windows
- **WHEN** pugixml is built from submodule
- **THEN** it SHALL compile as a static or shared library on Windows
- **AND** SHALL provide pugixml::pugixml target
- **AND** SHALL handle Windows paths correctly when parsing XML

#### Scenario: CppAD builds on Windows
- **WHEN** CppAD is used from submodule
- **THEN** it SHALL provide necessary headers for MSVC compilation
- **AND** temp_file.cpp SHALL compile successfully on Windows
- **AND** SHALL work with MSVC's C++20 implementation

#### Scenario: DaQP builds on Windows
- **WHEN** DaQP is built from submodule
- **THEN** it SHALL compile its C sources with MSVC
- **AND** SHALL provide daqp::daqp target
- **AND** SHALL link correctly with C++ code

#### Scenario: spdlog builds on Windows
- **WHEN** spdlog is built from submodule
- **THEN** it SHALL compile on Windows with MSVC
- **AND** SHALL provide spdlog::spdlog target
- **AND** SHALL support both static and shared builds

#### Scenario: GoogleTest builds on Windows
- **WHEN** BUILD_TESTING is ON
- **THEN** GoogleTest SHALL build from submodule on Windows
- **AND** SHALL provide GTest::gtest and GTest::gtest_main targets
- **AND** test executables SHALL link and run successfully

### Requirement: Documentation for Windows
The project documentation SHALL include clear instructions for building on Windows.

#### Scenario: README includes Windows instructions
- **WHEN** a user reads the README.md
- **THEN** Windows prerequisites SHALL be listed (Visual Studio, CMake, Python, Node.js)
- **AND** minimum versions SHALL be specified
- **AND** setup script usage SHALL be documented
- **AND** build commands for Windows SHALL be provided
- **AND** troubleshooting tips for common Windows issues SHALL be included

#### Scenario: project.md updated with Windows support
- **WHEN** reviewing project conventions in project.md
- **THEN** Windows SHALL be listed as a supported platform
- **AND** MSVC version requirements SHALL be documented
- **AND** Windows-specific build considerations SHALL be noted

## MODIFIED Requirements

### Requirement: Cross-Platform Build System
The build system SHALL support compilation on Linux, macOS, **and Windows** with appropriate platform-specific configurations.

#### Scenario: Platform detection
- **WHEN** CMake configuration runs
- **THEN** it SHALL detect the operating system (Linux, macOS, Windows)
- **AND** SHALL select appropriate compiler flags for each platform
- **AND** SHALL apply GCC/Clang flags on Linux/macOS
- **AND** SHALL apply MSVC flags on Windows

#### Scenario: Compiler-specific optimizations
- **WHEN** building in Release mode
- **THEN** on GCC/Clang, SHALL use `-O3 -march=native` (except Emscripten)
- **AND** on MSVC, SHALL use `/O2`
- **AND** on Emscripten, SHALL use `-O3` without `-march=native`

#### Scenario: Warning configuration
- **WHEN** configuring compiler warnings
- **THEN** on GCC/Clang, SHALL use `-Wall -Wextra -Wpedantic -Werror`
- **AND** on MSVC, SHALL use `/W4 /WX`
- **AND** both SHALL treat warnings as errors to maintain code quality

### Requirement: C++20 Compiler Support
The project SHALL require C++20 compatible compilers: GCC 10+, Clang 10+, **or MSVC 19.29+ (Visual Studio 2019 16.11+)**.

#### Scenario: MSVC version verification (ADDED to existing requirement)
- **WHEN** using MSVC compiler
- **THEN** CMake SHALL check version is at least 19.29
- **AND** SHALL display error with required Visual Studio version if too old
- **AND** SHALL configure C++20 standard correctly (`/std:c++20`)
