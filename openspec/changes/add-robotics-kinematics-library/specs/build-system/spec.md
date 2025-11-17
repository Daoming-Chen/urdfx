# Capability: Build System

## ADDED Requirements

### Requirement: System SHALL use CMake-based build configuration
The system SHALL use CMake 3.20+ as the primary build system.

#### Scenario: Configure project with CMake
**Given** the urdfx source directory  
**When** the user runs `cmake -B build -DCMAKE_BUILD_TYPE=Release`  
**Then** CMake configures the project successfully  
**And** all dependencies are found or fetched  
**And** build files are generated in the build/ directory

#### Scenario: Build C++ library
**Given** a configured CMake build directory  
**When** the user runs `cmake --build build -j$(nproc)`  
**Then** the core library liburdfx.so (or .dylib/.dll) is built  
**And** all tests are compiled  
**And** the build completes without errors

### Requirement: System SHALL manage dependencies via git submodules
The system SHALL use git submodules for third-party C++ libraries.

#### Scenario: Clone with submodules
**Given** the urdfx git repository URL  
**When** the user runs `git clone --recursive <url>`  
**Then** all submodules are cloned: Eigen, pugixml, CppAD, DaQP, spdlog, nanobind, googletest  
**And** submodules are at specified commits/tags

#### Scenario: Update submodules after clone
**Given** a cloned repository without submodules  
**When** the user runs `git submodule update --init --recursive`  
**Then** all submodules are initialized and checked out  
**And** the build can proceed

### Requirement: System SHALL provide setup script for system dependencies
The system SHALL include a script to install Emscripten and other system deps.

#### Scenario: Run setup script on Ubuntu
**Given** a fresh Ubuntu 22.04 system  
**When** the user runs `./scripts/setup.sh`  
**Then** the script installs CMake, Python, Node.js if missing  
**And** Emscripten SDK is installed to third_party/emsdk  
**And** the script prints success message

#### Scenario: Setup script is idempotent
**Given** a system with dependencies already installed  
**When** the user runs `./scripts/setup.sh` again  
**Then** the script detects existing installations  
**And** skips reinstallation  
**And** completes quickly

### Requirement: System SHALL support installation via make install
The system SHALL support installing the library system-wide or to a prefix.

#### Scenario: Install to system directory
**Given** a built project in build/  
**When** the user runs `sudo cmake --install build --prefix /usr/local`  
**Then** headers are installed to /usr/local/include/urdfx/  
**And** library is installed to /usr/local/lib/  
**And** CMake config files are installed to /usr/local/lib/cmake/urdfx/

#### Scenario: Install to custom prefix
**Given** a built project  
**When** the user runs `cmake --install build --prefix ~/local`  
**Then** all files are installed under ~/local/  
**And** no sudo is required

### Requirement: System SHALL export CMake targets for downstream projects
The system SHALL provide CMake targets for easy integration.

#### Scenario: Find and link urdfx in downstream project
**Given** urdfx is installed  
**When** a downstream CMakeLists.txt contains:  
```cmake
find_package(urdfx REQUIRED)
target_link_libraries(my_app urdfx::urdfx)
```  
**Then** CMake finds urdfx successfully  
**And** the target is linked with correct include paths and libraries

### Requirement: System SHALL build Python bindings with CMake
The system SHALL integrate Python binding compilation into CMake.

#### Scenario: Enable Python bindings in CMake
**Given** CMake configuration with `-DBUILD_PYTHON_BINDINGS=ON`  
**When** the project is built  
**Then** the Python extension module is compiled  
**And** the module is placed in python/urdfx/ directory  
**And** setup.py can find and package it

### Requirement: System SHALL build WebAssembly with Emscripten toolchain
The system SHALL support WASM builds via Emscripten CMake toolchain.

#### Scenario: Build WASM module
**Given** Emscripten SDK is installed  
**When** the user runs:  
```bash
emcmake cmake -B build-wasm -DBUILD_WASM=ON
cmake --build build-wasm
```  
**Then** urdfx.js and urdfx.wasm are generated in build-wasm/wasm/  
**And** the module is ready for browser use

### Requirement: System SHALL enforce C++20 standard
The system SHALL require and enforce C++20 compiler features.

#### Scenario: CMake enforces C++20
**Given** CMakeLists.txt with `set(CMAKE_CXX_STANDARD 20)`  
**When** CMake is configured  
**Then** the C++20 standard is set for all targets  
**And** C++20 features (concepts, ranges) can be used

#### Scenario: Reject incompatible compilers
**Given** a compiler without C++20 support (e.g., GCC 9)  
**When** CMake is configured  
**Then** an error is raised about compiler version  
**And** the user is informed of minimum requirements

### Requirement: System SHALL provide build options for components
The system SHALL allow enabling/disabling components via CMake options.

#### Scenario: Build without tests
**Given** CMake configuration with `-DBUILD_TESTING=OFF`  
**When** the project is built  
**Then** test executables are not compiled  
**And** GoogleTest is not required

#### Scenario: Build minimal library only
**Given** CMake options disabling Python, WASM, and tests  
**When** the project is built  
**Then** only the core C++ library is compiled  
**And** build time is minimized

### Requirement: System SHALL generate compile_commands.json for IDE support
The system SHALL generate compilation database for clangd and other tools.

#### Scenario: Enable compilation database
**Given** CMake configuration with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`  
**When** CMake configures the project  
**Then** compile_commands.json is generated in build/  
**And** clangd/VSCode can use it for code intelligence

### Requirement: System SHALL support out-of-tree builds
The system SHALL support building outside the source directory.

#### Scenario: Out-of-tree build
**Given** the urdfx source at /home/user/urdfx  
**When** the user builds in /tmp/urdfx-build  
**Then** the build succeeds  
**And** the source directory remains clean  
**And** multiple build configurations can coexist

### Requirement: System SHALL provide build documentation
The system SHALL include clear build instructions in documentation.

#### Scenario: Follow README build instructions
**Given** a user reading README.md  
**When** following the build steps  
**Then** the project builds successfully  
**And** all dependencies are explained  
**And** common issues are documented with solutions

### Requirement: System SHALL integrate spdlog for logging
The system SHALL use spdlog as the logging library for all C++ code.

#### Scenario: spdlog is available before C++ code compilation
**Given** the urdfx project source  
**When** CMake configures the project  
**Then** spdlog is found or built from submodule  
**And** spdlog is available to all C++ targets  
**And** logging headers are accessible via include/urdfx/logging.h

#### Scenario: Logging infrastructure is configured
**Given** a configured build  
**When** C++ code includes urdfx/logging.h  
**Then** logging macros (URDFX_LOG_DEBUG, URDFX_LOG_INFO, URDFX_LOG_WARN, URDFX_LOG_ERROR) are available  
**And** default log level is INFO  
**And** logs can be written to console and/or file  
**And** log level can be configured at runtime
