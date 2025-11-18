cmake_minimum_required(VERSION 3.20)

project(urdfx
    VERSION 1.0.0
    DESCRIPTION "Modern C++20 robotics kinematics library"
    LANGUAGES CXX
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Check compiler version
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "10.0")
        message(FATAL_ERROR "GCC version must be at least 10.0 for C++20 support. Current version: ${CMAKE_CXX_COMPILER_VERSION}")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "10.0")
        message(FATAL_ERROR "Clang version must be at least 10.0 for C++20 support. Current version: ${CMAKE_CXX_COMPILER_VERSION}")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "19.29")
        message(FATAL_ERROR "MSVC version must be at least 19.29 (VS 2019 16.11) for C++20 support. Current version: ${CMAKE_CXX_COMPILER_VERSION}")
    endif()
endif()

# Build options
option(BUILD_TESTING "Build tests" ON)
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
option(BUILD_WASM "Build WebAssembly bindings" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(BUILD_BENCHMARKS "Build benchmark suite" OFF)

# Compiler flags
if(MSVC)
    add_compile_options(/W4 /WX)
    add_compile_options(/bigobj)  # For large template instantiations
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
    add_compile_options(-Wno-unused-parameter)
    add_compile_options(-Wno-empty-body)
endif()

# Optimization flags for Release build
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(MSVC)
        add_compile_options(/O2)
    else()
        if(EMSCRIPTEN)
            add_compile_options(-O3)
        else()
            add_compile_options(-O3 -march=native)
        endif()
    endif()
endif()

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include dependencies
include(cmake/Dependencies.cmake)

# Core library
add_subdirectory(src)

# Tests
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()

# Examples
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Benchmarks
if(BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

# Python bindings
if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(python)
endif()

# WebAssembly bindings
if(BUILD_WASM)
    add_subdirectory(wasm)
endif()

# Installation (skip for Emscripten builds to avoid exporting host-only deps)
if(NOT EMSCRIPTEN)
    include(GNUInstallDirs)

    install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )

    install(TARGETS urdfx
        EXPORT urdfxTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/urdfx
    )

    # Export CMake targets
    install(EXPORT urdfxTargets
        FILE urdfxTargets.cmake
        NAMESPACE urdfx::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/urdfx
    )

    # Generate and install CMake config file
    include(CMakePackageConfigHelpers)

    configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/urdfxConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/urdfxConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/urdfx
    )

    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/urdfxConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/urdfxConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/urdfxConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/urdfx
    )
endif()

# Print configuration summary
message(STATUS "")
message(STATUS "urdfx ${PROJECT_VERSION} configuration summary:")
message(STATUS "  CMake version:       ${CMAKE_VERSION}")
message(STATUS "  C++ compiler:        ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Build type:          ${CMAKE_BUILD_TYPE}")
message(STATUS "  Build shared libs:   ${BUILD_SHARED_LIBS}")
message(STATUS "  Build testing:       ${BUILD_TESTING}")
message(STATUS "  Build examples:      ${BUILD_EXAMPLES}")
message(STATUS "  Build Python:        ${BUILD_PYTHON_BINDINGS}")
message(STATUS "  Build WASM:          ${BUILD_WASM}")
message(STATUS "  Install prefix:      ${CMAKE_INSTALL_PREFIX}")
message(STATUS "")
