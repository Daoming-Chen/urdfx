# Dependencies.cmake
# This module finds or builds all dependencies for urdfx

# Prefer using system-installed packages, fall back to submodules
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/third_party)

# Eigen3
find_package(Eigen3 3.4 QUIET)
if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not found on system, using submodule")
    add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/eigen EXCLUDE_FROM_ALL)
    # Eigen provides Eigen3::Eigen target
endif()
message(STATUS "Using Eigen3: ${EIGEN3_INCLUDE_DIR}")

# spdlog (must be available before any C++ code compilation)
find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
    message(STATUS "spdlog not found on system, using submodule")
    set(SPDLOG_BUILD_SHARED ${BUILD_SHARED_LIBS})
    add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/spdlog EXCLUDE_FROM_ALL)
    # spdlog provides spdlog::spdlog target
endif()
message(STATUS "Using spdlog")

# pugixml
find_package(pugixml QUIET)
if(NOT pugixml_FOUND)
    message(STATUS "pugixml not found on system, using submodule")
    add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/pugixml EXCLUDE_FROM_ALL)
    # Create alias target if not provided
    if(NOT TARGET pugixml::pugixml)
        add_library(pugixml::pugixml ALIAS pugixml)
    endif()
endif()
message(STATUS "Using pugixml")

# CppAD
find_package(CppAD QUIET)
if(NOT CppAD_FOUND)
    message(STATUS "CppAD not found on system, using submodule")
    set(cppad_prefix ${PROJECT_SOURCE_DIR}/third_party/CppAD)
    
    # CppAD is header-only for basic usage, so we can just add include path
    add_library(cppad_lib INTERFACE)
    target_include_directories(cppad_lib INTERFACE ${cppad_prefix}/include)
    
    # Create alias target
    add_library(CppAD::CppAD ALIAS cppad_lib)
endif()
message(STATUS "Using CppAD")

# DaQP
# DaQP doesn't have standard CMake config, so we build from submodule
if(NOT TARGET daqp)
    message(STATUS "Building DaQP from submodule")
    set(daqp_prefix ${PROJECT_SOURCE_DIR}/third_party/daqp)
    
    # Check if DaQP has CMakeLists.txt
    if(EXISTS ${daqp_prefix}/CMakeLists.txt)
        add_subdirectory(${daqp_prefix} EXCLUDE_FROM_ALL)
    else()
        # Build DaQP as a simple library if no CMake support
        file(GLOB DAQP_SOURCES ${daqp_prefix}/src/*.c)
        add_library(daqp STATIC ${DAQP_SOURCES})
        target_include_directories(daqp PUBLIC ${daqp_prefix}/include)
        set_target_properties(daqp PROPERTIES C_STANDARD 99)
    endif()
    
    # Create alias target
    if(NOT TARGET daqp::daqp)
        add_library(daqp::daqp ALIAS daqp)
    endif()
endif()
message(STATUS "Using DaQP")

# GoogleTest (only if building tests)
if(BUILD_TESTING)
    find_package(GTest QUIET)
    if(NOT GTest_FOUND)
        message(STATUS "GoogleTest not found on system, using submodule")
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
        add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/googletest EXCLUDE_FROM_ALL)
        # GoogleTest provides GTest::gtest and GTest::gtest_main
    endif()
    message(STATUS "Using GoogleTest")
endif()

# nanobind (only if building Python bindings)
if(BUILD_PYTHON_BINDINGS)
    find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
    
    # nanobind doesn't have a standard CMake config, use submodule
    if(NOT TARGET nanobind)
        message(STATUS "Using nanobind from submodule")
        add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/nanobind EXCLUDE_FROM_ALL)
    endif()
    message(STATUS "Using nanobind with Python ${Python_VERSION}")
endif()

# Emscripten is handled via toolchain file, not here
if(BUILD_WASM)
    if(NOT EMSCRIPTEN)
        message(FATAL_ERROR "BUILD_WASM is ON but not building with Emscripten. Use: emcmake cmake ...")
    endif()
    message(STATUS "Building for WebAssembly with Emscripten")
endif()
