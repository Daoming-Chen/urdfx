# Core C++ library for urdfx
cmake_minimum_required(VERSION 3.20)

# Core library sources
set(URDFX_SOURCES
    src/robot_model.cpp
    src/urdf_parser.cpp
    src/kinematics.cpp
    src/inverse_kinematics.cpp
)

set(URDFX_HEADERS
    include/urdfx/logging.h
    include/urdfx/robot_model.h
    include/urdfx/urdf_parser.h
    include/urdfx/kinematics.h
    include/urdfx/inverse_kinematics.h
    include/urdfx/export.h
)

# Create core library
add_library(urdfx ${URDFX_SOURCES} ${URDFX_HEADERS})

# Set library properties
set_target_properties(urdfx PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER "${URDFX_HEADERS}"
)

# Include directories
target_include_directories(urdfx
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link dependencies
target_link_libraries(urdfx
    PUBLIC
        Eigen3::Eigen
        spdlog::spdlog
    PRIVATE
        pugixml::pugixml
        daqp::daqp
)

# Compile definitions
target_compile_definitions(urdfx
    PRIVATE
        URDFX_BUILD
)

if(BUILD_SHARED_LIBS)
    target_compile_definitions(urdfx
        PUBLIC
            URDFX_SHARED
    )
endif()

# Installation rules
install(TARGETS urdfx
    EXPORT urdfxTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/urdfx
)

# Only export targets if all dependencies are system packages (not submodules)
# This avoids export errors when building with submodule dependencies
if(Eigen3_FOUND AND spdlog_FOUND AND pugixml_FOUND)
    # Export targets
    install(EXPORT urdfxTargets
        FILE urdfxTargets.cmake
        NAMESPACE urdfx::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/urdfx
    )
else()
    message(STATUS "Skipping CMake export (using submodule dependencies)")
endif()

# Tests
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()
