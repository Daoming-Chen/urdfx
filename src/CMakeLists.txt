# Core library sources
set(URDFX_SOURCES
    robot_model.cpp
    urdf_parser.cpp
    kinematics.cpp
    ${PROJECT_SOURCE_DIR}/third_party/CppAD/cppad_lib/temp_file.cpp
)

set(URDFX_HEADERS
    ${PROJECT_SOURCE_DIR}/include/urdfx/logging.h
    ${PROJECT_SOURCE_DIR}/include/urdfx/robot_model.h
    ${PROJECT_SOURCE_DIR}/include/urdfx/urdf_parser.h
    ${PROJECT_SOURCE_DIR}/include/urdfx/kinematics.h
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
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
)

# Link dependencies
target_link_libraries(urdfx
    PUBLIC
        Eigen3::Eigen
        spdlog::spdlog
    PRIVATE
        pugixml::pugixml
        CppAD::CppAD
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
