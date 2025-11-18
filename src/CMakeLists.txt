# Core library sources
set(URDFX_SOURCES
    # Will be populated in later phases
    # For now, create a placeholder to allow CMake to determine linker language
)

set(URDFX_HEADERS
    ${PROJECT_SOURCE_DIR}/include/urdfx/logging.h
)

# Create placeholder source file if no sources exist yet
if(NOT URDFX_SOURCES)
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/placeholder.cpp
        "// Placeholder file to satisfy CMake linker language detection\n"
        "// Will be replaced by actual implementation files in later phases\n"
    )
    set(URDFX_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/placeholder.cpp)
endif()

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
