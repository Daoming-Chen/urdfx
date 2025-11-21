cmake_minimum_required(VERSION 3.20)

# Define the Python extension module
# nanobind_add_module creates a shared library with the right suffix for Python
nanobind_add_module(_urdfx src/bindings.cpp)

# Link against the core library only
# nanobind is linked automatically by nanobind_add_module
target_link_libraries(_urdfx PRIVATE urdfx)

# Set C++ standard (should inherit from parent but good to be explicit)
target_compile_features(_urdfx PRIVATE cxx_std_20)

# Define version
target_compile_definitions(_urdfx PRIVATE URDFX_VERSION="${PROJECT_VERSION}")

# Set output directory to bindings/python/urdfx package folder
# This allows editable installs and testing to work easily
set_target_properties(_urdfx PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/urdfx
    OUTPUT_NAME _urdfx
)

# Avoid "lib" prefix on Windows/Linux (Python expects _urdfx.so/pyd not lib_urdfx.so)
set_target_properties(_urdfx PROPERTIES PREFIX "")

# Set RPATH to find liburdfx.so in the same directory
# $ORIGIN means the directory containing the binary
set_target_properties(_urdfx PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH FALSE
)

# For editable/development installs, copy all needed .so files to the Python package directory
# so the extension module can find them without needing to set LD_LIBRARY_PATH
add_custom_command(TARGET _urdfx POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:urdfx>
        ${CMAKE_CURRENT_SOURCE_DIR}/urdfx/$<TARGET_FILE_NAME:urdfx>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_SONAME_FILE:urdfx>
        ${CMAKE_CURRENT_SOURCE_DIR}/urdfx/$<TARGET_SONAME_FILE_NAME:urdfx>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:pugixml-shared>
        ${CMAKE_CURRENT_SOURCE_DIR}/urdfx/$<TARGET_FILE_NAME:pugixml-shared>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_SONAME_FILE:pugixml-shared>
        ${CMAKE_CURRENT_SOURCE_DIR}/urdfx/$<TARGET_SONAME_FILE_NAME:pugixml-shared>
    COMMENT "Copying shared libraries to Python package directory for editable install"
)

# Installation rules
install(TARGETS _urdfx
    LIBRARY DESTINATION urdfx
    ARCHIVE DESTINATION urdfx
    RUNTIME DESTINATION urdfx
)

# Also install liburdfx.so and dependencies alongside the Python module
install(TARGETS urdfx pugixml-shared
    LIBRARY DESTINATION urdfx
    ARCHIVE DESTINATION urdfx
    RUNTIME DESTINATION urdfx
)

