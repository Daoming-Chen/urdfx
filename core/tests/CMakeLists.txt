# Test sources
set(TEST_SOURCES
    test_main.cpp
    test_urdf_parser.cpp
    test_kinematics.cpp
    test_inverse_kinematics.cpp
)

# Create test executable
add_executable(urdfx_tests ${TEST_SOURCES})

# Link against core library and GoogleTest
target_link_libraries(urdfx_tests
    PRIVATE
        urdfx
        GTest::gtest
        GTest::gtest_main
)

# Include directories
target_include_directories(urdfx_tests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

# Add tests to CTest
include(GoogleTest)
# Disable test discovery during build to avoid PATH issues on Windows
# Tests can still be run manually with ctest
gtest_discover_tests(urdfx_tests
    DISCOVERY_MODE PRE_TEST
)

# Copy test data files from examples/models
file(COPY ${PROJECT_SOURCE_DIR}/examples/models/ur5
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
# For compatibility with old path (temporary)
file(COPY ${PROJECT_SOURCE_DIR}/examples/models/ur5
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/../
)

# On Windows with shared libraries, copy the DLL to the test directory
if(WIN32 AND BUILD_SHARED_LIBS)
    add_custom_command(TARGET urdfx_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:urdfx>
            $<TARGET_FILE_DIR:urdfx_tests>
        COMMENT "Copying urdfx DLL to test directory"
    )
    # Copy spdlog and pugixml DLLs if they exist
    add_custom_command(TARGET urdfx_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:spdlog>
            $<TARGET_FILE_DIR:urdfx_tests>
        COMMENT "Copying spdlog DLL to test directory"
    )
    add_custom_command(TARGET urdfx_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:pugixml-shared>
            $<TARGET_FILE_DIR:urdfx_tests>
        COMMENT "Copying pugixml DLL to test directory"
    )
endif()
