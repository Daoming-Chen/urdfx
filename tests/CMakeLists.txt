# Test sources
set(TEST_SOURCES
    test_main.cpp
    test_urdf_parser.cpp
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
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
)

# Add tests to CTest
include(GoogleTest)
gtest_discover_tests(urdfx_tests)

# Copy test data files
file(COPY ${PROJECT_SOURCE_DIR}/ur5_urdf
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
