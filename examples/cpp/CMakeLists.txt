# Examples for urdfx library

# Forward Kinematics Example
add_executable(forward_kinematics_example
    forward_kinematics_example.cpp
)

target_link_libraries(forward_kinematics_example
    PRIVATE
        urdfx
)

target_include_directories(forward_kinematics_example
    PRIVATE
        ${PROJECT_SOURCE_DIR}/core/include
)

# Install examples
install(TARGETS forward_kinematics_example
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/examples
)
