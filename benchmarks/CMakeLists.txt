add_executable(ik_benchmarks
    ik_benchmarks.cpp
)

add_executable(jacobian_benchmarks
    jacobian_benchmarks.cpp
)

target_compile_features(ik_benchmarks PRIVATE cxx_std_20)
target_compile_features(jacobian_benchmarks PRIVATE cxx_std_20)

target_link_libraries(ik_benchmarks
    PRIVATE
        urdfx
        benchmark::benchmark
        benchmark::benchmark_main
        spdlog::spdlog
)

target_link_libraries(jacobian_benchmarks
    PRIVATE
        urdfx
        benchmark::benchmark
        benchmark::benchmark_main
        spdlog::spdlog
)

target_compile_definitions(ik_benchmarks
    PRIVATE
    URDFX_UR5_URDF_PATH="${PROJECT_SOURCE_DIR}/examples/models/ur5/ur5e.urdf"
)

target_compile_definitions(jacobian_benchmarks
    PRIVATE
    URDFX_UR5_URDF_PATH="${PROJECT_SOURCE_DIR}/examples/models/ur5/ur5e.urdf"
)

# Copy DLLs to benchmark output directory on Windows
if(WIN32 AND BUILD_SHARED_LIBS)
    # Copy urdfx DLL
    add_custom_command(TARGET ik_benchmarks POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:urdfx>
            $<TARGET_FILE_DIR:ik_benchmarks>
        COMMENT "Copying urdfx DLL to benchmark directory"
    )
    
    # Copy spdlog DLL
    add_custom_command(TARGET ik_benchmarks POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:spdlog::spdlog>
            $<TARGET_FILE_DIR:ik_benchmarks>
        COMMENT "Copying spdlog DLL to benchmark directory"
    )
    
    # Copy pugixml DLL if it exists
    if(TARGET pugixml-shared)
        add_custom_command(TARGET ik_benchmarks POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:pugixml-shared>
                $<TARGET_FILE_DIR:ik_benchmarks>
            COMMENT "Copying pugixml DLL to benchmark directory"
        )
    endif()
    
    # Copy Google Benchmark DLLs
    add_custom_command(TARGET ik_benchmarks POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:benchmark::benchmark>
            $<TARGET_FILE_DIR:ik_benchmarks>
        COMMENT "Copying benchmark DLL to benchmark directory"
    )
    
    add_custom_command(TARGET ik_benchmarks POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:benchmark::benchmark_main>
            $<TARGET_FILE_DIR:ik_benchmarks>
        COMMENT "Copying benchmark_main DLL to benchmark directory"
    )
endif()

# Add a custom target to run benchmarks
add_custom_target(run_benchmarks
    COMMAND ik_benchmarks
    DEPENDS ik_benchmarks
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Running IK benchmarks"
    USES_TERMINAL
)
