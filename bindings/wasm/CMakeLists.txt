cmake_minimum_required(VERSION 3.20)

if(NOT EMSCRIPTEN)
    message(FATAL_ERROR "wasm bindings require the Emscripten toolchain. Configure the project with emcmake.")
endif()

set(URDFX_WASM_TARGET urdfx_wasm)

add_executable(${URDFX_WASM_TARGET}
    src/bindings.cpp
)

set_target_properties(${URDFX_WASM_TARGET} PROPERTIES
    OUTPUT_NAME "urdfx"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/wasm
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/wasm
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/wasm
)

target_compile_definitions(${URDFX_WASM_TARGET}
    PRIVATE
        URDFX_WASM_BUILD
)

target_include_directories(${URDFX_WASM_TARGET}
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${URDFX_WASM_TARGET}
    PRIVATE
        urdfx
)

target_compile_options(${URDFX_WASM_TARGET}
    PRIVATE
        -O3
        -msimd128
)

target_link_options(${URDFX_WASM_TARGET}
    PRIVATE
        -O3
        --bind
        -sMODULARIZE=1
        -sEXPORT_NAME=createUrdfxModule
        -sALLOW_MEMORY_GROWTH=1
        -sENVIRONMENT=web,worker,node
        -sFILESYSTEM=0
        -sASSERTIONS=0
        -sERROR_ON_UNDEFINED_SYMBOLS=1
        -sEXPORTED_RUNTIME_METHODS=['UTF8ToString','lengthBytesUTF8']
        -sALLOW_TABLE_GROWTH
)

set(URDFX_WASM_OUTPUT_DIR ${CMAKE_BINARY_DIR}/wasm)
set(URDFX_WASM_BINARY ${URDFX_WASM_OUTPUT_DIR}/urdfx.wasm)

add_custom_command(TARGET ${URDFX_WASM_TARGET} POST_BUILD
    COMMAND ${CMAKE_COMMAND}
        -DURDFX_WASM_PATH=${URDFX_WASM_BINARY}
        -P ${PROJECT_SOURCE_DIR}/bindings/wasm/cmake/CheckWasmSize.cmake
    BYPRODUCTS ${URDFX_WASM_BINARY}
    COMMENT "Checking urdfx.wasm binary size"
)

install(FILES
    ${URDFX_WASM_OUTPUT_DIR}/urdfx.js
    ${URDFX_WASM_OUTPUT_DIR}/urdfx.wasm
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/urdfx/wasm
)

install(FILES
    ${PROJECT_SOURCE_DIR}/bindings/wasm/urdfx.d.ts
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/urdfx/wasm
)
