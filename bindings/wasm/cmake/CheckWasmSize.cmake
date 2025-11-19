if(NOT DEFINED URDFX_WASM_PATH)
    message(FATAL_ERROR "URDFX_WASM_PATH is not set. Cannot verify WebAssembly binary size.")
endif()

if(NOT EXISTS "${URDFX_WASM_PATH}")
    message(FATAL_ERROR "Expected WebAssembly binary not found at ${URDFX_WASM_PATH}.")
endif()

set(URDFX_WASM_MAX_SIZE 2097152)
file(SIZE "${URDFX_WASM_PATH}" URDFX_WASM_SIZE)

if(URDFX_WASM_SIZE GREATER URDFX_WASM_MAX_SIZE)
    math(EXPR URDFX_WASM_SIZE_KB "${URDFX_WASM_SIZE} / 1024")
    math(EXPR URDFX_WASM_MAX_SIZE_KB "${URDFX_WASM_MAX_SIZE} / 1024")
    message(FATAL_ERROR "urdfx.wasm size ${URDFX_WASM_SIZE} bytes (${URDFX_WASM_SIZE_KB} KiB) exceeds limit of ${URDFX_WASM_MAX_SIZE} bytes (${URDFX_WASM_MAX_SIZE_KB} KiB).")
else()
    message(STATUS "urdfx.wasm size ${URDFX_WASM_SIZE} bytes within limit ${URDFX_WASM_MAX_SIZE} bytes.")
endif()
