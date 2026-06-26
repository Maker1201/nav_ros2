cmake_minimum_required(VERSION 3.10)

project(bpx_visual_programming LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找依赖
find_package(Threads REQUIRED)

# === BPX SDK ===
add_library(bpx_sdk SHARED IMPORTED)

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" BPX_SDK_ARCH)
set(BPX_SDK_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../lib")
set(BPX_SDK_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../include")

set(BPX_SDK_LIBRARY_CANDIDATES
    "${BPX_SDK_LIB_DIR}/libbpx_sdk_${BPX_SDK_ARCH}.so"
)

if(BPX_SDK_ARCH MATCHES "^(amd64|x64)$")
    list(APPEND BPX_SDK_LIBRARY_CANDIDATES
        "${BPX_SDK_LIB_DIR}/libbpx_sdk_x86_64.so"
    )
elseif(BPX_SDK_ARCH MATCHES "^(arm64)$")
    list(APPEND BPX_SDK_LIBRARY_CANDIDATES
        "${BPX_SDK_LIB_DIR}/libbpx_sdk_aarch64.so"
    )
endif()

foreach(BPX_SDK_LIBRARY_CANDIDATE IN LISTS BPX_SDK_LIBRARY_CANDIDATES)
    if(EXISTS "${BPX_SDK_LIBRARY_CANDIDATE}")
        set(BPX_SDK_LIBRARY_PATH "${BPX_SDK_LIBRARY_CANDIDATE}")
        break()
    endif()
endforeach()

if(NOT BPX_SDK_LIBRARY_PATH)
    message(FATAL_ERROR "No BPX SDK shared library found for architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set_target_properties(bpx_sdk PROPERTIES
    IMPORTED_LOCATION "${BPX_SDK_LIBRARY_PATH}"
    INTERFACE_INCLUDE_DIRECTORIES "${BPX_SDK_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES Threads::Threads
)

message(STATUS "BPX SDK library: ${BPX_SDK_LIBRARY_PATH}")

# === 第三方库 ===
set(THIRD_PARTY_DIR "${CMAKE_CURRENT_LIST_DIR}/third_party")

# cpp-httplib（需要 OpenSSL 支持 HTTPS，可选）
find_package(OpenSSL QUIET)
if(OPENSSL_FOUND)
    add_definitions(-DCPPHTTPLIB_OPENSSL_SUPPORT)
    message(STATUS "OpenSSL found, HTTPS support enabled")
endif()

# === 可执行文件 ===
add_executable(bpx_visual_programming
    server/main.cpp
    server/network_config.cpp
    server/robot_bridge.cpp
    server/command_executor.cpp
    server/state_publisher.cpp
)

target_include_directories(bpx_visual_programming PRIVATE
    server
    ${THIRD_PARTY_DIR}/cpp-httplib
    ${THIRD_PARTY_DIR}/nlohmann
)

target_link_libraries(bpx_visual_programming
    PRIVATE
        bpx_sdk
        Threads::Threads
)

if(OPENSSL_FOUND)
    target_link_libraries(bpx_visual_programming PRIVATE OpenSSL::SSL OpenSSL::Crypto)
endif()

# 设置 RPATH
set_target_properties(bpx_visual_programming PROPERTIES
    BUILD_RPATH "${BPX_SDK_LIB_DIR}"
    INSTALL_RPATH "${BPX_SDK_LIB_DIR}"
)

# 复制前端文件到构建目录
add_custom_command(TARGET bpx_visual_programming POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_LIST_DIR}/frontend"
        "$<TARGET_FILE_DIR:bpx_visual_programming>/frontend"
    COMMENT "Copying frontend files to build directory"
)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
