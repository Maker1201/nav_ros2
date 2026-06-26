cmake_minimum_required(VERSION 3.10)

project(bpx_sdk_examples LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(bpx_sdk SHARED IMPORTED)

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" BPX_SDK_ARCH)
set(BPX_SDK_LIBRARY_CANDIDATES
    "${CMAKE_CURRENT_LIST_DIR}/lib/libbpx_sdk_${BPX_SDK_ARCH}.so"
)

if(BPX_SDK_ARCH MATCHES "^(amd64|x64)$")
    list(APPEND BPX_SDK_LIBRARY_CANDIDATES
        "${CMAKE_CURRENT_LIST_DIR}/lib/libbpx_sdk_x86_64.so"
    )
elseif(BPX_SDK_ARCH MATCHES "^(arm64)$")
    list(APPEND BPX_SDK_LIBRARY_CANDIDATES
        "${CMAKE_CURRENT_LIST_DIR}/lib/libbpx_sdk_aarch64.so"
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
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/include"
    INTERFACE_LINK_LIBRARIES Threads::Threads
)

function(add_bpx_sdk_example target source)
    add_executable(${target}
        ${source}
    )
    target_link_libraries(${target}
        PRIVATE
            bpx_sdk
    )
    set_target_properties(${target} PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_LIST_DIR}/lib"
    )
endfunction()

add_bpx_sdk_example(request_robot_state_example example/request_robot_state_example.cpp)
add_bpx_sdk_example(motion_level_control_example example/motion_level_control_example.cpp)
add_bpx_sdk_example(joint_level_control_example example/joint_level_control_example.cpp)
