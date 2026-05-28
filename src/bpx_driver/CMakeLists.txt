cmake_minimum_required(VERSION 3.8)
project(bpx_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ensure RPATH is embedded in the installed binary so it finds the .so at runtime
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

# --- bpx_sdk (self-contained in third_party/) ---
set(BPX_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/bpx_sdk")
set(BPX_SDK_INCLUDE_DIR "${BPX_SDK_DIR}/include")
set(BPX_SDK_LIB_DIR "${BPX_SDK_DIR}/lib")

# Detect architecture and select the right shared library
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  set(BPX_SDK_LIB "${BPX_SDK_LIB_DIR}/libbpx_sdk_x86_64.so")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(BPX_SDK_LIB "${BPX_SDK_LIB_DIR}/libbpx_sdk_aarch64.so")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Create an imported target for the precompiled SDK
add_library(bpx_sdk SHARED IMPORTED)
set_target_properties(bpx_sdk PROPERTIES
  IMPORTED_LOCATION "${BPX_SDK_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${BPX_SDK_INCLUDE_DIR}"
)

# --- bpx_driver_node ---
add_executable(bpx_driver_node src/bpx_driver_node.cpp)
target_include_directories(bpx_driver_node PRIVATE ${BPX_SDK_INCLUDE_DIR})
ament_target_dependencies(bpx_driver_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)
target_link_libraries(bpx_driver_node bpx_sdk Threads::Threads)

install(TARGETS bpx_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the .so alongside the executable so it can be found at runtime
install(FILES "${BPX_SDK_LIB}"
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config bpx
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
