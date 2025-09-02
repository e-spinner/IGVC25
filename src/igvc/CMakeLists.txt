cmake_minimum_required(VERSION 3.8)
project(igvc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# generate msgs
set(msg_files
  "msg/Position.msg"
  "msg/Path.msg"
  "msg/Map.msg"
  "msg/PVA.msg"
  "msg/GPSFeedback.msg"
  "msg/IMUFeedback.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES builtin_interfaces geometry_msgs
)

# Create a header-only interface library
add_library(igvc_common  INTERFACE)

# Tell ROS and users where your headers are
target_include_directories(igvc_common  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the header directory
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install the interface library
install(
  TARGETS igvc_common
  EXPORT export_igvc_common
)

ament_export_targets(export_igvc_common HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()