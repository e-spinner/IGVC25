cmake_minimum_required(VERSION 3.8)
project(igvc_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(igvc REQUIRED)

# ----------------
# C++ nodes
# ----------------

# cmd_interpreter
add_executable(cmd_interpreter src/cmd_interpreter.cpp)
ament_target_dependencies(
  cmd_interpreter
  rclcpp
  geometry_msgs
  tf2_ros
  igvc
)

# transform
add_executable(transform src/transform.cpp)
ament_target_dependencies(
  transform
  rclcpp
  geometry_msgs
  tf2_ros
  sensor_msgs
  igvc
)

# --------------------
# Headers
# --------------------
include_directories(
  include
)

# --------------------
# Install Nodes
# --------------------
install(
  TARGETS cmd_interpreter transform
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------
# Install Extra Folders
# --------------------
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --------------------
# Python nodes
# --------------------
find_package(ament_cmake_python REQUIRED)

# Install Python module
ament_python_install_package(igvc_control_py)

# Install Python executables
install(PROGRAMS
  igvc_control_py/ack_calc.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

