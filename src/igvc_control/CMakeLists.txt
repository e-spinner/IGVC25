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
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(igvc REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
# find_package(nav2_ros_common REQUIRED)


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
# Builld Plugin
# --------------------

nav2_package()

add_library(pure_pursuit_controller SHARED
        src/pure_pursuit.cpp)

# prevent pluginlib from using boost
target_compile_definitions(pure_pursuit_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(pure_pursuit_controller
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  tf2
  tf2_ros
)

install(TARGETS pure_pursuit_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# AckermannAngleDriveController
add_library(ackermann_angle_drive_controller SHARED
        src/ackermann_angle_drive_controller.cpp)

# prevent pluginlib from using boost
target_compile_definitions(ackermann_angle_drive_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(ackermann_angle_drive_controller
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  controller_interface
  pluginlib
)

install(TARGETS ackermann_angle_drive_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --------------------
# Headers
# --------------------
include_directories(
  include
)

install(
  DIRECTORY include/
  DESTINATION include
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
  igvc_control_py/ackermann_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------
# Export Plugins
# --------------------
pluginlib_export_plugin_description_file(nav2_core pure_pursuit_controller.xml)
pluginlib_export_plugin_description_file(controller_interface ackermann_angle_drive_controller.xml)

ament_package()

