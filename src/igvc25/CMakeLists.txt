cmake_minimum_required(VERSION 3.8)
project(igvc25)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --------------------
# Message generation
# --------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RootInstruction.msg"
  DEPENDENCIES builtin_interfaces
)

# ----------------
# C++ nodes
# ----------------

# Fusion Node
add_executable(fusion_node src/fusion_node.cpp)
ament_target_dependencies(
  fusion_node
  rclcpp
  sensor_msgs
  nav_msgs
)

# Truth Gen
add_executable(truth_gen src/ground_truth_generator.cpp)
ament_target_dependencies(
  truth_gen
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
)

# Make executables depend on generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(truth_gen ${cpp_typesupport_target})
target_link_libraries(fusion_node ${cpp_typesupport_target})

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
  TARGETS fusion_node truth_gen
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------
# Install Extra Folders
# --------------------
install(
  DIRECTORY launch description config worlds
  DESTINATION share/${PROJECT_NAME}
)

# --------------------
# Python nodes
# --------------------
find_package(ament_cmake_python REQUIRED)

# Install Python module
ament_python_install_package(igvc25_py)

# Install Python executables
install(PROGRAMS
  igvc25_py/root_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()