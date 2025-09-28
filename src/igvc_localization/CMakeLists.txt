cmake_minimum_required(VERSION 3.8)
project(igvc_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

############
## C++ nodes
############

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
  nav_msgs
)
# Headers
include_directories(
  include
)

# Install Nodes
install(
  TARGETS fusion_node truth_gen
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
