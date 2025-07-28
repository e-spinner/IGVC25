cmake_minimum_required(VERSION 3.8)
project(sir_drivers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sir_msgs REQUIRED)

###############
## Python nodes
###############

install(PROGRAMS
  src/gps_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

############
## C++ nodes
############

# add_executable(imu_node src/imu_node.cpp)
# ament_target_dependencies(imu_node rclcpp)

# install(TARGETS
#   imu_node
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
