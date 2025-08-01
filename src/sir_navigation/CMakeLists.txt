cmake_minimum_required(VERSION 3.8)
project(sir_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sir REQUIRED)

############
## C++ nodes
############

add_executable(pathing_node src/pathing_node.cpp)

ament_target_dependencies(pathing_node
  rclcpp
  sir
)

# target_link_libraries(pathing_node sir_common)

include_directories(
  include
)

install(TARGETS pathing_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
