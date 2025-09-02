cmake_minimum_required(VERSION 3.8)
project(igvc_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(igvc REQUIRED)

############
## C++ nodes
############

add_executable(fusion_node src/fusion_node.cpp)

ament_target_dependencies(fusion_node
  rclcpp
  igvc
)

# target_link_libraries(fusion_node igvc_common)

include_directories(
  include
)

install(TARGETS fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
