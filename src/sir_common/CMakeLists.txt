cmake_minimum_required(VERSION 3.8)
project(sir_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Create a header-only interface library
add_library(${PROJECT_NAME} INTERFACE)

# Tell ROS and users where your headers are
target_include_directories(${PROJECT_NAME} INTERFACE
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
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)

ament_package()