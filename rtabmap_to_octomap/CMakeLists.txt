cmake_minimum_required(VERSION 3.8)
project(rtabmap_to_octomap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_octomap REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(octomap_msgs REQUIRED)

set(dependencies
  grid_map_msgs
  grid_map_ros
  rclcpp
)

include_directories(
  include
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(
  octomap_to_gridmap_demo
  src/octomap_to_gridmap_node.cpp
  src/OctomapToGridmap.cpp
)
ament_target_dependencies(
  octomap_to_gridmap_demo SYSTEM
  ${dependencies}
  grid_map_octomap
  octomap_msgs
)
install(
  TARGETS octomap_to_gridmap_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
