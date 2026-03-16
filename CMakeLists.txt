cmake_minimum_required(VERSION 3.5)
project(livox_to_pointcloud2)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Detect Livox driver
if(livox_ros_driver2_FOUND)
  message(STATUS "Found livox_ros_driver2")
  add_compile_definitions(LIVOX_ROS_DRIVER2)
endif()

# Component library
ament_auto_add_library(livox_to_pointcloud2 SHARED
  src/livox_to_pointcloud2_ros2.cpp
)

rclcpp_components_register_nodes(
  livox_to_pointcloud2
  "livox_to_pointcloud2::LivoxToPointCloud2"
)

# Executable node
ament_auto_add_executable(livox_to_pointcloud2_node
  src/livox_to_pointcloud2_ros2_node.cpp
)

target_include_directories(livox_to_pointcloud2_node PUBLIC
  include
)

target_link_libraries(livox_to_pointcloud2_node
  livox_to_pointcloud2
)

ament_auto_package()