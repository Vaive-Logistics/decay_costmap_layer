cmake_minimum_required(VERSION 3.8)
project(decay_costmap_layer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${nav2_costmap_2d_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

add_library(decay_layer SHARED
  src/persistent_obstacle_layer.cpp
  src/decay_obstacle_layer.cpp)

ament_target_dependencies(decay_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_costmap_2d decay_layer_plugin.xml)

install(TARGETS decay_layer
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

ament_package()