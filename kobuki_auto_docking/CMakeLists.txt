cmake_minimum_required(VERSION 3.5)
project(kobuki_auto_docking)

find_package(ament_cmake REQUIRED)
find_package(ecl_linear_algebra REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_core REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_kdl REQUIRED)

add_library(kobuki_auto_docking_ros SHARED
  src/auto_docking_ros.cpp)
target_include_directories(kobuki_auto_docking_ros PUBLIC
  include/)
ament_target_dependencies(kobuki_auto_docking_ros
  ecl_linear_algebra
  geometry_msgs
  kobuki_core
  kobuki_ros_interfaces
  message_filters
  orocos_kdl
  rclcpp
  rclcpp_components
  tf2_kdl
)

rclcpp_components_register_nodes(kobuki_auto_docking_ros
  "kobuki::AutoDockingROS")

install(TARGETS kobuki_auto_docking_ros
  DESTINATION lib/${project_name})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
