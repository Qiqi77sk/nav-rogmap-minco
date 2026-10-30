cmake_minimum_required(VERSION 3.8)
project(shengsai_bt)

if(CMAKE_COMPILER_ID MATCHES "Clang|GNU")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(shengsai_bt_node
  src/main.cpp
  src/bt_nodes.cpp
  src/shengsai_bt_node.cpp
)

target_include_directories(shengsai_bt_node PRIVATE
  include
)

ament_target_dependencies(shengsai_bt_node
  ament_index_cpp
  behaviortree_cpp_v3
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  ros_interfaces
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  shengsai_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  tree
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
