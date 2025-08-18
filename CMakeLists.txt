cmake_minimum_required(VERSION 3.8)
project(fastbot_waypoints)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# Action interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Waypoint.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Node
add_executable(fastbot_action_server src/fastbot_action_server.cpp)
ament_target_dependencies(fastbot_action_server
  rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_geometry_msgs
)
# Ensure generated action headers are available
add_dependencies(fastbot_action_server ${PROJECT_NAME}__rosidl_generator_cpp)

install(TARGETS fastbot_action_server
  DESTINATION lib/${PROJECT_NAME})

# Tests
ament_add_gtest(test_waypoints test/test_waypoints.cpp)
if(TARGET test_waypoints)
  target_link_libraries(test_waypoints)
  ament_target_dependencies(test_waypoints
    rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_geometry_msgs
  )
  add_dependencies(test_waypoints ${PROJECT_NAME}__rosidl_generator_cpp)
endif()

ament_package()
