cmake_minimum_required(VERSION 3.8)
project(fastbot_waypoints)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# 1) Generate the action interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Waypoint.action"
  DEPENDENCIES geometry_msgs
)

# Makes generated headers visible to consumers
ament_export_dependencies(rosidl_default_runtime)

add_executable(fastbot_action_server src/fastbot_action_server.cpp)
ament_target_dependencies(fastbot_action_server
  rclcpp rclcpp_action geometry_msgs nav_msgs tf2
)

# 2) Ensure the action types are generated BEFORE compiling, and link typesupport
add_dependencies(fastbot_action_server ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fastbot_action_server ${cpp_typesupport_target})

# (Optional but helpful) Include the generated headers explicitly during build
target_include_directories(fastbot_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR>/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>)

install(TARGETS fastbot_action_server
  DESTINATION lib/${PROJECT_NAME})

ament_add_gtest(test_waypoints test/test_waypoints.cpp)
if(TARGET test_waypoints)
  ament_target_dependencies(test_waypoints
    rclcpp rclcpp_action geometry_msgs nav_msgs tf2
  )
  add_dependencies(test_waypoints ${PROJECT_NAME}__rosidl_generator_cpp)
  rosidl_get_typesupport_target(cpp_ts_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(test_waypoints ${cpp_ts_target})
  target_include_directories(test_waypoints PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR>/rosidl_generator_cpp>
    $<INSTALL_INTERFACE:include>)
endif()

ament_package()
