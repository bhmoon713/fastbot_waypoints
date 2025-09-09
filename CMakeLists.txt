cmake_minimum_required(VERSION 3.8)
project(fastbot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# Add this to include generated headers
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

set(action_files
  "action/Waypoint.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
   ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

add_executable(fastbot_action_server src/fastbot_action_server.cpp)
ament_target_dependencies(fastbot_action_server rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_ros)

rosidl_target_interfaces(fastbot_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  fastbot_action_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)

  # Include the directories for the generated headers
  include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  )

  ament_add_gtest(test_fastbot_waypoint test/test_fastbot_waypoint.cpp)

  
  ament_target_dependencies(test_fastbot_waypoint rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_geometry_msgs )
 
  # Ensure the test links to the generated typesupport
  rosidl_target_interfaces(test_fastbot_waypoint ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

ament_package()