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
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)


# Generate the custom action messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES std_msgs geometry_msgs
)


## ADD Executable
add_executable(fastbot_action_server src/fastbot_action_server.cpp)

ament_target_dependencies(fastbot_action_server
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
)

# Add dependency to the custom action message
rosidl_target_interfaces(fastbot_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  fastbot_action_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}/
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  set(TESTFILES
      test/main.cpp
      test/test_fastbot_waypoint.cpp
    )
  # Add a gtest executables
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    rclcpp_action
    std_msgs
    geometry_msgs
    nav_msgs
	tf2
  )
  rosidl_target_interfaces(${PROJECT_NAME}_test
    ${PROJECT_NAME} "rosidl_typesupport_cpp")

  install(TARGETS
        ${PROJECT_NAME}_test
        # Install the generated test executable file to the path after DESTINATION
        DESTINATION lib/${PROJECT_NAME})
   
  # uncomment to enforce lintners and static code analyzers defined in ament_lint_common package        
  # ament_lint_auto_find_test_dependencies()

endif()


ament_package()