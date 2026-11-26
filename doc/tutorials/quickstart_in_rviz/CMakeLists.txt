install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(control_msgs REQUIRED)
  find_package(moveit_msgs REQUIRED)
  find_package(Boost CONFIG REQUIRED COMPONENTS filesystem)

  # Run all lint tests in package.xml
  ament_lint_auto_find_test_dependencies()


  # Basic "quickstart_in_rviz" bringup test
  ament_add_gtest_executable(bringup_test
    test/bringup_test.cpp
  )
  target_link_libraries(bringup_test ${control_msgs_TARGETS} ${moveit_msgs_TARGETS}
                        rclcpp_action::rclcpp_action)
  add_ros_test(test/bringup_test.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()
