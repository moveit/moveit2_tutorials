install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(0)#BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(control_msgs REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem)

  # Run all lint tests in package.xml
  ament_lint_auto_find_test_dependencies()

 # target_link_libraries(bringup_test PUBLIC control_msgs::control_msgs)

  # Basic "quickstart_in_rviz" bringup test
  ament_add_gtest_executable(bringup_test
    test/bringup_test.cpp
  )
  add_ros_test(test/bringup_test.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()
