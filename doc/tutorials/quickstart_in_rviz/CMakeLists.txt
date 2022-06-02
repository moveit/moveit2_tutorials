install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem)

  # Run all lint tests in package.xml
  ament_lint_auto_find_test_dependencies()

  # Basic "quickstart_in_rviz" bringup test
  ament_add_gtest_executable(bringup_test
    test/bringup_test.cpp
  )
  ament_target_dependencies(bringup_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(test/bringup_test.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()
