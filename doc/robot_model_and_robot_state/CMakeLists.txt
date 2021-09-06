cmake_minimum_required(VERSION 3.8)
project(robot_model_and_robot_state_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

find_package(moveit_common REQUIRED)

find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(robot_model_and_robot_state_tutorial src/robot_model_and_robot_state_tutorial.cpp)
ament_target_dependencies(robot_model_and_robot_state_tutorial rclcpp)
ament_target_dependencies(robot_model_and_robot_state_tutorial moveit_core)
ament_target_dependencies(robot_model_and_robot_state_tutorial moveit_ros_planning)

install(TARGETS
robot_model_and_robot_state_tutorial
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
