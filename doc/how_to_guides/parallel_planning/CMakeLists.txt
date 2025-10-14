find_package(moveit_visual_tools REQUIRED)

add_executable(parallel_planning_example src/parallel_planning_main.cpp)
target_include_directories(parallel_planning_example PRIVATE include)
#ament_target_dependencies(parallel_planning_example  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
target_link_libraries(parallel_planning_example PUBLIC rclcpp::rclcpp  moveit_ros_planning::moveit_cpp moveit_visual_tools::moveit_visual_tools moveit_ros_warehouse::moveit_warehouse)

install(TARGETS parallel_planning_example
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
