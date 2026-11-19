find_package(graph_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(OpenSSL REQUIRED)

add_executable(parallel_planning_example src/parallel_planning_main.cpp)
target_include_directories(parallel_planning_example PRIVATE include)
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
