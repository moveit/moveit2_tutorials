find_package(graph_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)

add_executable(pilz_move_group src/pilz_move_group.cpp)
target_include_directories(pilz_move_group PUBLIC include)
target_link_libraries(pilz_move_group PUBLIC ${moveit_task_constructor_core_TARGETS} rclcpp::rclcpp rclcpp_action::rclcpp_action moveit_ros_planning_interface::moveit_move_group_interface moveit_visual_tools::moveit_visual_tools)

add_executable(pilz_mtc src/pilz_mtc.cpp)
target_include_directories(pilz_mtc PUBLIC include)
target_link_libraries(pilz_mtc PUBLIC rclcpp::rclcpp rclcpp_action::rclcpp_action moveit_ros_planning_interface::moveit_move_group_interface moveit_visual_tools::moveit_visual_tools ${moveit_task_constructor_core_TARGETS})

add_executable(pilz_sequence src/pilz_sequence.cpp)
target_include_directories(pilz_sequence PUBLIC include)
target_link_libraries(pilz_sequence PUBLIC rclcpp::rclcpp rclcpp_action::rclcpp_action moveit_ros_planning_interface::moveit_move_group_interface moveit_visual_tools::moveit_visual_tools)

install(
  TARGETS
    pilz_move_group
    pilz_mtc
    pilz_sequence
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
