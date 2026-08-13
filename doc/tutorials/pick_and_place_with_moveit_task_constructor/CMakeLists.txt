find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

add_executable(mtc_tutorial src/mtc_node.cpp)
  target_link_libraries(mtc_tutorial PUBLIC rclcpp::rclcpp moveit_core::moveit_planning_interface moveit_core::moveit_planning_scene
  moveit_ros_planning::moveit_planning_pipeline
        moveit_ros_planning::moveit_planning_pipeline_interfaces
        moveit_ros_planning::moveit_planning_scene_monitor
        moveit_ros_planning_interface::moveit_move_group_interface
        moveit_task_constructor_core::moveit_task_constructor_core
        moveit_task_constructor_core::moveit_task_constructor_core_stages)

add_executable(minimal_mtc_tutorial src/minimal.cpp)
  target_link_libraries(minimal_mtc_tutorial PUBLIC rclcpp::rclcpp moveit_core::moveit_planning_interface moveit_core::moveit_planning_scene
  moveit_ros_planning::moveit_planning_pipeline
        moveit_ros_planning::moveit_planning_pipeline_interfaces
        moveit_ros_planning::moveit_planning_scene_monitor
        moveit_ros_planning_interface::moveit_move_group_interface
        moveit_task_constructor_core::moveit_task_constructor_core
        moveit_task_constructor_core::moveit_task_constructor_core_stages)

install(TARGETS mtc_tutorial minimal_mtc_tutorial
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
