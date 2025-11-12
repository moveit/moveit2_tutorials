find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)

add_executable(mtc_tutorial src/mtc_node.cpp)
#ament_target_dependencies(mtc_tutorial ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_link_libraries(mtc_tutorial PUBLIC rclcpp::rclcpp moveit_core::moveit_planning_interface moveit_core::moveit_planning_scene
  moveit_ros_planning::moveit_planning_pipeline
        moveit_ros_planning::moveit_planning_pipeline_interfaces
        moveit_ros_planning::moveit_planning_scene_monitor
        moveit_ros_planning_interface::moveit_move_group_interface
        ${moveit_task_constructor_core_TARGETS})

add_executable(minimal_mtc_tutorial src/minimal.cpp)
#ament_target_dependencies(minimal_mtc_tutorial ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_link_libraries(minimal_mtc_tutorial PUBLIC rclcpp::rclcpp moveit_core::moveit_planning_interface moveit_core::moveit_planning_scene
  moveit_ros_planning::moveit_planning_pipeline
        moveit_ros_planning::moveit_planning_pipeline_interfaces
        moveit_ros_planning::moveit_planning_scene_monitor
        moveit_ros_planning_interface::moveit_move_group_interface
        ${moveit_task_constructor_core_TARGETS})

install(TARGETS mtc_tutorial minimal_mtc_tutorial
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
