add_executable(motion_planning_api_tutorial
    src/motion_planning_api_tutorial.cpp)
target_include_directories(motion_planning_api_tutorial
  PUBLIC include)
target_link_libraries(motion_planning_api_tutorial PUBLIC pluginlib::pluginlib moveit_ros_planning::moveit_robot_model_loader moveit_visual_tools::moveit_visual_tools
moveit_ros_planning_interface::moveit_move_group_interface)

install(TARGETS motion_planning_api_tutorial
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
