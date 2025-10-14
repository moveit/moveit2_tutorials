add_executable(motion_planning_pipeline_tutorial
    src/motion_planning_pipeline_tutorial.cpp)
target_include_directories(motion_planning_pipeline_tutorial
    PUBLIC include)
#ament_target_dependencies(motion_planning_pipeline_tutorial  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(motion_planning_pipeline_tutorial PUBLIC pluginlib::pluginlib moveit_ros_planning::moveit_robot_model_loader   moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor moveit_visual_tools::moveit_visual_tools)


install(TARGETS motion_planning_pipeline_tutorial
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
