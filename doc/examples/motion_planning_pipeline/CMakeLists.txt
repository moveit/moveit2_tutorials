find_package(graph_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)

add_executable(motion_planning_pipeline_tutorial
    src/motion_planning_pipeline_tutorial.cpp)
target_include_directories(motion_planning_pipeline_tutorial
    PUBLIC include)
target_link_libraries(motion_planning_pipeline_tutorial PUBLIC
${graph_msgs_TARGETS}
pluginlib::pluginlib moveit_ros_planning::moveit_robot_model_loader
moveit_visual_tools::moveit_visual_tools rviz_visual_tools::rviz_visual_tools)

install(TARGETS motion_planning_pipeline_tutorial
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
