add_executable(planning_scene_ros_api_tutorial
    src/planning_scene_ros_api_tutorial.cpp)
target_include_directories(planning_scene_ros_api_tutorial
    PUBLIC include)

target_link_libraries(planning_scene_ros_api_tutorial PRIVATE  rclcpp::rclcpp   moveit_msgs::moveit_msgs__rosidl_typesupport_cpp rviz_visual_tools::rviz_visual_tools
)#${geometry_msgs_TARGETS} )# moveit_core::moveit_planning_scene moveit_ros_planning::moveit_robot_model_loader  moveit_visual_tools::moveit_visual_tools)

install(TARGETS planning_scene_ros_api_tutorial
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
