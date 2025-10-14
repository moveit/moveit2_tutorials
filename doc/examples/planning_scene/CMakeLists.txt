add_executable(planning_scene_tutorial
    src/planning_scene_tutorial.cpp)
target_include_directories(planning_scene_tutorial
    PUBLIC include)
#ament_target_dependencies(planning_scene_tutorial  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
target_link_libraries(planning_scene_tutorial PUBLIC rclcpp::rclcpp moveit_ros_planning::moveit_robot_model_loader)

install(TARGETS planning_scene_tutorial
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
