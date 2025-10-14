
add_executable(collision_scene_example
    src/collision_scene_example.cpp)
target_include_directories(collision_scene_example
    PUBLIC include)
target_link_libraries(collision_scene_example PUBLIC rclcpp::rclcpp moveit_ros_planning_interface::moveit_move_group_interface)

install(TARGETS collision_scene_example
    ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})
