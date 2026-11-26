add_executable(subframes_tutorial
    src/subframes_tutorial.cpp)
target_include_directories(subframes_tutorial
    PUBLIC include)
target_link_libraries(subframes_tutorial
    moveit_ros_planning_interface::moveit_move_group_interface
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_eigen::tf2_eigen
    Boost::boost
)

install(TARGETS subframes_tutorial
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
