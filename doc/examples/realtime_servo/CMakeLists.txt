find_package(graph_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rviz_visual_tools REQUIRED)

add_executable(pose_tracking_tutorial src/pose_tracking_tutorial.cpp)
target_include_directories(pose_tracking_tutorial PUBLIC include)
target_link_libraries(pose_tracking_tutorial PUBLIC
      ${graph_msgs_TARGETS} ${moveit_servo_TARGETS} tf2_eigen::tf2_eigen
      moveit_ros_planning::moveit_planning_scene_monitor
       rviz_visual_tools::rviz_visual_tools
)

install(
  TARGETS
    pose_tracking_tutorial
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
