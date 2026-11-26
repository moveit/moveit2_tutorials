add_executable(bag_publisher_maintain_time src/bag_publisher_maintain_time.cpp)
target_include_directories(bag_publisher_maintain_time
    PUBLIC include)
ament_target_dependencies(bag_publisher_maintain_time
    rclcpp rosbag2_cpp sensor_msgs Boost)

install(TARGETS bag_publisher_maintain_time
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY urdf launch meshes config worlds rviz2 bags
  DESTINATION share/moveit2_tutorials)
