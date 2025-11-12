find_package(moveit_visual_tools REQUIRED)

add_executable(moveit_cpp_tutorial src/moveit_cpp_tutorial.cpp)
target_include_directories(moveit_cpp_tutorial PUBLIC include)
#ament_target_dependencies(moveit_cpp_tutorial  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
target_link_libraries(moveit_cpp_tutorial PUBLIC rclcpp::rclcpp moveit_ros_planning::moveit_cpp moveit_visual_tools::moveit_visual_tools)

install(TARGETS moveit_cpp_tutorial
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
