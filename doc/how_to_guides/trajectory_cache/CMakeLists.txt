add_executable(trajectory_cache_demo src/trajectory_cache_demo.cpp)
ament_target_dependencies(trajectory_cache_demo ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost moveit_ros_trajectory_cache)

install(TARGETS trajectory_cache_demo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
