cmake_minimum_required(VERSION 3.22)
project(dual_arm_panda_moveit_config)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
