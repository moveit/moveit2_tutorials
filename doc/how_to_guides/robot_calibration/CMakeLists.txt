# cmake_minimum_required(VERSION 3.5)
# project(ur5_calibration)

# find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)
