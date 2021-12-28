add_executable(motion_planning_api_tutorial
    src/motion_planning_api_tutorial.cpp)
# target_link_libraries(motion_planning_api_tutorial
#     ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(motion_planning_api_tutorial
  PUBLIC include)
ament_target_dependencies(motion_planning_api_tutorial
  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS motion_planning_api_tutorial DESTINATION
  # ${CATKIN_PACKAGE_BIN_DESTINATION})
  lib/${PROJECT_NAME})

# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )
