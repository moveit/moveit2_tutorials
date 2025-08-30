set(INTERACTIVITY_LIB_NAME interactivity_utils)

add_library(${INTERACTIVITY_LIB_NAME}
  src/interactive_robot.cpp
  src/imarker.cpp
  src/pose_string.cpp
)

target_include_directories(${INTERACTIVITY_LIB_NAME}
    PUBLIC include)
ament_target_dependencies(${INTERACTIVITY_LIB_NAME}
    ${THIS_PACKAGE_INCLUDE_DEPENDS})


install(TARGETS ${INTERACTIVITY_LIB_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ! --- old code ---
# add_executable(interactivity_tutorial
#   src/interactivity_tutorial.cpp
#   src/interactive_robot.cpp
#   src/imarker.cpp
#   src/pose_string.cpp
# )
# target_link_libraries(interactivity_tutorial
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${interactive_markers_LIBRARIES}
# )

# add_executable(attached_body_tutorial
#   src/attached_body_tutorial.cpp
#   src/interactive_robot.cpp
#   src/imarker.cpp
#   src/pose_string.cpp
# )

# target_link_libraries(
#   ${INTERACTIVITY_LIB_NAME}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# install(
#   TARGETS
#     ${INTERACTIVITY_LIB_NAME}
#   LIBRARY
#   DESTINATION
#     ${CATKIN_PACKAGE_LIB_DESTINATION}
#   ARCHIVE
#   DESTINATION
#     ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME
#   DESTINATION
#     ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
