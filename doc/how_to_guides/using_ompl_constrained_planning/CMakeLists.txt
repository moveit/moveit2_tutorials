add_executable(ompl_constrained_planning
    src/ompl_constrained_planning_tutorial.cpp)
target_include_directories(ompl_constrained_planning
    PUBLIC include)
#ament_target_dependencies(ompl_constrained_planning  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
target_link_libraries(ompl_constrained_planning PUBLIC   moveit_ros_move_group::moveit_move_group_capabilities_base moveit_ros_planning_interface::moveit_move_group_interface moveit_visual_tools::moveit_visual_tools )

install(TARGETS ompl_constrained_planning
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
