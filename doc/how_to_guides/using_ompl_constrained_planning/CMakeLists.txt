find_package(graph_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
add_executable(ompl_constrained_planning
    src/ompl_constrained_planning_tutorial.cpp)
target_include_directories(ompl_constrained_planning
    PUBLIC include)
target_link_libraries(ompl_constrained_planning PUBLIC
${graph_msgs_TARGETS}
moveit_ros_move_group::moveit_move_group_capabilities_base moveit_ros_planning_interface::moveit_move_group_interface moveit_visual_tools::moveit_visual_tools )

install(TARGETS ompl_constrained_planning
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
