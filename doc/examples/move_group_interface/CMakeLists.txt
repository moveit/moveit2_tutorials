add_executable(move_group_interface_tutorial
    src/move_group_interface_tutorial.cpp)
target_include_directories(move_group_interface_tutorial
    PUBLIC include)
#ament_target_dependencies(move_group_interface_tutorial ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
target_link_libraries(move_group_interface_tutorial PUBLIC  moveit_ros_move_group::moveit_move_group_capabilities_base
moveit_ros_planning_interface::moveit_move_group_interface moveit_visual_tools::moveit_visual_tools)

install(TARGETS move_group_interface_tutorial
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
