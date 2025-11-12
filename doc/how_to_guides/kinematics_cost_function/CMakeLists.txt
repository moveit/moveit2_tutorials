add_executable(kinematics_cost_function_tutorial
    src/kinematics_cost_function_tutorial.cpp)
target_include_directories(kinematics_cost_function_tutorial
    PUBLIC include)
#ament_target_dependencies(kinematics_cost_function_tutorial  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(kinematics_cost_function_tutorial PUBLIC moveit_ros_move_group::moveit_move_group_capabilities_base
moveit_ros_planning_interface::moveit_move_group_interface moveit_visual_tools::moveit_visual_tools)

install(TARGETS kinematics_cost_function_tutorial
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
