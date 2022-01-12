add_executable(move_group_interface_tutorial
    src/move_group_interface_tutorial.cpp)
target_include_directories(move_group_interface_tutorial
    PUBLIC include)
ament_target_dependencies(move_group_interface_tutorial
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS move_group_interface_tutorial
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
