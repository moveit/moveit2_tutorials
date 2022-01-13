add_executable(robot_model_and_robot_state_tutorial
    src/robot_model_and_robot_state_tutorial.cpp)
target_include_directories(robot_model_and_robot_state_tutorial
    PUBLIC include)
ament_target_dependencies(robot_model_and_robot_state_tutorial
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS robot_model_and_robot_state_tutorial
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
