add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
target_include_directories(servo_keyboard_input PUBLIC include)
ament_target_dependencies(servo_keyboard_input  ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS servo_keyboard_input
  DESTINATION lib/${PROJECT_NAME}
)
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )
