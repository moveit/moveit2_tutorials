add_subdirectory(dual_arm_panda_moveit_config)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
