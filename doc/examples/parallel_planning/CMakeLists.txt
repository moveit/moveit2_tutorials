add_executable(parallel_planning_example src/parallel_planning_main.cpp)
target_include_directories(parallel_planning_example PRIVATE include)
ament_target_dependencies(parallel_planning_example  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS parallel_planning_example
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
