add_executable(pipeline_testbench_example src/pipeline_testbench_main.cpp)
target_include_directories(pipeline_testbench_example PRIVATE include)
ament_target_dependencies(pipeline_testbench_example  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS pipeline_testbench_example
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
