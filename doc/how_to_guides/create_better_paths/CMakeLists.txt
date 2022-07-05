add_executable(better_paths_demo src/better_paths_main.cpp)
target_include_directories(better_paths_demo PUBLIC include)
ament_target_dependencies(better_paths_demo  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS better_paths_demo
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
