add_executable(mtc_tutorial src/mtc_node.cpp)
target_compile_options(mtc_tutorial PUBLIC -g -O0)
ament_target_dependencies(mtc_tutorial ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(minimal_mtc_tutorial src/minimal.cpp)
target_compile_options(minimal_mtc_tutorial PUBLIC -g -O0)
ament_target_dependencies(minimal_mtc_tutorial ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS mtc_tutorial minimal_mtc_tutorial
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
