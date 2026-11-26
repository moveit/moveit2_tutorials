add_executable(subframes_tutorial
    src/subframes_tutorial.cpp)
ament_target_dependencies(subframes_tutorial
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS subframes_tutorial
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
