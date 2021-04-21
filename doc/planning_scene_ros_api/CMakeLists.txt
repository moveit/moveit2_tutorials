add_executable(planning_scene_ros_api_tutorial
    src/planning_scene_ros_api_tutorial.cpp)
target_include_directories(planning_scene_ros_api_tutorial
    PUBLIC include)
ament_target_dependencies(planning_scene_ros_api_tutorial
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS planning_scene_ros_api_tutorial
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
