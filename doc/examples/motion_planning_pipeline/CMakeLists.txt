find_package(moveit_visual_tools REQUIRED)

add_executable(motion_planning_pipeline_tutorial
               src/motion_planning_pipeline_tutorial.cpp)
target_include_directories(motion_planning_pipeline_tutorial PUBLIC include)
target_link_libraries(motion_planning_pipeline_tutorial PUBLIC
    moveit_ros_planning::moveit_planning_pipeline
    moveit_visual_tools::moveit_visual_tools)

install(TARGETS motion_planning_pipeline_tutorial
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
