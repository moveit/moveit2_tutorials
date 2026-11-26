add_executable(bullet_collision_checker_tutorial
    src/bullet_collision_checker_tutorial.cpp)

target_link_libraries(bullet_collision_checker_tutorial
    ${interactive_markers_LIBRARIES}
    interactivity_utils
    ${Boost_LIBRARIES})
