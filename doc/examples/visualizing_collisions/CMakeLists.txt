add_executable(visualizing_collisions_tutorial
  src/visualizing_collisions_tutorial.cpp
)

target_link_libraries(visualizing_collisions_tutorial
  ${interactive_markers_LIBRARIES}
  interactivity_utils
  ${Boost_LIBRARIES}
)
