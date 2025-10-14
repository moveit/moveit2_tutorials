include_directories(include)
include_directories(
  SYSTEM
  ${Boost_INCLUDE_DIRS}
)

add_executable(lerp_example  src/lerp_example.cpp)
target_link_libraries(lerp_example
  ${Boost_LIBRARIES}
)

# Lerp planning plugin
add_library(moveit_lerp_planner_plugin
  src/lerp_planner_manager.cpp
  src/lerp_interface.cpp
  src/lerp_planning_context.cpp)
set_target_properties(moveit_lerp_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_lerp_planner_plugin  ${Boost_LIBRARIES})

#############
## Install ##
#############
