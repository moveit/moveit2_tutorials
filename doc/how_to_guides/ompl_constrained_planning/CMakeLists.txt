add_executable(ompl_constrained_planning_tutorial
    src/ompl_constrained_planning_tutorial.cpp)
target_include_directories(ompl_constrained_planning_tutorial
    PUBLIC include)
ament_target_dependencies(ompl_constrained_planning_tutorial
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS ompl_constrained_planning_tutorial
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
