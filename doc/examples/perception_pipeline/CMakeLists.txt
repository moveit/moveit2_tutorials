find_package(PCL REQUIRED)
set(SUBTUTORIAL_DEPENDS
  geometry_msgs
  pcl_conversions
  sensor_msgs
)
foreach(dep IN ITEMS ${SUBTUTORIAL_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

include_directories(include ${PCL_INCLUDE_DIRS})

add_executable(cylinder_segment src/cylinder_segment.cpp)
ament_target_dependencies(cylinder_segment
  ${SUBTUTORIAL_DEPENDS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY bags launch
    DESTINATION share/${PROJECT_NAME}
)
