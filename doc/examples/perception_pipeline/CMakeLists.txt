#add_executable(cylinder_segment src/cylinder_segment.cpp)
#target_link_libraries(cylinder_segment ${ament_LIBRARIES})

add_executable(bag_publisher_maintain_time src/bag_publisher_maintain_time.cpp)
target_include_directories(bag_publisher_maintain_time
    PUBLIC include)
ament_target_dependencies(bag_publisher_maintain_time
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)


install(TARGETS bag_publisher_maintain_time
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY bags
  DESTINATION share/${PROJECT_NAME}
)

