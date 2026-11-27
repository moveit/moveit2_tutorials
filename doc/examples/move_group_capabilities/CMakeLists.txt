find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    srv/GetCollisionObjects.srv
    DEPENDENCIES
    builtin_interfaces
    moveit_msgs)

add_library(get_collision_objects_service_capability SHARED
    src/get_collision_objects_service_capability.cpp)
rosidl_get_typesupport_target(${PROJECT_NAME}_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(get_collision_objects_service_capability ${${PROJECT_NAME}_target})
ament_target_dependencies(get_collision_objects_service_capability ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
    TARGETS
    get_collision_objects_service_capability
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
