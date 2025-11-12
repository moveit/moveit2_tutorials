add_executable(pose_tracking_tutorial src/pose_tracking_tutorial.cpp)
target_include_directories(pose_tracking_tutorial PUBLIC include)
target_link_libraries(pose_tracking_tutorial PUBLIC tf2_eigen::tf2_eigen
   #   moveit_ros_planning::default_request_adapter_parameters
   #   moveit_ros_planning::default_response_adapter_parameters
   #   moveit_ros_planning::kinematics_parameters
   #   moveit_ros_planning::moveit_collision_plugin_loader
   #   moveit_ros_planning::moveit_constraint_sampler_manager_loader
   #   moveit_ros_planning::moveit_cpp
   #   moveit_ros_planning::moveit_kinematics_plugin_loader
   #   moveit_ros_planning::moveit_plan_execution
   #   moveit_ros_planning::moveit_planning_pipeline
   #   moveit_ros_planning::moveit_planning_pipeline_interfaces
      moveit_ros_planning::moveit_planning_scene_monitor
   #   moveit_ros_planning::moveit_rdf_loader
   #   moveit_ros_planning::moveit_robot_model_loader
   #   moveit_ros_planning::moveit_trajectory_execution_manager
   #   moveit_ros_planning::planning_pipeline_parameters
   #   moveit_ros_planning::srdf_publisher_node
 moveit_servo::moveit_servo_lib_cpp


)#rclcpp::rclcpp ${visualization_msgs_TARGETS} moveit_servo::moveit_servo_lib_cpp  ${moveit_msgs_TARGETS}  moveit_ros_planning::moveit_cpp control_msgs::control_msgs__rosidl_typesupport_fastrtps_cpp moveit_msgs::moveit_msgs__rosidl_typesupport_cpp visualization_msgs::visualization_msgs__rosidl_generator_c


install(
  TARGETS
    pose_tracking_tutorial
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
