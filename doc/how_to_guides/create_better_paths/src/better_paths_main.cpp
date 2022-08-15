#include <rclcpp/rclcpp.hpp>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <geometry_msgs/msg/point_stamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_tutorial");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  RCLCPP_INFO(LOGGER, "Initialize node");

  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "panda_arm";
  static const std::string LOGNAME = "better_paths_tutorial";

  /* Otherwise robot with zeros joint_states */
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(LOGGER, "Starting MoveIt Tutorials...");

  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto robot_start_state = planning_components->getStartState();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "panda_link0", "moveit_cpp_tutorial",
                                                      moveit_cpp_ptr->getPlanningSceneMonitor());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Better Paths Tutorial", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  const auto planning_pipeline_names = moveit_cpp_ptr->getPlanningPipelineNames(PLANNING_GROUP);

  for (auto it = planning_pipeline_names.begin(); it!=planning_pipeline_names.end();it++){
    RCLCPP_INFO_STREAM(LOGGER, "Pipeline names: '" << *it << "'");
  }

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  planning_components->setStartStateToCurrentState();

  // Benchmark function
  // Compute and print average path length, path similarity
  auto analyzeResult = [&](std::vector<moveit_cpp::PlanningComponent::PlanSolution> solutions) {
    // Adapted from https://github.com/ros-planning/moveit2/blob/main/moveit_ros/benchmarks/src/BenchmarkExecutor.cpp#L872
    // Analyzing the trajectories geometrically
    auto average_traj_len = 0.0;
    auto average_path_simularity = 0.0;

    // Calculate average path length
    for (auto solution : solutions)
    {
      // Only process successful solutions
      if (solution.error_code == moveit::core::MoveItErrorCode::SUCCESS)
      {
        // Compute path length
        for (std::size_t index = 1; index < solution.trajectory->getWayPointCount(); ++index)
          average_traj_len +=
              solution.trajectory->getWayPoint(index - 1).distance(solution.trajectory->getWayPoint(index));
      }
    }

    if (solutions.size() > 0)
    {
      average_traj_len /= solutions.size();
    }

    // Path similarity

    RCLCPP_INFO_STREAM(LOGGER, "Average path length (sum of L1 norms): '" << average_traj_len << " rad'");
    RCLCPP_INFO_STREAM(LOGGER, "Average path similarity: '" << average_path_simularity << "'");
  };

  // Some helper functions
  auto printAndPlan = [&](auto num_iterations, auto execute = false) {
    std::vector<moveit_cpp::PlanningComponent::PlanSolution> solutions;
    solutions.reserve(num_iterations);
    for (auto i = 0; i < num_iterations; i++)
    {
      planning_components->setStartStateToCurrentState();

      moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters multi_pipeline_plan_request;
      multi_pipeline_plan_request.load(node, { "one", "two", "three" });

      auto plan_solution = planning_components->plan(multi_pipeline_plan_request);

      // Check if PlanningComponents succeeded in finding the plan
      if (plan_solution)
      {
        // Visualize the trajectory in Rviz
        visual_tools.publishTrajectoryLine(plan_solution.trajectory, joint_model_group_ptr);
        visual_tools.trigger();
        solutions.push_back(plan_solution);
      }
      // Start the next plan
      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
      visual_tools.deleteAllMarkers();
      visual_tools.trigger();
      if (execute)
      {
        planning_components->execute();
      }
    }

    // Print result
    analyzeResult(solutions);
  };

  auto setJointGoal = [&](double const panda_joint1, double const panda_joint2, double const panda_joint3,
                          double const panda_joint4, double const panda_joint5, double const panda_joint6,
                          double const panda_joint7) {
    auto robot_goal_state = planning_components->getStartState();
    robot_goal_state->setJointPositions("panda_joint1", &panda_joint1);
    robot_goal_state->setJointPositions("panda_joint2", &panda_joint2);
    robot_goal_state->setJointPositions("panda_joint4", &panda_joint4);
    robot_goal_state->setJointPositions("panda_joint5", &panda_joint5);
    robot_goal_state->setJointPositions("panda_joint6", &panda_joint6);
    robot_goal_state->setJointPositions("panda_joint7", &panda_joint7);
    robot_goal_state->setJointPositions("panda_joint3", &panda_joint3);

    planning_components->setGoal(*robot_goal_state);
  };

  auto setCartGoal = [&](double const x, double const y, double const z, double const qx, double const qy,
                         double const qz, double const qw) {
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "panda_link0";
    target_pose.pose.orientation.x = qx;
    target_pose.pose.orientation.y = qy;
    target_pose.pose.orientation.z = qz;
    target_pose.pose.orientation.w = qw;
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
    planning_components->setGoal(target_pose, "panda_link8");
  };

  auto add_collision_objects = [&]() {
    moveit_msgs::msg::CollisionObject collision_object_1;
    moveit_msgs::msg::CollisionObject collision_object_2;
    moveit_msgs::msg::CollisionObject collision_object_3;

    collision_object_1.header.frame_id = "panda_link0";
    collision_object_1.id = "box1";

    collision_object_2.header.frame_id = "panda_link0";
    collision_object_2.id = "box2";

    collision_object_3.header.frame_id = "panda_link0";
    collision_object_3.id = "box3";

    shape_msgs::msg::SolidPrimitive box_1;
    box_1.type = box_1.BOX;
    box_1.dimensions = { 0.01, 0.12, 1.0 };

    shape_msgs::msg::SolidPrimitive box_2;
    box_2.type = box_2.BOX;
    box_2.dimensions = { 0.01, 1.0, 0.2 };

    // Add new collision objects
    geometry_msgs::msg::Pose box_pose_1;
    box_pose_1.position.x = 0.2;
    box_pose_1.position.y = 0.2;
    box_pose_1.position.z = 0.7;

    collision_object_1.primitives.push_back(box_1);
    collision_object_1.primitive_poses.push_back(box_pose_1);
    collision_object_1.operation = collision_object_1.ADD;

    geometry_msgs::msg::Pose box_pose_2;
    box_pose_2.position.x = 0.2;
    box_pose_2.position.y = -0.2;
    box_pose_2.position.z = 0.7;

    collision_object_2.primitives.push_back(box_1);
    collision_object_2.primitive_poses.push_back(box_pose_2);
    collision_object_2.operation = collision_object_2.ADD;

    geometry_msgs::msg::Pose box_pose_3;
    box_pose_3.position.x = -0.1;
    box_pose_3.position.y = 0.0;
    box_pose_3.position.z = 0.85;

    collision_object_3.primitives.push_back(box_2);
    collision_object_3.primitive_poses.push_back(box_pose_3);
    collision_object_3.operation = collision_object_3.ADD;

    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object_1);
      scene->processCollisionObjectMsg(collision_object_2);
      scene->processCollisionObjectMsg(collision_object_3);
    }  // Unlock PlanningScene
  };

  // Experiment 1 - Short motion - Cartesian goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  RCLCPP_INFO(LOGGER, "################ Short motion - Cartesian goal ##############");
  visual_tools.publishText(text_pose, "CartesianGoal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  setCartGoal(0.28, -0.2, 0.5, 0.92396, -0.3825, 1.3251e-12, 3.2002e-12);
  printAndPlan(3, false);

  // // Experiment 1 - Short motion - Joint goal
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  RCLCPP_INFO(LOGGER, "################ Short motion - Joint goal ##############");
  visual_tools.publishText(text_pose, "JointGoal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  setJointGoal(2.8886513579712823, 0.3875018855212286, 2.720914148047324, -2.876892569361917, 2.886120885454169,
               0.6383841437770176, 0.4312752220542212);
  printAndPlan(3, false);

  // // Experiment 2 - Same pose - Cartesian goal
  // // ^^^^^
  RCLCPP_INFO(LOGGER, "################ Same pose - Cartesian goal ##############");
  visual_tools.publishText(text_pose, "CartesianGoal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  setCartGoal(0.30702, -5.2214e-12, 0.59027, 0.92396, -0.3825, 1.3251e-12, 3.2002e-12);
  printAndPlan(3, false);

  //// Experiment 2 - Same pose - Joint goal
  //// ^^^^^
  RCLCPP_INFO(LOGGER, "################ Same pose - Joint goal ##############");
  visual_tools.publishText(text_pose, "JointGoal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  setJointGoal(0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785);
  printAndPlan(3, false);

  // // Experiment 3 - Long motion freespace - Joint goal
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  RCLCPP_INFO(LOGGER, "################ Long motion - Joint goal ##############");
  visual_tools.publishText(text_pose, "JointGoal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  setJointGoal(-2.583184932292678, -0.290335965663780, -1.030661387231159, -2.171781392507914, 2.897232510573447,
               1.1244922991023616, 2.708936891424673);
  printAndPlan(3, false);

  // // Experiment 3 - Long motion freespace - Cartesian goal
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  RCLCPP_INFO(LOGGER, "################ Long motion - Cartesian goal ##############");
  visual_tools.publishText(text_pose, "CartesianGoal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  setCartGoal(-0.42889, 0.26552, 0.6666, 0.92396, -0.3825, 1.3251e-12, 3.2002e-12);
  printAndPlan(3, false);

  // Experiment 4 - Long motion with collisions - Joint goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  RCLCPP_INFO(LOGGER, "################ Long motion - Joint goal ##############");
  add_collision_objects();
  visual_tools.publishText(text_pose, "JointGoal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  setJointGoal(-2.583184932292678, -0.290335965663780, -1.030661387231159, -2.171781392507914, 2.897232510573447,
               1.1244922991023616, 2.708936891424673);
  printAndPlan(3, false);

  //  // Experiment 4 - Long motion with collisions - Cartesian goal
  //  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  RCLCPP_INFO(LOGGER, "################ Long motion - Cartesian goal ##############");
  visual_tools.publishText(text_pose, "CartesianGoal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  setCartGoal(-0.42889, 0.26552, 0.6666, -0.013696, 0.005568, -0.38056, 0.92464);
  printAndPlan(3, false);

  RCLCPP_INFO(LOGGER, "Shutting down.");
  rclcpp::shutdown();
  return 0;
}
