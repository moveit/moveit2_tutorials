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
  visual_tools.publishText(text_pose, "MoveItCpp_Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  planning_components->setStartStateToCurrentState();


// Some helper functions
auto printAndPlan = [&](auto num_iterations, auto execute = false){
  for(auto i = 0; i < num_iterations; i++)
  {
      planning_components->setStartStateToCurrentState();

  auto plan_solution2 = planning_components->plan();

    // Check if PlanningComponents succeeded in finding the plan
    if (plan_solution2)
    {
      // Visualize the trajectory in Rviz
      visual_tools.publishTrajectoryLine(plan_solution2.trajectory, joint_model_group_ptr);
      visual_tools.trigger();
    }
    // Start the next plan
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    if(execute)
    {
    planning_components->execute();}
    }};

auto setJointGoal = [&](double const panda_joint1, double const panda_joint2, 
  double const panda_joint3, double const panda_joint4, double const panda_joint5,
  double const panda_joint6, double const panda_joint7){
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
  double const qz, double const qw){
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

  // Experiment 0 - Long motion - Joint goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
RCLCPP_INFO(LOGGER, "################ Long motion - Joint goal ##############");
setJointGoal(-2.583184932292678, -0.290335965663780, -1.030661387231159, -2.171781392507914, 2.897232510573447, 1.1244922991023616, 2.708936891424673);

printAndPlan(3, false);

  // Experiment 0 - Long motion - Cartesian goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
RCLCPP_INFO(LOGGER, "################ Long motion - Cartesian goal ##############");
setCartGoal(-0.013696,0.005568,-0.38056,0.92464,-0.42889,0.26552,0.6666);
printAndPlan(3, false);


  // Experiment 1 - Short motion - Cartesian goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
RCLCPP_INFO(LOGGER, "################ Short motion - Cartesian goal ##############");
setCartGoal(0.28,-0.2, 0.5, 0.0, 0.0, 0.0, 1.0);
printAndPlan(3, false);

  // Experiment 1 - Short motion - Joint goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
RCLCPP_INFO(LOGGER, "################ Short motion - Joint goal ##############");
setJointGoal(2.8886513579712823, 0.3875018855212286, -2.876892569361917, 2.886120885454169, 0.6383841437770176, 0.4312752220542212, 2.720914148047324);
printAndPlan(3, true);



  // Experiment 2 - Same pose - Cartesian goal
  // ^^^^^
RCLCPP_INFO(LOGGER, "################ Same pose - Cartesian goal ##############");
setJointGoal(2.8886513579712823, 0.3875018855212286, -2.876892569361917, 2.886120885454169, 0.6383841437770176, 0.4312752220542212, 2.720914148047324);
printAndPlan(3, true);

RCLCPP_INFO(LOGGER, "################ Same pose - Joint goal ##############");
setJointGoal(2.8886513579712823, 0.3875018855212286, -2.876892569361917, 2.886120885454169, 0.6383841437770176, 0.4312752220542212, 2.720914148047324);
printAndPlan(3, true);


  RCLCPP_INFO(LOGGER, "Shutting down.");
  rclcpp::shutdown();
  return 0;
}
