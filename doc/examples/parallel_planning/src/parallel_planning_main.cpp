#include <rclcpp/rclcpp.hpp>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <geometry_msgs/msg/point_stamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("parallel_planning_example");
const std::string PLANNING_GROUP = "panda_arm";
const std::string LOGNAME = "parallel_planning_example";
}  // namespace
namespace parallel_planning_example
{
/// \brief Utility class to create and interact with the parallel planning demo
class Demo
{
public:
  Demo(rclcpp::Node::SharedPtr node)
    : node_{ node }
    , moveit_cpp_{ std::make_shared<moveit_cpp::MoveItCpp>(node) }
    , planning_component_{ std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_) }
    , visual_tools_(node, "panda_link0", "parallel_planning_example", moveit_cpp_->getPlanningSceneMonitor())
  {
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();

    visual_tools_.deleteAllMarkers();
    visual_tools_.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools_.publishText(text_pose, "Parallel Planning Tutorial", rvt::WHITE, rvt::XLARGE);
    visual_tools_.trigger();

    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  }

  /// \brief Add a complex collision object to the planning scene
  void addCollisionObjects()
  {
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
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object_1);
      scene->processCollisionObjectMsg(collision_object_2);
      scene->processCollisionObjectMsg(collision_object_3);
    }  // Unlock PlanningScene
  }

  /// \brief Request a plan for the previously set joint goal and print debug information
  /// \param [in] panda_jointN Goal value for the Nth joint [rad]
  void planAndPrint(double const panda_joint1, double const panda_joint2, double const panda_joint3,
                    double const panda_joint4, double const panda_joint5, double const panda_joint6,
                    double const panda_joint7)
  {
    auto robot_goal_state = planning_component_->getStartState();
    robot_goal_state->setJointPositions("panda_joint1", &panda_joint1);
    robot_goal_state->setJointPositions("panda_joint2", &panda_joint2);
    robot_goal_state->setJointPositions("panda_joint4", &panda_joint4);
    robot_goal_state->setJointPositions("panda_joint5", &panda_joint5);
    robot_goal_state->setJointPositions("panda_joint6", &panda_joint6);
    robot_goal_state->setJointPositions("panda_joint7", &panda_joint7);
    robot_goal_state->setJointPositions("panda_joint3", &panda_joint3);

    planning_component_->setGoal(*robot_goal_state);

    planning_component_->setStartStateToCurrentState();

    moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters multi_pipeline_plan_request{
      node_, { "one", "two", "three" }
    };

    auto plan_solution = planning_component_->plan(multi_pipeline_plan_request);

    // Check if PlanningComponents succeeded in finding the plan
    if (plan_solution)
    {
      // Visualize the trajectory in Rviz
      auto robot_model_ptr = moveit_cpp_->getRobotModel();
      auto robot_start_state = planning_component_->getStartState();
      auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

      visual_tools_.publishTrajectoryLine(plan_solution.trajectory_, joint_model_group_ptr);
      visual_tools_.trigger();
    }

    planning_component_->execute();

    // Start the next plan
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    visual_tools_.deleteAllMarkers();
    visual_tools_.trigger();
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_;
  std::shared_ptr<moveit_cpp::PlanningComponent> planning_component_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
};
}  // namespace parallel_planning_example

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  RCLCPP_INFO(LOGGER, "Initialize node");

  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("parallel_planning_example", "", node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  parallel_planning_example::Demo demo(node);

  /* Otherwise robot with zeros joint_states */
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(LOGGER, "Starting MoveIt Tutorials...");

  // // Experiment 1 - Short free-space motion - Joint goal
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  RCLCPP_INFO(LOGGER, "################ Short motion - Joint goal ##############");
  demo.planAndPrint(2.8886513579712823, 0.3875018855212286, 2.720914148047324, -2.876892569361917, 2.886120885454169,
                    0.6383841437770176, 0.4312752220542212);

  // Experiment 2 - Long motion with collisions - Joint goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  RCLCPP_INFO(LOGGER, "################ Long motion - Joint goal ##############");
  demo.addCollisionObjects();
  demo.planAndPrint(-2.583184932292678, -0.290335965663780, -1.030661387231159, -2.171781392507914, 2.897232510573447,
                    1.1244922991023616, 2.708936891424673);

  rclcpp::shutdown();
  return 0;
}
