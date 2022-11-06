#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pilz_mtc");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  // psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto pilz_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
  pilz_planner->setPlannerId("CIRC");

  // PIPELINE
  {
    // Moves in a circle using the arc center pose
    auto stage =
        std::make_unique<mtc::stages::MoveTo>("move circle center", pilz_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(hand_frame);

    // Set the pose goal
    auto const goal_pose = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.orientation.x = 0.7071;
      msg.pose.orientation.y = 0.7071;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 0.0;
      msg.pose.position.x = 0.0;
      msg.pose.position.y = 0.3;
      msg.pose.position.z = 0.59;
      return msg;
    }();
    stage->setGoal(goal_pose);

    auto const center_pose = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.orientation.x = 0.707;
      msg.pose.orientation.y = 0.707;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 0.0;
      msg.pose.position.x = 0.0;
      msg.pose.position.y = 0.0;
      msg.pose.position.z = 0.59;
      return msg;
    }();

    moveit_msgs::msg::PositionConstraint pos_constraint;
    pos_constraint.header.frame_id = center_pose.header.frame_id;
    pos_constraint.link_name = hand_frame;
    pos_constraint.constraint_region.primitive_poses.resize(1);
    pos_constraint.constraint_region.primitive_poses[0] = center_pose.pose;
    pos_constraint.weight = 1.0;

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.name = "center";
    path_constraints.position_constraints.resize(1);
    path_constraints.position_constraints[0] = pos_constraint;
    stage->setPathConstraints(path_constraints);

    stage->properties().set("marker_ns", "move circle center");
    task.add(std::move(stage));
  }

  {
    // Return home
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  {
    // Moves in a circle using the arc interim pose
    auto stage =
        std::make_unique<mtc::stages::MoveTo>("move circle interim", pilz_planner);
    stage->setGroup(arm_group_name);

    // Set the pose goal
    auto const goal_pose = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.orientation.x = 0.707;
      msg.pose.orientation.y = -0.707;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 0.0;
      msg.pose.position.x = 0.0;
      msg.pose.position.y = -0.3;
      msg.pose.position.z = 0.59;
      return msg;
    }();
    stage->setIKFrame(hand_frame);
    stage->setGoal(goal_pose);

    auto const interim_pose = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.orientation.x = 0.9381665;
      msg.pose.orientation.y = -0.3461834;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 0.0;
      msg.pose.position.x = 0.21708;
      msg.pose.position.y = -0.21708;
      msg.pose.position.z = 0.69;
      return msg;
    }();

    moveit_msgs::msg::PositionConstraint pos_constraint;
    pos_constraint.header.frame_id = interim_pose.header.frame_id;
    pos_constraint.link_name = hand_frame;
    pos_constraint.constraint_region.primitive_poses.resize(1);
    pos_constraint.constraint_region.primitive_poses[0] = interim_pose.pose;
    pos_constraint.weight = 1.0;

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.name = "interim";
    path_constraints.position_constraints.resize(1);
    path_constraints.position_constraints[0] = pos_constraint;
    stage->setPathConstraints(path_constraints);

    stage->properties().set("marker_ns", "move circle interim");
    task.add(std::move(stage));
  }

  {
    // Return home again
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}