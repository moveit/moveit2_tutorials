#include <rclcpp/rclcpp.hpp>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <geometry_msgs/msg/point_stamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Cost functions
#include <moveit/cost_functions/cost_functions.hpp>
// Warehouse
#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/trajectory_constraints_storage.h>
#include <warehouse_ros/database_loader.h>

namespace rvt = rviz_visual_tools;

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("plannner_cost_fn_example");
const std::string PLANNING_GROUP = "panda_arm";
static const std::vector<std::string> CONTROLLERS(1, "panda_arm_controller");
}  // namespace
namespace plannner_cost_fn_example
{

/// \brief Utility class to create and interact with the parallel planning demo
class Demo
{
public:
  Demo(rclcpp::Node::SharedPtr node)
    : node_{ node }
    , moveit_cpp_{ std::make_shared<moveit_cpp::MoveItCpp>(node) }
    , planning_component_{ std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_) }
    , visual_tools_(node, "panda_link0", "plannner_cost_fn_example", moveit_cpp_->getPlanningSceneMonitorNonConst())
  {
    moveit_cpp_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

    visual_tools_.deleteAllMarkers();
    visual_tools_.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools_.publishText(text_pose, "Parallel Planning Tutorial", rvt::WHITE, rvt::XLARGE);
    visual_tools_.trigger();
  }

  bool loadPlanningSceneAndQuery()
  {
    std::string hostname = "";
    int port = 0.0;
    std::string scene_name = "";

    node_->get_parameter_or(std::string("warehouse.host"), hostname, std::string("127.0.0.1"));
    node_->get_parameter_or(std::string("warehouse.port"), port, 33829);

    if (!node_->get_parameter("warehouse.scene_name", scene_name))
    {
      RCLCPP_WARN(LOGGER, "Warehouse scene_name NOT specified");
    }

    moveit_warehouse::PlanningSceneStorage* planning_scene_storage = nullptr;

    // Initialize database connection
    try
    {
      warehouse_ros::DatabaseLoader db_loader(node_);
      warehouse_ros::DatabaseConnection::Ptr warehouse_connection = db_loader.loadDatabase();
      warehouse_connection->setParams(hostname, port, 20);
      if (warehouse_connection->connect())
      {
        planning_scene_storage = new moveit_warehouse::PlanningSceneStorage(warehouse_connection);
        RCLCPP_INFO(LOGGER, "Connected to database: '%s'", hostname.c_str());
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Failed to connect to database");
        return false;
      }
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(LOGGER, "Failed to initialize planning scene storage: '%s'", e.what());
      return false;
    }

    // Load planning scene
    moveit_msgs::msg::PlanningScene scene_msg;
    try
    {
      if (!planning_scene_storage)
      {
        RCLCPP_ERROR(LOGGER, "No planning scene storage");
        return false;
      }

      if (planning_scene_storage->hasPlanningScene(scene_name))  // Just the world (no robot)
      {
        moveit_msgs::msg::PlanningSceneWorld world_meta_data;
        if (!planning_scene_storage->getPlanningSceneWorld(world_meta_data, scene_name))
        {
          RCLCPP_ERROR(LOGGER, "Failed to load planning scene world '%s'", scene_name.c_str());
          return false;
        }
        scene_msg.world = world_meta_data;
        scene_msg.robot_model_name = "No robot information. Using only world geometry.";
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Failed to find planning scene '%s'", scene_name.c_str());
        return false;
      }
    }
    catch (std::exception& ex)
    {
      RCLCPP_ERROR(LOGGER, "Error loading planning scene: %s", ex.what());
      return false;
    }

    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitorNonConst());
      scene->processPlanningSceneWorldMsg(scene_msg.world);
    }  // Unlock PlanningScene

    RCLCPP_INFO(LOGGER, "Loaded planning scene successfully");

    // Get planning scene query
    moveit_warehouse::MotionPlanRequestWithMetadata planning_query;
    std::string query_name = scene_name + "_query";
    try
    {
      planning_scene_storage->getPlanningQuery(planning_query, scene_name, query_name);
    }
    catch (std::exception& ex)
    {
      RCLCPP_ERROR(LOGGER, "Error loading motion planning query '%s': %s", query_name.c_str(), ex.what());
    }

    planning_query_request_ = static_cast<moveit_msgs::msg::MotionPlanRequest>(*planning_query);
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    visual_tools_.trigger();
    return true;
  }

  /// \brief Set joint goal state for next planning attempt
  /// \param [in] panda_jointN Goal value for the Nth joint [rad]
  void setJointGoal(double const panda_joint1, double const panda_joint2, double const panda_joint3,
                    double const panda_joint4, double const panda_joint5, double const panda_joint6,
                    double const panda_joint7)
  {
    auto robot_goal_state = planning_component_->getStartState();
    robot_goal_state->setJointPositions("panda_joint1", &panda_joint1);
    robot_goal_state->setJointPositions("panda_joint2", &panda_joint2);
    robot_goal_state->setJointPositions("panda_joint3", &panda_joint3);
    robot_goal_state->setJointPositions("panda_joint4", &panda_joint4);
    robot_goal_state->setJointPositions("panda_joint5", &panda_joint5);
    robot_goal_state->setJointPositions("panda_joint6", &panda_joint6);
    robot_goal_state->setJointPositions("panda_joint7", &panda_joint7);

    // Set goal state
    planning_component_->setGoal(*robot_goal_state);
  }

  /// \brief Set goal state for next planning attempt based on query loaded from the database
  void setQueryGoal()
  {
    // Set goal state
    if (!planning_query_request_.goal_constraints.empty())
    {
      planning_component_->setGoal(planning_query_request_.goal_constraints);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Planning query request does not contain any goal constraints");
    }
  }

  /// \brief Request a motion plan based on the assumption that a goal is set and print debug information.
  void planAndPrint()
  {
    // Set start state as current state
    planning_component_->setStartStateToCurrentState();

    // Get start state
    auto robot_start_state = planning_component_->getStartState();

    // Get planning scene
    auto planning_scene_monitor = moveit_cpp_->getPlanningSceneMonitorNonConst();
    planning_scene_monitor->updateFrameTransforms();
    auto planning_scene = [planning_scene_monitor] {
      planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor);
      return planning_scene::PlanningScene::clone(ls);
    }();

    auto group_name = planning_query_request_.group_name;
    // Set cost function
    planning_component_->setStateCostFunction(
        [robot_start_state, group_name, planning_scene](const std::vector<double>& state_vector) mutable {
          // Publish robot state
          // auto const ee_tip = robot_state.getJointModelGroup(PLANNING_GROUP)->getOnlyOneEndEffectorTip();
          // this->getVisualTools().publishSphere(robot_state.getGlobalLinkTransform(ee_tip), rviz_visual_tools::GREEN,
          // rviz_visual_tools::MEDIUM); this->getVisualTools().trigger();
          auto clearance_cost_fn =
              moveit::cost_functions::getClearanceCostFn(*robot_start_state, group_name, planning_scene);
          return clearance_cost_fn(state_vector);
        });

    auto plan_solution = planning_component_->plan();

    // Check if PlanningComponents succeeded in finding the plan
    if (plan_solution)
    {
      // Visualize the trajectory in Rviz
      auto robot_model_ptr = moveit_cpp_->getRobotModel();
      auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

      visual_tools_.publishTrajectoryLine(plan_solution.trajectory, joint_model_group_ptr);
      visual_tools_.trigger();
    }

    // Execute the trajectory and block until it's finished
    moveit_cpp_->execute(plan_solution.trajectory, true /* blocking*/, CONTROLLERS);

    // Start the next plan
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    visual_tools_.deleteAllMarkers();
    visual_tools_.trigger();
  }

  moveit_visual_tools::MoveItVisualTools& getVisualTools()
  {
    return visual_tools_;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_;
  std::shared_ptr<moveit_cpp::PlanningComponent> planning_component_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  moveit_msgs::msg::MotionPlanRequest planning_query_request_;
};
}  // namespace plannner_cost_fn_example

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  RCLCPP_INFO(LOGGER, "Initialize node");

  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("plannner_cost_fn_example", "", node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  plannner_cost_fn_example::Demo demo(node);

  if (!demo.loadPlanningSceneAndQuery())
  {
    rclcpp::shutdown();
    return 0;
  }

  RCLCPP_INFO(LOGGER, "Starting MoveIt Tutorials...");

  // Experiment - Long motion with collisions, CHOMP and Pilz are likely to fail here due to the difficulty of the planning problem
  RCLCPP_INFO(LOGGER, "Experiment - Long motion with collisions");
  demo.setQueryGoal();
  demo.planAndPrint();

  rclcpp::shutdown();
  return 0;
}
