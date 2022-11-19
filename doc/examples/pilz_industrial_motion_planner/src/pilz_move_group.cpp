#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


/**
 * Pilz Example -- MoveGroup Interface
 * 
 * To run this example, first run this launch file:
 * ros2 launch moveit2_tutorials demo.launch.py rviz_config:=panda_hello_moveit.rviz
 *
 */

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "pilz_move_group_example",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit_log");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text)
  {
    auto const text_pose = []
    {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text)
  {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
       jmg = move_group_interface.getRobotModel()->getJointModelGroup(
           "panda_arm")](auto const trajectory)
  {
    moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
  };

  // Start planning //
  // Set up the Pilz planner to move in a circular motion
  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_interface.setPlannerId("CIRC");

  // Set a target pose.
  auto const target_pose = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 1.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.21708;
    msg.position.y = 0.21708;
    msg.position.z = 0.59;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Set a constraint pose. This is the center of the arc.
  auto const center_pose = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.0;
    msg.position.y = 0.0;
    msg.position.z = 0.59;
    return msg;
  }();
  moveit_msgs::msg::Constraints constraints;
  constraints.name = "center";
  moveit_msgs::msg::PositionConstraint pos_constraint;
  pos_constraint.header.frame_id = "panda_link0";
  pos_constraint.link_name = "panda_hand";
  pos_constraint.constraint_region.primitive_poses.push_back(center_pose);
  pos_constraint.weight = 1.0;
  constraints.position_constraints.push_back(pos_constraint);
  move_group_interface.setPathConstraints(constraints);

  // Create a plan to that target pose
  RCLCPP_INFO(logger, "Planning...");
  prompt("Press 'Next' in the RVizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    RCLCPP_INFO(logger, "Executing...");
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RVizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}