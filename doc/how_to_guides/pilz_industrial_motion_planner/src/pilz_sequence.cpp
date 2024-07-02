#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>  
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/planning_options.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>

/**
 * Pilz Example -- MoveGroupSequence service and action
 *
 * To run this example, first run this launch file:
 * ros2 launch moveit2_tutorials pilz_moveit.launch.py
 *
 * Then, run this file:
 * ros2 run moveit2_tutorials pilz_sequence
 * 
 */

/* Author: Alejo Mancinelli - 02/07/2024 */

using moveit_msgs::action::MoveGroupSequence;
using GoalHandleMoveGroupSequence = rclcpp_action::ClientGoalHandle<MoveGroupSequence>;

// Create a ROS logger
auto const LOGGER = rclcpp::get_logger("pilz_sequence_node");

int main(int argc, char** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared("pilz_sequence_node", node_options);

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Planning group
  static const std::string PLANNING_GROUP = "panda_arm";

  // Create the MoveIt MoveGroup Interface
  // In this case, this is just necessary por the visual interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "panda_link0", "move_group_tutorial",
                                              move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  moveit_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // [ --------------------------------------------------------------- ]
  // [ ----------------------- Motion Sequence ----------------------- ]
  // [ --------------------------------------------------------------- ]
  
  // TODO: Con el service no se devuelve el plan? Solo me sirve para poder graficarlo

  // Create a MotionSequenceRequest
  moveit_msgs::msg::MotionSequenceRequest sequence_request;

  
  // ----- Motion Sequence Item 1
  // Create a MotionSequenceItem
  moveit_msgs::msg::MotionSequenceItem item1;
  
  // Set pose blend radius
  item1.blend_radius = 0.1;

  // MotionSequenceItem configuration
  item1.req.group_name = PLANNING_GROUP;
  item1.req.planner_id = "LIN";
  item1.req.allowed_planning_time = 5;
  item1.req.max_velocity_scaling_factor = 0.1;
  item1.req.max_acceleration_scaling_factor = 0.1;
      
  moveit_msgs::msg::Constraints constraints_item1;
  moveit_msgs::msg::PositionConstraint pos_constraint_item1;
  pos_constraint_item1.header.frame_id = "world";
  pos_constraint_item1.link_name = "panda_hand";
  
  // Set a constraint pose
  auto target_pose_item1 = [] {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose.position.x = 0.3;
    msg.pose.position.y = -0.2;
    msg.pose.position.z = 0.6;
    msg.pose.orientation.x = 1.0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 0;
    return msg;
  } ();
  item1.req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints("panda_link8", target_pose_item1));

  // First MotionSequenceItem 
  sequence_request.items.push_back(item1);
  
  // ----- Motion Sequence Item 2
  // Create a MotionSequenceItem
  moveit_msgs::msg::MotionSequenceItem item2;
  
  // Set pose blend radius
  // For the last pose, it must be 0!
  item2.blend_radius = 0;
  
  // MotionSequenceItem configuration
  item2.req.group_name = PLANNING_GROUP;
  item2.req.planner_id = "LIN";
  item2.req.allowed_planning_time = 5;
  item2.req.max_velocity_scaling_factor = 0.1;
  item2.req.max_acceleration_scaling_factor = 0.1;
      
  // Set a constraint pose
  auto target_pose_item2 = [] {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose.position.x = 0.3;
    msg.pose.position.y = -0.2;
    msg.pose.position.z = 0.8;
    msg.pose.orientation.x = 1.0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 0;
    return msg;
  } ();
  item2.req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints("panda_link8", target_pose_item2));
  
  // Second MotionSequenceItem 
  sequence_request.items.push_back(item2);

  // [ --------------------------------------------------------------- ]
  // [ ------------------ MoveGroupSequence Service ------------------ ]
  // [ --------------------------------------------------------------- ]
  // The trajectory is returned but not executed

  using GetMotionSequence = moveit_msgs::srv::GetMotionSequence;
  auto service_client = node->create_client<GetMotionSequence>("/plan_sequence_path");

  while (!service_client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_WARN(LOGGER, "Waiting for service /plan_sequence_path to be available...");
  }
  
  // Create request
  auto service_request = std::make_shared<GetMotionSequence::Request>();
  service_request->request.items.push_back(item1);
  service_request->request.items.push_back(item2);

  // Call the service and process the result
  auto future = service_client->async_send_request(service_request);

  // Function to draw the trajectory
  auto const draw_trajectory_tool_path =
    [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("panda_arm")](
        auto const& trajectories) {
      for (const auto& trajectory : trajectories) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      }
    };

  auto response = future.get();
  if (response->response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_INFO(LOGGER, "Planning successful");

    // Access the planned trajectory
    auto trajectory = response->response.planned_trajectories;
    draw_trajectory_tool_path(trajectory);
    moveit_visual_tools.trigger();
  
  } else {
    RCLCPP_ERROR(LOGGER, "Planning failed with error code: %d", response->response.error_code.val);
  }
  
  moveit_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  // [ --------------------------------------------------------------- ]
  // [ ------------------ MoveGroupSequence Action ------------------- ]
  // [ --------------------------------------------------------------- ]
  // Plans and executes the trajectory

  // MoveGroupSequence action client
  using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;
  auto client = rclcpp_action::create_client<MoveGroupSequence>(node, "/sequence_move_group");

  // Verify that the action server is up and running
  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(LOGGER, "Error waiting for action server /sequence_move_group");
    return -1;
  }

  // Create action goal
  auto goal_msg = MoveGroupSequence::Goal();
  goal_msg.request = sequence_request;

  // Planning options
  goal_msg.planning_options.planning_scene_diff.is_diff = true;
  goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = true;
  // goal_msg.planning_options.plan_only = true;

  // Goal response callback
  auto send_goal_options = rclcpp_action::Client<MoveGroupSequence>::SendGoalOptions();
  send_goal_options.goal_response_callback = [](std::shared_ptr<GoalHandleMoveGroupSequence> goal_handle) {
    try {
      if (!goal_handle) {
        RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
      } else {
        RCLCPP_INFO(LOGGER, "Goal accepted by server, waiting for result");
      }
    }
    catch(const std::exception &e) {
      RCLCPP_ERROR(LOGGER, "Exception while waiting for goal response: %s", e.what());
    }
  };

  // Result callback
  send_goal_options.result_callback = [](const GoalHandleMoveGroupSequence::WrappedResult &result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(LOGGER, "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(LOGGER, "Goal was aborted. Status: %d", result.result->response.error_code.val);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(LOGGER, "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(LOGGER, "Unknown result code");
        break;
    }
    RCLCPP_INFO(LOGGER, "Result received");
  };

  // Send the action goal
  auto goal_handle_future = client->async_send_goal(goal_msg, send_goal_options);

  // Get result
  auto future_result = client->async_get_result(goal_handle_future.get());

  // Wait for the result
  if (future_result.valid()) {
    auto result = future_result.get();  // Blocks the program execution until it gets a response
    RCLCPP_INFO(LOGGER, "Action completed. Result: %d",  static_cast<int>(result.code));
  } else {
    RCLCPP_ERROR(LOGGER, "Action couldn't be completed.");
  }

  moveit_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // [ --------------------------------------------------------------- ]
  // [ ------------------------- Stop Motion ------------------------- ]
  // [ --------------------------------------------------------------- ]
  
  // Repeats the motion but after 2 seconds cancels the action
  auto goal_handle_future_new = client->async_send_goal(goal_msg, send_goal_options);
  sleep(2);
  auto future_cancel_motion = client->async_cancel_goal(goal_handle_future_new.get());

  // Wait until action cancel
  if (future_cancel_motion.valid()) {
    auto cancel_response = future_cancel_motion.get();  // Blocks the program execution until it gets a response

    if (!cancel_response->return_code) {
      RCLCPP_INFO(LOGGER, "The action has been cancel by the action server.");
    } else {
      RCLCPP_INFO(LOGGER, "Action cancel error. Code: %d.", cancel_response->return_code);
    }
  } else {
    RCLCPP_ERROR(LOGGER, "Action couldn't be cancel.");
  }
    
  rclcpp::shutdown();
  return 0;
}
