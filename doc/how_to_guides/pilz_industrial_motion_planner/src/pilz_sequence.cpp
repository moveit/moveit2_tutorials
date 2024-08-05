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
  // In this case, this is just necessary for the visual interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{ node, "panda_link0", "move_group_tutorial",
                                                                     move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  moveit_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // [ --------------------------------------------------------------- ]
  // [ ----------------------- Motion Sequence ----------------------- ]
  // [ --------------------------------------------------------------- ]

  // ----- Motion Sequence Item 1
  // Create a MotionSequenceItem
  moveit_msgs::msg::MotionSequenceItem item1;

  // Set pose blend radius
  item1.blend_radius = 0.1;

  // MotionSequenceItem configuration
  item1.req.group_name = PLANNING_GROUP;
  item1.req.planner_id = "LIN";
  item1.req.allowed_planning_time = 5.0;
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
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.0;
    return msg;
  }();
  item1.req.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints("panda_link8", target_pose_item1));

  // ----- Motion Sequence Item 2
  // Create a MotionSequenceItem
  moveit_msgs::msg::MotionSequenceItem item2;

  // Set pose blend radius
  // For the last pose, it must be 0!
  item2.blend_radius = 0.0;

  // MotionSequenceItem configuration
  item2.req.group_name = PLANNING_GROUP;
  item2.req.planner_id = "LIN";
  item2.req.allowed_planning_time = 5.0;
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
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.0;
    return msg;
  }();
  item2.req.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints("panda_link8", target_pose_item2));

  // [ --------------------------------------------------------------- ]
  // [ ------------------ MoveGroupSequence Service ------------------ ]
  // [ --------------------------------------------------------------- ]
  // The trajectory is returned but not executed

  // MoveGroupSequence service client
  using GetMotionSequence = moveit_msgs::srv::GetMotionSequence;
  auto service_client = node->create_client<GetMotionSequence>("/plan_sequence_path");

  // Verify that the action server is up and running
  while (!service_client->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_WARN(LOGGER, "Waiting for service /plan_sequence_path to be available...");
  }

  // Create request
  auto service_request = std::make_shared<GetMotionSequence::Request>();
  service_request->request.items.push_back(item1);
  service_request->request.items.push_back(item2);

  // Call the service and process the result
  auto service_future = service_client->async_send_request(service_request);

  // Function to draw the trajectory
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
       jmg = move_group_interface.getRobotModel()->getJointModelGroup("panda_arm")](auto const& trajectories) {
        for (const auto& trajectory : trajectories)
        {
          moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
        }
      };

  // Wait for the result
  std::future_status service_status;
  do
  {
    switch (service_status = service_future.wait_for(std::chrono::seconds(1)); service_status)
    {
      case std::future_status::deferred:
        RCLCPP_ERROR(LOGGER, "Deferred");
        break;
      case std::future_status::timeout:
        RCLCPP_INFO(LOGGER, "Waiting for trajectory plan...");
        break;
      case std::future_status::ready:
        RCLCPP_INFO(LOGGER, "Service ready!");
        break;
    }
  } while (service_status != std::future_status::ready);

  auto service_response = service_future.get();
  if (service_response->response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Planning successful");

    // Access the planned trajectory
    auto trajectory = service_response->response.planned_trajectories;
    draw_trajectory_tool_path(trajectory);
    moveit_visual_tools.trigger();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning failed with error code: %d", service_response->response.error_code.val);

    rclcpp::shutdown();
    return 0;
  }

  moveit_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // [ --------------------------------------------------------------- ]
  // [ ------------------ MoveGroupSequence Action ------------------- ]
  // [ --------------------------------------------------------------- ]
  // Plans and executes the trajectory

  // MoveGroupSequence action client
  using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;
  auto action_client = rclcpp_action::create_client<MoveGroupSequence>(node, "/sequence_move_group");

  // Verify that the action server is up and running
  if (!action_client->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(LOGGER, "Error waiting for action server /sequence_move_group");
    return -1;
  }

  // Create a MotionSequenceRequest
  moveit_msgs::msg::MotionSequenceRequest sequence_request;
  sequence_request.items.push_back(item1);
  sequence_request.items.push_back(item2);

  // Create action goal
  auto goal_msg = MoveGroupSequence::Goal();
  goal_msg.request = sequence_request;

  // Planning options
  goal_msg.planning_options.planning_scene_diff.is_diff = true;
  goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = true;
  // goal_msg.planning_options.plan_only = true; // Uncomment to only plan the trajectory

  // Goal response callback
  auto send_goal_options = rclcpp_action::Client<MoveGroupSequence>::SendGoalOptions();
  send_goal_options.goal_response_callback = [](std::shared_ptr<GoalHandleMoveGroupSequence> goal_handle) {
    try
    {
      if (!goal_handle)
      {
        RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
      }
      else
      {
        RCLCPP_INFO(LOGGER, "Goal accepted by server, waiting for result");
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(LOGGER, "Exception while waiting for goal response: %s", e.what());
    }
  };

  // Result callback
  send_goal_options.result_callback = [](const GoalHandleMoveGroupSequence::WrappedResult& result) {
    switch (result.code)
    {
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
  auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

  // Get result
  auto action_result_future = action_client->async_get_result(goal_handle_future.get());

  // Wait for the result
  std::future_status action_status;
  do
  {
    switch (action_status = action_result_future.wait_for(std::chrono::seconds(1)); action_status)
    {
      case std::future_status::deferred:
        RCLCPP_ERROR(LOGGER, "Deferred");
        break;
      case std::future_status::timeout:
        RCLCPP_INFO(LOGGER, "Executing trajectory...");
        break;
      case std::future_status::ready:
        RCLCPP_INFO(LOGGER, "Action ready!");
        break;
    }
  } while (action_status != std::future_status::ready);

  if (action_result_future.valid())
  {
    auto result = action_result_future.get();
    RCLCPP_INFO(LOGGER, "Action completed. Result: %d", static_cast<int>(result.code));
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Action couldn't be completed.");
  }

  moveit_visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // [ --------------------------------------------------------------- ]
  // [ ------------------------- Stop Motion ------------------------- ]
  // [ --------------------------------------------------------------- ]

  // Repeats the motion but after 2 seconds cancels the action
  auto cancel_goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
  sleep(2);
  auto cancel_action_result_future = action_client->async_cancel_goal(cancel_goal_handle_future.get());

  // Wait for the result
  std::future_status action_cancel_status;
  do
  {
    switch (action_cancel_status = cancel_action_result_future.wait_for(std::chrono::seconds(1)); action_cancel_status)
    {
      case std::future_status::deferred:
        RCLCPP_ERROR(LOGGER, "Deferred");
        break;
      case std::future_status::timeout:
        RCLCPP_INFO(LOGGER, "Waiting for trajectory stop...");
        break;
      case std::future_status::ready:
        RCLCPP_INFO(LOGGER, "Action cancel!");
        break;
    }
  } while (action_cancel_status != std::future_status::ready);

  if (cancel_action_result_future.valid())
  {
    auto cancel_response = cancel_action_result_future.get();

    if (!cancel_response->return_code)
    {
      RCLCPP_INFO(LOGGER, "The action has been canceled by the action server.");
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Action cancel error. Code: %d.", cancel_response->return_code);
    }
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Action couldn't be canceled.");
  }

  rclcpp::shutdown();
  return 0;
}
