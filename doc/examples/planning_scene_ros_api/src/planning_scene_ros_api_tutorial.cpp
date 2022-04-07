/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

// MoveIt
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <rviz_visual_tools/rviz_visual_tools.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_ros_api_tutorial");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("planning_scene_ros_api_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();
  // BEGIN_TUTORIAL
  //
  // Visualization
  // ^^^^^^^^^^^^^
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  rviz_visual_tools::RvizVisualTools visual_tools("panda_link0", "planning_scene_ros_api_tutorial", node);
  visual_tools.loadRemoteControl();
  visual_tools.deleteAllMarkers();

  // ROS API
  // ^^^^^^^
  // The ROS API to the planning scene publisher is through a topic interface
  // using "diffs". A planning scene diff is the difference between the current
  // planning scene (maintained by the move_group node) and the new planning
  // scene desired by the user.
  //
  // Advertise the required topic
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // We create a publisher and wait for subscribers.
  // Note that this topic may need to be remapped in the launch file.
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher->get_subscription_count() < 1)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Define the attached object message
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // We will use this message to add or
  // subtract the object from the world
  // and to attach the object to the robot.
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "panda_hand";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "panda_hand";
  /* The id of the object */
  attached_object.object.id = "box";

  /* A default pose */
  geometry_msgs::msg::Pose pose;
  pose.position.z = 0.11;
  pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.075;
  primitive.dimensions[1] = 0.075;
  primitive.dimensions[2] = 0.075;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  // Note that attaching an object to the robot requires
  // the corresponding operation to be specified as an ADD operation.
  attached_object.object.operation = attached_object.object.ADD;

  // Since we are attaching the object to the robot hand to simulate picking up the object,
  // we want the collision checker to ignore collisions between the object and the robot hand.
  attached_object.touch_links = std::vector<std::string>{ "panda_hand", "panda_leftfinger", "panda_rightfinger" };

  // Add an object into the environment
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Add the object into the environment by adding it to
  // the set of collision objects in the "world" part of the
  // planning scene. Note that we are using only the "object"
  // field of the attached_object message here.
  RCLCPP_INFO(LOGGER, "Adding the object into the world at the location of the hand.");
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher->publish(planning_scene);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Interlude: Synchronous vs Asynchronous updates
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // There are two separate mechanisms available to interact
  // with the move_group node using diffs:
  //
  // * Send a diff via a rosservice call and block until
  //   the diff is applied (synchronous update)
  // * Send a diff via a topic, continue even though the diff
  //   might not be applied yet (asynchronous update)
  //
  // While most of this tutorial uses the latter mechanism (given the long sleeps
  // inserted for visualization purposes asynchronous updates do not pose a problem),
  // it would be perfectly justified to replace the planning_scene_diff_publisher
  // by the following service client:
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_diff_client =
      node->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client->wait_for_service();
  // and send the diffs to the planning scene via a service call:
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = planning_scene;
  std::shared_future<std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response>> response_future;
  response_future = planning_scene_diff_client->async_send_request(request).future.share();

  // wait for the service to respond
  std::chrono::seconds wait_time(1);
  std::future_status fs = response_future.wait_for(wait_time);
  if (fs == std::future_status::timeout)
  {
    RCLCPP_ERROR(LOGGER, "Service timed out.");
  }
  else
  {
    std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response> planning_response;
    planning_response = response_future.get();
    if (planning_response->success)
    {
      RCLCPP_INFO(LOGGER, "Service successfully added object.");
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Service failed to add object.");
    }
  }

  // Note that this does not continue until we are sure the diff has been applied.
  //
  // Attach an object to the robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // When the robot picks up an object from the environment, we need to
  // "attach" the object to the robot so that any component dealing with
  // the robot model knows to account for the attached object, e.g. for
  // collision checking.
  //
  // Attaching an object requires two operations
  //  * Removing the original object from the environment
  //  * Attaching the object to the robot

  /* First, define the REMOVE object message*/
  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = "panda_hand";
  remove_object.operation = remove_object.REMOVE;

  // Note how we make sure that the diff message contains no other
  // attached objects or collisions objects by clearing those fields
  // first.
  /* Carry out the REMOVE + ATTACH operation */
  RCLCPP_INFO(LOGGER, "Attaching the object to the hand and removing it from the world.");
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene.robot_state.is_diff = true;
  planning_scene_diff_publisher->publish(planning_scene);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Detach an object from the robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Detaching an object from the robot requires two operations
  //  * Detaching the object from the robot
  //  * Re-introducing the object into the environment

  /* First, define the DETACH object message*/
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";
  detach_object.link_name = "panda_hand";
  detach_object.object.operation = attached_object.object.REMOVE;

  // Note how we make sure that the diff message contains no other
  // attached objects or collisions objects by clearing those fields
  // first.
  /* Carry out the DETACH + ADD operation */
  RCLCPP_INFO(LOGGER, "Detaching the object from the robot and returning it to the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.robot_state.is_diff = true;
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher->publish(planning_scene);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Remove the object from the collision world
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Removing the object from the collision world just requires
  // using the remove object message defined earlier.
  // Note, also how we make sure that the diff message contains no other
  // attached objects or collisions objects by clearing those fields
  // first.
  RCLCPP_INFO(LOGGER, "Removing the object from the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene_diff_publisher->publish(planning_scene);
  // END_TUTORIAL

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");

  rclcpp::shutdown();
  return 0;
}
