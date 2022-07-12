#include <rclcpp/rclcpp.hpp>
#include <string>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

moveit_msgs::msg::CollisionObject make_box(std::string name, std::string frame_id, float length, float width,
                                           float height, float x, float y, float z,
                                           geometry_msgs::msg::Quaternion orientation)
{
  moveit_msgs::msg::CollisionObject box;

  // Add the first box relative to the base link's frame.
  box.id = name.c_str();
  box.header.frame_id = frame_id.c_str();

  /* Define the primitive and its dimensions. */
  box.primitives.resize(1);
  box.primitives[0].type = box.primitives[0].BOX;
  box.primitives[0].dimensions.resize(3);
  box.primitives[0].dimensions[0] = length;
  box.primitives[0].dimensions[1] = width;
  box.primitives[0].dimensions[2] = height;

  /* Define the pose of the box. */
  box.primitive_poses.resize(1);
  box.primitive_poses[0].position.x = x;
  box.primitive_poses[0].position.y = y;
  box.primitive_poses[0].position.z = z;
  box.primitive_poses[0].orientation = orientation;

  box.operation = box.ADD;

  return box;
}
static const rclcpp::Logger LOGGER = rclcpp::get_logger("collision_scene_example");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto collision_scene_example_node = rclcpp::Node::make_shared("planning_scene_tutorial", node_options);

  // Check for robot_description_semantic param

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  // Create vector to hold 4 collision objects.
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(4);

  /* Create identity rotation quaternion */
  tf2::Quaternion zero_orientation;
  zero_orientation.setRPY(0, 0, 0);
  const geometry_msgs::msg::Quaternion zero_orientation_msg = tf2::toMsg(zero_orientation);

  // Add the boxes relative to the base link's frame.
  collision_objects[0] = make_box("box1", "panda_link0", 0.20, 0.20, 0.50, 0.20, 0.50, 0.25, zero_orientation_msg);
  collision_objects[1] = make_box("box2", "panda_link0", 0.25, 0.25, 1.75, -.55, -.55, 0.00, zero_orientation_msg);
  collision_objects[2] = make_box("box3", "panda_link0", 0.28, 0.28, 0.22, 0.50, -.55, 0.14, zero_orientation_msg);
  collision_objects[3] = make_box("box4", "panda_link0", 0.25, 0.25, 1.10, -0.4, 0.40, 0.50, zero_orientation_msg);

  planning_scene_interface_.applyCollisionObjects(collision_objects);

  rclcpp::shutdown();
  return 0;
}
