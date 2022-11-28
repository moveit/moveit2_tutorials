#include <chrono>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/msg/color_rgba.hpp>

static const auto LOGGER = rclcpp::get_logger("ompl_constrained_planning_demo");
int main(int argc, char** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ompl_constrained_planning_demo_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "panda_arm");
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };

  rclcpp::sleep_for(1s);

  // Create some helpful lambdas
  auto current_pose = move_group_interface.getCurrentPose();

  // Creates a pose at a given positional offset from the current pose
  auto get_relative_pose = [current_pose, &moveit_visual_tools](double x, double y, double z) {
    auto target_pose = current_pose;
    target_pose.pose.position.x += x;
    target_pose.pose.position.y += y;
    target_pose.pose.position.z += z;
    moveit_visual_tools.publishSphere(current_pose.pose, rviz_visual_tools::RED, 0.05);
    moveit_visual_tools.publishSphere(target_pose.pose, rviz_visual_tools::GREEN, 0.05);
    moveit_visual_tools.trigger();
    return target_pose;
  };

  // Resets the demo by cleaning up any constraints and markers
  auto reset_demo = [&move_group_interface, &moveit_visual_tools]() {
    move_group_interface.clearPathConstraints();
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.trigger();
  };

  reset_demo();
  moveit_visual_tools.loadRemoteControl();
  moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to start with the box constraint example");

  // Create the first planning problem
  auto target_pose = get_relative_pose(0.0, 0.3, -0.3);

  // Let's try the simple box constraints first!
  moveit_msgs::msg::PositionConstraint box_constraint;
  box_constraint.header.frame_id = move_group_interface.getPoseReferenceFrame();
  box_constraint.link_name = move_group_interface.getEndEffectorLink();
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { 0.1, 0.4, 0.4 };
  box_constraint.constraint_region.primitives.emplace_back(box);

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = current_pose.pose.position.x;
  box_pose.position.y = 0.15;
  box_pose.position.z = 0.45;
  box_pose.orientation.w = 1.0;
  box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
  box_constraint.weight = 1.0;

  // Visualize the box constraint
  Eigen::Vector3d box_point_1(box_pose.position.x - box.dimensions[0] / 2, box_pose.position.y - box.dimensions[1] / 2,
                              box_pose.position.z - box.dimensions[2] / 2);
  Eigen::Vector3d box_point_2(box_pose.position.x + box.dimensions[0] / 2, box_pose.position.y + box.dimensions[1] / 2,
                              box_pose.position.z + box.dimensions[2] / 2);
  moveit_visual_tools.publishCuboid(box_point_1, box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);
  moveit_visual_tools.trigger();

  // We need to wrap the constraints in a generic `Constraints` message.
  moveit_msgs::msg::Constraints box_constraints;
  box_constraints.position_constraints.emplace_back(box_constraint);

  // Don't forget the path constraints! That's the whole point of this tutorial.
  move_group_interface.setPathConstraints(box_constraints);

  // Now we have everything we need to configure and solve a planning problem - plan to the target pose
  move_group_interface.setPoseTarget(target_pose);
  move_group_interface.setPlanningTime(10.0);

  // And let the planner find a solution.
  // The move_group node should automatically visualize the solution in Rviz if a path is found.
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Plan with box constraint %s", success ? "" : "FAILED");

  // Now wait for the user to press Next before trying the planar constraints.
  moveit_visual_tools.prompt(
      "Press 'Next' in the RvizVisualToolsGui window to continue to the planar constraint example");

  // Clear the path constraints and markers for the next example
  reset_demo();

  // In the second problem we plan with the end-effector constrained to a plane.
  // We need to create a pose goal that lies in this plane.
  // The plane is tilted by 45 degrees, so moving an equal amount in the y and z direction should be ok.
  // Any goal or start state should also satisfy the path constraints.
  target_pose = get_relative_pose(0.0, 0.3, -0.3);

  // We create a plane perpendicular to the y-axis and tilt it by 45 degrees

  // Solving the problem using equality constraints is a bit more complicated. (Or should I say, hacky?)
  // We need to tell the planner explicitly that we want to use equality constraints for the small dimensions.
  // This is achieved by setting the name of the constraint to :code:`"use_equality_constraints"`.
  // In addition, any dimension of the box below a threshold of :code:`0.001` will be considered an equality constraint.
  // However, if we make it too small, the box will be thinner that the tolerance used by OMPL to evaluate constraints
  // (:code:`1e-4` by default). MoveIt will use the stricter tolerance (the box width) to check the constraints, and
  // many states will appear invalid. That's where the magic number :code:`0.0005` comes from, it is between
  // :code:`0.00001` and :code:`0.001`.
  moveit_msgs::msg::PositionConstraint plane_constraint;
  plane_constraint.header.frame_id = move_group_interface.getPoseReferenceFrame();
  plane_constraint.link_name = move_group_interface.getEndEffectorLink();
  shape_msgs::msg::SolidPrimitive plane;
  plane.type = shape_msgs::msg::SolidPrimitive::BOX;
  plane.dimensions = { 1.0, 0.0005, 1.0 };
  plane_constraint.constraint_region.primitives.emplace_back(plane);

  geometry_msgs::msg::Pose plane_pose;
  plane_pose.position.x = current_pose.pose.position.x;
  plane_pose.position.y = current_pose.pose.position.y;
  plane_pose.position.z = current_pose.pose.position.z;
  plane_pose.orientation.x = sin(M_PI_4 / 2);
  plane_pose.orientation.y = 0.0;
  plane_pose.orientation.z = 0.0;
  plane_pose.orientation.w = cos(M_PI_4 / 2);
  plane_constraint.constraint_region.primitive_poses.emplace_back(plane_pose);
  plane_constraint.weight = 1.0;

  // Visualize the constraint
  auto d = sqrt(pow(target_pose.pose.position.y, 2) + pow(target_pose.pose.position.z, 2));

  Eigen::Vector3d normal(0, 1, 1);
  moveit_visual_tools.publishNormalAndDistancePlane(normal, d, rviz_visual_tools::TRANSLUCENT_DARK);
  moveit_visual_tools.trigger();

  moveit_msgs::msg::Constraints plane_constraints;
  plane_constraints.position_constraints.emplace_back(plane_constraint);
  plane_constraints.name = "use_equality_constraints";
  move_group_interface.setPathConstraints(plane_constraints);

  // And again, configure and solve the planning problem
  move_group_interface.setPoseTarget(target_pose);
  move_group_interface.setPlanningTime(10.0);
  success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Plan with plane constraint %s", success ? "" : "FAILED");

  moveit_visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to continue to the linear constraint example");

  reset_demo();

  // We can also plan along a line. We can use the same pose as last time.
  target_pose = get_relative_pose(0.0, 0.3, -0.3);

  // Building on the previous constraint, we can make it a line, by also reducing the dimension of the box in the x-direction.
  moveit_msgs::msg::PositionConstraint line_constraint;
  line_constraint.header.frame_id = move_group_interface.getPoseReferenceFrame();
  line_constraint.link_name = move_group_interface.getEndEffectorLink();
  shape_msgs::msg::SolidPrimitive line;
  line.type = shape_msgs::msg::SolidPrimitive::BOX;
  line.dimensions = { 0.0005, 0.0005, 1.0 };
  line_constraint.constraint_region.primitives.emplace_back(line);

  geometry_msgs::msg::Pose line_pose;
  line_pose.position.x = current_pose.pose.position.x;
  line_pose.position.y = current_pose.pose.position.y;
  line_pose.position.z = current_pose.pose.position.z;
  line_pose.orientation.x = sin(M_PI_4 / 2);
  line_pose.orientation.y = 0.0;
  line_pose.orientation.z = 0.0;
  line_pose.orientation.w = cos(M_PI_4 / 2);
  line_constraint.constraint_region.primitive_poses.emplace_back(line_pose);
  line_constraint.weight = 1.0;

  // Visualize the constraint
  moveit_visual_tools.publishLine(current_pose.pose.position, target_pose.pose.position,
                                  rviz_visual_tools::TRANSLUCENT_DARK);
  moveit_visual_tools.trigger();

  moveit_msgs::msg::Constraints line_constraints;
  line_constraints.position_constraints.emplace_back(line_constraint);
  line_constraints.name = "use_equality_constraints";
  move_group_interface.setPathConstraints(line_constraints);
  move_group_interface.setPoseTarget(target_pose);

  success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Plan with line constraint %s", success ? "" : "FAILED");

  moveit_visual_tools.prompt(
      "Press 'Next' in the RvizVisualToolsGui window to continue to the orientation constraint example");
  reset_demo();

  // Finally, we can place constraints on orientation.
  // Set the target pose to be the other side of the robot
  target_pose = get_relative_pose(-0.6, 0.1, 0);

  // Create an orientation constraint
  moveit_msgs::msg::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = move_group_interface.getPoseReferenceFrame();
  orientation_constraint.link_name = move_group_interface.getEndEffectorLink();

  orientation_constraint.orientation = current_pose.pose.orientation;
  orientation_constraint.absolute_x_axis_tolerance = 0.4;
  orientation_constraint.absolute_y_axis_tolerance = 0.4;
  orientation_constraint.absolute_z_axis_tolerance = 0.4;
  orientation_constraint.weight = 1.0;

  moveit_msgs::msg::Constraints orientation_constraints;
  orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);
  move_group_interface.setPathConstraints(orientation_constraints);
  move_group_interface.setPoseTarget(target_pose);

  success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Plan with orientation constraint %s", success ? "" : "FAILED");

  moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to try mixed_constraints");
  reset_demo();

  // Finally, we can place constraints on orientation.
  // Use the target pose from the previous example
  target_pose = get_relative_pose(-0.6, 0.1, 0);

  // Reuse the orientation constraint, and make a new box constraint
  box.dimensions = { 1.0, 0.6, 1.0 };
  box_constraint.constraint_region.primitives[0] = box;

  box_pose.position.x = 0;
  box_pose.position.y = -0.1;
  box_pose.position.z = current_pose.pose.position.z;
  box_constraint.constraint_region.primitive_poses[0] = box_pose;
  box_constraint.weight = 1.0;

  // Visualize the box constraint
  Eigen::Vector3d new_box_point_1(box_pose.position.x - box.dimensions[0] / 2,
                                  box_pose.position.y - box.dimensions[1] / 2,
                                  box_pose.position.z - box.dimensions[2] / 2);
  Eigen::Vector3d new_box_point_2(box_pose.position.x + box.dimensions[0] / 2,
                                  box_pose.position.y + box.dimensions[1] / 2,
                                  box_pose.position.z + box.dimensions[2] / 2);
  moveit_msgs::msg::Constraints mixed_constraints;
  mixed_constraints.position_constraints.emplace_back(box_constraint);
  mixed_constraints.orientation_constraints.emplace_back(orientation_constraint);
  moveit_visual_tools.publishCuboid(new_box_point_1, new_box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);
  moveit_visual_tools.trigger();

  move_group_interface.setPathConstraints(mixed_constraints);
  move_group_interface.setPoseTarget(target_pose);
  move_group_interface.setPlanningTime(20.0);

  success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Plan with mixed constraint %s", success ? "" : "FAILED");

  // Done!
  moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to clear the markers");
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.trigger();
  move_group_interface.clearPathConstraints();

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
