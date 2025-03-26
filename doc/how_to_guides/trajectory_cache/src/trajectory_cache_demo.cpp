/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Intrinsic Innovation LLC.
 *  All rights reserved
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
 *   * Neither the name of Intrinsic Innovation LLC. nor the names of
 *     its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: methylDragon */

/** [TUTORIAL NOTE]
 *
 *                                  .     .
 *                               .  |\-^-/|  .
 *                              /| } O.=.O { |\
 *                             /´ \ \_ ~ _/ / `\
 *                           /´ |  \-/ ~ \-/  | `\
 *                           |   |  /\\ //\  |   |
 *                            \|\|\/-""-""-\/|/|/
 *                                    ______/ /
 *                                    '------'
 *                     _   _        _  ___
 *           _ __  ___| |_| |_ _  _| ||   \ _ _ __ _ __ _ ___ _ _
 *          | '  \/ -_)  _| ' \ || | || |) | '_/ _` / _` / _ \ ' \
 *          |_|_|_\___|\__|_||_\_, |_||___/|_| \__,_\__, \___/_||_|
 *                             |__/                 |___/
 *          -------------------------------------------------------
 *                          github.com/methylDragon
 *
 * NOTE:
 *   Tutorial notes will be commented like this block!
 *
 * PRE-REQUISITES
 * ^^^^^^^^^^^^^^
 * This tutorial assumes knowledge of the MoveGroupInterface.
 *
 * INTERACTIVITY
 * ^^^^^^^^^^^^^
 * This demo has four phases that can be advanced using the `rviz_visual_tools` dialog box.
 *
 * 1. Plan and cache (no pruning)
 * 2. Plan and cache (with pruning)
 * 3. Fetch from cache and execute (while still planning and caching with pruning)
 * 4. Fetch from cache and execute, except with large start tolerances
 *
 * This tutorial also supports "reconfigurable" parameters!:
 *
 * You can adjust them with:
 *   `ros2 param set /trajectory_cache_demo <parameter_name> <parameter_value>`
 *
 * Tutorial parameters:
 *   - planner:
 *       Defaults to "RRTstar". The OMPL planner used.
 *       It is better to use a random-sampling, non-optimal planner to see the cache working.
 *
 * Cache parameters:
 *   - start_tolerance:
 *       Defaults to 0.025. Determines the fuzziness of matching on planning start constraints.
 *   - goal_tolerance:
 *       Defaults to 0.001. Determines the fuzziness of matching on planning goal constraints.
 *   - prune_worse_trajectories:
 *       Defaults to true. Setting this to true will cause cached plans to be pruned.
 */

#include <atomic>
#include <optional>
#include <random>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_cache/cache_insert_policies/cache_insert_policy_interface.hpp>
#include <moveit/trajectory_cache/features/constant_features.hpp>
#include <moveit/trajectory_cache/features/features_interface.hpp>
#include <moveit/trajectory_cache/trajectory_cache.hpp>
#include <moveit/trajectory_cache/utils/utils.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <warehouse_ros/message_collection.h>

namespace  // Helpers.
{

// Consts and declarations. ========================================================================

namespace rvt = rviz_visual_tools;

using TrajectoryCacheEntryPtr = warehouse_ros::MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr;

using MotionPlanCacheInsertPolicy =
    moveit_ros::trajectory_cache::CacheInsertPolicyInterface<moveit_msgs::msg::MotionPlanRequest,
                                                             moveit::planning_interface::MoveGroupInterface::Plan,
                                                             moveit_msgs::msg::RobotTrajectory>;

using CartesianCacheInsertPolicy =
    moveit_ros::trajectory_cache::CacheInsertPolicyInterface<moveit_msgs::srv::GetCartesianPath::Request,
                                                             moveit_msgs::srv::GetCartesianPath::Response,
                                                             moveit_msgs::msg::RobotTrajectory>;

using CartesianFeatures = moveit_ros::trajectory_cache::FeaturesInterface<moveit_msgs::srv::GetCartesianPath::Request>;

static const size_t N_MOTION_PLANS_PER_ITERATION = 10;
static const double MIN_EXECUTABLE_FRACTION = 0.95;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("trajectory_cache_demo");

static const std::string HAIR_SPACE = " ";  // &hairsp; for rviz marker text display.
static const std::string PLANNING_GROUP = "panda_arm";

static std::random_device RD;
static std::mt19937 RAND_GEN(RD());

// Computation helpers. ============================================================================

// Generate a random 3D vector of some length.
geometry_msgs::msg::Point random3DVector(double length)
{
  std::uniform_real_distribution<> phiDist(0.0, 2.0 * 3.14159265358979323846);
  std::uniform_real_distribution<> cosThetaDist(-1.0, 1.0);

  double phi = phiDist(RAND_GEN);
  double cosTheta = cosThetaDist(RAND_GEN);
  double sinTheta = std::sqrt(1.0 - cosTheta * cosTheta);

  geometry_msgs::msg::Point out;
  out.x = sinTheta * std::sin(phi) * length;
  out.y = cosTheta * std::sin(phi) * length;
  out.z = std::cos(phi) * length;

  return out;
}

// Get start of a trajectory as a pose (wrt. base frame).
std::optional<geometry_msgs::msg::Pose>
getTrajectoryStartPose(moveit_visual_tools::MoveItVisualTools& visual_tools,
                       const moveit::planning_interface::MoveGroupInterface& move_group,
                       const moveit_msgs::msg::RobotTrajectory& trajectory_msg)
{
  if (trajectory_msg.joint_trajectory.points.empty())
  {
    return std::nullopt;
  }

  robot_trajectory::RobotTrajectoryPtr trajectory(
      new robot_trajectory::RobotTrajectory(move_group.getRobotModel(), move_group.getName()));
  trajectory->setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);

  const auto& tip_pose = trajectory->getWayPoint(0).getGlobalLinkTransform(move_group.getEndEffectorLink());
  if (tip_pose.translation().x() != tip_pose.translation().x())
  {
    RCLCPP_ERROR(LOGGER, "NAN DETECTED AT TRAJECTORY START POINT");
    return std::nullopt;
  }

  return visual_tools.convertPose(tip_pose);
}

// Get end of a trajectory as a pose (wrt. base frame).
std::optional<geometry_msgs::msg::Pose>
getTrajectoryEndPose(moveit_visual_tools::MoveItVisualTools& visual_tools,
                     const moveit::planning_interface::MoveGroupInterface& move_group,
                     const moveit_msgs::msg::RobotTrajectory& trajectory_msg)
{
  if (trajectory_msg.joint_trajectory.points.empty())
  {
    return std::nullopt;
  }

  robot_trajectory::RobotTrajectoryPtr trajectory(
      new robot_trajectory::RobotTrajectory(move_group.getRobotModel(), move_group.getName()));
  trajectory->setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);

  const auto& tip_pose = trajectory->getWayPoint(trajectory->getWayPointCount() - 1)
                             .getGlobalLinkTransform(move_group.getEndEffectorLink());
  if (tip_pose.translation().x() != tip_pose.translation().x())
  {
    RCLCPP_ERROR(LOGGER, "NAN DETECTED AT TRAJECTORY END POINT");
    return std::nullopt;
  }

  return visual_tools.convertPose(tip_pose);
}

// Parameters. =====================================================================================

struct ReconfigurableParameters
{
  ReconfigurableParameters(std::shared_ptr<rclcpp::Node> node) : node_(node)
  {
    update();
  }

  void update()
  {
    node_->get_parameter_or<double>("start_tolerance", start_tolerance, 0.025);
    node_->get_parameter_or<double>("goal_tolerance", goal_tolerance, 0.001);
    node_->get_parameter_or<bool>("prune_worse_trajectories", prune_worse_trajectories, false);
    node_->get_parameter_or<std::string>("planner", planner, "RRTstar");
  }

  double start_tolerance;
  double goal_tolerance;
  bool prune_worse_trajectories = true;
  std::string planner;

private:
  std::shared_ptr<rclcpp::Node> node_;
};

// Visualization helpers. ==========================================================================

// Visualize all cached trajectories.
void vizCachedTrajectories(moveit_visual_tools::MoveItVisualTools& visual_tools,
                           const moveit::core::JointModelGroup* joint_model_group,
                           const std::vector<TrajectoryCacheEntryPtr>& cached_trajectories, rvt::Colors color,
                           const std::vector<TrajectoryCacheEntryPtr>& exclude_trajectories = {})
{
  for (auto& cached_trajectory : cached_trajectories)
  {
    if (!cached_trajectory)
    {
      continue;
    }

    bool found_exclude = false;
    for (const auto& exclude_trajectory : exclude_trajectories)
    {
      if (!exclude_trajectory)
      {
        continue;
      }
      if (*exclude_trajectory == *cached_trajectory)
      {
        found_exclude = true;
        break;
      }
    }
    if (found_exclude)
    {
      continue;
    }

    visual_tools.publishTrajectoryLine(*cached_trajectory, joint_model_group, color);
  }
}

// Visualize all motion plan target poses.
void vizMotionPlanTargetPoses(moveit_visual_tools::MoveItVisualTools& visual_tools,
                              const std::vector<geometry_msgs::msg::PoseStamped>& target_poses,
                              const geometry_msgs::msg::PoseStamped& home_pose)
{
  visual_tools.publishAxisLabeled(home_pose.pose, "home", rvt::Scales::XXSMALL);
  for (const auto& target_pose : target_poses)
  {
    visual_tools.publishAxis(target_pose.pose, rvt::Scales::XXSMALL);
  }
}

// Visualize all cartesian path target poses.
void vizCartesianPathTargetPoses(moveit_visual_tools::MoveItVisualTools& visual_tools,
                                 const std::vector<geometry_msgs::msg::PoseStamped>& target_poses)
{
  for (const auto& target_pose : target_poses)
  {
    visual_tools.publishSphere(target_pose.pose, rvt::Colors::ORANGE, rvt::Scales::SMALL);
  }
}

// Visualize all cached motion plan trajectories.
void vizAllCachedMotionPlanTrajectories(moveit_visual_tools::MoveItVisualTools& visual_tools,
                                        const moveit::core::JointModelGroup* joint_model_group,
                                        moveit_ros::trajectory_cache::TrajectoryCache& trajectory_cache,
                                        const moveit::planning_interface::MoveGroupInterface& move_group,
                                        const moveit_msgs::msg::MotionPlanRequest& motion_plan_request_msg,
                                        rvt::Colors color,
                                        const std::vector<TrajectoryCacheEntryPtr>& exclude_trajectories = {})
{
  // We do this in a very hacky way, by specifying a ridiculously high tolerance and removing all constraints.
  for (const auto& cached_trajectory : trajectory_cache.fetchAllMatchingTrajectories(
           move_group, /*cache_namespace=*/PLANNING_GROUP,
           /*plan_request=*/motion_plan_request_msg,
           /*features=*/
           moveit_ros::trajectory_cache::TrajectoryCache::getDefaultFeatures(/*start_tolerance=*/9999,
                                                                             /*goal_tolerance=*/9999),
           /*sort_by=*/moveit_ros::trajectory_cache::TrajectoryCache::getDefaultSortFeature()))
  {
    bool found_exclude = false;
    for (const auto& exclude_trajectory : exclude_trajectories)
    {
      if (*exclude_trajectory == *cached_trajectory)
      {
        found_exclude = true;
        break;
      }
    }
    if (found_exclude)
    {
      continue;
    }

    visual_tools.publishTrajectoryLine(*cached_trajectory, joint_model_group, color);
  }
}

// Visualize all cached cartesian plan trajectories.
void vizAllCachedCartesianPlanTrajectories(moveit_visual_tools::MoveItVisualTools& visual_tools,
                                           const moveit::core::JointModelGroup* joint_model_group,
                                           moveit_ros::trajectory_cache::TrajectoryCache& trajectory_cache,
                                           const moveit::planning_interface::MoveGroupInterface& move_group,
                                           const moveit_msgs::srv::GetCartesianPath::Request& cartesian_plan_request_msg,
                                           rvt::Colors color,
                                           const std::vector<TrajectoryCacheEntryPtr>& exclude_trajectories = {})
{
  // We do this in a very hacky way, by specifying a ridiculously high tolerance and removing all constraints.
  for (const auto& cached_trajectory : trajectory_cache.fetchAllMatchingCartesianTrajectories(
           move_group, /*cache_namespace=*/PLANNING_GROUP,
           /*plan_request=*/cartesian_plan_request_msg,
           /*features=*/
           moveit_ros::trajectory_cache::TrajectoryCache::getDefaultCartesianFeatures(/*start_tolerance=*/9999,
                                                                                      /*goal_tolerance=*/9999,
                                                                                      /*min_fraction=*/0.0),
           /*sort_by=*/moveit_ros::trajectory_cache::TrajectoryCache::getDefaultSortFeature()))
  {
    bool found_exclude = false;
    for (const auto& exclude_trajectory : exclude_trajectories)
    {
      if (*exclude_trajectory == *cached_trajectory)
      {
        found_exclude = true;
        break;
      }
    }
    if (found_exclude)
    {
      continue;
    }

    visual_tools.publishTrajectoryLine(*cached_trajectory, joint_model_group, color);
  }
}

// Visualize diffs from trajectory.
// That is, the distance between plan start and goal poses, compared to the trajectory.
void vizTrajectoryDiffs(moveit_visual_tools::MoveItVisualTools& visual_tools,
                        const moveit::planning_interface::MoveGroupInterface& move_group,
                        const moveit_msgs::msg::RobotTrajectory& trajectory_msg,
                        const geometry_msgs::msg::Pose& goal_pose, rvt::Colors color, rvt::Scales scale = rvt::MEDIUM)
{
  std::optional<geometry_msgs::msg::Pose> trajectory_start_pose =
      getTrajectoryStartPose(visual_tools, move_group, trajectory_msg);
  if (!trajectory_start_pose.has_value())
  {
    return;
  }

  std::optional<geometry_msgs::msg::Pose> trajectory_end_pose =
      getTrajectoryEndPose(visual_tools, move_group, trajectory_msg);
  if (!trajectory_end_pose.has_value())
  {
    return;
  }

  visual_tools.publishLine(move_group.getCurrentPose().pose.position, trajectory_start_pose->position, color, scale);
  visual_tools.publishLine(goal_pose.position, trajectory_end_pose->position, color, scale);
}

// Visualize parameter text.
void vizParamText(moveit_visual_tools::MoveItVisualTools& visual_tools, const Eigen::Isometry3d& pose,
                  const std::string& cache_db_host, const ReconfigurableParameters& reconfigurable_parameters)
{
  auto param_text = std::stringstream();
  param_text << "[[PARAMETERS]]\n";
  param_text << "cache_db_host:" << HAIR_SPACE << cache_db_host << "\n";
  param_text << "start_tolerance:" << HAIR_SPACE << reconfigurable_parameters.start_tolerance << "\n";
  param_text << "goal_tolerance:" << HAIR_SPACE << reconfigurable_parameters.goal_tolerance << "\n";
  param_text << "prune_worse_trajectories:" << HAIR_SPACE
             << (reconfigurable_parameters.prune_worse_trajectories ? "true" : "false") << "\n";

  visual_tools.publishText(pose, param_text.str(), rvt::WHITE, rvt::XLARGE, false);
}

// Visualize demo phase text.
void vizDemoPhaseText(moveit_visual_tools::MoveItVisualTools& visual_tools, const Eigen::Isometry3d& pose,
                      size_t demo_phase)
{
  switch (demo_phase)
  {
    case 0:
    {
      visual_tools.publishText(pose, "Demo_CacheNoPruning(1/4)", rvt::WHITE, rvt::XXLARGE, false);
      return;
    }
    case 1:
    {
      visual_tools.publishText(pose, "Demo_CacheWithPruning(2/4)", rvt::WHITE, rvt::XXLARGE, false);
      return;
    }
    case 2:
    {
      visual_tools.publishText(pose, "Demo_ExecuteWithCache(3/4)", rvt::WHITE, rvt::XXLARGE, false);
      return;
    }
    case 3:
    default:
    {
      visual_tools.publishText(pose, "Demo_HighStartTolerance(4/4)", rvt::WHITE, rvt::XXLARGE, false);
      return;
    }
  }
}

// Visualize legend text.
void vizLegendText(moveit_visual_tools::MoveItVisualTools& visual_tools, const Eigen::Isometry3d& pose)
{
  static const std::string legend_text = []() {
    std::stringstream ss;
    ss << "[[LEGEND]]\n";
    ss << "TRANSLUCENT:" << HAIR_SPACE << "planner_plans"
       << "\n";
    ss << "GREY:" << HAIR_SPACE << "all_cached_plans"
       << "\n";
    ss << "WHITE:" << HAIR_SPACE << "matchable_cached_plans"
       << "\n";
    ss << "YELLOW:" << HAIR_SPACE << "matched_cached_plans"
       << "\n";
    ss << "GREEN:" << HAIR_SPACE << "best_cached_plan"
       << "\n";
    ss << "RED:" << HAIR_SPACE << "diff_to_trajectory"
       << "\n";
    return ss.str();
  }();

  visual_tools.publishText(pose, legend_text, rvt::WHITE, rvt::XLARGE, false);
}

// Visualize info text.
void vizInfoText(moveit_visual_tools::MoveItVisualTools& visual_tools, const Eigen::Isometry3d& pose,
                 moveit_ros::trajectory_cache::TrajectoryCache& trajectory_cache, TrajectoryCacheEntryPtr fetched_plan,
                 double fetch_time)
{
  auto info_text = std::stringstream();
  info_text << "cached-motion-plans:" << HAIR_SPACE << trajectory_cache.countTrajectories(PLANNING_GROUP) << "\n";
  info_text << "cached-cartesian-plans:" << HAIR_SPACE << trajectory_cache.countCartesianTrajectories(PLANNING_GROUP)
            << "\n";
  info_text << "fetched-plan-planning-time:" << HAIR_SPACE
            << (fetched_plan ? std::to_string(fetched_plan->lookupDouble("planning_time_s")) : "NO_ELIGIBLE_PLAN")
            << "\n";
  info_text << "fetched-plan-fetch-time:" << HAIR_SPACE << fetch_time << "\n";

  visual_tools.publishText(pose, info_text.str(), rvt::WHITE, rvt::XLARGE, false);
}

}  // namespace

int main(int argc, char** argv)
{
  // ===============================================================================================
  // SETUP
  // ===============================================================================================

  // ROS.
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("trajectory_cache_demo", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Move Group.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization.
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "panda_link0", "trajectory_cache_demo",
                                                      move_group.getRobotModel());
  visual_tools.loadRemoteControl();
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  Eigen::Isometry3d title_pose = Eigen::Isometry3d::Identity();
  title_pose.translation().y() = -1.5;
  title_pose.translation().z() = 1.2;

  Eigen::Isometry3d legend_pose = Eigen::Isometry3d::Identity();
  legend_pose.translation().y() = -1.5;

  Eigen::Isometry3d param_pose = Eigen::Isometry3d::Identity();
  param_pose.translation().y() = -1.52;
  param_pose.translation().z() = 0.5;

  Eigen::Isometry3d info_pose = Eigen::Isometry3d::Identity();
  info_pose.translation().y() = -1.4;
  info_pose.translation().z() = 1.0;

  // ===============================================================================================
  // CONFIGURE
  // ===============================================================================================

  // Get parameters (we set these in the launch file). =============================================

  // Set up reconfigurable parameters.
  ReconfigurableParameters reconfigurable_parameters(move_group_node);

  // Tutorial params.
  size_t num_target_poses;
  size_t num_cartesian_target_paths_per_target_pose;
  double cartesian_path_distance_m;

  move_group_node->get_parameter_or<size_t>("num_target_poses", num_target_poses, 4);
  move_group_node->get_parameter_or<size_t>("num_cartesian_target_paths_per_target_pose",
                                            num_cartesian_target_paths_per_target_pose, 2);
  move_group_node->get_parameter_or<double>("cartesian_path_distance_m", cartesian_path_distance_m, 0.10);

  // Cache DB init.
  std::string cache_db_plugin;
  std::string cache_db_host;
  int64_t cache_db_port;

  move_group_node->get_parameter_or<std::string>("cache_db_plugin", cache_db_plugin,
                                                 "warehouse_ros_sqlite::DatabaseConnection");
  move_group_node->get_parameter_or<std::string>("cache_db_host", cache_db_host, ":memory:");
  move_group_node->get_parameter_or<int64_t>("cache_db_port", cache_db_port, 0);

  // Cache params.

  /** [TUTORIAL NOTE]
   *
   * `exact_match_precision`:
   *   Tolerance for float precision comparison for what counts as an exact match.
   *
   *   An exact match is when:
   *     (candidate >= value - (exact_match_precision / 2)
   *      && candidate <= value + (exact_match_precision / 2))
   *
   * When using the default cache features and default cache insert policy (which this demo does):
   *   Cache entries are matched based off their start and goal states (and other parameters),
   *
   *   And are considered "better" if they higher priority in the sorting order specified by
   *   `sort_by` (execution time by default) than exactly matching entries.
   *
   *   A cache entry is "exactly matching" if its parameters are close enough to another cache entry.
   *   The tolerance for this depends on the `exact_match_precision` arg passed in the cache
   *   trajectory's init() method.
   *
   * `cartesian_max_step` and `cartesian_jump_threshold`:
   *   Used for constraining cartesian planning.
   */
  double exact_match_precision;
  double cartesian_max_step;
  double cartesian_jump_threshold;

  move_group_node->get_parameter_or<double>("exact_match_precision", exact_match_precision, 1e-6);
  move_group_node->get_parameter_or<double>("cartesian_max_step", cartesian_max_step, 0.001);
  move_group_node->get_parameter_or<double>("cartesian_jump_threshold", cartesian_jump_threshold, 0.0);

  // Generate targets. =============================================================================

  std::vector<geometry_msgs::msg::PoseStamped> target_poses;
  target_poses.reserve(num_target_poses);

  std::vector<geometry_msgs::msg::PoseStamped> target_cartesian_poses;
  target_cartesian_poses.reserve(num_target_poses * num_cartesian_target_paths_per_target_pose);

  move_group.rememberJointValues("home_pose");
  geometry_msgs::msg::PoseStamped home_pose = move_group.getCurrentPose();

  for (size_t i = 0; i < num_target_poses; ++i)
  {
    target_poses.push_back(move_group.getRandomPose());
  }

  for (const auto& target_pose : target_poses)
  {
    for (size_t i = 0; i < num_cartesian_target_paths_per_target_pose; ++i)
    {
      target_cartesian_poses.push_back(target_pose);

      geometry_msgs::msg::Point random_cartesian_diff = random3DVector(cartesian_path_distance_m);
      target_cartesian_poses.back().pose.position.x += random_cartesian_diff.x;
      target_cartesian_poses.back().pose.position.y += random_cartesian_diff.y;
      target_cartesian_poses.back().pose.position.z += random_cartesian_diff.z;
    }
  }

  // ===============================================================================================
  // DEMO
  // ===============================================================================================

  // Init trajectory cache. ========================================================================

  moveit_ros::trajectory_cache::TrajectoryCache trajectory_cache(move_group_node);

  /** [TUTORIAL NOTE]
   *
   * The trajectory cache must be initialized, to start a connection with the DB.
   *
   * NOTE:
   *   The `warehouse_plugin` parameter must be set. This was set in the launch file.
   *
   * We can also use this to set the exact match precision.
   * As noted above, this is the tolerance for float precision comparison for what counts as an
   * exact match.
   */

  moveit_ros::trajectory_cache::TrajectoryCache::Options options;
  options.db_path = cache_db_host;
  options.db_port = cache_db_port;
  options.exact_match_precision = exact_match_precision;
  options.num_additional_trajectories_to_preserve_when_pruning_worse = 0;

  if (!trajectory_cache.init(options))
  {
    RCLCPP_FATAL(LOGGER, "Could not init cache.");
    return 1;
  }

  // Interactivity. ================================================================================

  /** [TUTORIAL NOTE]
   * This demo has four phases that can be advanced using the `rviz_visual_tools` dialog box.
   *
   * 1. Plan and cache (no pruning)
   *   - Showcases basic cache functionality.
   * 2. Plan and cache (with pruning)
   *   - Showcases pruning functionality.
   * 3. Fetch from cache and execute (while still planning and caching with pruning)
   *   - Showcases cache fetches.
   * 4. Fetch from cache and execute, except with large start tolerances
   *   - Showcases fuzzy matching, and the impact of start tolerance.
   */

  std::atomic<size_t> demo_phase = 0;
  std::atomic<bool> run_execute = false;

  std::thread([&move_group_node, &visual_tools, &demo_phase, &run_execute]() {
    // Demo_CacheNoPruning.
    move_group_node->set_parameter(rclcpp::Parameter("prune_worse_trajectories", false));

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui to start pruning.");

    ++demo_phase;  // Demo_CacheWithPruning.
    move_group_node->set_parameter(rclcpp::Parameter("prune_worse_trajectories", true));

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui to start execution.");

    ++demo_phase;  // Demo_CacheAndExecute
    run_execute.store(true);

    visual_tools.prompt(
        "Press 'next' in the RvizVisualToolsGui to start execution with unreasonably high start tolerance.");

    ++demo_phase;  // Demo_CacheAndExecuteWithHighStartTolerance.
    move_group_node->set_parameter(rclcpp::Parameter("start_tolerance", 2.0));

    // We need the controller to also respect the start tolerance.
    auto set_param_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    set_param_request->parameters.emplace_back();
    set_param_request->parameters.back().name = "trajectory_execution.allowed_start_tolerance";
    set_param_request->parameters.back().value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;  // double.
    set_param_request->parameters.back().value.double_value = 2.5;  // With some buffer.
    move_group_node
        ->create_client<rcl_interfaces::srv::SetParameters>("/moveit_simple_controller_manager/set_parameters")
        ->async_send_request(set_param_request);
  }).detach();

  // Main loop. ====================================================================================

  moveit_msgs::msg::MotionPlanRequest to_target_motion_plan_request_msg;
  moveit_msgs::srv::GetCartesianPath::Request cartesian_path_request;
  moveit_msgs::msg::MotionPlanRequest to_home_motion_plan_request_msg;

  std::unique_ptr<MotionPlanCacheInsertPolicy> default_cache_insert_policy =
      moveit_ros::trajectory_cache::TrajectoryCache::getDefaultCacheInsertPolicy();

  std::unique_ptr<CartesianCacheInsertPolicy> default_cartesian_cache_insert_policy =
      moveit_ros::trajectory_cache::TrajectoryCache::getDefaultCartesianCacheInsertPolicy();

  /** [TUTORIAL NOTE]
   *
   * The loop will run a train-and-execute workflow where each iteration will do:
   *
   *   Permuting target pose and target cartesian diff sequentially:
   *     1. Motion plan N_MOTION_PLANS_PER_ITERATION times and execute to target pose.
   *     2. Cartesian plan and (best-effort) execute to target cartesian diff from pose.
   *     3. Motion plan N_MOTION_PLANS_PER_ITERATION times and execute to home pose.
   *
   * In each case, we will always plan and try to put into the cache.
   *
   * MULTIPLE PLANS
   * ^^^^^^^^^^^^^^
   * We plan multiple times to speed up cache convergence, and also to visualize how cached plans
   * are "better" than "worse", less-optimal plans.
   *
   * BEST PLANS
   * ^^^^^^^^^^
   * The executed plan will always be the best plan from the cache that matches the planning request
   * constraints, which will be from a plan in the current or prior iterations.
   *
   * (In this case "best" means smallest execution time, by default.)
   *
   * CACHE CONVERGENCE
   * ^^^^^^^^^^^^^^^^^
   * Over time execution time should improve as the cache collects more, and "better" plans.
   */

  while (rclcpp::ok())
  {
    for (size_t target_pose_i = 0; target_pose_i < target_poses.size(); ++target_pose_i)
    {
      for (size_t target_cartesian_pose_i = 0; target_cartesian_pose_i < num_cartesian_target_paths_per_target_pose;
           ++target_cartesian_pose_i)
      {
        auto target_pose = target_poses[target_pose_i];
        auto target_cartesian_pose =
            target_cartesian_poses[target_pose_i * num_cartesian_target_paths_per_target_pose + target_cartesian_pose_i];

        reconfigurable_parameters.update();
        move_group.setPlannerId(reconfigurable_parameters.planner);
        RCLCPP_INFO(LOGGER, "Set planner to: %s", reconfigurable_parameters.planner.c_str());

        // Plan and execute to target pose. ========================================================

        // Plan and Cache.
        move_group.setPoseTarget(target_pose.pose);

        to_target_motion_plan_request_msg = moveit_msgs::msg::MotionPlanRequest();
        move_group.constructMotionPlanRequest(to_target_motion_plan_request_msg);

        for (size_t i = 0; i < N_MOTION_PLANS_PER_ITERATION && rclcpp::ok(); ++i)
        {
          moveit::planning_interface::MoveGroupInterface::Plan to_target_motion_plan;
          if (move_group.plan(to_target_motion_plan) == moveit::core::MoveItErrorCode::SUCCESS)
          {
            RCLCPP_INFO(LOGGER, "Got plan to target pose from planner: %s. Planning time: %f",
                        reconfigurable_parameters.planner.c_str(), to_target_motion_plan.planning_time);
            visual_tools.publishTrajectoryLine(to_target_motion_plan.trajectory, joint_model_group,
                                               rvt::Colors::TRANSLUCENT);

            /** [TUTORIAL NOTE]
             *
             * For more information about how the cache works or the cache keying logic, see the
             * associated guide instead.
             *
             * INSERTING MOTION PLANS
             * ^^^^^^^^^^^^^^^^^^^^^^
             * The TrajectoryCache inserts plans based off a CacheInsertPolicy that you can specify.
             *
             * You can specify what cache insert policy to use using the `cache_insert_policy`
             * argument.
             *
             * The default policy inserts cache entries only if they have the best seen execution
             * time so far amongst other exactly matching cache entries.
             *
             * You may obtain a reusable instance of the default policy to pass by calling the
             * static `TrajectoryCache::getDefaultCacheInsertPolicy()` method.
             *
             * INSERT POLICIES AND FEATURES
             * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
             * The cache insert policy you use also constrains what features you can use to fetch
             * cache entries with. This is because the insert policy makes certain assumptions about
             * what cache features to extract and append to the cache entry to key the entry on.
             *
             * Be sure to check what features the cache insert policy you use supports!
             *
             * This will be further discussed in a later section of this demo.
             *
             * Additionally, a user may decide to attach additional features using the
             * `additional_features` field to append to the cache entry (provided it is also not
             * covered by the cache insert policy used) for use in fetching.
             *
             * CACHE PRUNING
             * ^^^^^^^^^^^^^
             * The cache insert policy selected also helps inform the cache for when an entry should
             * be pruned. Pruning allows for the cache memory/storage usage to be reduced, and also
             * reduces query time.
             *
             * The default policy marks "worse" (in terms of best seen execution time) exactly
             * matching cache entries for deletion, which the cache will use to execute cache
             * pruning.
             *
             * The cache's `num_additional_trajectories_to_preserve_when_pruning_worse` option can
             * be used to override the pruning behavior and preserve additional cache entries.
             *
             * NOTE:
             *   Pruning still happens even if a trajectory was not inserted, as long as the
             *   requirements for pruning are met.
             */
            trajectory_cache.insertTrajectory(  // Returns bool. True if put.
                move_group, /*cache_namespace=*/PLANNING_GROUP, /*plan_request=*/to_target_motion_plan_request_msg,
                /*plan=*/to_target_motion_plan,
                /*cache_insert_policy=*/*default_cache_insert_policy,
                /*prune_worse_trajectories=*/reconfigurable_parameters.prune_worse_trajectories,
                /*additional_features=*/{});
          }
          else
          {
            RCLCPP_WARN(LOGGER, "Could not get plan to target pose from planner: %s",
                        reconfigurable_parameters.planner.c_str());
          }
        }

        // Fetch.
        /** [TUTORIAL NOTE]
         *
         * FETCHING PLANS
         * ^^^^^^^^^^^^^^
         * The TrajectoryCache fetches plans based off a vector of Features of the plan request that
         * you can specify. These cache features will be used to fuzzily fetch either all matching
         * trajectories, or just the "best" one, sorted by some feature.
         *
         * You can specify what cache features to use to fetch and sort the fetched plans with using
         * the `features` and `sort_by` arguments respectively.
         *
         * As mentioned in the previous section, you can only fetch an inserted plan using a subset
         * of the features supported by the cache insert policy used to insert it (and/or any
         * `additional_features` provided at time of insert).
         *
         * Using any features not used during insert will cause the cache entry to fail to match,
         * and hence fail to be included in the returned fetch response.
         *
         * The default policy keys the entry on the plan request message (and the robot state from
         * the move group), including details such as:
         *   - Planning workspace
         *   - Starting joint state
         *   - Max velocity scaling, acceleration scaling, and cartesian speed
         *   - Goal constraints
         *   - Path constraints
         *   - Trajectory constraints
         *
         * The default sort feature is execution time.
         *
         * You may obtain a reusable instance of the default features and sort feature to pass for
         * given tolerances by calling the static `TrajectoryCache::getDefaultFeatures()` and
         * `TrajectoryCache::getDefaultSortFeature()` methods respectively.
         *
         * The default features uses all the features supported by the default cache insert policy,
         * and they are intended to be used together.
         *
         * MATCH TOLERANCES
         * ^^^^^^^^^^^^^^^^
         * For the default cache features, it is recommended to have a loose `start_tolerance` and a
         * stricter `goal_tolerance`.
         *
         * This is because, in most cases, the final position requested by a plan request is more
         * important. Consider:
         *   Even if the manipulator starts further away in configuration space than the first
         *   trajectory point of a fetched trajectory, on execution of the trajectory, the manipulator
         *   will "snap" to the start of the trajectory and follow it to the end point.
         *
         * As such, the start tolerance is only important to make sure that the "snapping" of the
         * manipulator to the start state of the fetched trajectory will be valid, whereas the goal
         * tolerance is important to make sure that the manipulator actually ends up close to the
         * requested goal.
         */
        std::vector<TrajectoryCacheEntryPtr> matched_to_target_trajectories;
        matched_to_target_trajectories = trajectory_cache.fetchAllMatchingTrajectories(
            move_group, /*cache_namespace=*/PLANNING_GROUP, /*plan_request=*/to_target_motion_plan_request_msg,
            /*features=*/
            moveit_ros::trajectory_cache::TrajectoryCache::getDefaultFeatures(reconfigurable_parameters.start_tolerance,
                                                                              reconfigurable_parameters.goal_tolerance),
            /*sort_by=*/moveit_ros::trajectory_cache::TrajectoryCache::getDefaultSortFeature(), /*ascending=*/true,
            /*metadata_only=*/false);

        TrajectoryCacheEntryPtr best_to_target_trajectory;

        auto best_to_target_fetch_start = move_group_node->now();
        best_to_target_trajectory = trajectory_cache.fetchBestMatchingTrajectory(
            move_group, /*cache_namespace=*/PLANNING_GROUP, /*plan_request=*/to_target_motion_plan_request_msg,
            moveit_ros::trajectory_cache::TrajectoryCache::getDefaultFeatures(reconfigurable_parameters.start_tolerance,
                                                                              reconfigurable_parameters.goal_tolerance),
            /*sort_by=*/moveit_ros::trajectory_cache::TrajectoryCache::getDefaultSortFeature(), /*ascending=*/true,
            /*metadata_only=*/false);
        auto best_to_target_fetch_end = move_group_node->now();

        if (matched_to_target_trajectories.empty() || !best_to_target_trajectory)
        {
          RCLCPP_FATAL(LOGGER, "No matched trajectories found.");
          return 1;
        }

        // Visualize.
        vizMotionPlanTargetPoses(visual_tools, target_poses, home_pose);
        vizCartesianPathTargetPoses(visual_tools, target_cartesian_poses);

        vizCachedTrajectories(visual_tools, joint_model_group, matched_to_target_trajectories, rvt::Colors::YELLOW,
                              /*exclude=*/{ best_to_target_trajectory });
        vizCachedTrajectories(visual_tools, joint_model_group, { best_to_target_trajectory }, rvt::Colors::LIME_GREEN);

        vizAllCachedMotionPlanTrajectories(visual_tools, joint_model_group, trajectory_cache, move_group,
                                           to_target_motion_plan_request_msg, rvt::Colors::WHITE,
                                           /*exclude=*/matched_to_target_trajectories);

        vizAllCachedCartesianPlanTrajectories(visual_tools, joint_model_group, trajectory_cache, move_group,
                                              cartesian_path_request, rvt::Colors::DARK_GREY,
                                              /*exclude=*/matched_to_target_trajectories);

        vizAllCachedMotionPlanTrajectories(visual_tools, joint_model_group, trajectory_cache, move_group,
                                           to_home_motion_plan_request_msg, rvt::Colors::DARK_GREY,
                                           /*exclude=*/matched_to_target_trajectories);

        vizTrajectoryDiffs(visual_tools, move_group, *best_to_target_trajectory, target_pose.pose, rvt::Colors::RED,
                           rvt::Scales::LARGE);

        vizParamText(visual_tools, param_pose, cache_db_host, reconfigurable_parameters);
        vizDemoPhaseText(visual_tools, title_pose, demo_phase.load());
        vizLegendText(visual_tools, legend_pose);
        vizInfoText(visual_tools, info_pose, trajectory_cache, best_to_target_trajectory,
                    (best_to_target_fetch_end - best_to_target_fetch_start).seconds());

        visual_tools.trigger();
        rclcpp::sleep_for(std::chrono::seconds(1));

        // Execute.
        if (run_execute.load())
        {
          move_group.execute(*best_to_target_trajectory);
        }

        // Cleanup.
        visual_tools.deleteAllMarkers();
        reconfigurable_parameters.update();

        // Interactivity Breakpoint. ===============================================================

        // We don't do the following steps unless we are in execute mode.
        if (!run_execute.load())
        {
          break;
        }

        // Plan and execute to target cartesian pose. ==============================================

        // Plan and Cache.
        cartesian_path_request = moveit_ros::trajectory_cache::constructGetCartesianPathRequest(
            move_group, { target_pose.pose, target_cartesian_pose.pose }, cartesian_max_step, cartesian_jump_threshold);

        // Cartesian plans are one-off, so we don't need to plan multiple times.
        moveit_msgs::srv::GetCartesianPath::Response cartesian_path_response;

        auto cartesian_plan_start = move_group_node->now();
        cartesian_path_response.fraction =
            move_group.computeCartesianPath(cartesian_path_request.waypoints, cartesian_max_step,
                                            cartesian_jump_threshold, cartesian_path_response.solution);
        auto cartesian_plan_end = move_group_node->now();

        if (cartesian_path_response.fraction >= MIN_EXECUTABLE_FRACTION)
        {
          /** [TUTORIAL NOTE]
           *
           * CARTESIAN PLANS
           * ^^^^^^^^^^^^^^^
           * The TrajectoryCache also supports inserting and fetching cartesian plans, with similar
           * capabilities to pass user-specified cache insert policies and cache features.
           *
           * Just call the cartesian variants of the cache methods, and use the cartesian variants of
           * the cache insert policies and cache features.
           *
           * For example:
           *   - `TrajectoryCache::getDefaultCartesianCacheInsertPolicy()`
           *   - `TrajectoryCache::getDefaultCartesianFeatures()`
           *
           * The default policy for cartesian plans keys the entry on the plan request message (and
           * the robot state from the move group), including details such as:
           *   - Planning workspace
           *   - Starting joint state
           *   - Max velocity scaling, acceleration scaling, and cartesian speed
           *   - Max end effector step and jump thresholds
           *   - Requested waypoints
           *   - Path constraints
           *
           * ADDITIONAL FEATURES
           * ^^^^^^^^^^^^^^^^^^^
           * In this example we also add a custom user-specified feature for planning time, since that
           * information is not found in the plan request or response to insert.
           *
           * Adding this feature will allow us to fetch and sort the cache entry we are inserting
           * using the "planning_time_s" feature.
           */
          std::vector<std::unique_ptr<CartesianFeatures>> additional_features;
          additional_features.push_back(
              std::make_unique<
                  moveit_ros::trajectory_cache::MetadataOnlyFeature<double, moveit_msgs::srv::GetCartesianPath::Request>>(
                  "planning_time_s", (cartesian_plan_end - cartesian_plan_start).seconds()));

          trajectory_cache.insertCartesianTrajectory(  // Returns bool. True if put.
              move_group, /*cache_namespace=*/PLANNING_GROUP, cartesian_path_request, cartesian_path_response,
              /*cache_insert_policy=*/*default_cartesian_cache_insert_policy,
              /*prune_worse_trajectories=*/reconfigurable_parameters.prune_worse_trajectories,
              /*additional_features=*/additional_features);
        }

        // Fetch.
        std::vector<TrajectoryCacheEntryPtr> matched_to_cartesian_trajectories;
        matched_to_cartesian_trajectories = trajectory_cache.fetchAllMatchingCartesianTrajectories(
            move_group, /*cache_namespace=*/PLANNING_GROUP, /*plan_request=*/cartesian_path_request,
            /*features=*/
            moveit_ros::trajectory_cache::TrajectoryCache::getDefaultCartesianFeatures(
                reconfigurable_parameters.start_tolerance, reconfigurable_parameters.goal_tolerance,
                MIN_EXECUTABLE_FRACTION),
            /*sort_by=*/moveit_ros::trajectory_cache::TrajectoryCache::getDefaultSortFeature(),
            /*ascending=*/true, /*metadata_only=*/false);

        TrajectoryCacheEntryPtr best_to_cartesian_trajectory;

        auto best_to_cartesian_fetch_start = move_group_node->now();
        best_to_cartesian_trajectory = trajectory_cache.fetchBestMatchingCartesianTrajectory(
            move_group, /*cache_namespace=*/PLANNING_GROUP, /*plan_request=*/cartesian_path_request,
            /*features=*/
            moveit_ros::trajectory_cache::TrajectoryCache::getDefaultCartesianFeatures(
                reconfigurable_parameters.start_tolerance, reconfigurable_parameters.goal_tolerance,
                MIN_EXECUTABLE_FRACTION),
            /*sort_by=*/moveit_ros::trajectory_cache::TrajectoryCache::getDefaultSortFeature(),
            /*ascending=*/true, /*metadata_only=*/false);
        auto best_to_cartesian_fetch_end = move_group_node->now();

        if (matched_to_cartesian_trajectories.empty() || !best_to_cartesian_trajectory)
        {
          RCLCPP_WARN(LOGGER, "No matched cartesian trajectories found.");
        }

        // Visualize.
        vizMotionPlanTargetPoses(visual_tools, target_poses, home_pose);
        vizCartesianPathTargetPoses(visual_tools, target_cartesian_poses);

        vizCachedTrajectories(visual_tools, joint_model_group, matched_to_cartesian_trajectories, rvt::Colors::YELLOW,
                              /*exclude=*/{ best_to_cartesian_trajectory });

        if (best_to_cartesian_trajectory)
        {
          vizCachedTrajectories(visual_tools, joint_model_group, { best_to_cartesian_trajectory },
                                rvt::Colors::LIME_GREEN);
          vizTrajectoryDiffs(visual_tools, move_group, *best_to_cartesian_trajectory, target_cartesian_pose.pose,
                             rvt::Colors::RED, rvt::Scales::LARGE);
        }

        vizAllCachedMotionPlanTrajectories(visual_tools, joint_model_group, trajectory_cache, move_group,
                                           to_target_motion_plan_request_msg, rvt::Colors::DARK_GREY,
                                           /*exclude=*/matched_to_cartesian_trajectories);

        vizAllCachedCartesianPlanTrajectories(visual_tools, joint_model_group, trajectory_cache, move_group,
                                              cartesian_path_request, rvt::Colors::WHITE,
                                              /*exclude=*/matched_to_cartesian_trajectories);

        vizAllCachedMotionPlanTrajectories(visual_tools, joint_model_group, trajectory_cache, move_group,
                                           to_home_motion_plan_request_msg, rvt::Colors::DARK_GREY,
                                           /*exclude=*/matched_to_cartesian_trajectories);

        vizParamText(visual_tools, param_pose, cache_db_host, reconfigurable_parameters);
        vizDemoPhaseText(visual_tools, title_pose, demo_phase.load());
        vizLegendText(visual_tools, legend_pose);
        vizInfoText(visual_tools, info_pose, trajectory_cache, best_to_cartesian_trajectory,
                    (best_to_cartesian_fetch_end - best_to_cartesian_fetch_start).seconds());

        visual_tools.trigger();
        rclcpp::sleep_for(std::chrono::seconds(1));

        // Execute.
        if (run_execute.load() && best_to_cartesian_trajectory &&
            best_to_cartesian_trajectory->lookupDouble("fraction") >= MIN_EXECUTABLE_FRACTION)
        {
          move_group.execute(*best_to_cartesian_trajectory);
        }

        // Cleanup.
        visual_tools.deleteAllMarkers();
        reconfigurable_parameters.update();

        // Plan and execute to home pose. ==========================================================

        // Plan and Cache.
        move_group.setNamedTarget("home_pose");  // Joint state target this time.

        to_home_motion_plan_request_msg = moveit_msgs::msg::MotionPlanRequest();
        move_group.constructMotionPlanRequest(to_home_motion_plan_request_msg);

        for (size_t i = 0; i < N_MOTION_PLANS_PER_ITERATION && rclcpp::ok(); ++i)
        {
          moveit::planning_interface::MoveGroupInterface::Plan to_home_motion_plan;
          if (move_group.plan(to_home_motion_plan) == moveit::core::MoveItErrorCode::SUCCESS)
          {
            RCLCPP_INFO(LOGGER, "Got plan to home pose from planner: %s. Planning time: %f",
                        reconfigurable_parameters.planner.c_str(), to_home_motion_plan.planning_time);
            visual_tools.publishTrajectoryLine(to_home_motion_plan.trajectory, joint_model_group,
                                               rvt::Colors::TRANSLUCENT_LIGHT);

            trajectory_cache.insertTrajectory(  // Returns bool. True if put.
                move_group, /*cache_namespace=*/PLANNING_GROUP, /*plan_request=*/to_home_motion_plan_request_msg,
                /*plan=*/to_home_motion_plan,
                /*cache_insert_policy=*/*default_cache_insert_policy,
                /*prune_worse_trajectories=*/reconfigurable_parameters.prune_worse_trajectories);
          }
          else
          {
            RCLCPP_WARN(LOGGER, "Could not get plan to target pose from planner: %s",
                        reconfigurable_parameters.planner.c_str());
          }
        }

        // Fetch.
        std::vector<TrajectoryCacheEntryPtr> matched_to_home_trajectories;
        matched_to_home_trajectories = trajectory_cache.fetchAllMatchingTrajectories(
            move_group, /*cache_namespace=*/PLANNING_GROUP, /*plan_request=*/to_home_motion_plan_request_msg,
            /*features=*/
            moveit_ros::trajectory_cache::TrajectoryCache::getDefaultFeatures(reconfigurable_parameters.start_tolerance,
                                                                              reconfigurable_parameters.goal_tolerance),
            /*sort_by=*/moveit_ros::trajectory_cache::TrajectoryCache::getDefaultSortFeature(), /*ascending=*/true,
            /*metadata_only=*/false);

        TrajectoryCacheEntryPtr best_to_home_trajectory;

        auto best_to_home_fetch_start = move_group_node->now();
        best_to_home_trajectory = trajectory_cache.fetchBestMatchingTrajectory(
            move_group, /*cache_namespace=*/PLANNING_GROUP, /*plan_request=*/to_home_motion_plan_request_msg,
            /*features=*/
            moveit_ros::trajectory_cache::TrajectoryCache::getDefaultFeatures(reconfigurable_parameters.start_tolerance,
                                                                              reconfigurable_parameters.goal_tolerance),
            /*sort_by=*/moveit_ros::trajectory_cache::TrajectoryCache::getDefaultSortFeature(), /*ascending=*/true,
            /*metadata_only=*/false);
        auto best_to_home_fetch_end = move_group_node->now();

        if (matched_to_home_trajectories.empty() || !best_to_home_trajectory)
        {
          RCLCPP_FATAL(LOGGER, "No matched trajectories found.");
          return 1;
        }

        // Visualize.
        vizMotionPlanTargetPoses(visual_tools, target_poses, home_pose);
        vizCartesianPathTargetPoses(visual_tools, target_cartesian_poses);

        vizCachedTrajectories(visual_tools, joint_model_group, matched_to_home_trajectories, rvt::Colors::YELLOW,
                              /*exclude=*/{ best_to_home_trajectory });
        vizCachedTrajectories(visual_tools, joint_model_group, { best_to_home_trajectory }, rvt::Colors::LIME_GREEN);

        vizAllCachedMotionPlanTrajectories(visual_tools, joint_model_group, trajectory_cache, move_group,
                                           to_target_motion_plan_request_msg, rvt::Colors::DARK_GREY,
                                           /*exclude=*/matched_to_home_trajectories);

        vizAllCachedCartesianPlanTrajectories(visual_tools, joint_model_group, trajectory_cache, move_group,
                                              cartesian_path_request, rvt::Colors::DARK_GREY,
                                              /*exclude=*/matched_to_home_trajectories);

        vizAllCachedMotionPlanTrajectories(visual_tools, joint_model_group, trajectory_cache, move_group,
                                           to_home_motion_plan_request_msg, rvt::Colors::WHITE,
                                           /*exclude=*/matched_to_home_trajectories);

        vizTrajectoryDiffs(visual_tools, move_group, *best_to_home_trajectory, home_pose.pose, rvt::Colors::RED,
                           rvt::Scales::LARGE);

        vizParamText(visual_tools, param_pose, cache_db_host, reconfigurable_parameters);
        vizDemoPhaseText(visual_tools, title_pose, demo_phase.load());
        vizLegendText(visual_tools, legend_pose);
        vizInfoText(visual_tools, info_pose, trajectory_cache, best_to_home_trajectory,
                    (best_to_home_fetch_end - best_to_home_fetch_start).seconds());

        visual_tools.trigger();
        rclcpp::sleep_for(std::chrono::seconds(1));

        // Execute.
        if (run_execute.load())
        {
          move_group.execute(*best_to_home_trajectory);
        }

        // Cleanup.
        visual_tools.deleteAllMarkers();
        reconfigurable_parameters.update();
      }
    }
  }

  rclcpp::shutdown();
  return 0;
}
