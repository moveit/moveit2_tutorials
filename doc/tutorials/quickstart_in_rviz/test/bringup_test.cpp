// Description: Test if launching the demo is successful

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <gtest/gtest.h>
#include <stdlib.h>

namespace moveit2_tutorials::quickstart_in_rviz
{
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("bringup_test");
}

class BringupTestFixture : public testing::Test
{
public:
  BringupTestFixture()
    : node_(std::make_shared<rclcpp::Node>("basic_bringup_test"))
    , executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>())
  {
  }

  void SetUp() override
  {
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;
};

// Check if the expected nodes and actions are available
TEST_F(BringupTestFixture, BasicBringupTest)
{
  // Check for several expected action servers
  auto control_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      node_, "/panda_arm_controller/follow_joint_trajectory");
  EXPECT_TRUE(control_client->wait_for_action_server());
  auto move_group_client = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node_, "/move_action");
  EXPECT_TRUE(move_group_client->wait_for_action_server());

  // Send a trajectory request
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                           "panda_joint5", "panda_joint6", "panda_joint7" };
  trajectory_msgs::msg::JointTrajectoryPoint point_msg;
  point_msg.positions = { 0, -0.785, 0, -2.356, 0, 1.571, 0.785 };
  point_msg.time_from_start.sec = 1;
  traj_msg.points.push_back(point_msg);
  control_msgs::action::FollowJointTrajectory::Goal joint_traj_request;
  joint_traj_request.trajectory = std::move(traj_msg);

  auto result_cb =
      [](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result) {
        EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
      };
  auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.result_callback = result_cb;
  control_client->async_send_goal(joint_traj_request, send_goal_options);
}
}  // namespace moveit2_tutorials::quickstart_in_rviz

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
