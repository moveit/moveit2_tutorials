// Description: Test if launching the demo is successful

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <gtest/gtest.h>
#include <stdlib.h>

namespace
{
// This is a bit of a hack to make thread sanitizer ignore a race condition
// in the constructor of the rclcpp::Node
#if defined(__has_feature)
#if __has_feature(thread_sanitizer)
__attribute__((no_sanitize("thread")))
#endif
#endif
rclcpp::Node::SharedPtr
make_node(std::string const& name, rclcpp::NodeOptions const& options)
{
  return std::make_shared<rclcpp::Node>(name, options);
}
}  // namespace

namespace moveit2_tutorials::quickstart_in_rviz
{
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

TEST_F(BringupTestFixture, BasicBringupTest)
{
  // Check for several expected action servers
  auto control_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      node_, "/panda_arm_controller/follow_joint_trajectory");
  EXPECT_TRUE(control_client->wait_for_action_server());
  auto move_group_client = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node_, "/move_action");
  EXPECT_TRUE(move_group_client->wait_for_action_server());

  // Check for the expected nodes
  const auto actual_node_names = node_->get_node_names();
  const std::vector<std::string> expected_node_names{ "/controller_manager",
                                                      "/joint_state_broadcaster",
                                                      "/move_group",
                                                      "/moveit_simple_controller_manager",
                                                      "/panda_arm_controller",
                                                      "/panda_hand_controller",
                                                      "/robot_state_publisher",
                                                      "/rviz2" };
  for (const std::string& node_name : expected_node_names)
  {
    EXPECT_NE(std::find(actual_node_names.begin(), actual_node_names.end(), node_name), actual_node_names.end());
  }
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
