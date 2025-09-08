#include <cmath>
#include <memory>
#include <thread>

#include "fastbot_waypoints/action/waypoint_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "gtest/gtest.h"

using WaypointAction = fastbot_waypoints::action::WaypointAction;
using GoalHandle = rclcpp_action::ClientGoalHandle<WaypointAction>;

using namespace std::chrono_literals;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class WaypointActionTest : public ::testing::Test {
protected:
  void SetUp() override {
    node_ = rclcpp::Node::make_shared("waypoint_action_test_node");
    client_ = rclcpp_action::create_client<WaypointAction>(node_, "fastbot_as");
    sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        std::bind(&WaypointActionTest::odom_callback, this,
                  std::placeholders::_1));

    ASSERT_TRUE(client_->wait_for_action_server(5s))
        << "Action server not available";

    goal1_.x = 0.8;
    goal1_.y = 1.;
    goal2_.x = 1.3;
    goal2_.y = 1.5;

    // Wait a bit for odom data to arrive
    rclcpp::spin_some(node_);
    rclcpp::sleep_for(1s);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_position_ = msg->pose.pose.position;

    double ox = msg->pose.pose.orientation.x;
    double oy = msg->pose.pose.orientation.y;
    double oz = msg->pose.pose.orientation.z;
    double ow = msg->pose.pose.orientation.w;

    tf2::Quaternion q(ox, oy, oz, ow);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    robot_yaw_ = yaw;
  }

  bool send_goal_and_wait(const geometry_msgs::msg::Point &target_point) {
    WaypointAction::Goal goal_msg;
    goal_msg.position = target_point;

    auto goal_future = client_->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node_, goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      return false;
    }

    auto result_future = client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return false;
    }

    auto result = result_future.get();
    return result.result->success;
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<WaypointAction>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

  geometry_msgs::msg::Point goal1_{}, goal2_{}, last_position_{};
  double robot_yaw_{0.0};
};

// ------------ TEST CASES ----------------------

TEST_F(WaypointActionTest, ReachGoal1) {

  ASSERT_TRUE(send_goal_and_wait(goal1_)) << "Failed to reach goal 1";

  double dist =
      std::hypot(last_position_.x - goal1_.x, last_position_.y - goal1_.y);
  EXPECT_LT(dist, 0.1) << "Robot too far from goal 1";

  // Normalized yaw in gazebo frame
  double yaw_offset = 1.47;
  double actual_yaw = robot_yaw_ - yaw_offset;
  while (actual_yaw > M_PI)
    actual_yaw -= 2 * M_PI;
  while (actual_yaw < -M_PI)
    actual_yaw += 2 * M_PI;

  // Expected yaw based on final position to goal 1
  double expected_yaw =
      std::atan2(goal1_.y - last_position_.y, goal1_.x - last_position_.x);
  while (expected_yaw > M_PI)
    expected_yaw -= 2 * M_PI;
  while (expected_yaw < -M_PI)
    expected_yaw += 2 * M_PI;

  EXPECT_NEAR(actual_yaw, expected_yaw, 0.3) << "Yaw mismatch for goal 1";
}

TEST_F(WaypointActionTest, ReachGoal2) {

  ASSERT_TRUE(send_goal_and_wait(goal1_))
      << "Failed to reach goal 1 before goal 2";
  rclcpp::sleep_for(1s);
  ASSERT_TRUE(send_goal_and_wait(goal2_)) << "Failed to reach goal 2";

  double dist =
      std::hypot(last_position_.x - goal2_.x, last_position_.y - goal2_.y);
  EXPECT_LT(dist, 0.1) << "Robot too far from goal 2";

  // Normalize actual yaw from Gazebo frame
  double yaw_offset = 1.47;
  double actual_yaw = robot_yaw_ - yaw_offset;
  while (actual_yaw > M_PI)
    actual_yaw -= 2 * M_PI;
  while (actual_yaw < -M_PI)
    actual_yaw += 2 * M_PI;

  // Expected yaw based on final position to goal2
  double expected_yaw =
      std::atan2(goal2_.y - last_position_.y, goal2_.x - last_position_.x);
  while (expected_yaw > M_PI)
    expected_yaw -= 2 * M_PI;
  while (expected_yaw < -M_PI)
    expected_yaw += 2 * M_PI;

  EXPECT_NEAR(actual_yaw, expected_yaw, 0.3) << "Yaw mismatch for goal 2";
}