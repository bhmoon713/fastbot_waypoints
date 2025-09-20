#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>

#include "fastbot_waypoints/action/waypoint_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "gtest/gtest.h"

using Waypoint = fastbot_waypoints::action::WaypointAction;
using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class FastbotActionServerTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Waypoint>::SharedPtr action_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::Point current_position_;
  bool odom_received_ = false;
  double current_yaw = 0.0;
  double init_yaw = 0.0;
  bool init_yaw_received = false;
  double init_yaw_offset = 1.52;

  void SetUp() override {
    node_ = rclcpp::Node::make_shared("test_fastbot_action_client");
    action_client_ =
        rclcpp_action::create_client<Waypoint>(node_, "fastbot_as");

    // Odometry subscriber
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          current_position_ = msg->pose.pose.position;
          tf2::Quaternion q(
              msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
          tf2::Matrix3x3 m(q);
          double roll, pitch;
          m.getRPY(roll, pitch, current_yaw);
          odom_received_ = true;

          if (!init_yaw_received) {
            init_yaw = current_yaw;
            init_yaw_received = true;
          }
        });

    // Wait for the action server to be available
    ASSERT_TRUE(action_client_->wait_for_action_server(10s));
  }

  void send_goal(Waypoint::Goal &goal) {
    // Send the goal to the action server
    auto send_goal_future = action_client_->async_send_goal(goal);
    rclcpp::spin_until_future_complete(node_, send_goal_future);
    auto goal_handle = send_goal_future.get();
    ASSERT_TRUE(goal_handle);

    // Wait for result
    auto result_future = action_client_->async_get_result(goal_handle);
    rclcpp::spin_until_future_complete(node_, result_future);
    auto result = result_future.get();
    ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
    ASSERT_TRUE(result.result->success);
  }

  // Returns true if the current position is within goal threshold
  bool position_close(double goal_x, double goal_y, double tolerance = 0.1) {

    bool is_close = false;
    double dx = goal_x - current_position_.x;
    double dy = goal_y - current_position_.y;

    is_close = (std::abs(dx) <= tolerance && std::abs(dy) <= tolerance);

    return is_close;
  }

  bool angle_close(double goal_yaw, double tolerance = 6.28) {

    bool is_close = false;
    double final_yaw = current_yaw;
    double yaw_diff = final_yaw - goal_yaw;

    is_close = (std::abs(yaw_diff) <= tolerance);

    return is_close;
  }
};

TEST_F(FastbotActionServerTest, RobotReachedGoal) {
  // Define the goal for pass
  Waypoint::Goal goal;
  goal.position.x = 2.0;
  goal.position.y = 2.0;
  double goal_yaw = 1.57;

  // Define the goal for fail
//   Waypoint::Goal goal;
//     goal.position.x = -1.0;
//     goal.position.y = 0.0;
//     double goal_yaw = 1.57;

  double global_goal_yaw = goal_yaw + init_yaw_offset;

  // Send the goal
  send_goal(goal);

  // Wait for odometry to settle
  rclcpp::Rate rate(10);
  for (int i = 0; i < 50 && !position_close(goal.position.x, goal.position.y);
       ++i) {
    rclcpp::spin_some(node_);
    rate.sleep();
  }

  EXPECT_TRUE(position_close(goal.position.x, goal.position.y))
      << "Robot did not reach the goal: "
      << "Expected (" << goal.position.x << ", " << goal.position.y << ") "
      << "but got (" << current_position_.x << ", " << current_position_.y
      << ")";

  EXPECT_TRUE(angle_close(global_goal_yaw))
      << "Robot did not reach the global goal yaw: " << global_goal_yaw
      << " Final yaw is: " << current_yaw << " ";
}