#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"

#include "fastbot_waypoints/action/waypoint.hpp"

using namespace std::chrono_literals;
using Waypoint = fastbot_waypoints::action::Waypoint;

static inline double normalize_angle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

class OdomWatcher : public rclcpp::Node {
public:
  OdomWatcher() : Node("odom_watcher") {
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        yaw_ = normalize_angle(tf2::getYaw(msg->pose.pose.orientation));
        got_odom_ = true;
      });
    // params (can be tweaked in source to force PASS/FAIL)
    declare_parameter<double>("goal_x", 1.0);
    declare_parameter<double>("goal_y", 0.0);
    declare_parameter<double>("expected_x", 1.0);
    declare_parameter<double>("expected_y", 0.0);
    declare_parameter<double>("expected_yaw_deg", 0.0);
    declare_parameter<double>("pos_tol", 0.15);
    declare_parameter<double>("yaw_tol_deg", 10.0);
    declare_parameter<double>("action_timeout", 60.0);
    declare_parameter<double>("settle_time", 1.0);
  }

  bool wait_for_odom(double timeout_sec) {
    const auto t0 = now();
    while (rclcpp::ok() && (now() - t0).seconds() < timeout_sec) {
      if (got_odom_) return true;
      rclcpp::sleep_for(50ms);
    }
    return false;
  }

  double x() const { return x_; }
  double y() const { return y_; }
  double yaw() const { return yaw_; }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  bool got_odom_{false};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
};

class WaypointTests : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<OdomWatcher>();
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);
    spin_thread_ = std::thread([]{ exec_->spin(); });
  }

  static void TearDownTestSuite() {
    exec_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    exec_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void send_goal_and_wait() {
    auto client = rclcpp_action::create_client<Waypoint>(node_, "fastbot_as");
    ASSERT_TRUE(client->wait_for_action_server(15s)) << "Action server not available";

    Waypoint::Goal goal;
    goal.position.x = node_->get_parameter("goal_x").as_double();
    goal.position.y = node_->get_parameter("goal_y").as_double();
    goal.position.z = 0.0;

    auto future_goal_handle = client->async_send_goal(goal);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node_, future_goal_handle, 5s),
              rclcpp::FutureReturnCode::SUCCESS) << "Send goal failed";
    auto goal_handle = future_goal_handle.get();
    ASSERT_TRUE(goal_handle) << "Null goal handle";

    const double action_timeout = node_->get_parameter("action_timeout").as_double();
    auto result_future = client->async_get_result(goal_handle);
    ASSERT_EQ(rclcpp::spin_until_future_complete(
                node_, result_future, std::chrono::duration<double>(action_timeout)),
              rclcpp::FutureReturnCode::SUCCESS) << "Action timed out";

    // extra time for odom to settle
    const double settle = node_->get_parameter("settle_time").as_double();
    rclcpp::sleep_for(std::chrono::duration<double>(settle));
  }

  static std::shared_ptr<OdomWatcher> node_;
  static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  static std::thread spin_thread_;
};

std::shared_ptr<OdomWatcher> WaypointTests::node_;
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> WaypointTests::exec_;
std::thread WaypointTests::spin_thread_;

TEST_F(WaypointTests, FinalPositionWithinTolerance) {
  ASSERT_TRUE(node_->wait_for_odom(10.0)) << "No /odom received";

  send_goal_and_wait();

  const double exp_x = node_->get_parameter("expected_x").as_double();
  const double exp_y = node_->get_parameter("expected_y").as_double();
  const double pos_tol = node_->get_parameter("pos_tol").as_double();

  const double dx = node_->x() - exp_x;
  const double dy = node_->y() - exp_y;
  const double dist = std::hypot(dx, dy);

  EXPECT_LE(dist, pos_tol) << "Position error " << dist << " > tol " << pos_tol;
}

TEST_F(WaypointTests, FinalYawWithinTolerance) {
  ASSERT_TRUE(node_->wait_for_odom(10.0)) << "No /odom received";

  send_goal_and_wait();

  const double exp_yaw_deg = node_->get_parameter("expected_yaw_deg").as_double();
  const double yaw_tol_deg = node_->get_parameter("yaw_tol_deg").as_double();

  const double err_deg = std::fabs((node_->yaw() * 180.0 / M_PI) - exp_yaw_deg);
  EXPECT_LE(err_deg, yaw_tol_deg) << "Yaw error " << err_deg << " > tol " << yaw_tol_deg;
}
