#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

#include "fastbot_waypoints/action/waypoint.hpp"

using namespace std::chrono_literals;
using Waypoint = fastbot_waypoints::action::Waypoint;
using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

static inline double normalize_angle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

class WaypointActionServer : public rclcpp::Node {
public:
  WaypointActionServer()
  : Node("fastbot_action_server"),
    yaw_precision_(M_PI / 90.0),   // ~2 deg
    dist_precision_(0.05)          // 5 cm
  {
    declare_parameter<double>("time_limit", 120.0);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&WaypointActionServer::odom_cb, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<Waypoint>(
      this,
      "fastbot_as",
      std::bind(&WaypointActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WaypointActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&WaypointActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "fastbot_as action server started");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;

  geometry_msgs::msg::Point position_;
  double yaw_{0.0};

  const double yaw_precision_;
  const double dist_precision_;

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position_ = msg->pose.pose.position;
    yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    yaw_ = normalize_angle(yaw_);
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const Waypoint::Goal> goal)
  {
    (void)goal;
    RCLCPP_INFO(get_logger(), "Received goal: [%.3f, %.3f]", goal->position.x, goal->position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleWaypoint> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Cancel requested");
    stop_robot();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    // Execute on a separate thread
    std::thread{std::bind(&WaypointActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    rclcpp::Rate rate(25);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto result   = std::make_shared<Waypoint::Result>();

    const double time_limit = get_parameter("time_limit").as_double();
    const auto t0 = now();

    while (rclcpp::ok()) {
      // Timeout
      if ((now() - t0).seconds() > time_limit) {
        RCLCPP_WARN(get_logger(), "Timeout reached; preempting.");
        goal_handle->canceled(result);
        stop_robot();
        return;
      }
      // Preempt?
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        stop_robot();
        return;
      }

      // Errors
      const double desired_yaw = std::atan2(goal->position.y - position_.y,
                                            goal->position.x - position_.x);
      const double err_yaw = normalize_angle(desired_yaw - yaw_);
      const double err_pos = std::hypot(goal->position.x - position_.x,
                                        goal->position.y - position_.y);

      geometry_msgs::msg::Twist twist;

      if (err_pos > dist_precision_) {
        if (std::fabs(err_yaw) > yaw_precision_) {
          twist.angular.z = (err_yaw > 0.0 ? 0.65 : -0.65);
        } else {
          twist.linear.x = 0.6;
        }
        cmd_pub_->publish(twist);
      } else {
        break; // Reached
      }

      feedback->position = position_;
      feedback->state = (std::fabs(err_yaw) > yaw_precision_) ? "fix yaw" : "go to point";
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    stop_robot();
    result->success = true;
    goal_handle->succeed(result);
  }

  void stop_robot() {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
