#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // Galactic uses .h
#include <tf2/utils.h>

#include "fastbot_waypoints/action/waypoint.hpp"

using namespace std::chrono_literals;
using Waypoint = fastbot_waypoints::action::Waypoint;
using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

class FastbotWaypointActionServer : public rclcpp::Node
{
public:
  FastbotWaypointActionServer()
  : Node("fastbot_action_server")
  {
    // Parameters (you can override via YAML/CLI)
    yaw_precision_deg_ = this->declare_parameter<double>("yaw_precision_deg", 2.0);  // +/- 2 deg
    dist_precision_    = this->declare_parameter<double>("dist_precision",     0.05);
    linear_speed_      = this->declare_parameter<double>("linear_speed",       0.6);
    angular_speed_     = this->declare_parameter<double>("angular_speed",      0.65);
    control_rate_hz_   = this->declare_parameter<double>("control_rate_hz",    25.0);

    // Publishers/Subscribers (fastbot names)
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/fastbot/cmd_vel", 10);
    sub_odom_    = this->create_subscription<nav_msgs::msg::Odometry>(
      "/fastbot/odom", 10,
      std::bind(&FastbotWaypointActionServer::odomCallback, this, std::placeholders::_1));

    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<Waypoint>(
      this,
      "/fastbot_waypoint",
      std::bind(&FastbotWaypointActionServer::handle_goal, this, _1, _2),
      std::bind(&FastbotWaypointActionServer::handle_cancel, this, _1),
      std::bind(&FastbotWaypointActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Fastbot Waypoint action server is up at /fastbot_waypoint");
  }

private:
  // --- ROS I/O ---
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;

  // --- Robot state (protected by mutex) ---
  std::mutex state_mtx_;
  geometry_msgs::msg::Point position_;
  double yaw_{0.0};
  bool have_odom_{false};

  // --- Params ---
  double yaw_precision_deg_;
  double dist_precision_;
  double linear_speed_;
  double angular_speed_;
  double control_rate_hz_;

  // --- Helpers ---
  static double normalize_angle(double a)
  {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::scoped_lock lk(state_mtx_);
    position_ = msg->pose.pose.position;
    yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  // --- Action server handlers ---
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const Waypoint::Goal> goal)
  {
    (void)goal;
    RCLCPP_INFO(get_logger(), "Received new goal: (%.3f, %.3f, %.3f)",
      goal->position.x, goal->position.y, goal->position.z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleWaypoint> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Goal cancel requested.");
    stopRobot();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle)
  {
    // Run the execute loop in a new thread so the executor isn't blocked
    std::thread{std::bind(&FastbotWaypointActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle)
  {
    rclcpp::Rate rate(control_rate_hz_);

    const auto goal = goal_handle->get_goal();
    const auto start_time = now();

    // Wait for odom if needed
    {
      rclcpp::Time wait_start = now();
      while (rclcpp::ok() && !have_odom_) {
        if (goal_handle->is_canceling()) {
          finishCanceled(goal_handle);
          return;
        }
        if ((now() - wait_start).seconds() > 5.0) {
          RCLCPP_WARN(get_logger(), "No odom after 5s; continuing anyway.");
          break;
        }
        rate.sleep();
      }
    }

    Waypoint::Feedback feedback;
    Waypoint::Result result;

    const double yaw_precision = yaw_precision_deg_ * M_PI / 180.0;

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        finishCanceled(goal_handle);
        return;
      }

      // Snapshot state
      geometry_msgs::msg::Point pos;
      double yaw;
      {
        std::scoped_lock lk(state_mtx_);
        pos = position_;
        yaw = yaw_;
      }

      // Errors
      const double dx = goal->position.x - pos.x;
      const double dy = goal->position.y - pos.y;
      const double desired_yaw = std::atan2(dy, dx);
      const double err_yaw = normalize_angle(desired_yaw - yaw);
      const double err_pos = std::hypot(dx, dy);

      RCLCPP_DEBUG(get_logger(), "yaw=%.3f desired=%.3f err=%.3f pos_err=%.3f",
                   yaw, desired_yaw, err_yaw, err_pos);

      geometry_msgs::msg::Twist cmd;

      std::string state_str;
      if (std::fabs(err_yaw) > yaw_precision) {
        state_str = "fix yaw";
        cmd.angular.z = (err_yaw > 0.0) ? angular_speed_ : -angular_speed_;
        cmd.linear.x = 0.0;
      } else if (err_pos > dist_precision_) {
        state_str = "go to point";
        cmd.linear.x = linear_speed_;
        cmd.angular.z = 0.0;
      } else {
        // Reached
        stopRobot();
        result.success = true;
        goal_handle->succeed(std::make_shared<Waypoint::Result>(result));
        RCLCPP_INFO(get_logger(), "Goal reached in %.2fs",
                    (now() - start_time).seconds());
        return;
      }

      pub_cmd_vel_->publish(cmd);

      // Feedback
      feedback.position = pos;
      feedback.state = state_str;
      goal_handle->publish_feedback(std::make_shared<Waypoint::Feedback>(feedback));

      rate.sleep();
    }

    // Node shutting down — cancel gracefully
    if (rclcpp::ok() == false) {
      stopRobot();
      goal_handle->canceled(std::make_shared<Waypoint::Result>());
    }
  }

  void stopRobot()
  {
    geometry_msgs::msg::Twist stop;
    pub_cmd_vel_->publish(stop);
  }

  void finishCanceled(const std::shared_ptr<GoalHandleWaypoint> &goal_handle)
  {
    stopRobot();
    auto res = std::make_shared<Waypoint::Result>();
    res->success = false;
    goal_handle->canceled(res);
    RCLCPP_INFO(get_logger(), "Goal canceled.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastbotWaypointActionServer>());
  rclcpp::shutdown();
  return 0;
}
