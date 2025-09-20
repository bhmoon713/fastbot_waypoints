#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "fastbot_waypoints/action/waypoint_action.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using Waypoint = fastbot_waypoints::action::WaypointAction;
using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class WaypointActionServer : public rclcpp::Node {
public:
  explicit WaypointActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("fastbot_as", options) {
    // Declare parameters (with defaults)
    cmd_vel_topic_        = this->declare_parameter<std::string>("cmd_vel_topic", "/fastbot/cmd_vel");
    odom_topic_           = this->declare_parameter<std::string>("odom_topic", "/fastbot/odom");
    yaw_precision_        = this->declare_parameter<double>("yaw_precision",  M_PI / 90.0); // ~2°
    dist_precision_       = this->declare_parameter<double>("dist_precision", 0.05);        // 5 cm
    initial_yaw_          = this->declare_parameter<double>("initial_yaw",    1.52);        // robot spawn offset
    k_lin_                = this->declare_parameter<double>("k_lin",          0.6);
    k_ang_                = this->declare_parameter<double>("k_ang",          1.2);
    max_linear_speed_     = this->declare_parameter<double>("max_linear_speed", 0.5);
    max_angular_speed_    = this->declare_parameter<double>("max_angular_speed", 0.6);
    timeout_sec_          = this->declare_parameter<double>("timeout_sec",    60.0);

    // Interfaces
    action_server_ = rclcpp_action::create_server<Waypoint>(
      this,
      "fastbot_as",
      std::bind(&WaypointActionServer::handle_goal,     this, _1, _2),
      std::bind(&WaypointActionServer::handle_cancel,   this, _1),
      std::bind(&WaypointActionServer::handle_accepted, this, _1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 50, std::bind(&WaypointActionServer::odom_callback, this, _1));

    RCLCPP_INFO(get_logger(),
      "WaypointActionServer up. cmd_vel: %s, odom: %s | yaw_precision=%.3f rad, dist_precision=%.3f m",
      cmd_vel_topic_.c_str(), odom_topic_.c_str(), yaw_precision_, dist_precision_);
  }

private:
  // ==== ROS interfaces ====
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // ==== Parameters / state ====
  std::string cmd_vel_topic_{"/fastbot/cmd_vel"};
  std::string odom_topic_{"/fastbot/odom"};

  geometry_msgs::msg::Point position_{}; // current odom position
  double yaw_{0.0};                      // current yaw (rad)
  double initial_yaw_{1.52};             // static offset on your platform
  std::string state_{"idle"};

  double yaw_precision_{M_PI / 90.0};
  double dist_precision_{0.05};

  // Motion tuning
  double k_lin_{0.6};
  double k_ang_{1.2};
  double max_linear_speed_{0.5};
  double max_angular_speed_{0.6};

  // Goal/loop state
  geometry_msgs::msg::Point des_pos_{};
  bool success_{false};
  double timeout_sec_{60.0};

  enum class Fsm { TURN, MOVE, FIX_YAW };
  Fsm fsm_{Fsm::TURN};

  // ==== Action handlers ====
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &,
                                          std::shared_ptr<const Waypoint::Goal> goal) {
    RCLCPP_INFO(get_logger(), "New goal: x=%.3f, y=%.3f", goal->position.x, goal->position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWaypoint> /*goal_handle*/) {
    RCLCPP_INFO(get_logger(), "Cancel requested");
    publish_stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    std::thread{std::bind(&WaypointActionServer::execute, this, _1), goal_handle}.detach();
  }

  // ==== Execution ====
  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing goal…");
    auto start_time = this->now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_sec_);

    des_pos_ = goal_handle->get_goal()->position;
    success_ = false;
    fsm_ = Fsm::TURN;

    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto result   = std::make_shared<Waypoint::Result>();
    rclcpp::Rate rate(25.0); // 25 Hz

    while (rclcpp::ok() && !success_) {
      if (goal_handle->is_canceling()) {
        publish_stop();
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return;
      }

      // Timeout?
      if (this->now() - start_time > timeout) {
        publish_stop();
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_WARN(get_logger(), "Goal aborted (timeout %.1f s)", timeout_sec_);
        return;
      }

      // Compute errors
      const double dx = des_pos_.x - position_.x;
      const double dy = des_pos_.y - position_.y;
      const double err_pos = std::hypot(dx, dy);
      const double desired_yaw = normalize_angle(std::atan2(dy, dx));
      const double err_yaw = normalize_angle(desired_yaw - (yaw_ - initial_yaw_));

      geometry_msgs::msg::Twist cmd; // auto-zeroed

      switch (fsm_) {
        case Fsm::TURN: {
          state_ = "fix yaw";
          const double ang = std::clamp(k_ang_ * err_yaw, -max_angular_speed_, max_angular_speed_);
          if (std::fabs(err_yaw) > yaw_precision_) {
            cmd.angular.z = ang;
            cmd.linear.x  = 0.0;
          } else {
            cmd.angular.z = 0.0;
            fsm_ = Fsm::MOVE;
            // continue to next iteration to compute fresh errors
          }
        } break;

        case Fsm::MOVE: {
          state_ = "go to point";
          if (err_pos < dist_precision_) {
            fsm_ = Fsm::FIX_YAW;
            break;
          }
          if (std::fabs(err_yaw) > yaw_precision_) {
            fsm_ = Fsm::TURN;
            break;
          }
          cmd.angular.z = 0.0;
          cmd.linear.x  = std::clamp(k_lin_ * err_pos, 0.0, max_linear_speed_);
        } break;

        case Fsm::FIX_YAW: {
          state_ = "fix final yaw";
          // Use err_yaw defined relative to desired_yaw; here we want to settle heading
          const double final_err_yaw = err_yaw;
          if (std::fabs(final_err_yaw) > yaw_precision_) {
            cmd.angular.z = std::clamp(k_ang_ * final_err_yaw, -max_angular_speed_, max_angular_speed_);
            cmd.linear.x  = 0.0;
          } else {
            cmd.angular.z = 0.0;
            cmd.linear.x  = 0.0;
            success_ = true;
          }
        } break;
      }

      cmd_vel_pub_->publish(cmd);

      // Feedback
      feedback->position = position_;
      feedback->state    = state_;
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    // Success: stop & report
    publish_stop();
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal reached successfully");
  }

  // ==== Callbacks ====
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position_ = msg->pose.pose.position;

    // Extract yaw
    const auto &o = msg->pose.pose.orientation;
    tf2::Quaternion q(o.x, o.y, o.z, o.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_);
  }

  // ==== Helpers ====
  static double normalize_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void publish_stop() {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointActionServer>());
  rclcpp::shutdown();
  return 0;
}
