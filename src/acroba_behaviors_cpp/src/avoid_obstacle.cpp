/**
 * @file avoid_obstacle.cpp
 * @brief Reactive obstacle avoidance behavior using direct LiDAR readings.
 *
 * Reads raw LaserScan data and computes an immediate velocity command
 * to steer away from nearby obstacles. Designed for low-latency reactive
 * control where Python's GIL overhead is unacceptable.
 *
 * The behavior divides the LiDAR scan into three sectors (left, front, right)
 * and applies a proportional repulsive force from the closest obstacles.
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "acroba_msgs/action/execute_behavior.hpp"
#include "acroba_msgs/msg/behavior_status.hpp"

using ExecuteBehavior = acroba_msgs::action::ExecuteBehavior;
using GoalHandleEB = rclcpp_action::ServerGoalHandle<ExecuteBehavior>;

class AvoidObstacle : public rclcpp::Node
{
public:
  AvoidObstacle()
  : Node("avoid_obstacle"),
    safe_distance_(0.8),
    linear_speed_(0.3),
    angular_speed_(1.2),
    active_(false)
  {
    // Parameters
    this->declare_parameter("safe_distance", 0.8);
    this->declare_parameter("linear_speed", 0.3);
    this->declare_parameter("angular_speed", 1.2);
    safe_distance_ = this->get_parameter("safe_distance").as_double();
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    angular_speed_ = this->get_parameter("angular_speed").as_double();

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    status_pub_ = this->create_publisher<acroba_msgs::msg::BehaviorStatus>(
      "/acroba/behavior_status", 10);

    // Subscriber
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&AvoidObstacle::scan_callback, this, std::placeholders::_1));

    // Action server
    action_server_ = rclcpp_action::create_server<ExecuteBehavior>(
      this,
      "/acroba/avoid_obstacle",
      std::bind(&AvoidObstacle::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&AvoidObstacle::handle_cancel, this, std::placeholders::_1),
      std::bind(&AvoidObstacle::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "AvoidObstacle ready — safe_distance=%.2f m, speed=%.2f m/s",
      safe_distance_, linear_speed_);
  }

private:
  // ── Parameters ──
  double safe_distance_;
  double linear_speed_;
  double angular_speed_;
  bool active_;

  // ── ROS interfaces ──
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<acroba_msgs::msg::BehaviorStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_action::Server<ExecuteBehavior>::SharedPtr action_server_;

  // ── Latest scan ──
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  // ── Scan processing ──
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    latest_scan_ = msg;

    if (!active_) return;

    auto cmd = geometry_msgs::msg::Twist();
    const size_t n = msg->ranges.size();
    if (n == 0) return;

    // Divide scan into 3 sectors
    const size_t third = n / 3;
    double min_right = std::numeric_limits<double>::max();
    double min_front = std::numeric_limits<double>::max();
    double min_left = std::numeric_limits<double>::max();

    for (size_t i = 0; i < n; ++i) {
      double r = msg->ranges[i];
      if (std::isnan(r) || std::isinf(r)) continue;
      if (r < msg->range_min) r = msg->range_min;

      if (i < third) {
        min_right = std::min(min_right, r);
      } else if (i < 2 * third) {
        min_front = std::min(min_front, r);
      } else {
        min_left = std::min(min_left, r);
      }
    }

    // Compute avoidance command
    if (min_front > safe_distance_ && min_left > safe_distance_ * 0.6
        && min_right > safe_distance_ * 0.6) {
      // Clear ahead — drive forward
      cmd.linear.x = linear_speed_;
      cmd.angular.z = 0.0;
    } else if (min_front <= safe_distance_) {
      // Obstacle ahead — turn toward the clearer side
      cmd.linear.x = linear_speed_ * 0.2;
      if (min_left > min_right) {
        cmd.angular.z = angular_speed_;
      } else {
        cmd.angular.z = -angular_speed_;
      }
    } else if (min_left <= safe_distance_ * 0.6) {
      // Obstacle on left — turn right
      cmd.linear.x = linear_speed_ * 0.5;
      cmd.angular.z = -angular_speed_ * 0.5;
    } else {
      // Obstacle on right — turn left
      cmd.linear.x = linear_speed_ * 0.5;
      cmd.angular.z = angular_speed_ * 0.5;
    }

    cmd_vel_pub_->publish(cmd);
  }

  // ── Action server callbacks ──
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteBehavior::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Goal received: %s", goal->behavior_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleEB>)
  {
    RCLCPP_INFO(this->get_logger(), "Preemption requested");
    active_ = false;
    stop_robot();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleEB> goal_handle)
  {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandleEB> goal_handle)
  {
    active_ = true;
    auto feedback = std::make_shared<ExecuteBehavior::Feedback>();
    auto result = std::make_shared<ExecuteBehavior::Result>();
    rclcpp::Rate rate(20);

    RCLCPP_INFO(this->get_logger(), "AvoidObstacle active");

    while (rclcpp::ok() && active_) {
      if (goal_handle->is_canceling()) {
        active_ = false;
        stop_robot();
        result->success = false;
        result->message = "AvoidObstacle preempted";
        goal_handle->canceled(result);
        publish_status("PREEMPTED");
        return;
      }

      feedback->current_status = "RUNNING";
      feedback->detail = "Avoiding obstacles reactively";
      goal_handle->publish_feedback(feedback);
      publish_status("RUNNING");

      rate.sleep();
    }

    // Behavior runs indefinitely until preempted — this is by design
    // for a reactive layer that stays active as a background safety.
    stop_robot();
    result->success = true;
    result->message = "AvoidObstacle deactivated";
    goal_handle->succeed(result);
    publish_status("SUCCEEDED");
  }

  void stop_robot()
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }

  void publish_status(const std::string & status_str)
  {
    auto msg = acroba_msgs::msg::BehaviorStatus();
    msg.behavior_name = "avoid_obstacle";
    msg.detail = status_str;
    msg.stamp = this->get_clock()->now();
    if (status_str == "RUNNING") msg.status = 1;
    else if (status_str == "SUCCEEDED") msg.status = 2;
    else if (status_str == "PREEMPTED") msg.status = 4;
    else msg.status = 0;
    status_pub_->publish(msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidObstacle>());
  rclcpp::shutdown();
  return 0;
}
