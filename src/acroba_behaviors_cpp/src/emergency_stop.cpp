/**
 * @file emergency_stop.cpp
 * @brief Emergency stop behavior — highest priority, preempts all others.
 *
 * When activated, immediately publishes zero velocity and continues to
 * publish at high frequency to override any other node. This behavior
 * is designed as a safety layer: it should be triggered by critical
 * proximity alerts or operator intervention.
 *
 * Unlike other behaviors, EmergencyStop can also self-activate when
 * LiDAR detects an obstacle within the critical_distance threshold,
 * without waiting for an action goal (autonomous safety mode).
 */

#include <atomic>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "acroba_msgs/action/execute_behavior.hpp"
#include "acroba_msgs/msg/behavior_status.hpp"

using ExecuteBehavior = acroba_msgs::action::ExecuteBehavior;
using GoalHandleEB = rclcpp_action::ServerGoalHandle<ExecuteBehavior>;

class EmergencyStop : public rclcpp::Node
{
public:
  EmergencyStop()
  : Node("emergency_stop"),
    critical_distance_(0.25),
    auto_trigger_(true),
    active_(false)
  {
    this->declare_parameter("critical_distance", 0.25);
    this->declare_parameter("auto_trigger", true);
    this->declare_parameter("stop_publish_rate", 50.0);

    critical_distance_ = this->get_parameter("critical_distance").as_double();
    auto_trigger_ = this->get_parameter("auto_trigger").as_bool();
    double rate_hz = this->get_parameter("stop_publish_rate").as_double();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    status_pub_ = this->create_publisher<acroba_msgs::msg::BehaviorStatus>(
      "/acroba/behavior_status", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&EmergencyStop::scan_callback, this, std::placeholders::_1));

    // High-frequency zero-velocity publisher when active
    stop_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&EmergencyStop::stop_timer_callback, this));

    action_server_ = rclcpp_action::create_server<ExecuteBehavior>(
      this,
      "/acroba/emergency_stop",
      std::bind(&EmergencyStop::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&EmergencyStop::handle_cancel, this, std::placeholders::_1),
      std::bind(&EmergencyStop::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "EmergencyStop ready — critical_distance=%.2f m, auto_trigger=%s",
      critical_distance_, auto_trigger_ ? "true" : "false");
  }

private:
  double critical_distance_;
  bool auto_trigger_;
  std::atomic<bool> active_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<acroba_msgs::msg::BehaviorStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr stop_timer_;
  rclcpp_action::Server<ExecuteBehavior>::SharedPtr action_server_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!auto_trigger_ || active_.load()) return;

    // Check if any reading is below critical distance
    for (const auto & r : msg->ranges) {
      if (!std::isnan(r) && !std::isinf(r) && r < critical_distance_ && r >= msg->range_min) {
        RCLCPP_WARN(this->get_logger(),
          "EMERGENCY: Obstacle at %.2f m (< %.2f m) — stopping immediately!",
          r, critical_distance_);
        active_.store(true);
        publish_status("RUNNING");
        return;
      }
    }

    // If was auto-triggered and obstacle cleared, deactivate
    // (only if no action goal is holding it active)
  }

  void stop_timer_callback()
  {
    if (active_.load()) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteBehavior::Goal>)
  {
    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP activated via action goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleEB>)
  {
    RCLCPP_INFO(this->get_logger(), "Emergency stop released");
    active_.store(false);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleEB> goal_handle)
  {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandleEB> goal_handle)
  {
    active_.store(true);
    auto feedback = std::make_shared<ExecuteBehavior::Feedback>();
    auto result = std::make_shared<ExecuteBehavior::Result>();
    rclcpp::Rate rate(10);

    while (rclcpp::ok() && active_.load()) {
      if (goal_handle->is_canceling()) {
        active_.store(false);
        result->success = true;
        result->message = "Emergency stop released";
        goal_handle->canceled(result);
        publish_status("PREEMPTED");
        return;
      }
      feedback->current_status = "RUNNING";
      feedback->detail = "EMERGENCY STOP — all motion halted";
      goal_handle->publish_feedback(feedback);
      publish_status("RUNNING");
      rate.sleep();
    }

    result->success = true;
    result->message = "Emergency stop complete";
    goal_handle->succeed(result);
    publish_status("SUCCEEDED");
  }

  void publish_status(const std::string & status_str)
  {
    auto msg = acroba_msgs::msg::BehaviorStatus();
    msg.behavior_name = "emergency_stop";
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
  rclcpp::spin(std::make_shared<EmergencyStop>());
  rclcpp::shutdown();
  return 0;
}
