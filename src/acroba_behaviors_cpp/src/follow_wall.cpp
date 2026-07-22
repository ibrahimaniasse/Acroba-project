/**
 * @file follow_wall.cpp
 * @brief Wall-following behavior using LiDAR range analysis.
 *
 * Maintains a configurable distance from the nearest wall by analyzing
 * LiDAR sectors. The robot follows the wall on its right side by default,
 * adjusting angular velocity to maintain the target distance.
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

class FollowWall : public rclcpp::Node
{
public:
  FollowWall()
  : Node("follow_wall"),
    wall_distance_(0.6),
    linear_speed_(0.25),
    kp_(3.0),
    active_(false)
  {
    this->declare_parameter("wall_distance", 0.6);
    this->declare_parameter("linear_speed", 0.25);
    this->declare_parameter("kp", 3.0);
    wall_distance_ = this->get_parameter("wall_distance").as_double();
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    kp_ = this->get_parameter("kp").as_double();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    status_pub_ = this->create_publisher<acroba_msgs::msg::BehaviorStatus>(
      "/acroba/behavior_status", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&FollowWall::scan_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<ExecuteBehavior>(
      this,
      "/acroba/follow_wall",
      std::bind(&FollowWall::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&FollowWall::handle_cancel, this, std::placeholders::_1),
      std::bind(&FollowWall::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "FollowWall ready — target distance=%.2f m", wall_distance_);
  }

private:
  double wall_distance_;
  double linear_speed_;
  double kp_;
  bool active_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<acroba_msgs::msg::BehaviorStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_action::Server<ExecuteBehavior>::SharedPtr action_server_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!active_ || msg->ranges.empty()) return;

    const size_t n = msg->ranges.size();
    auto cmd = geometry_msgs::msg::Twist();

    // Compute minimum distance in right sector (roughly 45° to 135° right)
    // For a 360° scan, right side is roughly indices [n*3/4 ... n-1] and [0 ... n/8]
    const size_t right_start = n * 5 / 8;
    const size_t right_end = n * 7 / 8;

    double min_right = std::numeric_limits<double>::max();
    for (size_t i = right_start; i < right_end && i < n; ++i) {
      double r = msg->ranges[i];
      if (!std::isnan(r) && !std::isinf(r) && r >= msg->range_min) {
        min_right = std::min(min_right, r);
      }
    }

    // Compute minimum front distance
    const size_t front_start = n * 3 / 8;
    const size_t front_end = n * 5 / 8;
    double min_front = std::numeric_limits<double>::max();
    for (size_t i = front_start; i < front_end && i < n; ++i) {
      double r = msg->ranges[i];
      if (!std::isnan(r) && !std::isinf(r) && r >= msg->range_min) {
        min_front = std::min(min_front, r);
      }
    }

    if (min_front < wall_distance_ * 0.8) {
      // Wall ahead — turn left sharply
      cmd.linear.x = 0.05;
      cmd.angular.z = 1.5;
    } else if (min_right > wall_distance_ * 2.0) {
      // Lost the wall — turn right to find it
      cmd.linear.x = linear_speed_ * 0.5;
      cmd.angular.z = -0.8;
    } else {
      // P-controller to maintain wall distance
      double error = min_right - wall_distance_;
      cmd.linear.x = linear_speed_;
      cmd.angular.z = -kp_ * error;
      // Clamp angular velocity
      cmd.angular.z = std::max(-1.5, std::min(1.5, cmd.angular.z));
    }

    cmd_vel_pub_->publish(cmd);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteBehavior::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleEB>)
  {
    active_ = false;
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
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

    RCLCPP_INFO(this->get_logger(), "FollowWall active");

    while (rclcpp::ok() && active_) {
      if (goal_handle->is_canceling()) {
        active_ = false;
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        result->success = false;
        result->message = "FollowWall preempted";
        goal_handle->canceled(result);
        return;
      }
      feedback->current_status = "RUNNING";
      feedback->detail = "Following wall on right side";
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }

    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    result->success = true;
    result->message = "FollowWall deactivated";
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowWall>());
  rclcpp::shutdown();
  return 0;
}
