#pragma once

#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace atlas_mission_manager {

class SafetyController {
 public:
  SafetyController(
      rclcpp::Node& node,
      const std::string& cmd_vel_topic,
      const std::string& brake_service,
      double zero_publish_rate_hz,
      rclcpp::CallbackGroup::SharedPtr callback_group);

  void enter_safe_stop(const std::string& reason);
  void allow_motion(const std::string& reason);
  void tick(const rclcpp::Time& now);
  void publish_zero_velocity();
  bool publish_motion_command(const geometry_msgs::msg::Twist& command);
  bool safe_stop_active() const;

 private:
  void request_brake(bool enable);

  rclcpp::Node& node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr brake_client_;

  mutable std::mutex mutex_;
  bool safe_stop_active_{false};
  bool brake_request_in_flight_{false};
  bool brake_target_{true};
  bool brake_confirmed_{false};
  bool brake_confirmed_value_{true};
  std::string reason_;
  rclcpp::Time last_zero_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_brake_attempt_{0, 0, RCL_ROS_TIME};
  double zero_publish_period_s_{0.1};
};

}  // namespace atlas_mission_manager
