#include "atlas_mission_manager/safety_controller.hpp"

#include <algorithm>
#include <exception>
#include <memory>

namespace atlas_mission_manager {

SafetyController::SafetyController(
    rclcpp::Node& node,
    const std::string& cmd_vel_topic,
    const std::string& brake_service,
    const double zero_publish_rate_hz,
    rclcpp::CallbackGroup::SharedPtr callback_group)
    : node_(node),
      zero_publish_period_s_(1.0 / std::max(1.0, zero_publish_rate_hz)) {
  cmd_vel_publisher_ = node_.create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  brake_client_ = node_.create_client<std_srvs::srv::SetBool>(
      brake_service,
      rmw_qos_profile_services_default,
      callback_group);
  last_zero_publish_ = node_.now();
  last_brake_attempt_ = node_.now();
}

void SafetyController::enter_safe_stop(const std::string& reason) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    safe_stop_active_ = true;
    reason_ = reason;
  }
  publish_zero_velocity();
  request_brake(true);
}

void SafetyController::allow_motion(const std::string& reason) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    safe_stop_active_ = false;
    reason_ = reason;
    brake_target_ = false;
    brake_confirmed_ = false;
  }
  request_brake(false);
}

void SafetyController::tick(const rclcpp::Time& now) {
  bool active = false;
  bool should_publish = false;
  bool should_retry_brake = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active = safe_stop_active_;
    should_publish = active &&
        (now - last_zero_publish_).seconds() >= zero_publish_period_s_;
    should_retry_brake = !brake_confirmed_ && !brake_request_in_flight_ &&
        (now - last_brake_attempt_).seconds() >= 1.0;
  }

  if (should_publish) {
    publish_zero_velocity();
  }
  if (should_retry_brake) {
    bool target = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target = brake_target_;
    }
    request_brake(target);
  }
}

void SafetyController::publish_zero_velocity() {
  geometry_msgs::msg::Twist zero;
  cmd_vel_publisher_->publish(zero);
  std::lock_guard<std::mutex> lock(mutex_);
  last_zero_publish_ = node_.now();
}

void SafetyController::request_brake(const bool enable) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    brake_target_ = enable;
    if (brake_request_in_flight_ ||
        (brake_confirmed_ && brake_confirmed_value_ == enable)) {
      return;
    }
    last_brake_attempt_ = node_.now();
    if (!brake_client_->service_is_ready()) {
      brake_confirmed_ = false;
      return;
    }
    brake_request_in_flight_ = true;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enable;
  brake_client_->async_send_request(
      request,
      [this, enable](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        bool success = false;
        try {
          const auto response = future.get();
          success = response && response->success;
        } catch (const std::exception& exception) {
          RCLCPP_WARN(
              node_.get_logger(),
              "Brake request failed with exception: %s",
              exception.what());
        }

        std::lock_guard<std::mutex> lock(mutex_);
        brake_request_in_flight_ = false;
        if (success && brake_target_ == enable) {
          brake_confirmed_ = true;
          brake_confirmed_value_ = enable;
        } else {
          brake_confirmed_ = false;
        }
      });
}


bool SafetyController::publish_motion_command(const geometry_msgs::msg::Twist& command) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (safe_stop_active_) {
      return false;
    }
  }
  cmd_vel_publisher_->publish(command);
  return true;
}

bool SafetyController::safe_stop_active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return safe_stop_active_;
}

}  // namespace atlas_mission_manager
