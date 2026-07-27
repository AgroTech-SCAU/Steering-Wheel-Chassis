#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "mcu_comm_bridge/srv/report_mission_result.hpp"

namespace atlas_mission_manager {

enum class ReportStatus : std::uint8_t {
  Idle,
  InFlight,
  Accepted,
  Rejected,
  ServiceUnavailable,
  TimedOut,
};

struct ReportSnapshot {
  ReportStatus status{ReportStatus::Idle};
  bool success{false};
  std::uint8_t sent_count{0};
  std::string message;
};

class MissionResultReporter {
 public:
  MissionResultReporter(
      rclcpp::Node& node,
      const std::string& service_name,
      rclcpp::CallbackGroup::SharedPtr callback_group);

  bool start(std::uint8_t result, std::int16_t code, const rclcpp::Time& now);
  void poll_timeout(const rclcpp::Time& now, double timeout_s);
  ReportSnapshot snapshot() const;
  void reset();

 private:
  rclcpp::Node& node_;
  rclcpp::Client<mcu_comm_bridge::srv::ReportMissionResult>::SharedPtr client_;

  mutable std::mutex mutex_;
  ReportStatus status_{ReportStatus::Idle};
  bool success_{false};
  std::uint8_t sent_count_{0};
  std::string message_;
  rclcpp::Time started_at_{0, 0, RCL_ROS_TIME};
  std::uint64_t generation_{0};
};

}  // namespace atlas_mission_manager
