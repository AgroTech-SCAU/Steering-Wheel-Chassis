#pragma once

#include <cstdint>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "mcu_comm_bridge/msg/mcu_status.hpp"

namespace atlas_mission_manager {

struct McuStateSnapshot {
  bool available{false};
  bool fresh{false};
  std::uint8_t app_state{0};
  bool auto_start_latched{false};
  std::uint8_t ready_flags{0};
  std::uint8_t online_flags{0};
  std::uint8_t fault_source{0};
  std::uint8_t fault_level{0};
  std::int16_t fault_code{0};
  std::uint32_t mcu_stamp_ms{0};
  rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
};

class McuStateCache {
 public:
  void update(
      const mcu_comm_bridge::msg::McuStatus& message,
      const rclcpp::Time& received_at);

  McuStateSnapshot snapshot(
      const rclcpp::Time& now,
      double timeout_s) const;

 private:
  mutable std::mutex mutex_;
  McuStateSnapshot state_;
};

constexpr std::uint8_t kReadyChassis = 1u << 0;
constexpr std::uint8_t kReadyArm = 1u << 1;
constexpr std::uint8_t kReadyOdom = 1u << 2;
constexpr std::uint8_t kOnlinePi = 1u << 2;
constexpr std::uint8_t kOnlineHasFault = 1u << 3;
constexpr std::uint8_t kOnlineEstop = 1u << 4;

bool is_auto_pi(const McuStateSnapshot& state) noexcept;
bool is_finished(const McuStateSnapshot& state) noexcept;
bool is_fault(const McuStateSnapshot& state) noexcept;
bool is_estop(const McuStateSnapshot& state) noexcept;
bool is_manual(const McuStateSnapshot& state) noexcept;
bool is_pi_online(const McuStateSnapshot& state) noexcept;
bool is_chassis_ready(const McuStateSnapshot& state) noexcept;
bool is_odom_ready(const McuStateSnapshot& state) noexcept;
bool is_arm_ready(const McuStateSnapshot& state) noexcept;

}  // namespace atlas_mission_manager
