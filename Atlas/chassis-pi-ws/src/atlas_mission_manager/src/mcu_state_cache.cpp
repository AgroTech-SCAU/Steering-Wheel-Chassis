#include "atlas_mission_manager/mcu_state_cache.hpp"

namespace atlas_mission_manager {

void McuStateCache::update(
    const mcu_comm_bridge::msg::McuStatus& message,
    const rclcpp::Time& received_at) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_.available = true;
  state_.fresh = true;
  state_.app_state = message.app_state;
  state_.auto_start_latched = message.auto_start_latched;
  state_.ready_flags = message.ready_flags;
  state_.online_flags = message.online_flags;
  state_.fault_source = message.fault_source;
  state_.fault_level = message.fault_level;
  state_.fault_code = message.fault_code;
  state_.mcu_stamp_ms = message.mcu_stamp_ms;
  state_.received_at = received_at;
}

McuStateSnapshot McuStateCache::snapshot(
    const rclcpp::Time& now,
    const double timeout_s) const {
  std::lock_guard<std::mutex> lock(mutex_);
  McuStateSnapshot copy = state_;
  if (!copy.available) {
    copy.fresh = false;
    return copy;
  }

  const double age_s = (now - copy.received_at).seconds();
  copy.fresh = age_s >= 0.0 && age_s <= timeout_s;
  return copy;
}

bool is_auto_pi(const McuStateSnapshot& state) noexcept {
  return state.app_state == mcu_comm_bridge::msg::McuStatus::STATE_AUTO_PI;
}

bool is_finished(const McuStateSnapshot& state) noexcept {
  return state.app_state == mcu_comm_bridge::msg::McuStatus::STATE_FINISHED;
}

bool is_fault(const McuStateSnapshot& state) noexcept {
  return state.app_state == mcu_comm_bridge::msg::McuStatus::STATE_FAULT ||
         (state.online_flags & kOnlineHasFault) != 0u;
}

bool is_estop(const McuStateSnapshot& state) noexcept {
  return state.app_state == mcu_comm_bridge::msg::McuStatus::STATE_ESTOP ||
         (state.online_flags & kOnlineEstop) != 0u;
}

bool is_manual(const McuStateSnapshot& state) noexcept {
  return state.app_state == mcu_comm_bridge::msg::McuStatus::STATE_MANUAL;
}

bool is_pi_online(const McuStateSnapshot& state) noexcept {
  return (state.online_flags & kOnlinePi) != 0u;
}

bool is_chassis_ready(const McuStateSnapshot& state) noexcept {
  return (state.ready_flags & kReadyChassis) != 0u;
}

bool is_odom_ready(const McuStateSnapshot& state) noexcept {
  return (state.ready_flags & kReadyOdom) != 0u;
}

bool is_arm_ready(const McuStateSnapshot& state) noexcept {
  return (state.ready_flags & kReadyArm) != 0u;
}

}  // namespace atlas_mission_manager
