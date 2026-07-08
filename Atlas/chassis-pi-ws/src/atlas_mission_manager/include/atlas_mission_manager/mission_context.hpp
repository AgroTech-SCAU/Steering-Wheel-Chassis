#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "atlas_mission_manager/mission_state.hpp"

namespace atlas_mission_manager {

struct MissionContext {
  std::uint32_t local_run_id{0};
  MissionState state{MissionState::Bootstrap};

  rclcpp::Time state_enter_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time run_start_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_mcu_status_time{0, 0, RCL_ROS_TIME};

  bool active{false};
  bool start_event_seen{false};
  bool reset_event_seen{false};
  bool result_reported{false};
  bool result_report_in_flight{false};
  bool cancellation_requested{false};

  std::int32_t last_error_code{0};
  std::string last_error_message;
};

}  // namespace atlas_mission_manager
