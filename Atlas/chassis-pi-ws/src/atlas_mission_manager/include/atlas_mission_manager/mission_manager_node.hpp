#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "atlas_mission_interfaces/msg/mission_status.hpp"
#include "atlas_mission_manager/mcu_state_cache.hpp"
#include "atlas_mission_manager/mission_context.hpp"
#include "atlas_mission_manager/mission_result_reporter.hpp"
#include "atlas_mission_manager/safety_controller.hpp"
#include "mcu_comm_bridge/msg/auto_task_event.hpp"
#include "mcu_comm_bridge/msg/mcu_status.hpp"

namespace atlas_mission_manager {

enum class FinalResultKind : std::uint8_t {
  None,
  Done,
  Fail,
};

class MissionManagerNode final : public rclcpp::Node {
 public:
  explicit MissionManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MissionManagerNode() override;

  void prepare_shutdown();

 private:
  void on_mcu_status(const mcu_comm_bridge::msg::McuStatus::SharedPtr message);
  void on_auto_task_event(const mcu_comm_bridge::msg::AutoTaskEvent::SharedPtr message);
  void update_state_machine();

  void transition_to(MissionState next_state, const std::string& reason);
  void request_abort(
      MissionState state_after_abort,
      const std::string& reason,
      std::int32_t error_code = 0);
  bool handle_global_conditions(
      const McuStateSnapshot& mcu,
      const rclcpp::Time& now);

  std::optional<std::pair<std::int32_t, std::string>> common_precheck(
      const McuStateSnapshot& mcu) const;

  void initialize_run(const rclcpp::Time& now);
  void clear_run_context();
  void begin_final_report(
      FinalResultKind kind,
      std::int16_t code,
      const std::string& message);
  void handle_reporting(const rclcpp::Time& now);
  void handle_confirmation_wait(
      const McuStateSnapshot& mcu,
      const rclcpp::Time& now);
  void handle_dry_run(const rclcpp::Time& now);
  void notify_task_flow_succeeded(const std::string& message);
  void notify_task_flow_failed(std::int32_t error_code, const std::string& message);
  void notify_task_flow_cancelled(const std::string& reason);

  void publish_mission_status(
      const McuStateSnapshot& mcu,
      const rclcpp::Time& now,
      bool force = false);

  bool take_pending_start(rclcpp::Time& event_time);
  bool has_pending_start(rclcpp::Time& event_time) const;
  bool take_pending_reset();
  void set_pending_reset();

  bool state_requires_auto_pi(MissionState state) const noexcept;
  bool state_is_active_lifecycle(MissionState state) const noexcept;
  bool is_reset_confirmed(const McuStateSnapshot& mcu) const noexcept;

  std::string mcu_status_topic_;
  std::string auto_task_event_topic_;
  std::string mission_result_service_;
  std::string brake_service_;
  std::string cmd_vel_topic_;
  std::string mission_status_topic_;

  double update_rate_hz_{20.0};
  double status_publish_rate_hz_{5.0};
  double zero_velocity_publish_rate_hz_{10.0};
  double mcu_status_timeout_s_{0.5};
  double start_confirm_timeout_s_{1.0};
  double result_service_timeout_s_{1.0};
  double result_confirm_timeout_s_{3.0};
  double result_retry_interval_s_{0.3};
  int result_report_retry_count_{2};
  bool require_arm_ready_in_common_precheck_{false};
  bool report_fail_on_common_precheck_error_{false};
  std::string dry_run_mode_{"hold"};
  double dry_run_success_delay_s_{2.0};
  std::int32_t dry_run_fail_code_{1003};

  McuStateCache mcu_state_cache_;
  MissionContext context_;

  rclcpp::CallbackGroup::SharedPtr status_callback_group_;
  rclcpp::CallbackGroup::SharedPtr event_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

  rclcpp::Subscription<mcu_comm_bridge::msg::McuStatus>::SharedPtr mcu_status_subscription_;
  rclcpp::Subscription<mcu_comm_bridge::msg::AutoTaskEvent>::SharedPtr auto_task_event_subscription_;
  rclcpp::Publisher<atlas_mission_interfaces::msg::MissionStatus>::SharedPtr mission_status_publisher_;
  rclcpp::TimerBase::SharedPtr state_machine_timer_;

  std::unique_ptr<SafetyController> safety_controller_;
  std::unique_ptr<MissionResultReporter> result_reporter_;

  mutable std::mutex event_mutex_;
  bool pending_start_{false};
  bool pending_reset_{false};
  rclcpp::Time pending_start_time_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> unexpected_latched_since_;

  bool previous_status_available_{false};
  bool previous_latched_{false};

  MissionState state_after_abort_{MissionState::WaitReset};
  FinalResultKind final_result_kind_{FinalResultKind::None};
  std::int16_t final_result_code_{0};
  int report_attempts_{0};
  rclcpp::Time last_report_attempt_time_{0, 0, RCL_ROS_TIME};

  std::string status_message_;
  rclcpp::Time last_status_publish_time_{0, 0, RCL_ROS_TIME};
  bool shutdown_requested_{false};
};

}  // namespace atlas_mission_manager
