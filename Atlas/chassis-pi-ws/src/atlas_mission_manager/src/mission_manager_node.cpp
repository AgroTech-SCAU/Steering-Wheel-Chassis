#include "atlas_mission_manager/mission_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <utility>

#include "atlas_mission_manager/mission_error.hpp"

namespace atlas_mission_manager {
namespace {

constexpr double kMinimumRateHz = 1.0;

std::chrono::nanoseconds period_from_hz(const double rate_hz) {
  const double safe_rate = std::max(kMinimumRateHz, rate_hz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / safe_rate));
}

bool is_valid_dry_run_mode(const std::string& mode) {
  return mode == "hold" || mode == "success_after_delay" ||
         mode == "fail_after_delay";
}

}  // namespace

MissionManagerNode::MissionManagerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("atlas_mission_manager", options) {
  mcu_status_topic_ = declare_parameter<std::string>("mcu_status_topic", "/mcu/status");
  auto_task_event_topic_ = declare_parameter<std::string>(
      "auto_task_event_topic", "/mcu/auto_task_event");
  mission_result_service_ = declare_parameter<std::string>(
      "mission_result_service", "/mcu/report_mission_result");
  brake_service_ = declare_parameter<std::string>("brake_service", "/mcu/set_brake");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/motor_cmd_vel");
  mission_status_topic_ = declare_parameter<std::string>(
      "mission_status_topic", "/atlas/mission/status");

  update_rate_hz_ = declare_parameter<double>("update_rate_hz", 20.0);
  status_publish_rate_hz_ = declare_parameter<double>("status_publish_rate_hz", 5.0);
  zero_velocity_publish_rate_hz_ = declare_parameter<double>(
      "zero_velocity_publish_rate_hz", 10.0);
  mcu_status_timeout_s_ = declare_parameter<double>("mcu_status_timeout_s", 0.5);
  start_confirm_timeout_s_ = declare_parameter<double>("start_confirm_timeout_s", 1.0);
  result_service_timeout_s_ = declare_parameter<double>("result_service_timeout_s", 1.0);
  result_confirm_timeout_s_ = declare_parameter<double>("result_confirm_timeout_s", 3.0);
  result_retry_interval_s_ = declare_parameter<double>("result_retry_interval_s", 0.3);
  result_report_retry_count_ = declare_parameter<int>("result_report_retry_count", 2);
  require_arm_ready_in_common_precheck_ = declare_parameter<bool>(
      "require_arm_ready_in_common_precheck", false);
  report_fail_on_common_precheck_error_ = declare_parameter<bool>(
      "report_fail_on_common_precheck_error", false);
  dry_run_mode_ = declare_parameter<std::string>("dry_run_mode", "hold");
  dry_run_success_delay_s_ = declare_parameter<double>("dry_run_success_delay_s", 2.0);
  dry_run_fail_code_ = declare_parameter<int>("dry_run_fail_code", error::kDryRunFailed);

  update_rate_hz_ = std::max(kMinimumRateHz, update_rate_hz_);
  status_publish_rate_hz_ = std::max(kMinimumRateHz, status_publish_rate_hz_);
  zero_velocity_publish_rate_hz_ = std::max(kMinimumRateHz, zero_velocity_publish_rate_hz_);
  mcu_status_timeout_s_ = std::max(0.05, mcu_status_timeout_s_);
  start_confirm_timeout_s_ = std::max(0.05, start_confirm_timeout_s_);
  result_service_timeout_s_ = std::max(0.05, result_service_timeout_s_);
  result_confirm_timeout_s_ = std::max(0.1, result_confirm_timeout_s_);
  result_retry_interval_s_ = std::max(0.0, result_retry_interval_s_);
  result_report_retry_count_ = std::clamp(result_report_retry_count_, 0, 10);
  dry_run_success_delay_s_ = std::max(0.0, dry_run_success_delay_s_);
  dry_run_fail_code_ = std::clamp(
      dry_run_fail_code_,
      static_cast<std::int32_t>(std::numeric_limits<std::int16_t>::min()),
      static_cast<std::int32_t>(std::numeric_limits<std::int16_t>::max()));

  if (!is_valid_dry_run_mode(dry_run_mode_)) {
    RCLCPP_WARN(
        get_logger(),
        "Unsupported dry_run_mode '%s'; falling back to 'hold'",
        dry_run_mode_.c_str());
    dry_run_mode_ = "hold";
  }

  status_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  event_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions status_options;
  status_options.callback_group = status_callback_group_;
  auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  status_qos.reliable();
  status_qos.transient_local();
  mcu_status_subscription_ = create_subscription<mcu_comm_bridge::msg::McuStatus>(
      mcu_status_topic_,
      status_qos,
      std::bind(&MissionManagerNode::on_mcu_status, this, std::placeholders::_1),
      status_options);

  rclcpp::SubscriptionOptions event_options;
  event_options.callback_group = event_callback_group_;
  auto event_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  event_qos.reliable();
  event_qos.durability_volatile();
  auto_task_event_subscription_ = create_subscription<mcu_comm_bridge::msg::AutoTaskEvent>(
      auto_task_event_topic_,
      event_qos,
      std::bind(&MissionManagerNode::on_auto_task_event, this, std::placeholders::_1),
      event_options);

  auto mission_status_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  mission_status_qos.reliable();
  mission_status_qos.transient_local();
  mission_status_publisher_ = create_publisher<atlas_mission_interfaces::msg::MissionStatus>(
      mission_status_topic_,
      mission_status_qos);

  safety_controller_ = std::make_unique<SafetyController>(
      *this,
      cmd_vel_topic_,
      brake_service_,
      zero_velocity_publish_rate_hz_,
      service_callback_group_);
  result_reporter_ = std::make_unique<MissionResultReporter>(
      *this,
      mission_result_service_,
      service_callback_group_);

  const auto current_time = now();
  context_.state_enter_time = current_time;
  context_.run_start_time = current_time;
  context_.last_mcu_status_time = current_time;
  last_status_publish_time_ = current_time;
  last_report_attempt_time_ = current_time;

  state_machine_timer_ = create_wall_timer(
      period_from_hz(update_rate_hz_),
      std::bind(&MissionManagerNode::update_state_machine, this));

  RCLCPP_INFO(
      get_logger(),
      "Atlas mission manager started in common-only mode; dry_run_mode=%s",
      dry_run_mode_.c_str());
}

MissionManagerNode::~MissionManagerNode() {
  prepare_shutdown();
}

void MissionManagerNode::prepare_shutdown() {
  if (shutdown_requested_) {
    return;
  }
  shutdown_requested_ = true;
  if (safety_controller_) {
    safety_controller_->enter_safe_stop("mission manager shutdown");
  }
}

void MissionManagerNode::on_mcu_status(
    const mcu_comm_bridge::msg::McuStatus::SharedPtr message) {
  const auto received_at = now();
  mcu_state_cache_.update(*message, received_at);
  std::lock_guard<std::mutex> lock(event_mutex_);
  if (previous_status_available_ && previous_latched_ && !message->auto_start_latched) {
    pending_reset_ = true;
  }
  previous_status_available_ = true;
  previous_latched_ = message->auto_start_latched;
}

void MissionManagerNode::on_auto_task_event(
    const mcu_comm_bridge::msg::AutoTaskEvent::SharedPtr message) {
  std::lock_guard<std::mutex> lock(event_mutex_);
  if (message->event == mcu_comm_bridge::msg::AutoTaskEvent::EVENT_START) {
    if (!pending_start_) {
      pending_start_ = true;
      pending_start_time_ = now();
    }
    return;
  }
  if (message->event == mcu_comm_bridge::msg::AutoTaskEvent::EVENT_RESET) {
    pending_reset_ = true;
    return;
  }

  RCLCPP_WARN(
      get_logger(),
      "Ignoring unsupported AutoTaskEvent value=%u",
      static_cast<unsigned int>(message->event));
}

void MissionManagerNode::update_state_machine() {
  const auto current_time = now();
  const auto mcu = mcu_state_cache_.snapshot(current_time, mcu_status_timeout_s_);
  safety_controller_->tick(current_time);

  if (handle_global_conditions(mcu, current_time)) {
    publish_mission_status(mcu, current_time);
    return;
  }

  switch (context_.state) {
    case MissionState::Bootstrap:
      safety_controller_->enter_safe_stop("bootstrap");
      transition_to(MissionState::WaitMcuStatus, "waiting for first MCU status");
      break;

    case MissionState::WaitMcuStatus:
      if (!mcu.available || !mcu.fresh) {
        break;
      }
      if (mcu.auto_start_latched || is_auto_pi(mcu)) {
        context_.last_error_code = error::kMissingRunContext;
        context_.last_error_message =
            "node started while MCU already owns an active or latched auto task";
        transition_to(MissionState::RecoveryRequired, context_.last_error_message);
      } else if (is_fault(mcu) || is_estop(mcu)) {
        transition_to(MissionState::WaitReset, "MCU is in Fault/EStop during startup");
      } else {
        clear_run_context();
        transition_to(MissionState::WaitStart, "MCU status is available");
      }
      break;

    case MissionState::WaitStart: {
      safety_controller_->enter_safe_stop("waiting for auto task start");
      if (!mcu.available || !mcu.fresh) {
        transition_to(MissionState::WaitMcuStatus, "MCU status is unavailable");
        break;
      }
      if (is_fault(mcu) || is_estop(mcu)) {
        transition_to(MissionState::WaitReset, "MCU is in Fault/EStop while waiting for START");
        break;
      }

      rclcpp::Time start_event_time(0, 0, get_clock()->get_clock_type());
      if (has_pending_start(start_event_time)) {
        if (is_auto_pi(mcu) && mcu.auto_start_latched) {
          (void)take_pending_start(start_event_time);
          unexpected_latched_since_.reset();
          ++context_.local_run_id;
          context_.start_event_seen = true;
          context_.reset_event_seen = false;
          context_.last_error_code = error::kNone;
          context_.last_error_message.clear();
          transition_to(MissionState::Precheck, "START event confirmed by MCU status");
        } else if ((current_time - start_event_time).seconds() > start_confirm_timeout_s_) {
          (void)take_pending_start(start_event_time);
          context_.last_error_code = error::kStartConfirmationTimeout;
          context_.last_error_message = "START event was not confirmed by MCU status";
          transition_to(MissionState::RecoveryRequired, context_.last_error_message);
        }
        break;
      }

      if (is_auto_pi(mcu) && mcu.auto_start_latched) {
        if (!unexpected_latched_since_.has_value()) {
          unexpected_latched_since_ = current_time;
        } else if ((current_time - *unexpected_latched_since_).seconds() >
                   start_confirm_timeout_s_) {
          context_.last_error_code = error::kMissingRunContext;
          context_.last_error_message =
              "MCU entered AutoPi without a matching START event";
          transition_to(MissionState::RecoveryRequired, context_.last_error_message);
        }
      } else {
        unexpected_latched_since_.reset();
      }
      break;
    }

    case MissionState::Precheck: {
      const auto failure = common_precheck(mcu);
      if (!failure.has_value()) {
        transition_to(MissionState::Initializing, "common precheck passed");
        break;
      }

      context_.last_error_code = failure->first;
      context_.last_error_message = failure->second;
      safety_controller_->enter_safe_stop(context_.last_error_message);
      if (report_fail_on_common_precheck_error_ && is_auto_pi(mcu) &&
          mcu.auto_start_latched) {
        begin_final_report(
            FinalResultKind::Fail,
            static_cast<std::int16_t>(failure->first),
            failure->second);
      } else {
        transition_to(MissionState::WaitReset, failure->second);
      }
      break;
    }

    case MissionState::Initializing:
      initialize_run(current_time);
      transition_to(MissionState::Running, "common run context initialized");
      break;

    case MissionState::Running:
      handle_dry_run(current_time);
      break;

    case MissionState::Aborting:
      safety_controller_->enter_safe_stop(status_message_);
      context_.active = false;
      context_.cancellation_requested = true;
      context_.result_report_in_flight = false;
      result_reporter_->reset();
      final_result_kind_ = FinalResultKind::None;
      if (state_after_abort_ == MissionState::WaitStart) {
        clear_run_context();
      }
      transition_to(state_after_abort_, status_message_);
      break;

    case MissionState::ReportingDone:
    case MissionState::ReportingFail:
      handle_reporting(current_time);
      break;

    case MissionState::WaitMcuFinished:
    case MissionState::WaitMcuFault:
      handle_confirmation_wait(mcu, current_time);
      break;

    case MissionState::WaitReset:
      safety_controller_->enter_safe_stop("waiting for MCU reset");
      if (is_reset_confirmed(mcu)) {
        clear_run_context();
        transition_to(MissionState::WaitStart, "auto task latch reset confirmed");
      }
      break;

    case MissionState::RecoveryRequired:
      safety_controller_->enter_safe_stop("recovery required");
      if (mcu.available && mcu.fresh && !mcu.auto_start_latched &&
          !is_auto_pi(mcu) && !is_fault(mcu) && !is_estop(mcu)) {
        clear_run_context();
        transition_to(MissionState::WaitStart, "MCU reset restored a safe baseline");
      }
      break;

    case MissionState::ShuttingDown:
      safety_controller_->enter_safe_stop("shutting down");
      break;
  }

  publish_mission_status(mcu, current_time);
}

void MissionManagerNode::transition_to(
    const MissionState next_state,
    const std::string& reason) {
  if (context_.state == next_state) {
    status_message_ = reason;
    return;
  }

  const MissionState previous = context_.state;
  context_.state = next_state;
  context_.state_enter_time = now();
  status_message_ = reason;
  if (next_state == MissionState::WaitReset ||
      next_state == MissionState::RecoveryRequired ||
      next_state == MissionState::ShuttingDown) {
    context_.active = false;
  }

  RCLCPP_INFO(
      get_logger(),
      "Mission state: %s -> %s, run=%u, reason=%s",
      mission_state_name(previous),
      mission_state_name(next_state),
      context_.local_run_id,
      reason.c_str());

  const auto current_time = now();
  const auto mcu = mcu_state_cache_.snapshot(current_time, mcu_status_timeout_s_);
  publish_mission_status(mcu, current_time, true);
}

void MissionManagerNode::request_abort(
    const MissionState state_after_abort,
    const std::string& reason,
    const std::int32_t error_code) {
  if (context_.state == MissionState::Aborting ||
      context_.state == MissionState::ShuttingDown) {
    return;
  }
  state_after_abort_ = state_after_abort;
  context_.cancellation_requested = true;
  if (error_code != error::kNone) {
    context_.last_error_code = error_code;
    context_.last_error_message = reason;
  }
  transition_to(MissionState::Aborting, reason);
}

bool MissionManagerNode::handle_global_conditions(
    const McuStateSnapshot& mcu,
    const rclcpp::Time& /*now*/) {
  if (shutdown_requested_ && context_.state != MissionState::ShuttingDown) {
    if (context_.state == MissionState::Aborting) {
      state_after_abort_ = MissionState::ShuttingDown;
      return false;
    }
    request_abort(MissionState::ShuttingDown, "shutdown requested");
    return true;
  }

  if (take_pending_reset()) {
    context_.reset_event_seen = true;
    if (context_.state == MissionState::WaitStart ||
        context_.state == MissionState::WaitMcuStatus) {
      clear_run_context();
      return false;
    }
    const MissionState target =
        (mcu.available && mcu.fresh && !mcu.auto_start_latched &&
         !is_auto_pi(mcu) && !is_fault(mcu) && !is_estop(mcu))
            ? MissionState::WaitStart
            : MissionState::WaitReset;
    request_abort(target, "MCU RESET event received");
    return true;
  }

  if (state_is_active_lifecycle(context_.state) && (!mcu.available || !mcu.fresh)) {
    request_abort(
        MissionState::RecoveryRequired,
        "MCU status timed out during an active mission lifecycle",
        error::kMcuStatusTimeout);
    return true;
  }

  if (!mcu.available || !mcu.fresh) {
    return false;
  }

  if (context_.state == MissionState::ReportingDone && is_finished(mcu)) {
    context_.result_reported = true;
    context_.active = false;
    transition_to(MissionState::WaitReset, "MCU confirmed Finished during result reporting");
    return true;
  }
  if (context_.state == MissionState::ReportingFail && is_fault(mcu)) {
    context_.result_reported = true;
    context_.active = false;
    transition_to(MissionState::WaitReset, "MCU confirmed Fault during result reporting");
    return true;
  }

  if (!state_requires_auto_pi(context_.state)) {
    return false;
  }

  if (is_estop(mcu)) {
    request_abort(MissionState::WaitReset, "MCU entered EStop");
    return true;
  }
  if (is_fault(mcu)) {
    request_abort(MissionState::WaitReset, "MCU entered Fault");
    return true;
  }
  if (is_manual(mcu)) {
    request_abort(MissionState::WaitReset, "manual control took over");
    return true;
  }
  if (!is_auto_pi(mcu) || !mcu.auto_start_latched) {
    request_abort(MissionState::WaitReset, "MCU left AutoPi or cleared the task latch");
    return true;
  }

  return false;
}

std::optional<std::pair<std::int32_t, std::string>>
MissionManagerNode::common_precheck(const McuStateSnapshot& mcu) const {
  if (!mcu.available) {
    return std::make_pair(error::kMcuStatusUnavailable, "MCU status is unavailable");
  }
  if (!mcu.fresh) {
    return std::make_pair(error::kMcuStatusTimeout, "MCU status is stale");
  }
  if (is_estop(mcu)) {
    return std::make_pair(error::kMcuNotAutoPi, "MCU is in EStop");
  }
  if (is_fault(mcu)) {
    return std::make_pair(error::kMcuNotAutoPi, "MCU is in Fault");
  }
  if (!is_auto_pi(mcu)) {
    return std::make_pair(error::kMcuNotAutoPi, "MCU is not in AutoPi");
  }
  if (!mcu.auto_start_latched) {
    return std::make_pair(
        error::kAutoStartLatchMismatch,
        "auto_start_latched is false during task startup");
  }
  if (!is_pi_online(mcu)) {
    return std::make_pair(error::kPiOffline, "MCU reports Pi offline");
  }
  if (!is_chassis_ready(mcu)) {
    return std::make_pair(error::kChassisNotReady, "chassis is not ready");
  }
  if (!is_odom_ready(mcu)) {
    return std::make_pair(error::kOdomNotReady, "odom is not ready");
  }
  if (require_arm_ready_in_common_precheck_ && !is_arm_ready(mcu)) {
    return std::make_pair(error::kArmNotReady, "arm is not ready");
  }
  return std::nullopt;
}

void MissionManagerNode::initialize_run(const rclcpp::Time& now_time) {
  safety_controller_->enter_safe_stop("initializing common mission context");
  context_.active = true;
  context_.result_reported = false;
  context_.result_report_in_flight = false;
  context_.cancellation_requested = false;
  context_.run_start_time = now_time;
  context_.last_error_code = error::kNone;
  context_.last_error_message.clear();
  final_result_kind_ = FinalResultKind::None;
  final_result_code_ = 0;
  report_attempts_ = 0;
  result_reporter_->reset();
}

void MissionManagerNode::clear_run_context() {
  context_.active = false;
  context_.start_event_seen = false;
  context_.reset_event_seen = false;
  context_.result_reported = false;
  context_.result_report_in_flight = false;
  context_.cancellation_requested = false;
  context_.last_error_code = error::kNone;
  context_.last_error_message.clear();
  final_result_kind_ = FinalResultKind::None;
  final_result_code_ = 0;
  report_attempts_ = 0;
  unexpected_latched_since_.reset();
  result_reporter_->reset();
  {
    std::lock_guard<std::mutex> lock(event_mutex_);
    pending_start_ = false;
  }
}

void MissionManagerNode::begin_final_report(
    const FinalResultKind kind,
    const std::int16_t code,
    const std::string& message) {
  final_result_kind_ = kind;
  final_result_code_ = kind == FinalResultKind::Done ? 0 : code;
  report_attempts_ = 0;
  context_.result_reported = false;
  context_.result_report_in_flight = false;
  result_reporter_->reset();
  last_report_attempt_time_ = now() - rclcpp::Duration::from_seconds(result_retry_interval_s_);

  if (kind == FinalResultKind::Done) {
    transition_to(MissionState::ReportingDone, message);
  } else {
    transition_to(MissionState::ReportingFail, message);
  }
}

void MissionManagerNode::handle_reporting(const rclcpp::Time& current_time) {
  safety_controller_->enter_safe_stop("reporting mission result");
  result_reporter_->poll_timeout(current_time, result_service_timeout_s_);
  auto report = result_reporter_->snapshot();
  const int maximum_attempts = 1 + result_report_retry_count_;

  if (report.status == ReportStatus::Accepted) {
    context_.result_reported = true;
    context_.result_report_in_flight = false;
    RCLCPP_INFO(
        get_logger(),
        "Mission result accepted by bridge: kind=%s sent_count=%u message=%s",
        final_result_kind_ == FinalResultKind::Done ? "DONE" : "FAIL",
        static_cast<unsigned int>(report.sent_count),
        report.message.c_str());
    transition_to(
        final_result_kind_ == FinalResultKind::Done
            ? MissionState::WaitMcuFinished
            : MissionState::WaitMcuFault,
        "mission result written to Pi-MCU serial bridge");
    return;
  }

  const bool terminal_attempt_failure =
      report.status == ReportStatus::Rejected ||
      report.status == ReportStatus::ServiceUnavailable ||
      report.status == ReportStatus::TimedOut;

  if (terminal_attempt_failure) {
    context_.result_report_in_flight = false;
    if (report_attempts_ >= maximum_attempts) {
      context_.last_error_code =
          report.status == ReportStatus::ServiceUnavailable
              ? error::kResultServiceUnavailable
              : error::kResultServiceRejected;
      context_.last_error_message = report.message;
      transition_to(MissionState::RecoveryRequired, report.message);
      return;
    }
    if ((current_time - last_report_attempt_time_).seconds() >= result_retry_interval_s_) {
      result_reporter_->reset();
      report = result_reporter_->snapshot();
    }
  }

  if (report.status != ReportStatus::Idle) {
    return;
  }
  if (report_attempts_ >= maximum_attempts) {
    context_.last_error_code = error::kResultServiceRejected;
    context_.last_error_message = "mission result retry limit reached";
    transition_to(MissionState::RecoveryRequired, context_.last_error_message);
    return;
  }
  if ((current_time - last_report_attempt_time_).seconds() < result_retry_interval_s_) {
    return;
  }

  const std::uint8_t result =
      final_result_kind_ == FinalResultKind::Done
          ? mcu_comm_bridge::srv::ReportMissionResult::Request::RESULT_DONE
          : mcu_comm_bridge::srv::ReportMissionResult::Request::RESULT_FAIL;
  ++report_attempts_;
  last_report_attempt_time_ = current_time;
  context_.result_report_in_flight = true;
  (void)result_reporter_->start(result, final_result_code_, current_time);
}

void MissionManagerNode::handle_confirmation_wait(
    const McuStateSnapshot& mcu,
    const rclcpp::Time& current_time) {
  safety_controller_->enter_safe_stop("waiting for MCU result confirmation");

  if (context_.state == MissionState::WaitMcuFinished && is_finished(mcu)) {
    context_.active = false;
    transition_to(MissionState::WaitReset, "MCU confirmed Finished");
    return;
  }
  if (context_.state == MissionState::WaitMcuFault && is_fault(mcu)) {
    context_.active = false;
    transition_to(MissionState::WaitReset, "MCU confirmed Fault");
    return;
  }

  if (is_estop(mcu) || is_manual(mcu)) {
    request_abort(MissionState::WaitReset, "MCU changed state while confirming result");
    return;
  }
  if (context_.state == MissionState::WaitMcuFinished && is_fault(mcu)) {
    context_.last_error_message = "MCU entered Fault while confirming DONE";
    transition_to(MissionState::WaitReset, context_.last_error_message);
    return;
  }
  if (context_.state == MissionState::WaitMcuFault && is_finished(mcu)) {
    context_.last_error_message = "MCU entered Finished while confirming FAIL";
    transition_to(MissionState::WaitReset, context_.last_error_message);
    return;
  }

  if ((current_time - context_.state_enter_time).seconds() <= result_confirm_timeout_s_) {
    return;
  }

  // The bridge marks a mission result as reported after the first successful
  // serial write and rejects duplicate DONE/FAIL calls until RESET. Therefore
  // an MCU confirmation timeout cannot be recovered by calling the service
  // again; enter RecoveryRequired without replaying the task or result.
  context_.last_error_code =
      final_result_kind_ == FinalResultKind::Done
          ? error::kDoneConfirmationTimeout
          : error::kFailConfirmationTimeout;
  context_.last_error_message = "MCU did not confirm the reported mission result";
  transition_to(MissionState::RecoveryRequired, context_.last_error_message);
}

void MissionManagerNode::handle_dry_run(const rclcpp::Time& current_time) {
  if (dry_run_mode_ == "hold") {
    return;
  }
  if ((current_time - context_.state_enter_time).seconds() < dry_run_success_delay_s_) {
    return;
  }

  if (dry_run_mode_ == "success_after_delay") {
    notify_task_flow_succeeded("dry-run task flow succeeded");
    return;
  }

  notify_task_flow_failed(dry_run_fail_code_, "dry-run task flow failed");
}

void MissionManagerNode::notify_task_flow_succeeded(const std::string& message) {
  if (context_.state != MissionState::Running) {
    RCLCPP_WARN(
        get_logger(),
        "Ignoring task-flow success outside RUNNING: state=%s",
        mission_state_name(context_.state));
    return;
  }
  begin_final_report(FinalResultKind::Done, 0, message);
}

void MissionManagerNode::notify_task_flow_failed(
    const std::int32_t error_code,
    const std::string& message) {
  if (context_.state != MissionState::Running) {
    RCLCPP_WARN(
        get_logger(),
        "Ignoring task-flow failure outside RUNNING: state=%s",
        mission_state_name(context_.state));
    return;
  }
  context_.last_error_code = error_code;
  context_.last_error_message = message;
  begin_final_report(
      FinalResultKind::Fail,
      static_cast<std::int16_t>(error_code),
      message);
}

void MissionManagerNode::notify_task_flow_cancelled(const std::string& reason) {
  if (context_.state != MissionState::Running) {
    return;
  }
  request_abort(MissionState::WaitReset, reason);
}

void MissionManagerNode::publish_mission_status(
    const McuStateSnapshot& mcu,
    const rclcpp::Time& current_time,
    const bool force) {
  const double publish_period_s = 1.0 / status_publish_rate_hz_;
  if (!force && (current_time - last_status_publish_time_).seconds() < publish_period_s) {
    return;
  }

  atlas_mission_interfaces::msg::MissionStatus message;
  message.header.stamp = current_time;
  message.header.frame_id = "";
  message.state = static_cast<std::uint8_t>(context_.state);
  message.local_run_id = context_.local_run_id;
  message.active = context_.active;
  message.mcu_status_fresh = mcu.available && mcu.fresh;
  message.auto_start_latched = mcu.available && mcu.auto_start_latched;
  message.mcu_app_state = mcu.available ? mcu.app_state : 0u;
  message.result_reported = context_.result_reported;
  message.error_code = context_.last_error_code;
  message.state_name = mission_state_name(context_.state);
  message.message = !context_.last_error_message.empty()
      ? context_.last_error_message
      : status_message_;
  mission_status_publisher_->publish(message);
  last_status_publish_time_ = current_time;
}

bool MissionManagerNode::take_pending_start(rclcpp::Time& event_time) {
  std::lock_guard<std::mutex> lock(event_mutex_);
  if (!pending_start_) {
    return false;
  }
  event_time = pending_start_time_;
  pending_start_ = false;
  return true;
}

bool MissionManagerNode::has_pending_start(rclcpp::Time& event_time) const {
  std::lock_guard<std::mutex> lock(event_mutex_);
  if (!pending_start_) {
    return false;
  }
  event_time = pending_start_time_;
  return true;
}

bool MissionManagerNode::take_pending_reset() {
  std::lock_guard<std::mutex> lock(event_mutex_);
  if (!pending_reset_) {
    return false;
  }
  pending_reset_ = false;
  return true;
}

void MissionManagerNode::set_pending_reset() {
  std::lock_guard<std::mutex> lock(event_mutex_);
  pending_reset_ = true;
}

bool MissionManagerNode::state_requires_auto_pi(const MissionState state) const noexcept {
  return state == MissionState::Precheck ||
         state == MissionState::Initializing ||
         state == MissionState::Running ||
         state == MissionState::ReportingDone ||
         state == MissionState::ReportingFail;
}

bool MissionManagerNode::state_is_active_lifecycle(const MissionState state) const noexcept {
  return state == MissionState::Precheck ||
         state == MissionState::Initializing ||
         state == MissionState::Running ||
         state == MissionState::ReportingDone ||
         state == MissionState::WaitMcuFinished ||
         state == MissionState::ReportingFail ||
         state == MissionState::WaitMcuFault;
}

bool MissionManagerNode::is_reset_confirmed(const McuStateSnapshot& mcu) const noexcept {
  return mcu.available && mcu.fresh && !mcu.auto_start_latched &&
         !is_auto_pi(mcu) && !is_fault(mcu) && !is_estop(mcu);
}

}  // namespace atlas_mission_manager
