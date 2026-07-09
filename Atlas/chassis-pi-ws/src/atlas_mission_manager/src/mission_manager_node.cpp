#include "atlas_mission_manager/mission_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "atlas_mission_manager/mission_error.hpp"

namespace atlas_mission_manager {
namespace {
constexpr double kMinimumRateHz = 1.0;

std::chrono::nanoseconds period_from_hz(const double rate_hz) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(kMinimumRateHz, rate_hz)));
}

std::string yaml_string_or(const YAML::Node& node, const std::string& key, const std::string& fallback) {
  const YAML::Node child = node[key];
  return child ? child.as<std::string>() : fallback;
}

double yaml_double_or(const YAML::Node& node, const std::string& key, const double fallback) {
  const YAML::Node child = node[key];
  return child ? child.as<double>() : fallback;
}

std::size_t yaml_size_or(const YAML::Node& node, const std::string& key, const std::size_t fallback) {
  const YAML::Node child = node[key];
  return child ? child.as<std::size_t>() : fallback;
}

bool yaml_bool_or(const YAML::Node& node, const std::string& key, const bool fallback) {
  const YAML::Node child = node[key];
  return child ? child.as<bool>() : fallback;
}

const char* route_stage_name(const RouteStage stage) {
  switch (stage) {
    case RouteStage::Idle: return "IDLE";
    case RouteStage::StartPreMove: return "START_PRE_MOVE";
    case RouteStage::WaitPreMove: return "WAIT_PRE_MOVE";
    case RouteStage::StartNavigation: return "START_NAVIGATION";
    case RouteStage::WaitNavigation: return "WAIT_NAVIGATION";
    case RouteStage::StartManipulation: return "START_MANIPULATION";
    case RouteStage::WaitManipulation: return "WAIT_MANIPULATION";
    case RouteStage::Completed: return "COMPLETED";
    case RouteStage::Failed: return "FAILED";
    default: return "UNKNOWN";
  }
}

ManipulationJobConfig parse_arrival_job(const YAML::Node& node, const std::string& waypoint_id, const std::size_t index) {
  ManipulationJobConfig job;
  job.id = yaml_string_or(node, "id", "");
  if (job.id.empty()) {
    std::ostringstream stream;
    stream << waypoint_id << "_job_" << (index + 1u);
    job.id = stream.str();
  }
  job.prepare_action = yaml_string_or(node, "prepare_action", "noop");
  job.arrival_task = yaml_string_or(node, "arrival_task", yaml_string_or(node, "task", "noop"));
  return job;
}

WaypointConfig parse_waypoint(const YAML::Node& node, const std::size_t index) {
  WaypointConfig wp;
  wp.id = yaml_string_or(node, "id", "");
  if (wp.id.empty()) {
    std::ostringstream stream;
    stream << "waypoint_" << index;
    wp.id = stream.str();
  }
  wp.x = yaml_double_or(node, "x", yaml_double_or(node, "x_m", 0.0));
  wp.y = yaml_double_or(node, "y", yaml_double_or(node, "y_m", 0.0));
  wp.yaw = yaml_double_or(node, "yaw", yaml_double_or(node, "yaw_rad", 0.0));
  wp.timeout_s = std::max(0.1, yaml_double_or(node, "timeout_s", 20.0));
  wp.area = yaml_string_or(node, "area", "PASS_BY");
  wp.pre_move_action = yaml_string_or(node, "pre_move_action", "noop");

  const YAML::Node jobs = node["arrival_jobs"];
  if (jobs && jobs.IsSequence()) {
    for (std::size_t i = 0; i < jobs.size(); ++i) {
      wp.arrival_jobs.push_back(parse_arrival_job(jobs[i], wp.id, i));
    }
  } else {
    const std::string legacy_prepare = yaml_string_or(node, "prepare_action", "noop");
    const std::string legacy_task = yaml_string_or(node, "arrival_task", "noop");
    if (legacy_task != "noop") {
      ManipulationJobConfig job;
      job.id = wp.id + "_job_1";
      job.prepare_action = legacy_prepare;
      job.arrival_task = legacy_task;
      wp.arrival_jobs.push_back(job);
    }
  }
  return wp;
}
}  // namespace

MissionManagerNode::MissionManagerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("atlas_mission_manager", options) {
  mcu_status_topic_ = declare_parameter<std::string>("mcu_status_topic", "/mcu/status");
  auto_task_event_topic_ = declare_parameter<std::string>("auto_task_event_topic", "/mcu/auto_task_event");
  mission_result_service_ = declare_parameter<std::string>("mission_result_service", "/mcu/report_mission_result");
  brake_service_ = declare_parameter<std::string>("brake_service", "/mcu/set_brake");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/motor_cmd_vel");
  mission_status_topic_ = declare_parameter<std::string>("mission_status_topic", "/atlas/mission/status");
  route_yaml_path_ = declare_parameter<std::string>("route_yaml_path", "");

  navigation_backend_ = declare_parameter<std::string>("navigation_backend", "pseudo");
  navigation_status_topic_ = declare_parameter<std::string>("navigation_status_topic", "/atlas/navigation/status");
  navigation_cmd_vel_topic_ = declare_parameter<std::string>("navigation_cmd_vel_topic", "/atlas/navigation/cmd_vel");
  navigation_start_service_ = declare_parameter<std::string>("navigation_start_service", "/atlas/navigation/start");
  navigation_cancel_service_ = declare_parameter<std::string>("navigation_cancel_service", "/atlas/navigation/cancel");

  manipulation_backend_ = declare_parameter<std::string>("manipulation_backend", "vision_pollination");
  manipulation_status_topic_ = declare_parameter<std::string>("manipulation_status_topic", "/atlas/manipulation/status");
  manipulation_start_service_ = declare_parameter<std::string>("manipulation_start_service", "/atlas/manipulation/start");
  manipulation_cancel_service_ = declare_parameter<std::string>("manipulation_cancel_service", "/atlas/manipulation/cancel");

  update_rate_hz_ = std::max(kMinimumRateHz, declare_parameter<double>("update_rate_hz", 20.0));
  status_publish_rate_hz_ = std::max(kMinimumRateHz, declare_parameter<double>("status_publish_rate_hz", 5.0));
  zero_velocity_publish_rate_hz_ = std::max(kMinimumRateHz, declare_parameter<double>("zero_velocity_publish_rate_hz", 10.0));
  mcu_status_timeout_s_ = std::max(0.05, declare_parameter<double>("mcu_status_timeout_s", 0.5));
  start_confirm_timeout_s_ = std::max(0.05, declare_parameter<double>("start_confirm_timeout_s", 1.0));
  result_service_timeout_s_ = std::max(0.05, declare_parameter<double>("result_service_timeout_s", 1.0));
  result_confirm_timeout_s_ = std::max(0.1, declare_parameter<double>("result_confirm_timeout_s", 3.0));
  result_retry_interval_s_ = std::max(0.0, declare_parameter<double>("result_retry_interval_s", 0.3));
  result_report_retry_count_ = std::clamp<int>(static_cast<int>(declare_parameter<int>("result_report_retry_count", 2)), 0, 10);
  require_arm_ready_in_common_precheck_ = declare_parameter<bool>("require_arm_ready_in_common_precheck", true);
  report_fail_on_common_precheck_error_ = declare_parameter<bool>("report_fail_on_common_precheck_error", false);

  status_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  event_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions status_options;
  status_options.callback_group = status_callback_group_;
  auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  status_qos.reliable();
  status_qos.transient_local();
  mcu_status_subscription_ = create_subscription<mcu_comm_bridge::msg::McuStatus>(
      mcu_status_topic_, status_qos,
      std::bind(&MissionManagerNode::on_mcu_status, this, std::placeholders::_1), status_options);

  rclcpp::SubscriptionOptions event_options;
  event_options.callback_group = event_callback_group_;
  auto event_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  event_qos.reliable();
  event_qos.durability_volatile();
  auto_task_event_subscription_ = create_subscription<mcu_comm_bridge::msg::AutoTaskEvent>(
      auto_task_event_topic_, event_qos,
      std::bind(&MissionManagerNode::on_auto_task_event, this, std::placeholders::_1), event_options);

  navigation_status_subscription_ = create_subscription<atlas_mission_interfaces::msg::NavigationStatus>(
      navigation_status_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&MissionManagerNode::on_navigation_status, this, std::placeholders::_1));
  manipulation_status_subscription_ = create_subscription<atlas_mission_interfaces::msg::ManipulationStatus>(
      manipulation_status_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&MissionManagerNode::on_manipulation_status, this, std::placeholders::_1));
  navigation_cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      navigation_cmd_vel_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&MissionManagerNode::on_navigation_cmd_vel, this, std::placeholders::_1));

  navigation_start_client_ = create_client<atlas_mission_interfaces::srv::StartNavigation>(
      navigation_start_service_, rmw_qos_profile_services_default, service_callback_group_);
  navigation_cancel_client_ = create_client<atlas_mission_interfaces::srv::CancelNavigation>(
      navigation_cancel_service_, rmw_qos_profile_services_default, service_callback_group_);
  manipulation_start_client_ = create_client<atlas_mission_interfaces::srv::StartManipulation>(
      manipulation_start_service_, rmw_qos_profile_services_default, service_callback_group_);
  manipulation_cancel_client_ = create_client<atlas_mission_interfaces::srv::CancelManipulation>(
      manipulation_cancel_service_, rmw_qos_profile_services_default, service_callback_group_);

  auto mission_status_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  mission_status_qos.reliable();
  mission_status_qos.transient_local();
  mission_status_publisher_ = create_publisher<atlas_mission_interfaces::msg::MissionStatus>(
      mission_status_topic_, mission_status_qos);

  safety_controller_ = std::make_unique<SafetyController>(
      *this, cmd_vel_topic_, brake_service_, zero_velocity_publish_rate_hz_, service_callback_group_);
  result_reporter_ = std::make_unique<MissionResultReporter>(
      *this, mission_result_service_, service_callback_group_);

  const auto current_time = now();
  context_.state_enter_time = current_time;
  context_.run_start_time = current_time;
  context_.last_mcu_status_time = current_time;
  last_status_publish_time_ = current_time;
  last_report_attempt_time_ = current_time;
  route_stage_enter_time_ = current_time;

  if (!route_yaml_path_.empty() && !load_route_config(route_yaml_path_)) {
    RCLCPP_ERROR(get_logger(), "Failed to load route YAML: %s", route_yaml_path_.c_str());
  }

  state_machine_timer_ = create_wall_timer(
      period_from_hz(update_rate_hz_), std::bind(&MissionManagerNode::update_state_machine, this));

  RCLCPP_INFO(
      get_logger(),
      "Atlas mission manager started: nav_backend=%s manipulation_backend=%s forward=%zu return=%zu",
      navigation_backend_.c_str(), manipulation_backend_.c_str(), forward_waypoints_.size(), return_waypoints_.size());
}

MissionManagerNode::~MissionManagerNode() { prepare_shutdown(); }

void MissionManagerNode::prepare_shutdown() {
  if (shutdown_requested_) return;
  shutdown_requested_ = true;
  cancel_backends("mission manager shutdown");
  if (safety_controller_) safety_controller_->enter_safe_stop("mission manager shutdown");
}

void MissionManagerNode::on_mcu_status(const mcu_comm_bridge::msg::McuStatus::SharedPtr message) {
  const auto received_at = now();
  mcu_state_cache_.update(*message, received_at);
  std::lock_guard<std::mutex> lock(event_mutex_);
  if (previous_status_available_ && previous_latched_ && !message->auto_start_latched) {
    pending_reset_ = true;
  }
  previous_status_available_ = true;
  previous_latched_ = message->auto_start_latched;
}

void MissionManagerNode::on_auto_task_event(const mcu_comm_bridge::msg::AutoTaskEvent::SharedPtr message) {
  std::lock_guard<std::mutex> lock(event_mutex_);
  if (message->event == mcu_comm_bridge::msg::AutoTaskEvent::EVENT_START) {
    if (!pending_start_) {
      pending_start_ = true;
      pending_start_time_ = now();
    }
  } else if (message->event == mcu_comm_bridge::msg::AutoTaskEvent::EVENT_RESET) {
    pending_reset_ = true;
  }
}

void MissionManagerNode::on_navigation_status(
    const atlas_mission_interfaces::msg::NavigationStatus::SharedPtr message) {
  std::lock_guard<std::mutex> lock(backend_status_mutex_);
  latest_navigation_status_ = *message;
  navigation_status_available_ = true;
}

void MissionManagerNode::on_manipulation_status(
    const atlas_mission_interfaces::msg::ManipulationStatus::SharedPtr message) {
  std::lock_guard<std::mutex> lock(backend_status_mutex_);
  latest_manipulation_status_ = *message;
  manipulation_status_available_ = true;
}

void MissionManagerNode::on_navigation_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message) {
  if (!message) return;
  if (context_.state == MissionState::Running && route_stage_ == RouteStage::WaitNavigation) {
    (void)safety_controller_->publish_motion_command(*message);
  }
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
      if (!mcu.available || !mcu.fresh) break;
      if (mcu.auto_start_latched || is_auto_pi(mcu)) {
        context_.last_error_code = error::kMissingRunContext;
        context_.last_error_message = "node started while MCU already owns an active or latched auto task";
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
      rclcpp::Time start_event_time(0, 0, get_clock()->get_clock_type());
      if (has_pending_start(start_event_time)) {
        if (is_auto_pi(mcu) && mcu.auto_start_latched) {
          (void)take_pending_start(start_event_time);
          unexpected_latched_since_.reset();
          ++context_.local_run_id;
          context_.start_event_seen = true;
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
        if (!unexpected_latched_since_) unexpected_latched_since_ = current_time;
        else if ((current_time - *unexpected_latched_since_).seconds() > start_confirm_timeout_s_) {
          context_.last_error_code = error::kMissingRunContext;
          context_.last_error_message = "MCU entered AutoPi without a matching START event";
          transition_to(MissionState::RecoveryRequired, context_.last_error_message);
        }
      } else {
        unexpected_latched_since_.reset();
      }
      break;
    }

    case MissionState::Precheck: {
      const auto failure = common_precheck(mcu);
      if (!failure) {
        transition_to(MissionState::Initializing, "common precheck passed");
        break;
      }
      context_.last_error_code = failure->first;
      context_.last_error_message = failure->second;
      safety_controller_->enter_safe_stop(context_.last_error_message);
      if (report_fail_on_common_precheck_error_ && is_auto_pi(mcu) && mcu.auto_start_latched) {
        begin_final_report(FinalResultKind::Fail, static_cast<std::int16_t>(failure->first), failure->second);
      } else {
        transition_to(MissionState::WaitReset, failure->second);
      }
      break;
    }

    case MissionState::Initializing:
      initialize_run(current_time);
      transition_to(MissionState::Running, "route mission initialized");
      break;

    case MissionState::Running:
      handle_route_flow(current_time);
      break;

    case MissionState::Aborting:
      safety_controller_->enter_safe_stop(status_message_);
      cancel_backends(status_message_);
      context_.active = false;
      context_.cancellation_requested = true;
      context_.result_report_in_flight = false;
      result_reporter_->reset();
      final_result_kind_ = FinalResultKind::None;
      if (state_after_abort_ == MissionState::WaitStart) clear_run_context();
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
      if (mcu.available && mcu.fresh && !mcu.auto_start_latched && !is_auto_pi(mcu) && !is_fault(mcu) && !is_estop(mcu)) {
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

void MissionManagerNode::transition_to(const MissionState next_state, const std::string& reason) {
  if (context_.state == next_state) {
    status_message_ = reason;
    return;
  }
  const auto previous = context_.state;
  context_.state = next_state;
  context_.state_enter_time = now();
  status_message_ = reason;
  if (next_state == MissionState::WaitReset || next_state == MissionState::RecoveryRequired || next_state == MissionState::ShuttingDown) {
    context_.active = false;
  }
  RCLCPP_INFO(get_logger(), "Mission state: %s -> %s, run=%u, reason=%s", mission_state_name(previous), mission_state_name(next_state), context_.local_run_id, reason.c_str());
  const auto current_time = now();
  publish_mission_status(mcu_state_cache_.snapshot(current_time, mcu_status_timeout_s_), current_time, true);
}

void MissionManagerNode::request_abort(const MissionState state_after_abort, const std::string& reason, const std::int32_t error_code) {
  if (context_.state == MissionState::Aborting || context_.state == MissionState::ShuttingDown) return;
  state_after_abort_ = state_after_abort;
  context_.cancellation_requested = true;
  if (error_code != error::kNone) {
    context_.last_error_code = error_code;
    context_.last_error_message = reason;
  }
  transition_to(MissionState::Aborting, reason);
}

bool MissionManagerNode::handle_global_conditions(const McuStateSnapshot& mcu, const rclcpp::Time& /*now*/) {
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
    if (context_.state == MissionState::WaitStart || context_.state == MissionState::WaitMcuStatus) {
      clear_run_context();
      return false;
    }
    const MissionState target = (mcu.available && mcu.fresh && !mcu.auto_start_latched && !is_auto_pi(mcu) && !is_fault(mcu) && !is_estop(mcu)) ? MissionState::WaitStart : MissionState::WaitReset;
    request_abort(target, "MCU RESET event received");
    return true;
  }
  if (state_is_active_lifecycle(context_.state) && (!mcu.available || !mcu.fresh)) {
    request_abort(MissionState::RecoveryRequired, "MCU status timed out during active mission", error::kMcuStatusTimeout);
    return true;
  }
  if (!mcu.available || !mcu.fresh) return false;
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
  if (!state_requires_auto_pi(context_.state)) return false;
  if (is_estop(mcu)) { request_abort(MissionState::WaitReset, "MCU entered EStop"); return true; }
  if (is_fault(mcu)) { request_abort(MissionState::WaitReset, "MCU entered Fault"); return true; }
  if (is_manual(mcu)) { request_abort(MissionState::WaitReset, "manual control took over"); return true; }
  if (!is_auto_pi(mcu) || !mcu.auto_start_latched) { request_abort(MissionState::WaitReset, "MCU left AutoPi or cleared task latch"); return true; }
  return false;
}

std::optional<std::pair<std::int32_t, std::string>> MissionManagerNode::common_precheck(const McuStateSnapshot& mcu) const {
  if (!mcu.available) return std::make_pair(error::kMcuStatusUnavailable, "MCU status is unavailable");
  if (!mcu.fresh) return std::make_pair(error::kMcuStatusTimeout, "MCU status is stale");
  if (is_estop(mcu)) return std::make_pair(error::kMcuNotAutoPi, "MCU is in EStop");
  if (is_fault(mcu)) return std::make_pair(error::kMcuNotAutoPi, "MCU is in Fault");
  if (!is_auto_pi(mcu)) return std::make_pair(error::kMcuNotAutoPi, "MCU is not in AutoPi");
  if (!mcu.auto_start_latched) return std::make_pair(error::kAutoStartLatchMismatch, "auto_start_latched is false");
  if (!is_pi_online(mcu)) return std::make_pair(error::kPiOffline, "MCU reports Pi offline");
  if (!is_chassis_ready(mcu)) return std::make_pair(error::kChassisNotReady, "chassis is not ready");
  if (!is_odom_ready(mcu)) return std::make_pair(error::kOdomNotReady, "odom is not ready");
  if (require_arm_ready_in_common_precheck_ && !is_arm_ready(mcu)) return std::make_pair(error::kArmNotReady, "arm is not ready");
  if (active_route_.empty()) return std::make_pair(error::kRouteConfigError, "mission route is empty");
  return std::nullopt;
}

void MissionManagerNode::initialize_run(const rclcpp::Time& now_time) {
  safety_controller_->enter_safe_stop("initializing mission route");
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
  reset_route_flow();
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
  reset_route_flow();
  std::lock_guard<std::mutex> lock(event_mutex_);
  pending_start_ = false;
}

void MissionManagerNode::begin_final_report(const FinalResultKind kind, const std::int16_t code, const std::string& message) {
  final_result_kind_ = kind;
  final_result_code_ = kind == FinalResultKind::Done ? 0 : code;
  report_attempts_ = 0;
  context_.result_reported = false;
  context_.result_report_in_flight = false;
  result_reporter_->reset();
  last_report_attempt_time_ = now() - rclcpp::Duration::from_seconds(result_retry_interval_s_);
  transition_to(kind == FinalResultKind::Done ? MissionState::ReportingDone : MissionState::ReportingFail, message);
}

void MissionManagerNode::handle_reporting(const rclcpp::Time& current_time) {
  safety_controller_->enter_safe_stop("reporting mission result");
  result_reporter_->poll_timeout(current_time, result_service_timeout_s_);
  auto report = result_reporter_->snapshot();
  const int maximum_attempts = 1 + result_report_retry_count_;
  if (report.status == ReportStatus::Accepted) {
    context_.result_reported = true;
    context_.result_report_in_flight = false;
    transition_to(final_result_kind_ == FinalResultKind::Done ? MissionState::WaitMcuFinished : MissionState::WaitMcuFault, "mission result written to bridge");
    return;
  }
  const bool terminal = report.status == ReportStatus::Rejected || report.status == ReportStatus::ServiceUnavailable || report.status == ReportStatus::TimedOut;
  if (terminal) {
    context_.result_report_in_flight = false;
    if (report_attempts_ >= maximum_attempts) {
      context_.last_error_code = report.status == ReportStatus::ServiceUnavailable ? error::kResultServiceUnavailable : error::kResultServiceRejected;
      context_.last_error_message = report.message;
      transition_to(MissionState::RecoveryRequired, report.message);
      return;
    }
    if ((current_time - last_report_attempt_time_).seconds() >= result_retry_interval_s_) result_reporter_->reset();
  }
  if (result_reporter_->snapshot().status != ReportStatus::Idle) return;
  if (report_attempts_ >= maximum_attempts) {
    context_.last_error_code = error::kResultServiceRejected;
    context_.last_error_message = "mission result retry limit reached";
    transition_to(MissionState::RecoveryRequired, context_.last_error_message);
    return;
  }
  if ((current_time - last_report_attempt_time_).seconds() < result_retry_interval_s_) return;
  const std::uint8_t result = final_result_kind_ == FinalResultKind::Done ? mcu_comm_bridge::srv::ReportMissionResult::Request::RESULT_DONE : mcu_comm_bridge::srv::ReportMissionResult::Request::RESULT_FAIL;
  ++report_attempts_;
  last_report_attempt_time_ = current_time;
  context_.result_report_in_flight = true;
  (void)result_reporter_->start(result, final_result_code_, current_time);
}

void MissionManagerNode::handle_confirmation_wait(const McuStateSnapshot& mcu, const rclcpp::Time& current_time) {
  safety_controller_->enter_safe_stop("waiting for MCU result confirmation");
  if (context_.state == MissionState::WaitMcuFinished && is_finished(mcu)) { context_.active = false; transition_to(MissionState::WaitReset, "MCU confirmed Finished"); return; }
  if (context_.state == MissionState::WaitMcuFault && is_fault(mcu)) { context_.active = false; transition_to(MissionState::WaitReset, "MCU confirmed Fault"); return; }
  if (is_estop(mcu) || is_manual(mcu)) { request_abort(MissionState::WaitReset, "MCU changed state while confirming result"); return; }
  if ((current_time - context_.state_enter_time).seconds() <= result_confirm_timeout_s_) return;
  context_.last_error_code = final_result_kind_ == FinalResultKind::Done ? error::kDoneConfirmationTimeout : error::kFailConfirmationTimeout;
  context_.last_error_message = "MCU did not confirm the reported mission result";
  transition_to(MissionState::RecoveryRequired, context_.last_error_message);
}

bool MissionManagerNode::load_route_config(const std::string& path) {
  forward_waypoints_.clear(); return_waypoints_.clear(); active_route_.clear();
  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node mission = root["mission"] ? root["mission"] : root["route"];
    if (!mission) throw std::runtime_error("missing mission/route root");
    navigation_backend_ = yaml_string_or(mission, "navigation_backend", navigation_backend_);
    manipulation_backend_ = yaml_string_or(mission, "manipulation_backend", manipulation_backend_);
    return_home_enabled_ = yaml_bool_or(mission, "return_home_enabled", false);
    max_forward_waypoints_ = yaml_size_or(mission, "max_forward_waypoints", 1);
    const YAML::Node waypoints = mission["waypoints"] ? mission["waypoints"] : mission;
    if (waypoints && waypoints.IsSequence()) {
      for (std::size_t i = 0; i < waypoints.size(); ++i) forward_waypoints_.push_back(parse_waypoint(waypoints[i], i));
    }
    const YAML::Node returns = mission["return_waypoints"];
    if (returns && returns.IsSequence()) {
      for (std::size_t i = 0; i < returns.size(); ++i) return_waypoints_.push_back(parse_waypoint(returns[i], i));
    }
    active_route_ = forward_waypoints_;
    if (max_forward_waypoints_ > 0 && max_forward_waypoints_ < active_route_.size()) active_route_.resize(max_forward_waypoints_);
    if (return_home_enabled_) active_route_.insert(active_route_.end(), return_waypoints_.begin(), return_waypoints_.end());
    RCLCPP_INFO(get_logger(), "Loaded mission YAML: forward=%zu active=%zu return_enabled=%s nav=%s manipulation=%s", forward_waypoints_.size(), active_route_.size(), return_home_enabled_ ? "true" : "false", navigation_backend_.c_str(), manipulation_backend_.c_str());
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to load route YAML '%s': %s", path.c_str(), e.what());
    return false;
  }
}

void MissionManagerNode::reset_route_flow() {
  active_waypoint_index_ = 0;
  active_job_index_ = 0;
  route_stage_ = RouteStage::Idle;
  route_stage_enter_time_ = now();
  navigation_start_requested_ = false;
  manipulation_start_requested_ = false;
  active_manipulation_request_id_.clear();
}

void MissionManagerNode::cancel_backends(const std::string& reason) {
  if (navigation_cancel_client_ && navigation_cancel_client_->service_is_ready()) {
    auto req = std::make_shared<atlas_mission_interfaces::srv::CancelNavigation::Request>();
    req->reason = reason;
    (void)navigation_cancel_client_->async_send_request(req);
  }
  if (manipulation_cancel_client_ && manipulation_cancel_client_->service_is_ready()) {
    auto req = std::make_shared<atlas_mission_interfaces::srv::CancelManipulation::Request>();
    req->reason = reason;
    (void)manipulation_cancel_client_->async_send_request(req);
  }
}

void MissionManagerNode::handle_route_flow(const rclcpp::Time& current_time) {
  if (active_route_.empty()) { route_failed(error::kRouteConfigError, "active route is empty"); return; }
  if (route_stage_ == RouteStage::Idle) { route_stage_ = RouteStage::StartPreMove; route_stage_enter_time_ = current_time; }
  switch (route_stage_) {
    case RouteStage::StartPreMove:
      start_pre_move_for_current_waypoint(current_time);
      break;
    case RouteStage::WaitPreMove: {
      safety_controller_->enter_safe_stop("waiting pre-move manipulation");
      atlas_mission_interfaces::msg::ManipulationStatus status;
      bool available = false;
      { std::lock_guard<std::mutex> lock(backend_status_mutex_); status = latest_manipulation_status_; available = manipulation_status_available_; }
      if (available && status.waypoint_id == active_manipulation_request_id_) {
        if (status.state == atlas_mission_interfaces::msg::ManipulationStatus::STATE_SUCCEEDED) {
          route_stage_ = RouteStage::StartNavigation;
          route_stage_enter_time_ = current_time;
        } else if (status.state == atlas_mission_interfaces::msg::ManipulationStatus::STATE_FAILED || status.state == atlas_mission_interfaces::msg::ManipulationStatus::STATE_CANCELLED) {
          route_failed(status.error_code ? status.error_code : error::kTaskFlowUnavailable, "pre-move manipulation failed: " + status.message);
        }
      }
      break;
    }
    case RouteStage::StartNavigation:
      start_navigation_for_current_waypoint(current_time);
      break;
    case RouteStage::WaitNavigation: {
      safety_controller_->allow_motion("navigation backend owns cmd_vel");
      atlas_mission_interfaces::msg::NavigationStatus status;
      bool available = false;
      { std::lock_guard<std::mutex> lock(backend_status_mutex_); status = latest_navigation_status_; available = navigation_status_available_; }
      const auto& wp = active_route_[active_waypoint_index_];
      if (available && status.waypoint_id == wp.id) {
        if (status.state == atlas_mission_interfaces::msg::NavigationStatus::STATE_SUCCEEDED) {
          safety_controller_->enter_safe_stop("waypoint reached");
          active_job_index_ = 0;
          if (wp.arrival_jobs.empty()) {
            advance_job_or_waypoint(current_time);
          } else {
            route_stage_ = RouteStage::StartManipulation;
            route_stage_enter_time_ = current_time;
          }
        } else if (status.state == atlas_mission_interfaces::msg::NavigationStatus::STATE_FAILED || status.state == atlas_mission_interfaces::msg::NavigationStatus::STATE_CANCELLED) {
          route_failed(status.error_code ? status.error_code : error::kTaskFlowUnavailable, "navigation failed: " + status.message);
        }
      }
      break;
    }
    case RouteStage::StartManipulation:
      start_manipulation_for_current_job(current_time);
      break;
    case RouteStage::WaitManipulation: {
      safety_controller_->enter_safe_stop("waiting manipulation backend");
      atlas_mission_interfaces::msg::ManipulationStatus status;
      bool available = false;
      { std::lock_guard<std::mutex> lock(backend_status_mutex_); status = latest_manipulation_status_; available = manipulation_status_available_; }
      if (available && status.waypoint_id == active_manipulation_request_id_) {
        if (status.state == atlas_mission_interfaces::msg::ManipulationStatus::STATE_SUCCEEDED) {
          advance_job_or_waypoint(current_time);
        } else if (status.state == atlas_mission_interfaces::msg::ManipulationStatus::STATE_FAILED || status.state == atlas_mission_interfaces::msg::ManipulationStatus::STATE_CANCELLED) {
          route_failed(status.error_code ? status.error_code : error::kTaskFlowUnavailable, "manipulation failed: " + status.message);
        }
      }
      break;
    }
    case RouteStage::Completed:
      notify_task_flow_succeeded("all route waypoints completed");
      break;
    case RouteStage::Failed:
    case RouteStage::Idle:
    default:
      break;
  }
}

void MissionManagerNode::start_pre_move_for_current_waypoint(const rclcpp::Time& current_time) {
  if (active_waypoint_index_ >= active_route_.size()) { route_stage_ = RouteStage::Completed; return; }
  const auto& wp = active_route_[active_waypoint_index_];
  if (wp.pre_move_action.empty() || wp.pre_move_action == "noop") {
    route_stage_ = RouteStage::StartNavigation;
    route_stage_enter_time_ = current_time;
    return;
  }
  if (!manipulation_start_client_->service_is_ready()) { route_failed(error::kTaskFlowUnavailable, "manipulation start service unavailable for pre-move action"); return; }
  auto req = std::make_shared<atlas_mission_interfaces::srv::StartManipulation::Request>();
  req->backend = manipulation_backend_;
  active_manipulation_request_id_ = wp.id + "__pre_move";
  req->waypoint_id = active_manipulation_request_id_;
  req->prepare_action = wp.pre_move_action;
  req->arrival_task = "prepare_only";
  {
    std::lock_guard<std::mutex> lock(backend_status_mutex_);
    manipulation_status_available_ = false;
  }
  (void)manipulation_start_client_->async_send_request(req);
  manipulation_start_requested_ = true;
  route_stage_ = RouteStage::WaitPreMove;
  route_stage_enter_time_ = current_time;
  RCLCPP_INFO(get_logger(), "Starting pre-move action waypoint=%s request=%s prepare=%s", wp.id.c_str(), active_manipulation_request_id_.c_str(), wp.pre_move_action.c_str());
}

void MissionManagerNode::start_navigation_for_current_waypoint(const rclcpp::Time& current_time) {
  if (active_waypoint_index_ >= active_route_.size()) { route_stage_ = RouteStage::Completed; return; }
  if (!navigation_start_client_->service_is_ready()) { route_failed(error::kTaskFlowUnavailable, "navigation start service unavailable"); return; }
  const auto& wp = active_route_[active_waypoint_index_];
  auto req = std::make_shared<atlas_mission_interfaces::srv::StartNavigation::Request>();
  req->backend = navigation_backend_;
  req->waypoint_id = wp.id;
  req->x_m = wp.x;
  req->y_m = wp.y;
  req->yaw_rad = wp.yaw;
  req->reset_origin = active_waypoint_index_ == 0;
  req->timeout_s = wp.timeout_s;
  {
    std::lock_guard<std::mutex> lock(backend_status_mutex_);
    navigation_status_available_ = false;
  }
  (void)navigation_start_client_->async_send_request(req);
  navigation_start_requested_ = true;
  route_stage_ = RouteStage::WaitNavigation;
  route_stage_enter_time_ = current_time;
  RCLCPP_INFO(get_logger(), "Starting navigation waypoint=%s x=%.3f y=%.3f yaw=%.3f", wp.id.c_str(), wp.x, wp.y, wp.yaw);
}

void MissionManagerNode::start_manipulation_for_current_job(const rclcpp::Time& current_time) {
  if (active_waypoint_index_ >= active_route_.size()) { route_stage_ = RouteStage::Completed; return; }
  const auto& wp = active_route_[active_waypoint_index_];
  if (active_job_index_ >= wp.arrival_jobs.size()) { advance_job_or_waypoint(current_time); return; }
  if (!manipulation_start_client_->service_is_ready()) { route_failed(error::kTaskFlowUnavailable, "manipulation start service unavailable"); return; }
  const auto& job = wp.arrival_jobs[active_job_index_];
  auto req = std::make_shared<atlas_mission_interfaces::srv::StartManipulation::Request>();
  req->backend = manipulation_backend_;
  active_manipulation_request_id_ = job.id;
  req->waypoint_id = active_manipulation_request_id_;
  req->prepare_action = job.prepare_action;
  req->arrival_task = job.arrival_task;
  {
    std::lock_guard<std::mutex> lock(backend_status_mutex_);
    manipulation_status_available_ = false;
  }
  (void)manipulation_start_client_->async_send_request(req);
  manipulation_start_requested_ = true;
  route_stage_ = RouteStage::WaitManipulation;
  route_stage_enter_time_ = current_time;
  RCLCPP_INFO(get_logger(), "Starting manipulation waypoint=%s job=%s prepare=%s task=%s", wp.id.c_str(), job.id.c_str(), job.prepare_action.c_str(), job.arrival_task.c_str());
}

void MissionManagerNode::advance_job_or_waypoint(const rclcpp::Time& current_time) {
  if (active_waypoint_index_ >= active_route_.size()) { route_stage_ = RouteStage::Completed; return; }
  const auto& wp = active_route_[active_waypoint_index_];
  if (active_job_index_ + 1u < wp.arrival_jobs.size()) {
    ++active_job_index_;
    manipulation_start_requested_ = false;
    active_manipulation_request_id_.clear();
    route_stage_ = RouteStage::StartManipulation;
    route_stage_enter_time_ = current_time;
    return;
  }
  ++active_waypoint_index_;
  active_job_index_ = 0;
  navigation_start_requested_ = false;
  manipulation_start_requested_ = false;
  active_manipulation_request_id_.clear();
  if (active_waypoint_index_ >= active_route_.size()) {
    route_stage_ = RouteStage::Completed;
    route_stage_enter_time_ = current_time;
  } else {
    route_stage_ = RouteStage::StartPreMove;
    route_stage_enter_time_ = current_time;
  }
}

void MissionManagerNode::route_failed(const std::int32_t code, const std::string& message) {
  route_stage_ = RouteStage::Failed;
  context_.last_error_code = code;
  context_.last_error_message = message;
  notify_task_flow_failed(code, message);
}

void MissionManagerNode::notify_task_flow_succeeded(const std::string& message) {
  if (context_.state != MissionState::Running) return;
  begin_final_report(FinalResultKind::Done, 0, message);
}

void MissionManagerNode::notify_task_flow_failed(const std::int32_t error_code, const std::string& message) {
  if (context_.state != MissionState::Running) return;
  context_.last_error_code = error_code;
  context_.last_error_message = message;
  begin_final_report(FinalResultKind::Fail, static_cast<std::int16_t>(std::clamp(error_code, static_cast<std::int32_t>(std::numeric_limits<std::int16_t>::min()), static_cast<std::int32_t>(std::numeric_limits<std::int16_t>::max()))), message);
}

void MissionManagerNode::publish_mission_status(const McuStateSnapshot& mcu, const rclcpp::Time& current_time, const bool force) {
  const double period_s = 1.0 / status_publish_rate_hz_;
  if (!force && (current_time - last_status_publish_time_).seconds() < period_s) return;
  atlas_mission_interfaces::msg::MissionStatus msg;
  msg.header.stamp = current_time;
  msg.state = static_cast<std::uint8_t>(context_.state);
  msg.local_run_id = context_.local_run_id;
  msg.active = context_.active;
  msg.mcu_status_fresh = mcu.available && mcu.fresh;
  msg.auto_start_latched = mcu.available && mcu.auto_start_latched;
  msg.mcu_app_state = mcu.available ? mcu.app_state : 0u;
  msg.result_reported = context_.result_reported;
  msg.error_code = context_.last_error_code;
  msg.state_name = mission_state_name(context_.state);
  std::ostringstream stream;
  if (!context_.last_error_message.empty()) stream << context_.last_error_message;
  else stream << status_message_;
  if (context_.state == MissionState::Running) {
    stream << " route_stage=" << route_stage_name(route_stage_);
    if (active_waypoint_index_ < active_route_.size()) stream << " waypoint=" << active_route_[active_waypoint_index_].id;
    stream << " nav=" << navigation_backend_ << " manipulation=" << manipulation_backend_;
  }
  msg.message = stream.str();
  mission_status_publisher_->publish(msg);
  last_status_publish_time_ = current_time;
}

bool MissionManagerNode::take_pending_start(rclcpp::Time& event_time) {
  std::lock_guard<std::mutex> lock(event_mutex_);
  if (!pending_start_) return false;
  event_time = pending_start_time_;
  pending_start_ = false;
  return true;
}

bool MissionManagerNode::has_pending_start(rclcpp::Time& event_time) const {
  std::lock_guard<std::mutex> lock(event_mutex_);
  if (!pending_start_) return false;
  event_time = pending_start_time_;
  return true;
}

bool MissionManagerNode::take_pending_reset() {
  std::lock_guard<std::mutex> lock(event_mutex_);
  if (!pending_reset_) return false;
  pending_reset_ = false;
  return true;
}

bool MissionManagerNode::state_requires_auto_pi(const MissionState state) const noexcept {
  return state == MissionState::Precheck || state == MissionState::Initializing || state == MissionState::Running || state == MissionState::ReportingDone || state == MissionState::ReportingFail;
}

bool MissionManagerNode::state_is_active_lifecycle(const MissionState state) const noexcept {
  return state == MissionState::Precheck || state == MissionState::Initializing || state == MissionState::Running || state == MissionState::ReportingDone || state == MissionState::WaitMcuFinished || state == MissionState::ReportingFail || state == MissionState::WaitMcuFault;
}

bool MissionManagerNode::is_reset_confirmed(const McuStateSnapshot& mcu) const noexcept {
  return mcu.available && mcu.fresh && !mcu.auto_start_latched && !is_auto_pi(mcu) && !is_fault(mcu) && !is_estop(mcu);
}

}  // namespace atlas_mission_manager
