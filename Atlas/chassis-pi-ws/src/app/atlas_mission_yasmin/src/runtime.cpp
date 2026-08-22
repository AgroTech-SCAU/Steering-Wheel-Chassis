// Copyright 2026 yangxuan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "atlas_mission_yasmin/runtime.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <utility>

#include "yaml-cpp/yaml.h"

namespace atlas_mission_yasmin
{

namespace
{

template<typename T>
T yaml_value(const YAML::Node & node, const char * key, const T & default_value)
{
  if (!node || !node[key]) {
    return default_value;
  }
  return node[key].as<T>();
}

Waypoint parse_waypoint(const YAML::Node & node)
{
  Waypoint waypoint;
  waypoint.id = yaml_value<std::string>(node, "id", "");
  waypoint.x_m = yaml_value<double>(node, "x_m", 0.0);
  waypoint.y_m = yaml_value<double>(node, "y_m", 0.0);
  waypoint.yaw_rad = yaml_value<double>(node, "yaw_rad", 0.0);
  waypoint.pre_move_action = yaml_value<std::string>(node, "pre_move_action", "");
  waypoint.timeout_s = yaml_value<double>(node, "timeout_s", 0.0);

  const auto jobs = node["arrival_jobs"];
  if (jobs && jobs.IsSequence()) {
    for (const auto job_node : jobs) {
      Job job;
      if (job_node.IsScalar()) {
        job.task_id = job_node.as<std::string>();
      } else {
        job.task_id = yaml_value<std::string>(job_node, "task_id", "");
      }
      waypoint.arrival_jobs.push_back(job);
    }
  }

  return waypoint;
}

bool is_navigation_terminal(const uint8_t state)
{
  using atlas_mission_interfaces::msg::NavigationStatus;
  return state == NavigationStatus::STATE_SUCCEEDED ||
         state == NavigationStatus::STATE_FAILED ||
         state == NavigationStatus::STATE_CANCELLED;
}

bool is_manipulation_terminal(const uint8_t state)
{
  using atlas_mission_interfaces::msg::ManipulationStatus;
  return state == ManipulationStatus::STATE_SUCCEEDED ||
         state == ManipulationStatus::STATE_FAILED ||
         state == ManipulationStatus::STATE_CANCELLED;
}

}  // namespace

Runtime::Runtime(const rclcpp::NodeOptions & options)
: rclcpp::Node("atlas_mission_yasmin", options)
{
  config_ = load_config();
  if (!config_.route_yaml_path.empty()) {
    try {
      plan_ = load_plan(config_.route_yaml_path);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "failed to load route yaml: %s", error.what());
    }
  }
  configure_ros_interfaces();
}

Plan Runtime::load_plan(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  Plan plan;
  if (!root || root.IsNull()) {
    return plan;
  }

  plan.navigation_backend = yaml_value<std::string>(root, "navigation_backend", "");
  plan.manipulation_backend = yaml_value<std::string>(root, "manipulation_backend", "");
  plan.return_home_enabled = yaml_value<bool>(root, "return_home_enabled", false);

  const auto waypoints = root["waypoints"];
  if (waypoints && waypoints.IsSequence()) {
    for (const auto waypoint : waypoints) {
      plan.waypoints.push_back(parse_waypoint(waypoint));
    }
  }

  const auto return_waypoints = root["return_waypoints"];
  if (return_waypoints && return_waypoints.IsSequence()) {
    for (const auto waypoint : return_waypoints) {
      plan.return_waypoints.push_back(parse_waypoint(waypoint));
    }
  }

  return plan;
}

const Plan & Runtime::plan() const
{
  return plan_;
}

Runtime::RuntimeConfig Runtime::load_config()
{
  RuntimeConfig config;
  config.route_yaml_path = declare_parameter<std::string>("route_yaml_path", "");
  config.mcu_status_timeout_s = declare_parameter<double>("mcu_status_timeout_s", 1.0);
  config.service_timeout_s = declare_parameter<double>("service_timeout_s", 3.0);
  config.backend_grace_s = declare_parameter<double>("backend_grace_s", 0.2);
  config.result_confirm_timeout_s =
    declare_parameter<double>("result_confirm_timeout_s", 30.0);
  config.required_ready_mask = declare_parameter<int64_t>("required_ready_mask", 0);
  config.navigation_result_timeout_s =
    declare_parameter<double>("navigation_result_timeout_s", 60.0);
  config.manipulation_result_timeout_s =
    declare_parameter<double>("manipulation_result_timeout_s", 30.0);
  return config;
}

void Runtime::configure_ros_interfaces()
{
  const auto mcu_status_topic =
    declare_parameter<std::string>("topics.mcu_status", "/mcu/status");
  const auto auto_task_event_topic =
    declare_parameter<std::string>("topics.mcu_auto_task_event", "/mcu/auto_task_event");
  const auto navigation_status_topic =
    declare_parameter<std::string>("topics.navigation_status", "/atlas/navigation/status");
  const auto navigation_cmd_vel_topic =
    declare_parameter<std::string>("topics.navigation_cmd_vel", "/atlas/navigation/cmd_vel");
  const auto manipulation_status_topic =
    declare_parameter<std::string>("topics.manipulation_status", "/atlas/manipulation/status");
  const auto mission_status_topic =
    declare_parameter<std::string>("topics.mission_status", "/atlas/mission/status");
  const auto motor_cmd_vel_topic =
    declare_parameter<std::string>("topics.motor_cmd_vel", "/motor_cmd_vel");

  const auto report_mission_result_service =
    declare_parameter<std::string>("services.report_mission_result", "/mcu/report_mission_result");
  const auto set_brake_service =
    declare_parameter<std::string>("services.set_brake", "/mcu/set_brake");
  const auto navigation_start_service =
    declare_parameter<std::string>("services.navigation_start", "/atlas/navigation/start");
  const auto navigation_cancel_service =
    declare_parameter<std::string>("services.navigation_cancel", "/atlas/navigation/cancel");
  const auto manipulation_start_service =
    declare_parameter<std::string>("services.manipulation_start", "/atlas/manipulation/start");
  const auto manipulation_cancel_service =
    declare_parameter<std::string>("services.manipulation_cancel", "/atlas/manipulation/cancel");

  mcu_sub_ = create_subscription<McuStatus>(
    mcu_status_topic,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&Runtime::handle_mcu_status, this, std::placeholders::_1));
  auto_task_event_sub_ = create_subscription<AutoTaskEvent>(
    auto_task_event_topic,
    rclcpp::QoS(10),
    std::bind(&Runtime::handle_auto_task_event, this, std::placeholders::_1));
  navigation_status_sub_ = create_subscription<NavigationStatus>(
    navigation_status_topic,
    rclcpp::QoS(10),
    std::bind(&Runtime::handle_navigation_status, this, std::placeholders::_1));
  navigation_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    navigation_cmd_vel_topic,
    rclcpp::QoS(10),
    std::bind(&Runtime::handle_navigation_cmd_vel, this, std::placeholders::_1));
  manipulation_status_sub_ = create_subscription<ManipulationStatus>(
    manipulation_status_topic,
    rclcpp::QoS(10),
    std::bind(&Runtime::handle_manipulation_status, this, std::placeholders::_1));

  motor_pub_ = create_publisher<geometry_msgs::msg::Twist>(motor_cmd_vel_topic, 10);
  mission_status_pub_ = create_publisher<MissionStatus>(mission_status_topic, 10);

  nav_start_ =
    create_client<atlas_mission_interfaces::srv::StartNavigation>(navigation_start_service);
  nav_cancel_ =
    create_client<atlas_mission_interfaces::srv::CancelNavigation>(navigation_cancel_service);
  manip_start_ =
    create_client<atlas_mission_interfaces::srv::StartManipulation>(manipulation_start_service);
  manip_cancel_ =
    create_client<atlas_mission_interfaces::srv::CancelManipulation>(manipulation_cancel_service);
  result_client_ =
    create_client<mcu_comm_bridge::srv::ReportMissionResult>(report_mission_result_service);
  brake_client_ = create_client<std_srvs::srv::SetBool>(set_brake_service);
}

void Runtime::handle_mcu_status(McuStatus::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mcu_received_ = true;
    mcu_ = *msg;
    mcu_receive_time_ = Clock::now();
  }
  condition_.notify_all();
}

void Runtime::handle_auto_task_event(AutoTaskEvent::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->event == AutoTaskEvent::EVENT_START) {
      start_event_ = true;
    }
    if (msg->event == AutoTaskEvent::EVENT_RESET) {
      reset_event_ = true;
    }
  }
  condition_.notify_all();
}

void Runtime::handle_navigation_status(NavigationStatus::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_navigation_status_ = *msg;
  }
  condition_.notify_all();
}

void Runtime::handle_manipulation_status(ManipulationStatus::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_manipulation_status_ = *msg;
  }
  condition_.notify_all();
}

void Runtime::handle_navigation_cmd_vel(geometry_msgs::msg::Twist::SharedPtr msg)
{
  const auto now = Clock::now();
  const bool allowed = [&]() {
      std::lock_guard<std::mutex> lock(mutex_);
      return can_move_locked(now);
    }();

  if (allowed) {
    motor_pub_->publish(*msg);
  } else {
    publish_motor_zero();
  }
}

bool Runtime::mcu_fresh_locked(const Clock::time_point now) const
{
  if (!mcu_received_) {
    return false;
  }
  const auto age = std::chrono::duration<double>(now - mcu_receive_time_).count();
  return age <= config_.mcu_status_timeout_s;
}

GuardResult Runtime::guard_locked(const Clock::time_point now) const
{
  if (!rclcpp::ok()) {
    return GuardResult::kShutdown;
  }
  if (reset_event_) {
    return GuardResult::kReset;
  }
  if (!mcu_fresh_locked(now)) {
    return GuardResult::kRecovery;
  }
  if (mcu_.app_state == McuStatus::STATE_FAULT ||
    mcu_.app_state == McuStatus::STATE_ESTOP)
  {
    return GuardResult::kRecovery;
  }
  if (mcu_.app_state != McuStatus::STATE_AUTO_PI) {
    return GuardResult::kRecovery;
  }
  if (config_.required_ready_mask != 0) {
    const auto required = static_cast<uint8_t>(config_.required_ready_mask & 0xff);
    if ((mcu_.ready_flags & required) != required) {
      return GuardResult::kRecovery;
    }
  }
  return GuardResult::kOk;
}

bool Runtime::can_move_locked(const Clock::time_point now) const
{
  return active_ && motion_enabled_ && guard_locked(now) == GuardResult::kOk;
}

bool Runtime::mcu_fresh() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mcu_fresh_locked(Clock::now());
}

GuardResult Runtime::guard() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return guard_locked(Clock::now());
}

bool Runtime::can_move() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return can_move_locked(Clock::now());
}

bool Runtime::wait_start()
{
  std::unique_lock<std::mutex> lock(mutex_);
  return condition_.wait_for(
    lock, std::chrono::milliseconds(100), [&]() {
      return start_event_ || reset_event_ || !rclcpp::ok();
    }) && start_event_;
}

void Runtime::begin_run()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = true;
  motion_enabled_ = false;
  start_event_ = false;
  reset_event_ = false;
  result_reported_ = false;
  error_code_ = 0;
  ++local_run_id_;
  mission_state_ = MissionStatus::STATE_RUNNING;
  mission_state_name_ = "RUNNING";
  mission_message_.clear();
  publish_mission_status_locked();
}

void Runtime::clear_run()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  motion_enabled_ = false;
  start_event_ = false;
  reset_event_ = false;
  result_reported_ = false;
  last_navigation_status_.reset();
  last_manipulation_status_.reset();
  publish_mission_status_locked();
}

void Runtime::set_state(
  const uint8_t state,
  const std::string & name,
  const std::string & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  mission_state_ = state;
  mission_state_name_ = name;
  mission_message_ = message;
  publish_mission_status_locked();
}

void Runtime::set_error(const int32_t error_code, const std::string & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  error_code_ = error_code;
  mission_message_ = message;
  publish_mission_status_locked();
}

bool Runtime::report_done()
{
  auto request = std::make_shared<mcu_comm_bridge::srv::ReportMissionResult::Request>();
  request->result = mcu_comm_bridge::srv::ReportMissionResult::Request::RESULT_DONE;
  request->code = 0;
  if (!result_client_->wait_for_service(std::chrono::duration<double>(config_.service_timeout_s))) {
    return false;
  }
  auto future = result_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(config_.service_timeout_s)) !=
    std::future_status::ready)
  {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  result_reported_ = future.get()->success;
  publish_mission_status_locked();
  return result_reported_;
}

bool Runtime::report_fail(const int16_t code)
{
  auto request = std::make_shared<mcu_comm_bridge::srv::ReportMissionResult::Request>();
  request->result = mcu_comm_bridge::srv::ReportMissionResult::Request::RESULT_FAIL;
  request->code = code;
  if (!result_client_->wait_for_service(std::chrono::duration<double>(config_.service_timeout_s))) {
    return false;
  }
  auto future = result_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(config_.service_timeout_s)) !=
    std::future_status::ready)
  {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  result_reported_ = future.get()->success;
  publish_mission_status_locked();
  return result_reported_;
}

bool Runtime::request_brake(const bool enabled)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  if (!brake_client_->wait_for_service(std::chrono::duration<double>(config_.service_timeout_s))) {
    return false;
  }
  auto future = brake_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(config_.service_timeout_s)) !=
    std::future_status::ready)
  {
    return false;
  }
  return future.get()->success;
}

ActionResult Runtime::run_navigation(const Waypoint & waypoint)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_navigation_status_.reset();
    motion_enabled_ = false;
  }

  if (guard() != GuardResult::kOk) {
    return ActionResult::kRecovery;
  }
  if (!nav_start_->wait_for_service(std::chrono::duration<double>(config_.service_timeout_s))) {
    return ActionResult::kTimeout;
  }

  auto request = std::make_shared<atlas_mission_interfaces::srv::StartNavigation::Request>();
  request->backend = plan_.navigation_backend;
  request->waypoint_id = waypoint.id;
  request->x_m = waypoint.x_m;
  request->y_m = waypoint.y_m;
  request->yaw_rad = waypoint.yaw_rad;
  request->reset_origin = false;
  request->timeout_s = waypoint.timeout_s;

  auto future = nav_start_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(config_.service_timeout_s)) !=
    std::future_status::ready)
  {
    call_cancel_navigation("navigation start service timeout");
    return ActionResult::kTimeout;
  }
  if (!future.get()->success) {
    return ActionResult::kRejected;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    motion_enabled_ = true;
  }

  const double timeout_s =
    waypoint.timeout_s > 0.0 ? waypoint.timeout_s : config_.navigation_result_timeout_s;
  const auto result = wait_navigation_terminal(waypoint, timeout_s);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    motion_enabled_ = false;
  }
  safe_stop("navigation terminal");
  if (result != ActionResult::kSucceeded) {
    call_cancel_navigation("navigation interrupted");
  }
  return result;
}

ActionResult Runtime::run_pre_move(const Waypoint & waypoint)
{
  if (waypoint.pre_move_action.empty()) {
    return ActionResult::kSucceeded;
  }
  Job pre_move;
  pre_move.task_id = waypoint.pre_move_action;
  return run_job(waypoint, pre_move);
}

ActionResult Runtime::run_job(const Waypoint & waypoint, const Job & job)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_manipulation_status_.reset();
  }

  if (guard() != GuardResult::kOk) {
    return ActionResult::kRecovery;
  }
  if (!manip_start_->wait_for_service(std::chrono::duration<double>(config_.service_timeout_s))) {
    return ActionResult::kTimeout;
  }

  auto request = std::make_shared<atlas_mission_interfaces::srv::StartManipulation::Request>();
  request->backend = plan_.manipulation_backend;
  request->waypoint_id = waypoint.id;
  request->prepare_action = waypoint.pre_move_action;
  request->arrival_task = job.task_id;

  auto future = manip_start_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(config_.service_timeout_s)) !=
    std::future_status::ready)
  {
    call_cancel_manipulation("manipulation start service timeout");
    return ActionResult::kTimeout;
  }
  if (!future.get()->success) {
    return ActionResult::kRejected;
  }

  const auto result =
    wait_manipulation_terminal(waypoint.id, job.task_id, config_.manipulation_result_timeout_s);
  if (result != ActionResult::kSucceeded) {
    call_cancel_manipulation("manipulation interrupted");
  }
  return result;
}

bool Runtime::cancel_navigation(const std::string & reason)
{
  return call_cancel_navigation(reason);
}

bool Runtime::cancel_manipulation(const std::string & reason)
{
  return call_cancel_manipulation(reason);
}

bool Runtime::call_cancel_navigation(const std::string & reason)
{
  if (!nav_cancel_->wait_for_service(std::chrono::duration<double>(config_.service_timeout_s))) {
    return false;
  }
  auto request = std::make_shared<atlas_mission_interfaces::srv::CancelNavigation::Request>();
  request->reason = reason;
  auto future = nav_cancel_->async_send_request(request);
  return future.wait_for(std::chrono::duration<double>(config_.service_timeout_s)) ==
         std::future_status::ready && future.get()->success;
}

bool Runtime::call_cancel_manipulation(const std::string & reason)
{
  if (!manip_cancel_->wait_for_service(std::chrono::duration<double>(config_.service_timeout_s))) {
    return false;
  }
  auto request = std::make_shared<atlas_mission_interfaces::srv::CancelManipulation::Request>();
  request->reason = reason;
  auto future = manip_cancel_->async_send_request(request);
  return future.wait_for(std::chrono::duration<double>(config_.service_timeout_s)) ==
         std::future_status::ready && future.get()->success;
}

ActionResult Runtime::wait_navigation_terminal(const Waypoint & waypoint, const double timeout_s)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_s);
  std::unique_lock<std::mutex> lock(mutex_);
  while (Clock::now() < deadline) {
    const auto guard_result = guard_locked(Clock::now());
    if (guard_result == GuardResult::kReset) {
      return ActionResult::kReset;
    }
    if (guard_result == GuardResult::kRecovery) {
      return ActionResult::kRecovery;
    }
    if (guard_result == GuardResult::kShutdown) {
      return ActionResult::kShutdown;
    }
    if (last_navigation_status_ &&
      last_navigation_status_->waypoint_id == waypoint.id &&
      is_navigation_terminal(last_navigation_status_->state))
    {
      if (last_navigation_status_->state == NavigationStatus::STATE_SUCCEEDED) {
        return ActionResult::kSucceeded;
      }
      return ActionResult::kFailed;
    }
    condition_.wait_until(lock, deadline);
  }
  return ActionResult::kTimeout;
}

ActionResult Runtime::wait_manipulation_terminal(
  const std::string & waypoint_id,
  const std::string & task_id,
  const double timeout_s)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_s);
  std::unique_lock<std::mutex> lock(mutex_);
  while (Clock::now() < deadline) {
    const auto guard_result = guard_locked(Clock::now());
    if (guard_result == GuardResult::kReset) {
      return ActionResult::kReset;
    }
    if (guard_result == GuardResult::kRecovery) {
      return ActionResult::kRecovery;
    }
    if (guard_result == GuardResult::kShutdown) {
      return ActionResult::kShutdown;
    }
    if (last_manipulation_status_ &&
      last_manipulation_status_->waypoint_id == waypoint_id &&
      last_manipulation_status_->task_id == task_id &&
      is_manipulation_terminal(last_manipulation_status_->state))
    {
      if (last_manipulation_status_->state == ManipulationStatus::STATE_SUCCEEDED) {
        return ActionResult::kSucceeded;
      }
      return ActionResult::kFailed;
    }
    condition_.wait_until(lock, deadline);
  }
  return ActionResult::kTimeout;
}

void Runtime::safe_stop(const std::string & reason)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    motion_enabled_ = false;
    ++safe_stop_count_;
  }
  publish_motor_zero();
  RCLCPP_WARN(get_logger(), "safe_stop: %s", reason.c_str());
}

void Runtime::publish_mission_status_locked()
{
  if (!mission_status_pub_) {
    return;
  }
  MissionStatus status;
  status.header.stamp = now();
  status.state = mission_state_;
  status.local_run_id = local_run_id_;
  status.active = active_;
  status.mcu_status_fresh = mcu_fresh_locked(Clock::now());
  status.auto_start_latched = start_event_ || mcu_.auto_start_latched;
  status.mcu_app_state = mcu_.app_state;
  status.result_reported = result_reported_;
  status.error_code = error_code_;
  status.state_name = mission_state_name_;
  status.message = mission_message_;
  mission_status_pub_->publish(status);
}

void Runtime::publish_motor_zero()
{
  if (!motor_pub_) {
    return;
  }
  geometry_msgs::msg::Twist zero;
  motor_pub_->publish(zero);
}

void Runtime::inject_mcu_status_for_test(const McuStatus & status)
{
  handle_mcu_status(std::make_shared<McuStatus>(status));
}

void Runtime::inject_auto_task_event_for_test(const AutoTaskEvent & event)
{
  handle_auto_task_event(std::make_shared<AutoTaskEvent>(event));
}

void Runtime::set_mcu_status_timeout_for_test(const double timeout_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_.mcu_status_timeout_s = timeout_s;
}

void Runtime::set_motion_enabled_for_test(const bool enabled)
{
  std::lock_guard<std::mutex> lock(mutex_);
  motion_enabled_ = enabled;
}

std::size_t Runtime::safe_stop_count_for_test() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return safe_stop_count_;
}

}  // namespace atlas_mission_yasmin
