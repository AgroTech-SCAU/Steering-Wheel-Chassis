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

#include <chrono>
#include <future>
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

Waypoint parse_waypoint(const YAML::Node & node, const std::string & fallback_id)
{
  Waypoint waypoint;
  waypoint.id = fallback_id;
  waypoint.timeout_s = 30.0;

  if (!node) {
    return waypoint;
  }
  if (node.IsScalar()) {
    waypoint.id = node.as<std::string>();
    return waypoint;
  }

  waypoint.id = yaml_value<std::string>(node, "id", fallback_id);
  waypoint.timeout_s = yaml_value<double>(node, "timeout_s", 30.0);
  return waypoint;
}

bool navigation_terminal(const uint8_t state)
{
  using atlas_mission_interfaces::msg::NavigationStatus;
  return state == NavigationStatus::STATE_SUCCEEDED ||
         state == NavigationStatus::STATE_FAILED ||
         state == NavigationStatus::STATE_CANCELLED;
}

bool manipulation_terminal(const uint8_t state)
{
  using atlas_mission_interfaces::msg::ManipulationStatus;
  return state == ManipulationStatus::STATE_SUCCEEDED ||
         state == ManipulationStatus::STATE_FAILED ||
         state == ManipulationStatus::STATE_CANCELLED;
}

template<typename HeaderT>
bool status_is_current(
  const HeaderT & header,
  const std::optional<rclcpp::Time> & request_start)
{
  if (!request_start) {
    return true;
  }
  const rclcpp::Time stamp(header.stamp);
  return stamp.nanoseconds() == 0 || stamp.nanoseconds() >= request_start->nanoseconds();
}

bool observation_valid(const Observation & observation)
{
  return observation.result == ActionResult::kSucceeded &&
         observation.layer_ok && observation.complete;
}

}  // namespace

Runtime::Runtime(const rclcpp::NodeOptions & options)
: rclcpp::Node("atlas_mission_yasmin", options)
{
  config_ = load_config();
  if (!config_.route_yaml_path.empty()) {
    plan_ = load_plan(config_.route_yaml_path);
  }
  configure_ros_interfaces();
}

Plan Runtime::load_plan(const std::string & path)
{
  const YAML::Node file = YAML::LoadFile(path);
  const YAML::Node root = file["mission"] ? file["mission"] : file;
  if (!root || root.IsNull()) {
    throw std::runtime_error("mission route yaml is empty");
  }

  Plan plan;
  plan.navigation_backend = yaml_value<std::string>(root, "navigation_backend", "");
  plan.manipulation_backend = yaml_value<std::string>(root, "manipulation_backend", "");

  const auto waypoints = root["waypoints"];
  if (!waypoints || !waypoints.IsMap()) {
    throw std::runtime_error("mission.waypoints must be a map");
  }

  plan.pickup = parse_waypoint(waypoints["pickup"], "pickup");
  plan.park_1 = parse_waypoint(waypoints["park_1"], "park_1");
  plan.park_2 = parse_waypoint(waypoints["park_2"], "park_2");

  if (plan.pickup.id.empty() || plan.park_1.id.empty() || plan.park_2.id.empty()) {
    throw std::runtime_error("pickup, park_1 and park_2 waypoint ids are required");
  }
  return plan;
}

const Plan & Runtime::plan() const
{
  return plan_;
}

const Waypoint * Runtime::waypoint(const std::string & id) const
{
  if (id == "pickup") {
    return &plan_.pickup;
  }
  if (id == "park_1") {
    return &plan_.park_1;
  }
  if (id == "park_2") {
    return &plan_.park_2;
  }
  return nullptr;
}

CompetitionModel & Runtime::model()
{
  return model_;
}

const CompetitionModel & Runtime::model() const
{
  return model_;
}

Runtime::RuntimeConfig Runtime::load_config()
{
  RuntimeConfig config;
  config.route_yaml_path = declare_parameter<std::string>("route_yaml_path", "");
  config.mcu_status_timeout_s = declare_parameter<double>("mcu_status_timeout_s", 1.0);
  config.service_timeout_s = declare_parameter<double>("service_timeout_s", 3.0);
  config.navigation_result_timeout_s =
    declare_parameter<double>("navigation_result_timeout_s", 60.0);
  config.manipulation_result_timeout_s =
    declare_parameter<double>("manipulation_result_timeout_s", 30.0);
  config.required_ready_mask = declare_parameter<int64_t>("required_ready_mask", 0);
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

  const auto report_result_service =
    declare_parameter<std::string>("services.report_mission_result", "/mcu/report_mission_result");
  const auto set_brake_service =
    declare_parameter<std::string>("services.set_brake", "/mcu/set_brake");
  const auto navigation_start_service =
    declare_parameter<std::string>("services.navigation_start", "/atlas/navigation/start");
  const auto navigation_cancel_service =
    declare_parameter<std::string>("services.navigation_cancel", "/atlas/navigation/cancel");
  const auto navigation_view_scan_service =
    declare_parameter<std::string>("services.navigation_view_scan", "/atlas/navigation/view_scan");
  const auto manipulation_start_service =
    declare_parameter<std::string>("services.manipulation_start", "/atlas/manipulation/start");
  const auto manipulation_cancel_service =
    declare_parameter<std::string>("services.manipulation_cancel", "/atlas/manipulation/cancel");
  const auto classify_sorting_service =
    declare_parameter<std::string>(
    "services.classify_sorting",
    "/atlas/vision/classify_sorting_rule");
  const auto detect_target_service =
    declare_parameter<std::string>("services.detect_target", "/atlas/vision/detect_target");

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
  nav_view_scan_client_ = create_client<std_srvs::srv::SetBool>(navigation_view_scan_service);
  manip_start_ =
    create_client<atlas_mission_interfaces::srv::StartManipulation>(manipulation_start_service);
  manip_cancel_ =
    create_client<atlas_mission_interfaces::srv::CancelManipulation>(manipulation_cancel_service);
  classify_sorting_ =
    create_client<atlas_mission_interfaces::srv::ClassifySortingRule>(classify_sorting_service);
  detect_target_ =
    create_client<atlas_mission_interfaces::srv::DetectCameraTarget>(detect_target_service);
  result_client_ = create_client<mcu_comm_bridge::srv::ReportMissionResult>(report_result_service);
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
      auto_event_ = true;
    } else if (msg->event == AutoTaskEvent::EVENT_RESET) {
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
  return std::chrono::duration<double>(now - mcu_receive_time_).count() <=
         config_.mcu_status_timeout_s;
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
  if (mcu_.app_state == McuStatus::STATE_FAULT || mcu_.app_state == McuStatus::STATE_ESTOP) {
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

WaitResult Runtime::wait_mcu()
{
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait_for(lock, std::chrono::milliseconds(100));
  if (!rclcpp::ok()) {
    return WaitResult::kShutdown;
  }
  if (reset_event_) {
    return WaitResult::kReset;
  }
  return mcu_fresh_locked(Clock::now()) ? WaitResult::kSuccess : WaitResult::kRetry;
}

WaitResult Runtime::wait_auto()
{
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait_for(lock, std::chrono::milliseconds(100));
  if (!rclcpp::ok()) {
    return WaitResult::kShutdown;
  }
  if (reset_event_) {
    return WaitResult::kReset;
  }
  if (!(auto_event_ || mcu_.auto_start_latched)) {
    return WaitResult::kRetry;
  }
  return guard_locked(Clock::now()) == GuardResult::kOk ?
         WaitResult::kSuccess : WaitResult::kRecovery;
}

WaitResult Runtime::wait_reset()
{
  safe_stop("wait reset");
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait_for(lock, std::chrono::milliseconds(100));
  if (!rclcpp::ok()) {
    return WaitResult::kShutdown;
  }
  if (reset_event_) {
    reset_event_ = false;
    auto_event_ = false;
    active_ = false;
    motion_enabled_ = false;
    return WaitResult::kSuccess;
  }
  return WaitResult::kRetry;
}

void Runtime::begin_run()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = true;
  motion_enabled_ = false;
  auto_event_ = false;
  reset_event_ = false;
  result_reported_ = false;
  error_code_ = 0;
  ++local_run_id_;
  model_.reset();
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
  auto_event_ = false;
  result_reported_ = false;
  last_navigation_status_.reset();
  last_manipulation_status_.reset();
  navigation_request_start_.reset();
  manipulation_request_start_.reset();
  model_.reset();
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

SortingResult Runtime::inspect_sorting_zone()
{
  const auto arm_result = manipulate("sorting", "pre_recognition", 0, 0);
  if (arm_result != ActionResult::kSucceeded) {
    return SortingResult{arm_result, "", "", "", "pre-recognition failed"};
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (next_sorting_result_for_test_) {
      auto result = *next_sorting_result_for_test_;
      next_sorting_result_for_test_.reset();
      if (result.result == ActionResult::kSucceeded &&
        !model_.set_sorting_rule(result.arena, result.park_1_cargo, result.park_2_cargo))
      {
        result.result = ActionResult::kFailed;
      }
      return result;
    }
  }

  if (guard() != GuardResult::kOk) {
    return SortingResult{ActionResult::kRecovery, "", "", "", "guard rejected"};
  }
  if (!classify_sorting_->wait_for_service(
      std::chrono::duration<double>(config_.service_timeout_s)))
  {
    return SortingResult{ActionResult::kTimeout, "", "", "", "sorting service unavailable"};
  }

  auto request = std::make_shared<atlas_mission_interfaces::srv::ClassifySortingRule::Request>();
  auto future = classify_sorting_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(config_.service_timeout_s)) !=
    std::future_status::ready)
  {
    return SortingResult{ActionResult::kTimeout, "", "", "", "sorting service timeout"};
  }

  const auto response = future.get();
  SortingResult result;
  result.result = response->success ? ActionResult::kSucceeded : ActionResult::kFailed;
  result.arena = response->arena;
  result.park_1_cargo = response->park_1_cargo;
  result.park_2_cargo = response->park_2_cargo;
  result.message = response->message;

  if (result.result == ActionResult::kSucceeded &&
    !model_.set_sorting_rule(result.arena, result.park_1_cargo, result.park_2_cargo))
  {
    result.result = ActionResult::kFailed;
    result.message = "invalid arena or sorting rule";
  }
  return result;
}

ActionResult Runtime::navigate(const std::string & waypoint_id)
{
  if (model_.arena() != "A" && model_.arena() != "B") {
    return ActionResult::kFailed;
  }
  const auto * target = waypoint(waypoint_id);
  return target == nullptr ? ActionResult::kFailed : run_navigation_request(*target);
}

ActionResult Runtime::manipulate(
  const std::string & area,
  const std::string & task,
  const std::size_t slot,
  const uint8_t layer,
  const std::string & cargo_class)
{
  return run_manipulation_request(area, task, slot, layer, cargo_class);
}

Observation Runtime::observe_once(
  const std::string & area,
  const std::size_t slot,
  const uint8_t expected_layer)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (next_observation_for_test_) {
      auto result = *next_observation_for_test_;
      next_observation_for_test_.reset();
      return result;
    }
  }

  if (guard() != GuardResult::kOk) {
    return Observation{ActionResult::kRecovery, "", false, false, "guard rejected"};
  }
  if (!detect_target_->wait_for_service(std::chrono::duration<double>(config_.service_timeout_s))) {
    return Observation{ActionResult::kTimeout, "", false, false, "vision service unavailable"};
  }

  auto request = std::make_shared<atlas_mission_interfaces::srv::DetectCameraTarget::Request>();
  request->waypoint_id = area;
  request->task_id = "verify_layer";
  request->slot = static_cast<uint8_t>(slot);
  request->expected_layer = expected_layer;
  request->max_targets = 1;
  request->target_class = "";

  auto future = detect_target_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(config_.service_timeout_s)) !=
    std::future_status::ready)
  {
    return Observation{ActionResult::kTimeout, "", false, false, "vision service timeout"};
  }

  const auto response = future.get();
  Observation observation;
  observation.result = response->success ? ActionResult::kSucceeded : ActionResult::kFailed;
  observation.cargo_class = response->cargo_class;
  observation.layer_ok = response->layer_ok;
  observation.complete = response->complete;
  observation.message = response->message;
  return observation;
}

Observation Runtime::observe_with_recovery(
  const std::string & area,
  const std::size_t slot,
  const uint8_t expected_layer)
{
  auto action = manipulate(area, "pre_recognition", slot, expected_layer);
  if (action != ActionResult::kSucceeded) {
    return Observation{action, "", false, false, "pre-recognition failed"};
  }

  auto observation = observe_once(area, slot, expected_layer);
  if (observation_valid(observation)) {
    return observation;
  }

  action = manipulate(area, "view_scan", slot, expected_layer);
  if (action != ActionResult::kSucceeded) {
    return Observation{action, "", false, false, "arm view scan failed"};
  }
  observation = observe_once(area, slot, expected_layer);
  const auto restore_arm = manipulate(area, "pre_recognition", slot, expected_layer);
  if (restore_arm != ActionResult::kSucceeded) {
    return Observation{restore_arm, "", false, false, "arm restore failed"};
  }
  if (observation_valid(observation)) {
    return observation;
  }

  if (!set_navigation_view_scan(true)) {
    return Observation{ActionResult::kFailed, "", false, false, "base yaw scan failed"};
  }
  observation = observe_once(area, slot, expected_layer);
  const bool restored = set_navigation_view_scan(false);
  if (!restored) {
    return Observation{ActionResult::kRecovery, "", false, false, "base yaw restore failed"};
  }
  if (!observation_valid(observation)) {
    observation.result = ActionResult::kFailed;
  }
  return observation;
}

ActionResult Runtime::run_navigation_request(const Waypoint & waypoint)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_navigation_status_.reset();
    navigation_request_start_ = now();
    motion_enabled_ = false;
    if (next_navigation_result_for_test_) {
      const auto result = *next_navigation_result_for_test_;
      next_navigation_result_for_test_.reset();
      return result;
    }
  }

  if (guard() != GuardResult::kOk) {
    return ActionResult::kRecovery;
  }
  if (!nav_start_->wait_for_service(std::chrono::duration<double>(config_.service_timeout_s))) {
    return ActionResult::kTimeout;
  }

  auto request = std::make_shared<atlas_mission_interfaces::srv::StartNavigation::Request>();
  request->backend = plan_.navigation_backend;
  request->arena = model_.arena();
  request->waypoint_id = waypoint.id;
  request->x_m = 0.0;
  request->y_m = 0.0;
  request->yaw_rad = 0.0;
  request->reset_origin = false;
  request->timeout_s = waypoint.timeout_s;

  auto future = nav_start_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(config_.service_timeout_s)) !=
    std::future_status::ready)
  {
    call_cancel_navigation("navigation start timeout");
    return ActionResult::kTimeout;
  }
  if (!future.get()->success) {
    return ActionResult::kRejected;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    motion_enabled_ = true;
  }
  const auto result = wait_navigation_terminal(waypoint, waypoint.timeout_s);
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

ActionResult Runtime::run_manipulation_request(
  const std::string & area,
  const std::string & task,
  const std::size_t slot,
  const uint8_t layer,
  const std::string & cargo_class)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_manipulation_status_.reset();
    manipulation_request_start_ = now();
    if (next_manipulation_result_for_test_) {
      const auto result = *next_manipulation_result_for_test_;
      next_manipulation_result_for_test_.reset();
      return result;
    }
  }

  if (guard() != GuardResult::kOk) {
    return ActionResult::kRecovery;
  }
  if (!manip_start_->wait_for_service(std::chrono::duration<double>(config_.service_timeout_s))) {
    return ActionResult::kTimeout;
  }

  auto request = std::make_shared<atlas_mission_interfaces::srv::StartManipulation::Request>();
  request->backend = plan_.manipulation_backend;
  request->waypoint_id = area;
  request->prepare_action = task == "pre_recognition" ? "pre_recognition" : "";
  request->arrival_task = task;
  request->slot = static_cast<uint8_t>(slot);
  request->layer = layer;
  request->cargo_class = cargo_class;

  auto future = manip_start_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(config_.service_timeout_s)) !=
    std::future_status::ready)
  {
    call_cancel_manipulation("manipulation start timeout");
    return ActionResult::kTimeout;
  }
  if (!future.get()->success) {
    return ActionResult::kRejected;
  }

  const auto result = wait_manipulation_terminal(area, task, config_.manipulation_result_timeout_s);
  if (result != ActionResult::kSucceeded) {
    call_cancel_manipulation("manipulation interrupted");
  }
  return result;
}

bool Runtime::set_navigation_view_scan(const bool enabled)
{
  if (guard() != GuardResult::kOk) {
    return false;
  }
  if (!nav_view_scan_client_->wait_for_service(
      std::chrono::duration<double>(config_.service_timeout_s)))
  {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    motion_enabled_ = true;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  auto future = nav_view_scan_client_->async_send_request(request);
  const bool success =
    future.wait_for(std::chrono::duration<double>(config_.service_timeout_s)) ==
    std::future_status::ready && future.get()->success;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    motion_enabled_ = false;
  }
  safe_stop("navigation view scan");
  return success;
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
      status_is_current(last_navigation_status_->header, navigation_request_start_) &&
      navigation_terminal(last_navigation_status_->state))
    {
      return last_navigation_status_->state == NavigationStatus::STATE_SUCCEEDED ?
             ActionResult::kSucceeded : ActionResult::kFailed;
    }
    condition_.wait_until(lock, deadline);
  }
  return ActionResult::kTimeout;
}

ActionResult Runtime::wait_manipulation_terminal(
  const std::string & area,
  const std::string & task,
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
      last_manipulation_status_->waypoint_id == area &&
      last_manipulation_status_->task_id == task &&
      status_is_current(last_manipulation_status_->header, manipulation_request_start_) &&
      manipulation_terminal(last_manipulation_status_->state))
    {
      return last_manipulation_status_->state == ManipulationStatus::STATE_SUCCEEDED ?
             ActionResult::kSucceeded : ActionResult::kFailed;
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
  status.auto_start_latched = auto_event_ || mcu_.auto_start_latched;
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

void Runtime::set_plan_for_test(const Plan & plan)
{
  std::lock_guard<std::mutex> lock(mutex_);
  plan_ = plan;
}

void Runtime::set_next_navigation_result_for_test(const ActionResult result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  next_navigation_result_for_test_ = result;
}

void Runtime::set_next_manipulation_result_for_test(const ActionResult result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  next_manipulation_result_for_test_ = result;
}

void Runtime::set_next_sorting_result_for_test(const SortingResult & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  next_sorting_result_for_test_ = result;
}

void Runtime::set_next_observation_for_test(const Observation & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  next_observation_for_test_ = result;
}

}  // namespace atlas_mission_yasmin
