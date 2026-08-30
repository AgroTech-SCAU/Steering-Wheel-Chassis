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

#ifndef ATLAS_MISSION_YASMIN__RUNTIME_HPP_
#define ATLAS_MISSION_YASMIN__RUNTIME_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "atlas_mission_interfaces/msg/manipulation_status.hpp"
#include "atlas_mission_interfaces/msg/mission_status.hpp"
#include "atlas_mission_interfaces/msg/navigation_status.hpp"
#include "atlas_mission_interfaces/srv/cancel_manipulation.hpp"
#include "atlas_mission_interfaces/srv/cancel_navigation.hpp"
#include "atlas_mission_interfaces/srv/classify_sorting_rule.hpp"
#include "atlas_mission_interfaces/srv/detect_camera_target.hpp"
#include "atlas_mission_interfaces/srv/start_manipulation.hpp"
#include "atlas_mission_interfaces/srv/start_navigation.hpp"
#include "atlas_mission_yasmin/competition_model.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mcu_comm_bridge/msg/auto_task_event.hpp"
#include "mcu_comm_bridge/msg/mcu_status.hpp"
#include "mcu_comm_bridge/srv/report_mission_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace atlas_mission_yasmin
{

enum class GuardResult
{
  kOk,
  kReset,
  kRecovery,
  kShutdown,
};

enum class ActionResult
{
  kSucceeded,
  kFailed,
  kRejected,
  kTimeout,
  kReset,
  kRecovery,
  kShutdown,
};

enum class WaitResult
{
  kSuccess,
  kRetry,
  kReset,
  kRecovery,
  kShutdown,
};

struct Waypoint
{
  std::string id;
  double timeout_s{30.0};
};

struct Plan
{
  std::string navigation_backend;
  std::string manipulation_backend;
  Waypoint pickup;
  Waypoint park_1;
  Waypoint park_2;
};

struct SortingResult
{
  ActionResult result{ActionResult::kFailed};
  std::string arena;
  std::string park_1_cargo;
  std::string park_2_cargo;
  std::string message;
};

struct Observation
{
  ActionResult result{ActionResult::kFailed};
  std::string cargo_class;
  bool layer_ok{false};
  bool complete{false};
  std::string message;
};

class Runtime final : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<Runtime>;
  using Clock = std::chrono::steady_clock;

  explicit Runtime(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  static Plan load_plan(const std::string & path);
  const Plan & plan() const;
  const Waypoint * waypoint(const std::string & id) const;

  CompetitionModel & model();
  const CompetitionModel & model() const;

  bool mcu_fresh() const;
  GuardResult guard() const;
  bool can_move() const;

  WaitResult wait_mcu();
  WaitResult wait_auto();
  WaitResult wait_reset();
  void begin_run();
  void clear_run();
  void set_state(uint8_t state, const std::string & name, const std::string & message);
  void set_error(int32_t error_code, const std::string & message);
  bool report_done();
  bool report_fail(int16_t code);

  SortingResult inspect_sorting_zone();
  Observation observe_with_recovery(
    const std::string & area,
    std::size_t slot,
    uint8_t expected_layer);
  ActionResult navigate(const std::string & waypoint_id);
  ActionResult manipulate(
    const std::string & area,
    const std::string & task,
    std::size_t slot,
    uint8_t layer,
    const std::string & cargo_class = "");

  bool cancel_navigation(const std::string & reason);
  bool cancel_manipulation(const std::string & reason);
  void safe_stop(const std::string & reason);

  void inject_mcu_status_for_test(const mcu_comm_bridge::msg::McuStatus & status);
  void inject_auto_task_event_for_test(const mcu_comm_bridge::msg::AutoTaskEvent & event);
  void set_mcu_status_timeout_for_test(double timeout_s);
  void set_motion_enabled_for_test(bool enabled);
  std::size_t safe_stop_count_for_test() const;
  void set_plan_for_test(const Plan & plan);
  void set_next_navigation_result_for_test(ActionResult result);
  void set_next_manipulation_result_for_test(ActionResult result);
  void set_next_sorting_result_for_test(const SortingResult & result);
  void set_next_observation_for_test(const Observation & result);

private:
  using McuStatus = mcu_comm_bridge::msg::McuStatus;
  using AutoTaskEvent = mcu_comm_bridge::msg::AutoTaskEvent;
  using NavigationStatus = atlas_mission_interfaces::msg::NavigationStatus;
  using ManipulationStatus = atlas_mission_interfaces::msg::ManipulationStatus;
  using MissionStatus = atlas_mission_interfaces::msg::MissionStatus;

  struct RuntimeConfig
  {
    std::string route_yaml_path;
    double mcu_status_timeout_s{1.0};
    double service_timeout_s{3.0};
    double navigation_result_timeout_s{60.0};
    double manipulation_result_timeout_s{30.0};
    int64_t required_ready_mask{0};
  };

  RuntimeConfig load_config();
  void configure_ros_interfaces();
  void handle_mcu_status(McuStatus::SharedPtr msg);
  void handle_auto_task_event(AutoTaskEvent::SharedPtr msg);
  void handle_navigation_status(NavigationStatus::SharedPtr msg);
  void handle_manipulation_status(ManipulationStatus::SharedPtr msg);
  void handle_navigation_cmd_vel(geometry_msgs::msg::Twist::SharedPtr msg);

  bool mcu_fresh_locked(Clock::time_point now) const;
  GuardResult guard_locked(Clock::time_point now) const;
  bool can_move_locked(Clock::time_point now) const;
  void publish_mission_status_locked();
  void publish_motor_zero();

  ActionResult run_navigation_request(const Waypoint & waypoint);
  ActionResult run_manipulation_request(
    const std::string & area,
    const std::string & task,
    std::size_t slot,
    uint8_t layer,
    const std::string & cargo_class);
  Observation observe_once(
    const std::string & area,
    std::size_t slot,
    uint8_t expected_layer);
  bool set_navigation_view_scan(bool enabled);

  bool call_cancel_navigation(const std::string & reason);
  bool call_cancel_manipulation(const std::string & reason);
  ActionResult wait_navigation_terminal(const Waypoint & waypoint, double timeout_s);
  ActionResult wait_manipulation_terminal(
    const std::string & area,
    const std::string & task,
    double timeout_s);

  RuntimeConfig config_;
  Plan plan_;
  CompetitionModel model_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;

  bool mcu_received_{false};
  McuStatus mcu_;
  Clock::time_point mcu_receive_time_{};
  bool auto_event_{false};
  bool reset_event_{false};
  bool active_{false};
  bool motion_enabled_{false};
  uint32_t local_run_id_{0};
  bool result_reported_{false};
  int32_t error_code_{0};
  uint8_t mission_state_{MissionStatus::STATE_BOOTSTRAP};
  std::string mission_state_name_{"BOOTSTRAP"};
  std::string mission_message_;
  std::size_t safe_stop_count_{0};

  std::optional<NavigationStatus> last_navigation_status_;
  std::optional<ManipulationStatus> last_manipulation_status_;
  std::optional<rclcpp::Time> navigation_request_start_;
  std::optional<rclcpp::Time> manipulation_request_start_;

  std::optional<ActionResult> next_navigation_result_for_test_;
  std::optional<ActionResult> next_manipulation_result_for_test_;
  std::optional<SortingResult> next_sorting_result_for_test_;
  std::optional<Observation> next_observation_for_test_;

  rclcpp::Subscription<McuStatus>::SharedPtr mcu_sub_;
  rclcpp::Subscription<AutoTaskEvent>::SharedPtr auto_task_event_sub_;
  rclcpp::Subscription<NavigationStatus>::SharedPtr navigation_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_cmd_vel_sub_;
  rclcpp::Subscription<ManipulationStatus>::SharedPtr manipulation_status_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_pub_;
  rclcpp::Publisher<MissionStatus>::SharedPtr mission_status_pub_;

  rclcpp::Client<atlas_mission_interfaces::srv::StartNavigation>::SharedPtr nav_start_;
  rclcpp::Client<atlas_mission_interfaces::srv::CancelNavigation>::SharedPtr nav_cancel_;
  rclcpp::Client<atlas_mission_interfaces::srv::StartManipulation>::SharedPtr manip_start_;
  rclcpp::Client<atlas_mission_interfaces::srv::CancelManipulation>::SharedPtr manip_cancel_;
  rclcpp::Client<atlas_mission_interfaces::srv::ClassifySortingRule>::SharedPtr classify_sorting_;
  rclcpp::Client<atlas_mission_interfaces::srv::DetectCameraTarget>::SharedPtr detect_target_;
  rclcpp::Client<mcu_comm_bridge::srv::ReportMissionResult>::SharedPtr result_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr brake_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr nav_view_scan_client_;
};

}  // namespace atlas_mission_yasmin

#endif  // ATLAS_MISSION_YASMIN__RUNTIME_HPP_
