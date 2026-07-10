#include "atlas_mission_manager/mission_result_reporter.hpp"

#include <exception>
#include <memory>

namespace atlas_mission_manager {

MissionResultReporter::MissionResultReporter(
    rclcpp::Node& node,
    const std::string& service_name,
    rclcpp::CallbackGroup::SharedPtr callback_group)
    : node_(node) {
  client_ = node_.create_client<mcu_comm_bridge::srv::ReportMissionResult>(
      service_name,
      rmw_qos_profile_services_default,
      callback_group);
}

bool MissionResultReporter::start(
    const std::uint8_t result,
    const std::int16_t code,
    const rclcpp::Time& now) {
  std::uint64_t request_generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == ReportStatus::InFlight) {
      return false;
    }
    ++generation_;
    request_generation = generation_;
    success_ = false;
    sent_count_ = 0;
    message_.clear();
    started_at_ = now;

    if (!client_->service_is_ready()) {
      status_ = ReportStatus::ServiceUnavailable;
      message_ = "mission result service is unavailable";
      return false;
    }
    status_ = ReportStatus::InFlight;
  }

  auto request = std::make_shared<mcu_comm_bridge::srv::ReportMissionResult::Request>();
  request->result = result;
  request->code = code;

  client_->async_send_request(
      request,
      [this, request_generation](
          rclcpp::Client<mcu_comm_bridge::srv::ReportMissionResult>::SharedFuture future) {
        bool accepted = false;
        std::uint8_t sent_count = 0;
        std::string message;
        try {
          const auto response = future.get();
          if (response) {
            accepted = response->success;
            sent_count = response->sent_count;
            message = response->message;
          } else {
            message = "mission result service returned an empty response";
          }
        } catch (const std::exception& exception) {
          message = exception.what();
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (request_generation != generation_) {
          return;
        }
        success_ = accepted;
        sent_count_ = sent_count;
        message_ = message;
        status_ = accepted ? ReportStatus::Accepted : ReportStatus::Rejected;
      });
  return true;
}

void MissionResultReporter::poll_timeout(
    const rclcpp::Time& now,
    const double timeout_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ != ReportStatus::InFlight) {
    return;
  }
  if ((now - started_at_).seconds() > timeout_s) {
    ++generation_;
    status_ = ReportStatus::TimedOut;
    success_ = false;
    sent_count_ = 0;
    message_ = "mission result service request timed out";
  }
}

ReportSnapshot MissionResultReporter::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ReportSnapshot{status_, success_, sent_count_, message_};
}

void MissionResultReporter::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++generation_;
  status_ = ReportStatus::Idle;
  success_ = false;
  sent_count_ = 0;
  message_.clear();
}

}  // namespace atlas_mission_manager
