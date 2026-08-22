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

#include <exception>
#include <memory>
#include <thread>

#include "atlas_mission_yasmin/machine.hpp"
#include "atlas_mission_yasmin/runtime.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin/state_machine_cancel_exception.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto runtime = std::make_shared<atlas_mission_yasmin::Runtime>();
  yasmin_ros::set_ros_loggers(runtime);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
  executor.add_node(runtime);

  std::thread executor_thread([&executor]() {
      executor.spin();
    });

  int exit_code = 0;
  bool enable_viewer = false;
  try {
    auto machine = atlas_mission_yasmin::build_machine(runtime);
    auto blackboard = yasmin::Blackboard::make_shared();
    enable_viewer = runtime->declare_parameter<bool>("enable_viewer", true);
    std::unique_ptr<yasmin_viewer::YasminViewerPub> viewer;
    if (enable_viewer) {
      viewer = std::make_unique<yasmin_viewer::YasminViewerPub>(
        runtime, machine, "ATLAS_MISSION_YASMIN");
    }

    (*machine)(blackboard);
    viewer.reset();
  } catch (const yasmin::StateMachineCancelException & error) {
    RCLCPP_INFO(runtime->get_logger(), "atlas_mission_yasmin canceled: %s", error.what());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(runtime->get_logger(), "atlas_mission_yasmin failed: %s", error.what());
    exit_code = 1;
  }

  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  executor.remove_node(runtime);
  runtime.reset();
  if (enable_viewer) {
    try {
      yasmin_ros::YasminNode::destroy_instance();
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        rclcpp::get_logger("atlas_mission_yasmin"),
        "yasmin node destroy skipped after cleanup: %s",
        error.what());
    }
  }
  if (rclcpp::ok()) {
    try {
      rclcpp::shutdown();
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        rclcpp::get_logger("atlas_mission_yasmin"),
        "rclcpp shutdown skipped after cleanup: %s",
        error.what());
    }
  }
  return exit_code;
}
