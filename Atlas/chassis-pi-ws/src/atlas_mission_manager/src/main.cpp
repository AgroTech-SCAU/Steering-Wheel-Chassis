#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "atlas_mission_manager/mission_manager_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<atlas_mission_manager::MissionManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  node->prepare_shutdown();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
