#include <memory>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("atlas_mission_yasmin");
  RCLCPP_INFO(
    node->get_logger(),
    "atlas_mission_yasmin skeleton node started; Runtime and YASMIN states are not implemented yet");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
