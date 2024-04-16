#include <bms_interface/bms_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BMSInterface>();
  rclcpp::executors::MultiThreadedExecutor exector;
  exector.add_node(node);
  exector.spin();
  rclcpp::shutdown();
  // rclcpp::spin(node);
  // rclcpp::shutdown();
  return 0;
}
