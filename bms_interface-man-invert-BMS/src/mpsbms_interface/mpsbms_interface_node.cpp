#include <mpsbms_interface/mpsbms_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MPSBMSInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
