#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <charging_robot_system_msgs/msg/battery_system.hpp>
#include <charging_robot_system_msgs/msg/charging_robot_state.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

using std::placeholders::_1;

class MinimalPublisher : public rclcpp::Node
{
public:
MinimalPublisher()
  : Node("minimal_publisher"), count_(0) {
    subscription_ = this->create_subscription<charging_robot_system_msgs::msg::BatterySystem>(
      "/charging_robot/status/battery_system_state", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
    publisher_ = this->create_publisher<charging_robot_system_msgs::msg::ChargingRobotState>(
      "/charging_robot/state", 10);
    timer_ = this->create_wall_timer(
    20ms, std::bind(&MinimalPublisher::timer_callback, this));
}

private:
  int response_;

  void timer_callback() {
    auto message = charging_robot_system_msgs::msg::ChargingRobotState();
    message.charging_robot_state = 18;
    if (response_ == 2){
      message.charging_robot_state = 10;
    }
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.charging_robot_state);
    publisher_->publish(message);
  }

  void topic_callback(const charging_robot_system_msgs::msg::BatterySystem::SharedPtr msg) {
      response_ = msg->bs_state;
    RCLCPP_INFO(this->get_logger(), "feedback: '%d'", msg->bs_state);
  }
  rclcpp::Subscription<charging_robot_system_msgs::msg::BatterySystem>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<charging_robot_system_msgs::msg::ChargingRobotState>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
