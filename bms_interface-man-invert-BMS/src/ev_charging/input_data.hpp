#ifndef _CHADEMO_INPUT_DATA_HPP_
#define _CHADEMO_INPUT_DATA_HPP_

#include <charging_robot_system_msgs/msg/charging_robot_state.hpp>
#include <charging_robot_system_msgs/msg/bms_state.hpp>
#include <charging_robot_system_msgs/msg/inverter_state.hpp>
#include <charging_robot_system_msgs/msg/mpsbms_state.hpp>
#include <charging_robot_system_msgs/msg/battery_system.hpp>
#include <charging_robot_system_msgs/msg/inverter_cmd.hpp>
#include <charging_robot_system_msgs/srv/pcb_switch.hpp>
#include <can_msgs/msg/frame.hpp>

namespace HKPC{

struct InputData{
    can_msgs::msg::Frame can_msg;
    charging_robot_system_msgs::msg::ChargingRobotState task_request_msg;
    charging_robot_system_msgs::msg::BmsState bms_state_msg;
    charging_robot_system_msgs::msg::InverterState inverter_state_msg;
};

} //
#endif