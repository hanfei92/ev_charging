#ifndef _EV_CHARGING_HPP_
#define _EV_CHARGING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>


#include <can_msgs/msg/frame.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <charging_robot_system_msgs/msg/charging_robot_state.hpp>
#include <charging_robot_system_msgs/msg/bms_state.hpp>
#include <charging_robot_system_msgs/msg/inverter_state.hpp>
#include <charging_robot_system_msgs/msg/mpsbms_state.hpp>
#include <charging_robot_system_msgs/srv/pcb_switch.hpp>
#include <charging_robot_system_msgs/msg/battery_system.hpp>
#include <charging_robot_system_msgs/msg/inverter_cmd.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "chademo_data.hpp"
#include "input_data.hpp"
#include "charging_mode_base.hpp"
#include "modbus_interface.hpp"

namespace HKPC{

using namespace std;

enum  ChargingGunType {
    INVALID = 0,
    CHADEMO = 1,
    CCS = 2,
};

class EVCharging : public ::rclcpp::Node
{
public:
    explicit EVCharging(const rclcpp::NodeOptions & options);
    virtual ~EVCharging() = default;

private:
    rclcpp::TimerBase::SharedPtr timer_control_;
    // rclcpp::CallbackGroup::SharedPtr callback_group_organization;
    std::shared_ptr<ChargingModeBase> charging_type_;

    /*parameters*/
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    int fps_; //example for dynamic parameters 
    /*service*/
    rclcpp::Service<charging_robot_system_msgs::srv::PCBSwitch>::SharedPtr charging_control_service_;
    /*Subscriber*/ 
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_msgs_sub_;
    rclcpp::Subscription<charging_robot_system_msgs::msg::BmsState>::SharedPtr bat_state_sub_;
    rclcpp::Subscription<charging_robot_system_msgs::msg::InverterState>::SharedPtr inv_state_sub_;
    rclcpp::Subscription<charging_robot_system_msgs::msg::ChargingRobotState>::SharedPtr task_sub_;
    /*Publisher*/ 
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_msgs_pub_;
    rclcpp::Publisher<charging_robot_system_msgs::msg::MpsbmsState>::SharedPtr charging_state_pub_;
    rclcpp::Publisher<charging_robot_system_msgs::msg::BatterySystem>::SharedPtr charging_report_pub_;
    rclcpp::Publisher<charging_robot_system_msgs::msg::InverterCmd>::SharedPtr inv_cmd_pub_;
    /*data*/
    can_msgs::msg::Frame::SharedPtr can_msgs_ptr_;
    charging_robot_system_msgs::msg::ChargingRobotState::SharedPtr task_request_ptr_;
    charging_robot_system_msgs::msg::BmsState::SharedPtr bms_state_ptr_;
    charging_robot_system_msgs::msg::InverterState::SharedPtr inverter_state_ptr_;
    charging_robot_system_msgs::msg::MpsbmsState::SharedPtr mpsbms_state_ptr_;
    charging_robot_system_msgs::msg::BatterySystem::SharedPtr battery_system_ptr_;
    charging_robot_system_msgs::msg::InverterCmd::SharedPtr inverter_cmd_ptr_;

private:
    std::unique_ptr<ModbusInterface> modbus_ptr_;
    ModbusPtr ctx;
private:
    std::optional<InputData> CreatInputData(rclcpp::Clock &clock) const;
    /*function*/

    void CallbackTimerControl();
    /*callbacks*/
    rcl_interfaces::msg::SetParametersResult ParametersCallback(const std::vector<rclcpp::Parameter> & parameters);
    void CANMsgPharserCallback(const can_msgs::msg::Frame::SharedPtr msg);
    void BatStsCallback(const charging_robot_system_msgs::msg::BmsState::SharedPtr msg);
    void InvStsCallback(const charging_robot_system_msgs::msg::InverterState::SharedPtr msg);
    void TaskReqCallback(const charging_robot_system_msgs::msg::ChargingRobotState::SharedPtr msg);
    void OnOffService(const charging_robot_system_msgs::srv::PCBSwitch::Request::SharedPtr request, 
        const charging_robot_system_msgs::srv::PCBSwitch::Response::SharedPtr response);
    
    ChargingGunType GetChargingGunType(const std::string &gun_type) const;
    /*debug*/
    bool IsTimeOut();
    void PublishDebugInfo(const InputData &input_data, const OutputData &output_data) const;
    void PublishProcessingTime();

    bool charging_flag_ = true; ///todo


};

} //

#endif // MPSBMS_INTERFACE__MPSBMS_INTERFACE_HPP_
