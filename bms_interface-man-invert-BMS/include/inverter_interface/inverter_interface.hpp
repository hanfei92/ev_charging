#ifndef INVERTER_INTERFACE__INVERTER_INTERFACE_HPP_
#define INVERTER_INTERFACE__INVERTER_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <vector>

#include <sensor_msgs/msg/battery_state.hpp>
#include <charging_robot_system_msgs/msg/charging_robot_state.hpp>
#include <charging_robot_system_msgs/msg/bms_state.hpp>
#include <charging_robot_system_msgs/msg/inverter_state.hpp>
#include <charging_robot_system_msgs/msg/mpsbms_state.hpp>
#include <charging_robot_system_msgs/msg/inverter_cmd.hpp>
#include <charging_robot_system_msgs/srv/inverter_switch.hpp>

#include <algorithm>
#include <limits>
#include <memory>

using namespace std;

union U_INVERTER_DATA {
  unsigned int INVERTER_DATA;
  struct{
    unsigned int d:8;
    unsigned int c:8;
    unsigned int b:8;
    unsigned int a:8;
  }S_INVERTER_DATA;
};

typedef struct {
    float request_voltage;
    float request_current;
} INVERTER_INPUT;

class InverterInterface : public rclcpp::Node
{
public:
    InverterInterface();

private:
    //callback group
    rclcpp::CallbackGroup::SharedPtr callback_group_organization;
    // service
    rclcpp::Service<charging_robot_system_msgs::srv::InverterSwitch>::SharedPtr inverter_service_;
    void serviceControl(const charging_robot_system_msgs::srv::InverterSwitch::Request::SharedPtr request, 
      const charging_robot_system_msgs::srv::InverterSwitch::Response::SharedPtr response);
    bool inverter_flag_ = false;
    // Subscriber
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_inverter_sub_;
    rclcpp::Subscription<charging_robot_system_msgs::msg::InverterCmd>::SharedPtr inverter_cmd_sub_;
    rclcpp::Subscription<charging_robot_system_msgs::msg::MpsbmsState>::SharedPtr mps_state_sub_;
    // rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;


    // Publisher
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_inverter_pub_;
    rclcpp::Publisher<charging_robot_system_msgs::msg::InverterState>::SharedPtr inverter_state_pub_;
    // rclcpp::Publisher<charging_robot_system_msgs::msg::MpsbmsState>::SharedPtr mpsbms_state_pub_;
    // rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time velocity_status_received_time_;
    rclcpp::Time current_time_;
    rclcpp::Time delta_time_;
    rclcpp::Time _received_time_;
    rclcpp::Time start_time_BMS;

    // Parameters
    int persetID = 0x02208000;
    int power_supply_health = 0;

    /*=============Flag=============*/
    int AC_input_failure = 0;
    int ACModule_protection = 0;
    int PFC_Bus_over_voltage = 0;
    int PFC_Bus_under_voltage = 0;
    int PFC_Bus_unbalance = 0;
    int DCOutPutVoltage = 0;
    int DCModule_protection = 0;
    int DCoutputUndervoltage = 0;
    int FanFailure = 0;
    int FanDrivencircuitDamaged = 0;
    int PfcNotRun = 0;
    /*==============================*/

    double voltageOut = 0;
    double currentOutSlow = 0;
    double voltageRef = 0;
    double limitCurrent = 0;
    double statusFlag = 0;
    double BMSdurationOn = 120.0;
    const double loop_rate_ = 20.0;

    bool BMS_systemFlag = false;
    bool startFlag = true;
    bool checkFlag = true;
    bool chargingFlag = false;

    bool chargingFlagx = false;  //xxx

    bool setModeFlag = true;
    // uint32_t sample = 0;

    sensor_msgs::msg::BatteryState::ConstSharedPtr battery_state_rpt_ptr_ = nullptr;
    charging_robot_system_msgs::msg::MpsbmsState::ConstSharedPtr mpsbms_state_rpt_ptr_ = nullptr;
    charging_robot_system_msgs::msg::InverterState inverter_state_msg;

    // Functions
    void Requestvoltage();
    void Requestcurrent();

    void readInfor();
    void publishInveterState();
    void inverterOn();
    void invertersetmode();
    void invertersetlimitcurrent();
    void invertersetReferVoltage();
    void inverterReadOutputCurrent();
    void inverterReadFlag();
    void inverterInsolutionTest();
    void inverterReadOutputVoltage();
    void inverterReadMode();
    void inverterPreSetModeOn();
    void controller();
    void inverterOff();
    void invertcontrol();
    uint8_t reverseBit(uint8_t value);
        //callback
    void invertermsg(const can_msgs::msg::Frame::ConstSharedPtr msg);
    void batterystatuscallback(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg);
    void inverterCmdCallback(charging_robot_system_msgs::msg::InverterCmd::ConstSharedPtr msg);
    void mpsStateCallback(charging_robot_system_msgs::msg::MpsbmsState::ConstSharedPtr msg);

    void inverterSetVoltage();

    uint8_t start_flag_ = 0;
    INVERTER_INPUT inverter_input;
    INVERTER_INPUT mannual_input;

    // can_msgs::msg::Frame msg;
};
// namespace bms_interface

#endif // HKPC_INTERFACE__HKPC_INTERFACE_HPP_