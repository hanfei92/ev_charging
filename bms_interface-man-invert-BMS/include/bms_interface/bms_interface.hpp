#ifndef BMS_INTERFACE__BMS_INTERFACE_HPP_
#define BMS_INTERFACE__BMS_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <vector>

#include <sensor_msgs/msg/battery_state.hpp>
#include <charging_robot_system_msgs/msg/charging_robot_state.hpp>
#include <charging_robot_system_msgs/msg/bms_state.hpp>
#include <charging_robot_system_msgs/msg/inverter_state.hpp>
#include <charging_robot_system_msgs/msg/mpsbms_state.hpp>
#include <charging_robot_system_msgs/srv/bms_switch.hpp>

#include <algorithm>
#include <limits>
#include <memory>

using namespace std;
class BMSInterface : public rclcpp::Node
{
public:
    BMSInterface();

private:
    //callback group
    rclcpp::CallbackGroup::SharedPtr callback_group_organization;

    // service
    rclcpp::Service<charging_robot_system_msgs::srv::BMSSwitch>::SharedPtr mannual_control_service_;

    // Subscriber
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_bms_sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_inverter_sub_;

    // Publisher
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_bms_pub_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_inverter_pub_;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;

    rclcpp::Publisher<charging_robot_system_msgs::msg::BmsState>::SharedPtr bms_state_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time velocity_status_received_time_;
    rclcpp::Time current_time_;
    rclcpp::Time delta_time_;
    rclcpp::Time _received_time_;
    rclcpp::Time start_time_BMS;

    double BMSdurationOn = 30.0;

    double soc_ = 0.0;
    // const double loop_rate_ = 20.0;
    double current_ = 0.0;
    double voltage_ = 0.0;
    double cell_current = 0.0;
    double cell_voltage = 0.0;
    double pack_totalcap = 0.0;
    double pack_leftcap = 0.0;
    double SOH_ = 0.0;
    double TempHMax = 0.0;
    double TempLMax = 0.0;
    double VolChargeMaxLimit = 0.0;
    double CurChargeMaxLimit = 0.0;
    double powerDischargepeak = 0.0;
    double powerDischargecountine = 0.0;
    double powerChargepeak = 0.0;
    double powerChargecountine = 0.0;
    double isolatedResistor = 0.0;
    double batteryPower = 0.0;
    double cumCap_Discharge = 0.0;
    double cumCap_charge = 0.0;

    double StatisticAverageVoltage = 0.0;
    double StatisticAverageTemperature = 0.0;
    double StatisticDeltaVoltage = 0.0;
    double StatisticDeltaTemperature = 0.0;
    double StatisticChargeTimes = 0.0;

    // vector<double> array_cell_voltage;

    sensor_msgs::msg::BatteryState battery_state_msg;
    charging_robot_system_msgs::msg::BmsState bms_state_msg;

    int BMS_chargeMode = 0;
    int BMScharge_status = 0;
    int BMS_Status_of_charging = 0;
    int BMShealth_status = 1;
    int BMS_charge_status = 0;
    int BMS_current_status = 0;
    int count_cell_voltage = 0;
    int BMS_ElCurFlow = 0;
    int BMS_alarm = 0;
    int BMS_alarm_Lvl = 0;
    int BMS_readySignal = 0;
    int BMS_warningStatus = 0;
    int BMS_warningAlarm = 0;

    int inver_Msgtype = 0;
    int inver_MsgBit0 = 0;

    int Data_readySignal = 0;
    int readySignal = 0;

    /*==================Relay==================*/
    int NumberofRelays = 0;
    int No1_relay = 0;
    int No2_relay = 0;
    int No3_relay = 0;
    int No4_relay = 0;
    int No5_relay = 0;

    int No1_relayStatus = 0;
    int No2_relayStatus = 0;
    int No3_relayStatus = 0;
    int No4_relayStatus = 0;
    int No5_relayStatus = 0;

    int No1relayStatus = 0;
    int No2relayStatus = 0;
    int No3relayStatus = 0;
    int No4relayStatus = 0;
    int No5relayStatus = 0;

    int No1_relayFlagstatus = 0;
    int No2_relayFlagstatus = 0;
    int No3_relayFlagstatus = 0;
    int No4_relayFlagstatus = 0;
    int No5_relayFlagstatus = 0;

    int No1relayFlagstatus = 0;
    int No2relayFlagstatus = 0;
    int No3relayFlagstatus = 0;
    int No4relayFlagstatus = 0;
    int No5relayFlagstatus = 0;

    /*=========================================*/
    bool BMS_systemFlag = false;
    bool System_flag = false;
    bool inverter = false;
    bool BMS_alarmFlag = false;
    bool BMSdurationOnFlag = true;

    /*==================Control==================*/
    bool charging_flag_ = true;
    bool startFlag = true;
    bool checkFlag = true;
    bool chargingFlag = false;
    bool Onflag = false;
    bool eStopFlag = false;
    bool resetFlag = false;

    // Functions
    void mannualControl(const charging_robot_system_msgs::srv::BMSSwitch::Request::SharedPtr request, const charging_robot_system_msgs::srv::BMSSwitch::Response::SharedPtr response);
    void publishBatteryState();
    void publishBmsState();
    void powerOn();
    void powerOff();
    void eStop();
    void onCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg);
    bool eStopCheck();
    bool readyCheck();
    void controller();
    uint8_t reverseBit(uint8_t value);
    int count = 0;
};
// namespace bms_interface

#endif // HKPC_INTERFACE__HKPC_INTERFACE_HPP_