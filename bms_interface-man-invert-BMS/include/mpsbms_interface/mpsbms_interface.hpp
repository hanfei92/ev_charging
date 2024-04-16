#ifndef MPSBMS_INTERFACE__MPSBMS_INTERFACE_HPP_
#define MPSBMS_INTERFACE__MPSBMS_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <vector>

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
#include <modbus/modbus.h>

#define MODBUS_RTU_RS232 0
#define MODBUS_RTU_RS485 1

#define MODBUS_RTU_RTS_NONE 0
#define MODBUS_RTU_RTS_UP 1
#define MODBUS_RTU_RTS_DOWN 2

using namespace std;

union U_PCB_DATA {
  unsigned short PCB_DATA;
  struct{
    unsigned short a:8;
    unsigned short b:8;
  }S_PCB_DATA;
};

class MPSBMSInterface : public rclcpp::Node
{
public:
    MPSBMSInterface();
    ~MPSBMSInterface();
private:
    //Callback
    //can feedback
    void mpsbmsmsg(const can_msgs::msg::Frame::ConstSharedPtr msg);
    //bms state feedback
    void batterystatuscallback(const charging_robot_system_msgs::msg::BmsState::ConstSharedPtr msg);
    //inverter state feedback
    void inverterStatusCallback(const charging_robot_system_msgs::msg::InverterState::ConstSharedPtr msg);
    //task request
    void taskRequestCallback(const charging_robot_system_msgs::msg::ChargingRobotState::ConstSharedPtr msg);
private:
    //callback group
    rclcpp::CallbackGroup::SharedPtr callback_group_organization;

    void serviceControl(const charging_robot_system_msgs::srv::PCBSwitch::Request::SharedPtr request, 
      const charging_robot_system_msgs::srv::PCBSwitch::Response::SharedPtr response);

    // // service
    rclcpp::Service<charging_robot_system_msgs::srv::PCBSwitch>::SharedPtr charging_control_service_;

    // //client
    // rclcpp::Client<charging_robot_system_msgs::srv::InverterSwitch>::SharedPtr inverter_control_client_;

    // Subscriber
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_mpsbms_sub_;
    rclcpp::Subscription<charging_robot_system_msgs::msg::BmsState>::SharedPtr bms_state_sub_;
    rclcpp::Subscription<charging_robot_system_msgs::msg::InverterState>::SharedPtr inverter_state_sub_;
    rclcpp::Subscription<charging_robot_system_msgs::msg::ChargingRobotState>::SharedPtr task_sub_;

    // Publisher
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_mpsbms_pub_;
    rclcpp::Publisher<charging_robot_system_msgs::msg::MpsbmsState>::SharedPtr mpsbms_state_pub_;
    rclcpp::Publisher<charging_robot_system_msgs::msg::BatterySystem>::SharedPtr battery_system_pub_;
    rclcpp::Publisher<charging_robot_system_msgs::msg::InverterCmd>::SharedPtr inverter_cmd_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters

    uint32_t sample = 0;

    // sensor_msgs::msg::BatteryState::ConstSharedPtr battery_state_rpt_ptr_ = nullptr;

    charging_robot_system_msgs::msg::ChargingRobotState task_request_msg;
    charging_robot_system_msgs::msg::BmsState bms_state_msg;
    charging_robot_system_msgs::msg::InverterState inverter_state_msg;
    charging_robot_system_msgs::msg::MpsbmsState mpsbms_state_msg;
    charging_robot_system_msgs::msg::BatterySystem battery_system_msg;
    charging_robot_system_msgs::msg::InverterCmd inverter_cmd_msg;

    //0x401
    int cha_req = 0;
    int cha_imix = 0;
    int chb_req = 0;
    int chb_imix = 0;

    int measured_voltage = 0;
    int measured_output_current =0;
    int ground_fault_self_test =0;
    int ground_fault_status = 0;
    int insulation_test_status = 0;
    int charger_status =0;
    int charger_enable_display = 0;
    int charger_fault_status =0;

    //0X406
    int PlugType = 0;
    int PlugState = 0;
    int CHAdeMO = 0;
    int Combo = 0;
    int Charger_Fault = 0;
    int Vehicle_Fault = 0;
    int ProtocolNumber = 0;
    int VersionInfoMinor = 0;
    int ChargerStatusExt = 0;
    int SequenceNo = 0;
    int Present_SOC = 0;
    int Remaining_charginf_time = 0;
    int Version_Info_Major = 0;
    int Remaining_charging_time = 0;
    int Sequence_no = 0;

    //0X408
    int PlugType_CCS = 0;
    int PlugState_CCS = 0;
    int CHAdeMO_CCS = 0;
    int Combo_CCS = 0;
    int Charger_Fault_CCS = 0;
    int Vehicle_Fault_CCS = 0;
    int ProtocolNumber_CCS = 0;
    int VersionInfoMinor_CCS = 0;
    int ChargerStatusExt_CCS = 0;
    int SequenceNo_CCS = 0;
    int Present_SOC_CCS = 0;
    int Remaining_charginf_time_CCS = 0;
    int Version_Info_Major_CCS = 0;
    int Remaining_charging_time_CCS = 0;
    int Sequence_no_CCS = 0;
    // 0x201
    int ChargerOnOffRequest = 0;
    int RequestInsulationTest = 0;
    int RequestChargerRest = 0;
    int RequestGroundFault = 0;
    int RelayStatus = 0;
    int WorkingMode = 0;
    int PreCharge = 0;
    //0x202
    int ChargerOnOffRequest_CCS = 0;  
    int RequestInsulationTest_CCS = 0;
    int RequestChargerRest_CCS = 0;
    int RequestGroundFault_CCS = 0;
    int RelayStatus_CCS = 0;
    int WorkingMode_CCS = 0;
    int PreCharge_CCS = 0;

    int template_date = 0;

    int insulation_self_test_flag = 0;
    int Fault_Flag = 0;
    int insulation_flag = 0;
    int charger_status_flag = 0;
    int charger_enable = 0;

    /*---------- Flag----------*/
    int start_flag = 0;
    int stop_flag = 0;
    int estop_flag = 0;
    int wake_flag = 0;

    int insulationTest = 0;

    int count_x = 0;
    //0X407
    double Present_voltage = 0;
    double Present_current = 0;
    //0X201
    double RequestChargeVoltage = 0;
    double RequestChargeCurrent = 0;
    double PreChargeVoltage = 0;

    //0X202
    double RequestChargeVoltage_CCS = 0;
    double RequestChargeCurrent_CCS = 0;
    double PreChargeVoltage_CCS = 0;

    const double loop_rate_ = 50.0;

    bool charging_flag_ = true;

    // Functions
    void control();
    void mpsbmscontrol();
    void publishMPSBMSState();
    void MPSBMSMeasure();
    void MPSBMSon();
    void MPSBMSonChASlp();
    void MPSBMSonChAWake();
    void MPSBMSonChAStart();
    void MPSBMSonChAStop();
    void MPSBMSonChAEStop();
    void MPSBMSonChAReset();
    void MPSBMSonChAResume();

    void MPSBMSonIntial();
    void MPSBMSonStop();
    void MPSBMSReset();
    void MPSBMSonChargerStatusStandby();
    void MPSBMSonInsulationTestInProgress();
    void MPSBMSonInsulationTestPass();
    void MPSBMSonChargerStatusOn();
    void MPSBMSonInsulationTestOff();
    void MPSBMSonMeasuredVoltageCurrent();
    void MPSBMSonInsulationTest_5();

    void MPSBMSonChBSlp();
    void MPSBMSonChBWake();
    void MPSBMSonChBStart();
    void MPSBMSonChBStop();
    void MPSBMSonChBEStop();
    void MPSBMSonChBReset();
    void MPSBMSonChBResume();
    void MPSBMSMeasureTest();

    void MPSBMSInverterReading();

    void MPSBMTesting();
    
    void MPSBMSCheckFault();

    bool checkProtocolFault();
    bool checkInverterFault();

    int running_count = 0;
    // int connection_count = 0;
    can_msgs::msg::Frame msg_401;
    can_msgs::msg::Frame msg_208;
    can_msgs::msg::Frame msg_209;
    // can_msgs::msg::Frame msg_208;
    bool start_counting_flag_ = false;
    rclcpp::Time start_time_;
    // rclcpp::Time start_time_;

    //modbus params
    const int REMOTE_ID = 1;
    modbus_t *ctx = nullptr;
    int recharge_switch_ = 0;
    int recharge_switch_backup_ = 0;
};
// namespace bms_interface

#endif // MPSBMS_INTERFACE__MPSBMS_INTERFACE_HPP_
