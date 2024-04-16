#include <inverter_interface/inverter_interface.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <cmath>
#include <chrono>
#include <thread>
#include <string>
#include <iostream>
// #include <windows.h> // For windows
#include <unistd.h> // For Linux
#include <yaml-cpp/yaml.h>

using std::placeholders::_1;
using std::placeholders::_2;

InverterInterface::InverterInterface()
    : Node("inverter_interface")
{
  

      //yaml
  std::string package_path = ament_index_cpp::get_package_share_directory("bms_interface");
  std::string config_path = package_path + "/config/bs.yaml";
  YAML::Node config = YAML::LoadFile(config_path);

  callback_group_organization = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);  

  //service 
  inverter_service_ = this->create_service<charging_robot_system_msgs::srv::InverterSwitch>(
    config["inverter_interface"]["service_name"]["InverterSwitch"].as<std::string>(), 
    std::bind(&InverterInterface::serviceControl, this, _1,_2), rmw_qos_profile_services_default, callback_group_organization);

  // Subscriber

  can_inverter_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      config["inverter_interface"]["sub_topic_name"]["from_can"].as<std::string>(), 
      rclcpp::QoS(500), std::bind(&InverterInterface::invertermsg, this, _1));

  inverter_cmd_sub_ = this->create_subscription<charging_robot_system_msgs::msg::InverterCmd>(
      config["inverter_interface"]["sub_topic_name"]["inverter_cmd_sub_"].as<std::string>(), 
      rclcpp::QoS(1), std::bind(&InverterInterface::inverterCmdCallback, this, _1));

  mps_state_sub_ = this->create_subscription<charging_robot_system_msgs::msg::MpsbmsState>(
      config["inverter_interface"]["sub_topic_name"]["mps_state_sub_"].as<std::string>(), 
      rclcpp::QoS(1), std::bind(&InverterInterface::mpsStateCallback, this, _1));


  // Publisher
  can_inverter_pub_ = this->create_publisher<can_msgs::msg::Frame>(
      config["inverter_interface"]["pub_topic_name"]["to_can"].as<std::string>(), 
      rclcpp::QoS(500));

  inverter_state_pub_ = this->create_publisher<charging_robot_system_msgs::msg::InverterState>(
      config["inverter_interface"]["pub_topic_name"]["inverter_state_pub_"].as<std::string>(), 
      rclcpp::QoS(10));
  
  mannual_input.request_voltage = config["inverter_interface"]["mannual_voltage"].as<double>();
  mannual_input.request_current = config["inverter_interface"]["mannual_current"].as<double>();

  // Timer
  const auto period_ns = rclcpp::Rate(config["inverter_interface"]["loop_rate"].as<double>()).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period_ns, [this]()
                                {
      std::bind(&InverterInterface::publishInveterState, this)();
      std::bind(&InverterInterface::invertcontrol, this)();
      std::bind(&InverterInterface::inverterReadFlag, this)(); });

  velocity_status_received_time_ = this->now();

}

/*
  questions:
  Is every x secs, functions that sending msgs shall be called
  e.g. invertermsg

*/



void InverterInterface::inverterCmdCallback(charging_robot_system_msgs::msg::InverterCmd::ConstSharedPtr msg){
  start_flag_ = msg->inverter_enable;
}

void InverterInterface::mpsStateCallback(charging_robot_system_msgs::msg::MpsbmsState::ConstSharedPtr msg){
  inverter_input.request_voltage = msg->request_charge_voltage;
  inverter_input.request_current = msg->request_charge_current;
}

void InverterInterface::publishInveterState()
{
  inverter_state_msg.header.stamp = this->now();
  inverter_state_msg.outputvoltage = voltageOut;
  inverter_state_msg.outputcurrent = currentOutSlow;
  inverter_state_msg.referencevoltage = voltageRef;
  inverter_state_msg.limitoutputcurrent = limitCurrent;
  inverter_state_msg.ac_input_failure = AC_input_failure;
  inverter_state_msg.ac_module_protection = ACModule_protection;
  inverter_state_msg.pfc_bus_overvoltage = PFC_Bus_over_voltage;
  inverter_state_msg.pfc_bus_undervoltage = PFC_Bus_under_voltage;
  inverter_state_msg.pfc_bus_unbalance = PFC_Bus_unbalance;
  inverter_state_msg.dc_output_voltage = DCOutPutVoltage;
  inverter_state_msg.dc_module_protection = DCModule_protection;
  inverter_state_msg.dc_output_undervoltage = DCoutputUndervoltage;
  inverter_state_msg.fan_failure = FanFailure;
  inverter_state_msg.fan_driven_circuit_damaged = FanDrivencircuitDamaged;
  inverter_state_msg.pfc_not_run = PfcNotRun;
  inverter_state_pub_->publish(inverter_state_msg);
}

/*Inverter On*/
void InverterInterface::inverterOn()
{

  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x02200000;
  /*
  msg.data.at(0) = 0x1b;
  msg.data.at(1) = 0x00;
  msg.data.at(2) = 0x00;
  msg.data.at(3) = 0x10;
  msg.data.at(4) = 0x12;
  msg.data.at(5) = 0x16;
  msg.data.at(6) = 0x12;
  msg.data.at(7) = 0x93;
  // msg.data = [0x10,0x02,0x00,0x00,0x00,0x07,0x41,0x9e]; //475.55V
  can_inverter_pub_->publish(msg);
  */

  // Set low or high voltage mode
  // 1000V mode
  #if 0
  msg.data.at(0) = 0x12;
  msg.data.at(1) = 0x00;
  msg.data.at(2) = 0x5F;
  msg.data.at(3) = 0x00;
  msg.data.at(4) = 0x00;
  msg.data.at(5) = 0x00;
  msg.data.at(6) = 0x00;
  msg.data.at(7) = 0x01;
  #endif
  

  // Power On
  msg.data.at(0) = 0x10;
  msg.data.at(1) = 0x04;
  msg.data.at(2) = 0x00;
  msg.data.at(3) = 0x00;
  msg.data.at(4) = 0x00;
  msg.data.at(5) = 0x00;
  msg.data.at(6) = 0x00;
  msg.data.at(7) = 0x00;
  // msg.dada = [0x10,0x02,0x00,0x00,0x00,0x07,0x41,0x9e]; //475.55V
  can_inverter_pub_->publish(msg);
}

void InverterInterface::inverterOff()
{
  // if (battery_state_rpt_ptr_->power_supply_health == 1)

  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x02200000;

  msg.data.at(0) = 0x10;
  msg.data.at(1) = 0x04;
  msg.data.at(2) = 0x00;
  msg.data.at(3) = 0x00;
  msg.data.at(4) = 0x00;
  msg.data.at(5) = 0x00;
  msg.data.at(6) = 0x00;
  msg.data.at(7) = 0x01;
  // msg.data = [0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x01]; //475.55V
  can_inverter_pub_->publish(msg);
}

void InverterInterface::invertersetmode()
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x02200000;

  msg.data.at(0) = 0x10;
  msg.data.at(1) = 0x5F;
  msg.data.at(2) = 0x00;
  msg.data.at(3) = 0x00;
  msg.data.at(4) = 0x00;
  msg.data.at(5) = 0x00;
  msg.data.at(6) = 0x00;
  msg.data.at(7) = 0x01;
  // msg.data = [0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x01]; //475.55V
  can_inverter_pub_->publish(msg);
}

void InverterInterface::invertersetlimitcurrent()
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x02200000;
  // reference output current
  msg.data.at(0) = 0x10;
  msg.data.at(1) = 0x03;
  msg.data.at(2) = 0x00;
  msg.data.at(3) = 0x00;

  U_INVERTER_DATA request_current;
  if(inverter_flag_){
    request_current.INVERTER_DATA = mannual_input.request_current * 1000;
  } else {
    request_current.INVERTER_DATA = inverter_input.request_current * 1000;
    // request_current.INVERTER_DATA = 10 * 1000;
  }
  // request_current.INVERTER_DATA = inverter_input.request_current * 1000;

  msg.data.at(4) = request_current.S_INVERTER_DATA.a;
  msg.data.at(5) = request_current.S_INVERTER_DATA.b;
  msg.data.at(6) = request_current.S_INVERTER_DATA.c;
  msg.data.at(7) = request_current.S_INVERTER_DATA.d;  

  // msg.data.at(4) = 0x00;
  // msg.data.at(5) = 0x00;
  // msg.data.at(6) = 0x07;
  // msg.data.at(7) = 0xd0;
  // msg.data = [0x10,0x03,0x00,0x00,0x00,0x00,0x07,0xD0]; //2A
  can_inverter_pub_->publish(msg);
}

void InverterInterface::invertersetReferVoltage()
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x02200000;

  // reference output voltage
  msg.data.at(0) = 0x10;
  msg.data.at(1) = 0x02;
  msg.data.at(2) = 0x00;
  msg.data.at(3) = 0x00;

  U_INVERTER_DATA request_voltage;
  if (inverter_flag_){
    request_voltage.INVERTER_DATA = mannual_input.request_voltage * 1000;
  } else {
    request_voltage.INVERTER_DATA = inverter_input.request_voltage * 1000;
  }

  msg.data.at(4) = request_voltage.S_INVERTER_DATA.a;
  msg.data.at(5) = request_voltage.S_INVERTER_DATA.b;
  msg.data.at(6) = request_voltage.S_INVERTER_DATA.c;
  msg.data.at(7) = request_voltage.S_INVERTER_DATA.d;  



  // msg.data.at(4) = 0x00;
  // msg.data.at(5) = 0x05;
  // msg.data.at(6) = 0xcc;
  // msg.data.at(7) = 0x60;
  // msg.data = [0x10,0x02,0x00,0x00,0x00,0x07,0x41,0x9e]; //475.55V
  can_inverter_pub_->publish(msg);
}

void InverterInterface::inverterReadOutputVoltage()
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x02200000;

  // reference output voltage
  msg.data.at(0) = 0x12;
  msg.data.at(1) = 0x00;
  msg.data.at(2) = 0x00;
  msg.data.at(3) = 0x00;
  msg.data.at(4) = 0x00;
  msg.data.at(5) = 0x00;
  msg.data.at(6) = 0x00;
  msg.data.at(7) = 0x00;
  // msg.data = [0x10,0x02,0x00,0x00,0x00,0x07,0x41,0x9e]; //475.55V
  can_inverter_pub_->publish(msg);
}

void InverterInterface::inverterReadOutputCurrent()
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x02200000;

  msg.data.at(0) = 0x12;
  msg.data.at(1) = 0x01;
  msg.data.at(2) = 0x00;
  msg.data.at(3) = 0x00;
  msg.data.at(4) = 0x00;
  msg.data.at(5) = 0x00;
  msg.data.at(6) = 0x00;
  msg.data.at(7) = 0x00;
  // msg.data = [0x10,0x02,0x00,0x00,0x00,0x07,0x41,0x9e]; //475.55V
  can_inverter_pub_->publish(msg);
}

void InverterInterface::inverterReadMode()
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x02200000;

  msg.data.at(0) = 0x12;
  msg.data.at(1) = 0x60;
  msg.data.at(2) = 0x00;
  msg.data.at(3) = 0x00;
  msg.data.at(4) = 0x00;
  msg.data.at(5) = 0x00;
  msg.data.at(6) = 0x00;
  msg.data.at(7) = 0x00;
  // msg.data = [0x10,0x02,0x00,0x00,0x00,0x07,0x41,0x9e]; //475.55V
  can_inverter_pub_->publish(msg);
}

void InverterInterface::inverterReadFlag()
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x02208000;

  msg.data.at(0) = 0x01;
  msg.data.at(1) = 0x08;
  msg.data.at(2) = 0x00;
  msg.data.at(3) = 0x00;
  msg.data.at(4) = 0x00;
  msg.data.at(5) = 0x00;
  msg.data.at(6) = 0x00;
  msg.data.at(7) = 0x00;
  // msg.data = [0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x01]; //475.55V
  can_inverter_pub_->publish(msg);
}

void InverterInterface::inverterInsolutionTest()
{

  battery_state_rpt_ptr_->power_supply_health;
  
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x02200000;

  // reference output voltage
  msg.data.at(0) = 0x10;
  msg.data.at(1) = 0x02;
  msg.data.at(2) = 0x00;
  msg.data.at(3) = 0x00;
  msg.data.at(4) = 0x00;
  msg.data.at(5) = 0x07;
  msg.data.at(6) = 0xA1;
  msg.data.at(7) = 0x20;
  // msg.data = [0x10,0x02,0x00,0x00,0x00,0x07,0x41,0x9e]; //475.55V
  can_inverter_pub_->publish(msg);
}

void InverterInterface::inverterPreSetModeOn()
{
}
//---------Reverse Bit---------
uint8_t InverterInterface::reverseBit(uint8_t value)
{
  uint8_t result = 0;
  for (int i = 0; i < 8; ++i)
  {
    result <<= 1;
    result |= (value & 0x01);
    value >>= 1;
  }

  return result;
}

// msg from invertor
void InverterInterface::invertermsg(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  uint32_t sample = 0;

  switch (msg->data[0])
  {
  case 0x13:
    switch (msg->data[1])
    {
    case 0x00:

      // uint32_t sample = 0;
      sample |= msg->data[4] << 24;
      sample |= msg->data[5] << 16;
      sample |= msg->data[6] << 8;
      sample |= msg->data[7];
      // voltage_ = static_cast<double>(sample) * 0.001;
      voltageOut = (msg->data[4] * 16777216 + msg->data[5] * 65536 + msg->data[6] * 256 + msg->data[7]) * 0.001;
      std::cout << "voltageOut" << voltageOut << std::endl;
      break;

    case 0x01:

      // uint32_t sample = 0;
      sample |= msg->data[4] << 24;
      sample |= msg->data[5] << 16;
      sample |= msg->data[6] << 8;
      sample |= msg->data[7];
      // voltage_ = static_cast<double>(sample) * 0.001;
      currentOutSlow = (msg->data[4] * 16777216 + msg->data[5] * 65536 + msg->data[6] * 256 + msg->data[7]) * 0.001;
      std::cout << "currentOutSlow = " << currentOutSlow << std::endl;
      break;

    case 0x02:
      // uint32_t sample = 0;
      sample |= msg->data[4] << 24;
      sample |= msg->data[5] << 16;
      sample |= msg->data[6] << 8;
      sample |= msg->data[7];
      // voltage_ = static_cast<double>(sample) * 0.001;
      voltageRef = (msg->data[4] * 16777216 + msg->data[5] * 65536 + msg->data[6] * 256 + msg->data[7]) * 0.001;
      std::cout << "voltageRef =" << voltageRef << std::endl;
      break;

    case 0x03:
      // uint32_t sample = 0;
      sample |= msg->data[4] << 24;
      sample |= msg->data[5] << 16;
      sample |= msg->data[6] << 8;
      sample |= msg->data[7];
      // voltage_ = static_cast<double>(sample) * 0.001;
      limitCurrent = (msg->data[4] * 16777216 + msg->data[5] * 65536 + msg->data[6] * 256 + msg->data[7]) * 0.001;
      std::cout << "limitCurrent" << limitCurrent << std::endl;
      break;

    case 0x08:
      // uint32_t sample = 0;
      sample |= msg->data[4] << 24;
      sample |= msg->data[5] << 16;
      sample |= msg->data[6] << 8;
      sample |= msg->data[7];
      // voltage_ = static_cast<double>(sample) * 0.001;
      statusFlag = (msg->data[4] * 16777216 + msg->data[5] * 65536 + msg->data[6] * 256 + msg->data[7]) * 0.001;
      std::cout << "statusFlag = " << statusFlag << std::endl;
      break;
    default:
      break;
    }
    break;
#if 0
  case 0x13:
    switch (msg->data[1]){
      case 0x00:
            // uint32_t sample = 0;
        sample |= msg->data[4] << 24;
        sample |= msg->data[5] << 16;
        sample |= msg->data[6] << 8;
        sample |= msg->data[7];
        voltageOut = static_cast<double>(sample) * 0.001;
        break;
      case 0x01:
        sample |= msg->data[4] << 24;
        sample |= msg->data[5] << 16;
        sample |= msg->data[6] << 8;
        sample |= msg->data[7];
        currentOut = static_cast<double>(sample) * 0.001;
        std::cout << "[inverter read]  output current" <<currentOut<< std::endl;
    }
    // uint32_t sample = 0;
    sample |= msg->data[4] << 24;
    sample |= msg->data[5] << 16;
    sample |= msg->data[6] << 8;
    sample |= msg->data[7];
    // voltageOut = static_cast<double>(sample) * 0.001;
    // voltageOut = (msg->data[4] * 16777216 + msg->data[5] * 65536 + msg->data[6] * 256 + msg->data[7]) * 0.001;
    // std::cout << voltageOut << std::endl;

    /*=============Flag=============*/
    switch (msg->data[1])
    {
    case 0x08:
      switch (((msg->data[7]) & (0x03)) >> 2)
      {
      case 0:
        AC_input_failure = 0;
        break;
      case 1:
        AC_input_failure = 1;
        break;
      case 2:
        AC_input_failure = 2;
        break;
      case 3:
        AC_input_failure = 3;
        break;
      default:
        break;
      }

      switch (((msg->data[7]) & (0x04)) >> 1)
      {
      case 0:
        ACModule_protection = 0;
        break;
      case 1:
        ACModule_protection = 1;
        break;
      default:
        break;
      }

      switch (((msg->data[7]) & (0x08)) >> 1)
      {
      case 0:
        PFC_Bus_over_voltage = 0;
        break;
      case 1:
        PFC_Bus_over_voltage = 1;
        break;
      default:
        break;
      }

      switch (((msg->data[7]) & (0x010)) >> 1)
      {
      case 0:
        PFC_Bus_under_voltage = 0;
        break;
      case 1:
        PFC_Bus_under_voltage = 1;
        break;
      default:
        break;
      }

      switch (((msg->data[7]) & (0x020)) >> 1)
      {
      case 0:
        PFC_Bus_unbalance = 0;
        break;
      case 1:
        PFC_Bus_unbalance = 1;
        break;
      default:
        break;
      }

      switch (((msg->data[7]) & (0x040)) >> 1)
      {
      case 0:
        DCOutPutVoltage = 0;
        break;
      case 1:
        DCOutPutVoltage = 1;
        break;
      default:
        break;
      }

      switch (((msg->data[7]) & (0x080)) >> 1)
      {
      case 0:
        DCModule_protection = 0;
        break;
      case 1:
        DCModule_protection = 1;
        break;
      default:
        break;
      }

      switch (((msg->data[6]) & (0x01)) >> 1)
      {
      case 0:
        DCoutputUndervoltage = 0;
        break;
      case 1:
        DCoutputUndervoltage = 1;
        break;
      default:
        break;
      }

      switch (((msg->data[6]) & (0x02)) >> 1)
      {
      case 0:
        FanFailure = 0;
        break;
      case 1:
        FanFailure = 1;
        break;
      default:
        break;
      }

      switch (((msg->data[6]) & (0x08)) >> 1)
      {
      case 0:
        FanDrivencircuitDamaged = 0;
        break;
      case 1:
        FanDrivencircuitDamaged = 1;
        break;
      default:
        break;
      }

      switch (((msg->data[5]) & (0x08)) >> 1)
      {
      case 0:
        PfcNotRun = 0;
        break;
      case 1:
        PfcNotRun = 1;
        break;
      default:
        break;
      }

      /*=============Flag=============*/
      break;

    default:
      break;
    }
    break;

 #endif 
 
  default:
    break;
  }
}

void InverterInterface::Requestcurrent()
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = persetID;

  msg.data.at(0) = 0x01;
  msg.data.at(1) = 0x00;
  msg.data.at(2) = 0x04;
  msg.data.at(3) = 0x00;
  msg.data.at(4) = 0x00;
  msg.data.at(5) = 0x00;
  msg.data.at(6) = 0x00;
  msg.data.at(7) = 0x01;
  // msg.data = [0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x01]; //475.55V
  can_inverter_pub_->publish(msg);
}

void InverterInterface::serviceControl(const charging_robot_system_msgs::srv::InverterSwitch::Request::SharedPtr request, 
      const charging_robot_system_msgs::srv::InverterSwitch::Response::SharedPtr response){
  if(request->enable){
    inverter_flag_ = true;
  } else {
    inverter_flag_ = false;
  }
  response->success = true;
  response->message = "inverter flag updated";
}

void InverterInterface::inverterSetVoltage(){
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x02200000;

  // reference output voltage
  msg.data.at(0) = 0x10;
  msg.data.at(1) = 0x02;
  msg.data.at(2) = 0x00;
  msg.data.at(3) = 0x00;

  U_INVERTER_DATA request_voltage;

  request_voltage.INVERTER_DATA = voltageOut * 1000;

  msg.data.at(4) = request_voltage.S_INVERTER_DATA.a;
  msg.data.at(5) = request_voltage.S_INVERTER_DATA.b;
  msg.data.at(6) = request_voltage.S_INVERTER_DATA.c;
  msg.data.at(7) = request_voltage.S_INVERTER_DATA.d;  
  can_inverter_pub_->publish(msg);
}

void InverterInterface::invertcontrol()
{
  inverterReadOutputVoltage();
  inverterReadOutputCurrent();
  if(start_flag_ == 1){
    inverterOn();
    // if (voltageOut > 210){
    //   inverterSetVoltage();
    // } else {
    //   invertersetReferVoltage();
    // }
    
    // inverterReadOutputVoltage();
    // inverterReadOutputCurrent();
    invertersetReferVoltage();
    invertersetlimitcurrent();
  } else {
    inverterOff();
  }
  // can_inverter_pub_->publish(msg);
}
