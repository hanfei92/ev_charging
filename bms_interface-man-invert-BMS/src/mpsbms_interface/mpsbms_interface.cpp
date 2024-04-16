#include <mpsbms_interface/mpsbms_interface.hpp>
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
#include <atomic>
// #include <windows.h> // For windows
#include <unistd.h> // For Linux
#include <yaml-cpp/yaml.h>

using std::placeholders::_1;
using std::placeholders::_2;

MPSBMSInterface::MPSBMSInterface()
    : Node("mpsbms_interface")
{
  //yaml
  std::string package_path = ament_index_cpp::get_package_share_directory("bms_interface");
  std::string config_path = package_path + "/config/bs.yaml";
  YAML::Node config = YAML::LoadFile(config_path);

  callback_group_organization = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);  

  //service
  charging_control_service_ = this->create_service<charging_robot_system_msgs::srv::PCBSwitch>(
    config["mpsbms_interface"]["service_name"]["PCBSwitch"].as<std::string>(), 
    std::bind(&MPSBMSInterface::serviceControl, this, _1,_2), rmw_qos_profile_services_default, callback_group_organization);

  //client
  // inverter_control_client_ = this->create_client<charging_robot_system_msgs::srv::InverterSwitch>("bs/inverter_control");

  // Subscriber
  can_mpsbms_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      config["mpsbms_interface"]["sub_topic_name"]["from_can"].as<std::string>(), rclcpp::QoS(500), std::bind(&MPSBMSInterface::mpsbmsmsg, this, _1));
  bms_state_sub_ = this->create_subscription<charging_robot_system_msgs::msg::BmsState>(
      config["mpsbms_interface"]["sub_topic_name"]["bms_state_sub_"].as<std::string>(), rclcpp::QoS(1), std::bind(&MPSBMSInterface::batterystatuscallback, this, _1));
  inverter_state_sub_ = this->create_subscription<charging_robot_system_msgs::msg::InverterState>(
      config["mpsbms_interface"]["sub_topic_name"]["inverter_state_sub_"].as<std::string>(), rclcpp::QoS(1), std::bind(&MPSBMSInterface::inverterStatusCallback, this, _1));
  task_sub_ = this->create_subscription<charging_robot_system_msgs::msg::ChargingRobotState>(
      config["mpsbms_interface"]["sub_topic_name"]["task_sub_"].as<std::string>(), rclcpp::QoS(1), std::bind(&MPSBMSInterface::taskRequestCallback, this, _1));

  // Publisher
  can_mpsbms_pub_ = this->create_publisher<can_msgs::msg::Frame>(config["mpsbms_interface"]["pub_topic_name"]["to_can"].as<std::string>(), rclcpp::QoS(500));

  battery_system_pub_ = this->create_publisher<charging_robot_system_msgs::msg::BatterySystem>(config["mpsbms_interface"]["pub_topic_name"]["battery_system_pub_"].as<std::string>(), rclcpp::QoS(10));

  mpsbms_state_pub_ = this->create_publisher<charging_robot_system_msgs::msg::MpsbmsState>(config["mpsbms_interface"]["pub_topic_name"]["mpsbms_state_pub_"].as<std::string>(), rclcpp::QoS(10));

  inverter_cmd_pub_ = this->create_publisher<charging_robot_system_msgs::msg::InverterCmd>(config["mpsbms_interface"]["pub_topic_name"]["inverter_cmd_pub_"].as<std::string>(), rclcpp::QoS(10));

  // std::string palarity_ = 'N';
  // char palarity_ = 'N';

  ctx = modbus_new_rtu(config["mpsbms_interface"]["modbus_para"]["port"].as<string>().c_str(), config["mpsbms_interface"]["modbus_para"]["baudrate"].as<int>(), config["mpsbms_interface"]["modbus_para"]["parity"].as<char>(), config["mpsbms_interface"]["modbus_para"]["size"].as<int>(), config["mpsbms_interface"]["modbus_para"]["stop_bit"].as<int>()  );

  modbus_set_slave(ctx, REMOTE_ID);
  modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS485);
  modbus_rtu_set_rts(ctx, MODBUS_RTU_RTS_UP);
  modbus_rtu_set_rts_delay(ctx, 500);
  modbus_rtu_get_rts_delay(ctx);
  if(ctx == nullptr){
  	std::cout << "Unable to create the libmodbus contex" << std::endl;
  }
  if(modbus_connect(ctx) == -1) {
  	std::cout << "Connection failed " << modbus_strerror(errno) << std::endl;
	modbus_free(ctx);
  }

  //   // Timer
  const auto period_ns = rclcpp::Rate(config["mpsbms_interface"]["loop_rate"].as<double>()).period();
  timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, [this]
      {
        // std::bind(&MPSBMSInterface::publishMPSBMSState, this)();
        std::bind(&MPSBMSInterface::mpsbmscontrol, this)();
      });

  msg_401.header.stamp = this->now();
  msg_401.is_error = false;
  msg_401.is_extended = true;
  msg_401.is_rtr = false;
  msg_401.dlc = 8;
  msg_401.id = 0x00000401;
  msg_401.data = {0};

  msg_208.header.stamp = this->now();
  msg_208.is_error = false;
  msg_208.is_extended = true;
  msg_208.is_rtr = false;
  msg_208.dlc = 8;
  msg_208.id = 0x00000208;
  msg_208.data = {0};

  msg_209.header.stamp = this->now();
  msg_209.is_error = false;
  msg_209.is_extended = true;
  msg_209.is_rtr = false;
  msg_209.dlc = 8;
  msg_209.id = 0x00000209;
  msg_209.data = {0};
}
MPSBMSInterface::~MPSBMSInterface(){
  std::cout << " &&&&&&&&&&&&&&&&&&&&&&&&&" << std::endl;
  msg_401.header.stamp = this->now();
  msg_401.is_error = false;
  msg_401.is_extended = true;
  msg_401.is_rtr = false;
  msg_401.dlc = 8;
  msg_401.id = 0x00000401;
  msg_401.data = {0};

  msg_208.header.stamp = this->now();
  msg_208.is_error = false;
  msg_208.is_extended = true;
  msg_208.is_rtr = false;
  msg_208.dlc = 8;
  msg_208.id = 0x00000208;
  msg_208.data = {0};

  msg_209.header.stamp = this->now();
  msg_209.is_error = false;
  msg_209.is_extended = true;
  msg_209.is_rtr = false;
  msg_209.dlc = 8;
  msg_209.id = 0x00000209;
  msg_209.data = {0};

  // for (int i = 0; i < 10000; i++){
    MPSBMSonStop();
    std::cout << "deconstructor" << std::endl;
    can_mpsbms_pub_->publish(msg_401);
    can_mpsbms_pub_->publish(msg_208);
    // can_mpsbms_pub_->publish(msg_209);
  // }
	  modbus_write_register(ctx, 0, 0);
  
  modbus_close(ctx);
  modbus_free(ctx);
}



void MPSBMSInterface::taskRequestCallback(const charging_robot_system_msgs::msg::ChargingRobotState::ConstSharedPtr msg){
  task_request_msg.header = msg->header;
  task_request_msg.charging_robot_state = msg->charging_robot_state;
}

void MPSBMSInterface::batterystatuscallback(const charging_robot_system_msgs::msg::BmsState::ConstSharedPtr msg)
{
  bms_state_msg.header = msg->header;
  bms_state_msg.voltage = msg->voltage;
  bms_state_msg.current = msg->current;
  bms_state_msg.charge = msg->charge;
  bms_state_msg.capacity = msg->capacity;
  bms_state_msg.design_capacity = msg->design_capacity;
  bms_state_msg.soc = msg->soc;
  bms_state_msg.soh = msg->soh;
  bms_state_msg.totalcapacity = msg->totalcapacity;
  bms_state_msg.leftcapacity = msg->leftcapacity;
  bms_state_msg.maxmihightemp = msg->maxmihightemp;
  bms_state_msg.maxmilowtemp = msg->maxmilowtemp;
  bms_state_msg.peakpowerdischarge = msg->peakpowerdischarge;
  bms_state_msg.countinepowerdischarge = msg->countinepowerdischarge;
  bms_state_msg.peakpowercharge = msg->peakpowercharge;
  bms_state_msg.countinepowercharge = msg->countinepowercharge;
  bms_state_msg.maxlimitvoltagecharge = msg->maxlimitvoltagecharge;
  bms_state_msg.maxlimitcurcharge = msg->maxlimitcurcharge;
  bms_state_msg.isolatedresistor = msg->isolatedresistor;
  bms_state_msg.batterypower = msg->batterypower;
  bms_state_msg.consumptioncapdischarge = msg->consumptioncapdischarge;
  bms_state_msg.consumptioncapcharge = msg->consumptioncapcharge;
  bms_state_msg.statisticaveragevoltage = msg->statisticaveragevoltage;
  bms_state_msg.statisticaveragetemperature = msg->statisticaveragetemperature;
  bms_state_msg.statisticdeltavoltage = msg->statisticdeltavoltage;
  bms_state_msg.statisticdeltatemperature = msg->statisticdeltatemperature;
  bms_state_msg.statisticchargetimes = msg->statisticchargetimes;
  bms_state_msg.numberofrelays = msg->numberofrelays;
  bms_state_msg.bms_charging_mode = msg->bms_charging_mode;
  bms_state_msg.bms_charging_status = msg->bms_charging_status;
  bms_state_msg.bms_alarm_level = msg->bms_alarm_level;
  bms_state_msg.bms_ready_signal = msg->bms_ready_signal;
  bms_state_msg.bms_warning_alarm = msg->bms_warning_alarm;
  bms_state_msg.onoffstate = msg->onoffstate;
  bms_state_msg.estopstate = msg->estopstate;
}

void MPSBMSInterface::inverterStatusCallback(const charging_robot_system_msgs::msg::InverterState::ConstSharedPtr msg){

  inverter_state_msg.header = msg->header;

  inverter_state_msg.outputvoltage = msg->outputvoltage;
  inverter_state_msg.outputcurrent = msg->outputcurrent;
  inverter_state_msg.referencevoltage = msg->referencevoltage;
  inverter_state_msg.limitoutputcurrent = msg->limitoutputcurrent;

  inverter_state_msg.ac_input_failure = msg->ac_input_failure;
  inverter_state_msg.ac_module_protection = msg->ac_module_protection;
  inverter_state_msg.pfc_bus_overvoltage = msg->pfc_bus_overvoltage;
  inverter_state_msg.pfc_bus_undervoltage = msg->pfc_bus_undervoltage;
  inverter_state_msg.pfc_bus_unbalance = msg->pfc_bus_unbalance;
  inverter_state_msg.dc_output_voltage = msg->dc_output_voltage;
  inverter_state_msg.dc_module_protection = msg->dc_module_protection;
  inverter_state_msg.dc_output_undervoltage = msg->dc_output_undervoltage;
  inverter_state_msg.fan_failure = msg->fan_failure;
  inverter_state_msg.fan_driven_circuit_damaged = msg->fan_driven_circuit_damaged;
  inverter_state_msg.pfc_not_run = msg->pfc_not_run;
}

void MPSBMSInterface::mpsbmsmsg(const can_msgs::msg::Frame::ConstSharedPtr msg)
{

  switch (msg->id)
  {
  case 0x00000208:
    measured_voltage = (msg->data[1]) * 256 + (msg->data[0]);
    measured_output_current = (msg->data[3]) * 256 + (msg->data[2]);
    ;
    switch ((msg->data[4]) & 0x3)
    {
    case 0:
      ground_fault_self_test = 0;
      break;
    case 1:
      ground_fault_self_test = 1;
      break;
    case 2:
      ground_fault_self_test = 2;
      break;
    case 3:
      ground_fault_self_test = 3;
      break;
    default:
      break;
    }

    switch ((msg->data[4]) & 0x4 >> 2)
    {
    case 0:
      ground_fault_status = 0;
      break;
    case 1:
      ground_fault_status = 1;
      break;
    default:
      break;
    }

    switch ((msg->data[4]) & 0x18 >> 3)
    {
    case 0:
      insulation_test_status = 0;
      break;
    case 1:
      insulation_test_status = 1;
      break;
    case 2:
      insulation_test_status = 2;
      break;
    case 3:
      insulation_test_status = 3;
      break;
    default:
      break;
    }

    switch ((msg->data[4]) & 0x60 >> 5)
    {
    case 0:
      charger_status = 0;
      break;
    case 1:
      charger_status = 1;
      break;
    case 2:
      charger_status = 2;
      break;
    case 3:
      charger_status = 3;
      break;
    default:
      break;
    }

    switch ((msg->data[4]) & 0x80 >> 7)
    {
    case 0:
      charger_enable_display = 0;
      break;
    case 1:
      charger_enable_display = 1;
      break;
    default:
      break;
    }

    switch ((msg->data[5]))
    {
    case 0:
      charger_fault_status = 0;
      break;
    case 1:
      charger_fault_status = 1;
      break;
    case 2:
      charger_fault_status = 2;
      break;
    case 3:
      charger_fault_status = 3;
      break;
    case 4:
      charger_fault_status = 4;
      break;
    case 5:
      charger_fault_status = 5;
      break;
    case 6:
      charger_fault_status = 6;
      break;
    case 7:
      charger_fault_status = 7;
      break;
    default:
      break;
    }

    break;

  case 0x00000401:
    cha_imix = (msg->data[1]);
    chb_imix = (msg->data[3]);
    switch ((msg->data[0]))
    {
    case 0:
      cha_req = 0;
      break;
    case 1:
      cha_req = 1;
      break;
    case 2:
      cha_req = 2;
      break;
    case 3:
      cha_req = 3;
      break;
    case 4:
      cha_req = 4;
      break;
    case 5:
      cha_req = 5;
      break;
    case 6:
      cha_req = 6;
      break;
    default:
      break;
    }

    switch ((msg->data[3]))
    {
    case 0:
      chb_req = 0;
      break;
    case 1:
      chb_req = 1;
      break;
    case 2:
      chb_req = 2;
      break;
    case 3:
      chb_req = 3;
      break;
    case 4:
      chb_req = 4;
      break;
    case 5:
      chb_req = 5;
      break;
    case 6:
      chb_req = 6;
      break;
    default:
      break;
    }

    break;

  case 0x00000406:
    ProtocolNumber = (msg->data[3]);
    VersionInfoMinor = (msg->data[4]);
    SequenceNo = (msg->data[4]);
    switch (((msg->data[0]) & 0x07))
    {
    case 0:
      PlugType = 0;
      break;
    case 1:
      PlugType = 1;
      break;
    case 2:
      PlugType = 2;
      break;
    case 3:
      PlugType = 3;
      break;
    case 4:
      PlugType = 4;
      break;
    case 5:
      PlugType = 5;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x18) >> 3)
    {
    case 0:
      PlugState = 0;
      break;
    case 1:
      PlugState = 1;
      break;
    case 2:
      PlugState = 2;
      break;
    case 3:
      PlugState = 3;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x20) >> 5)
    {
    case 0:
      CHAdeMO = 0;
      break;
    case 1:
      CHAdeMO = 1;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x40) >> 6)
    {
    case 0:
      Combo = 0;
      break;
    case 1:
      Combo = 1;
      break;
    default:
      break;
    }

    switch (((msg->data[1]) & 0x01))
    {
    case 0:
      Charger_Fault = 0;
      break;
    case 1:
      Charger_Fault = 1;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x02) >> 1)
    {
    case 0:
      Charger_Fault = 2;
      break;
    case 1:
      Charger_Fault = 3;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x04) >> 2)
    {
    case 0:
      Charger_Fault = 4;
      break;
    case 1:
      Charger_Fault = 5;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x08) >> 3)
    {
    case 0:
      Charger_Fault = 6;
      break;
    case 1:
      Charger_Fault = 7;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x10) >> 4)
    {
    case 0:
      Charger_Fault = 8;
      break;
    case 1:
      Charger_Fault = 9;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x20) >> 5)
    {
    case 0:
      Charger_Fault = 10;
      break;
    case 1:
      Charger_Fault = 11;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x40) >> 6)
    {
    case 0:
      Charger_Fault = 12;
      break;
    case 1:
      Charger_Fault = 13;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x80) >> 7)
    {
    case 0:
      Charger_Fault = 14;
      break;
    case 1:
      Charger_Fault = 15;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x01))
    {
    case 0:
      Vehicle_Fault = 0;
      break;
    case 1:
      Vehicle_Fault = 1;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x02) >> 1)
    {
    case 0:
      Vehicle_Fault = 2;
      break;
    case 1:
      Vehicle_Fault = 3;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x04) >> 2)
    {
    case 0:
      Vehicle_Fault = 4;
      break;
    case 1:
      Vehicle_Fault = 5;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x08) >> 3)
    {
    case 0:
      Vehicle_Fault = 6;
      break;
    case 1:
      Vehicle_Fault = 7;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x10) >> 4)
    {
    case 0:
      Vehicle_Fault = 8;
      break;
    case 1:
      Vehicle_Fault = 9;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x20) >> 5)
    {
    case 0:
      Vehicle_Fault = 10;
      break;
    case 1:
      Vehicle_Fault = 11;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x40) >> 6)
    {
    case 0:
      Charger_Fault = 12;
      break;
    case 1:
      Charger_Fault = 13;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x80) >> 7)
    {
    case 0:
      Charger_Fault = 14;
      break;
    case 1:
      Charger_Fault = 15;
      break;

    default:
      break;
    }

    switch (((msg->data[6])))
    {
    case 0:
      ChargerStatusExt = 0;
      break;
    case 1:
      ChargerStatusExt = 1;
      break;
    case 2:
      ChargerStatusExt = 2;
      break;
    case 3:
      ChargerStatusExt = 3;
      break;
    case 4:
      ChargerStatusExt = 4;
      break;
    case 5:
      ChargerStatusExt = 5;
      break;
    case 6:
      ChargerStatusExt = 6;
      break;
    case 7:
      ChargerStatusExt = 7;
      break;
    case 8:
      ChargerStatusExt = 8;
      break;
    case 9:
      ChargerStatusExt = 9;
      break;
    case 10:
      ChargerStatusExt = 10;
      break;
    case 11:
      ChargerStatusExt = 11;
      break;
    case 12:
      ChargerStatusExt = 12;
      break;
    case 13:
      ChargerStatusExt = 13;
      break;
    case 14:
      ChargerStatusExt = 14;
      break;
    default:
      break;
    }

    break;

  case 0x00000407:
    Present_voltage = (msg->data[1] * 256 + msg->data[0]) * 1;
    Present_current = (msg->data[2]);
    Present_SOC = (msg->data[3]);
    Remaining_charginf_time = (msg->data[4]);
    Version_Info_Major = (msg->data[5]);
    Remaining_charging_time = (msg->data[6]);
    Sequence_no = (msg->data[7]);
    break;

  case 0x00000408:
    ProtocolNumber_CCS = (msg->data[3]);
    VersionInfoMinor_CCS = (msg->data[4]);
    SequenceNo_CCS = (msg->data[4]);
    switch (((msg->data[0]) & 0x07))
    {
    case 0:
      PlugType_CCS = 0;
      break;
    case 1:
      PlugType_CCS = 1;
      break;
    case 2:
      PlugType_CCS = 2;
      break;
    case 3:
      PlugType_CCS = 3;
      break;
    case 4:
      PlugType_CCS = 4;
      break;
    case 5:
      PlugType_CCS = 5;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x18) >> 3)
    {
    case 0:
      PlugState_CCS = 0;
      break;
    case 1:
      PlugState_CCS = 1;
      break;
    case 2:
      PlugState_CCS = 2;
      break;
    case 3:
      PlugState_CCS = 3;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x20) >> 5)
    {
    case 0:
      CHAdeMO_CCS = 0;
      break;
    case 1:
      CHAdeMO_CCS = 1;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x40) >> 6)
    {
    case 0:
      Combo_CCS = 0;
      break;
    case 1:
      Combo_CCS = 1;
      break;
    default:
      break;
    }

    switch (((msg->data[1]) & 0x01))
    {
    case 0:
      Charger_Fault_CCS = 0;
      break;
    case 1:
      Charger_Fault_CCS =1;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x02) >> 1)
    {
    case 0:
      Charger_Fault_CCS = 2;
      break;
    case 1:
      Charger_Fault_CCS = 3;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x04) >> 2)
    {
    case 0:
      Charger_Fault_CCS = 4;
      break;
    case 1:
      Charger_Fault_CCS = 5;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x08) >> 3)
    {
    case 0:
      Charger_Fault_CCS = 6;
      break;
    case 1:
      Charger_Fault_CCS = 7;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x10) >> 4)
    {
    case 0:
      Charger_Fault_CCS = 8;
      break;
    case 1:
      Charger_Fault_CCS = 9;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x20) >> 5)
    {
    case 0:
      Charger_Fault_CCS = 10;
      break;
    case 1:
      Charger_Fault_CCS = 11;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x40) >> 6)
    {
    case 0:
      Charger_Fault_CCS = 12;
      break;
    case 1:
      Charger_Fault_CCS = 13;
      break;

    default:
      break;
    }

    switch (((msg->data[1]) & 0x80) >> 7)
    {
    case 0:
      Charger_Fault_CCS = 14;
      break;
    case 1:
      Charger_Fault_CCS = 15;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x01))
    {
    case 0:
      Vehicle_Fault_CCS = 0;
      break;
    case 1:
      Vehicle_Fault_CCS = 1;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x02) >> 1)
    {
    case 0:
      Vehicle_Fault_CCS = 2;
      break;
    case 1:
      Vehicle_Fault_CCS = 3;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x04) >> 2)
    {
    case 0:
      Vehicle_Fault_CCS = 4;
      break;
    case 1:
      Vehicle_Fault_CCS = 5;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x08) >> 3)
    {
    case 0:
      Vehicle_Fault_CCS = 6;
      break;
    case 1:
      Vehicle_Fault_CCS = 7;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x10) >> 4)
    {
    case 0:
      Vehicle_Fault_CCS = 8;
      break;
    case 1:
      Vehicle_Fault_CCS = 9;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x20) >> 5)
    {
    case 0:
      Vehicle_Fault_CCS = 10;
      break;
    case 1:
      Vehicle_Fault_CCS = 11;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x40) >> 6)
    {
    case 0:
      Charger_Fault_CCS = 12;
      break;
    case 1:
      Charger_Fault_CCS = 13;
      break;

    default:
      break;
    }

    switch (((msg->data[2]) & 0x80) >> 7)
    {
    case 0:
      Charger_Fault_CCS = 14;
      break;
    case 1:
      Charger_Fault_CCS = 15;
      break;

    default:
      break;
    }

    switch (((msg->data[6])))
    {
    case 0:
      ChargerStatusExt_CCS = 0;
      break;
    case 1:
      ChargerStatusExt_CCS = 1;
      break;
    case 2:
      ChargerStatusExt_CCS = 2;
      break;
    case 3:
      ChargerStatusExt_CCS = 3;
      break;
    case 4:
      ChargerStatusExt_CCS = 4;
      break;
    case 5:
      ChargerStatusExt_CCS = 5;
      break;
    case 6:
      ChargerStatusExt_CCS = 6;
      break;
    case 7:
      ChargerStatusExt_CCS = 7;
      break;
    case 8:
      ChargerStatusExt_CCS = 8;
      break;
    case 9:
      ChargerStatusExt_CCS = 9;
      break;
    case 10:
      ChargerStatusExt_CCS = 10;
      break;
    case 11:
      ChargerStatusExt_CCS = 11;
      break;
    case 12:
      ChargerStatusExt_CCS = 12;
      break;
    case 13:
      ChargerStatusExt_CCS = 13;
      break;
    case 14:
      ChargerStatusExt_CCS = 14;
      break;
    default:
      break;
    }
    break;

  case 0x00000201:

    RequestChargeVoltage = (msg->data[2] * 256 + msg->data[1]) * 0.1;
    RequestChargeCurrent = (msg->data[4] * 256 + msg->data[3]) * 0.1;
    PreChargeVoltage = (msg->data[7] * 256 + msg->data[6]) * 0.1;

    switch (((msg->data[0]) & 0x03))
    {
    case 0:
      ChargerOnOffRequest = 0;
      break;
    case 1:
      ChargerOnOffRequest = 1;
      break;
    case 2:
      ChargerOnOffRequest = 2;
      break;
    case 3:
      ChargerOnOffRequest = 3;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0xC) >> 2)
    {
    case 0:
      RequestInsulationTest = 0;
      break;
    case 1:
      RequestInsulationTest = 1;
      break;
    case 2:
      RequestInsulationTest = 2;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x10) >> 4)
    {
    case 0:
      RequestChargerRest = 0;
      break;
    case 1:
      RequestChargerRest = 1;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x20) >> 5)
    {
    case 0:
      RequestGroundFault = 0;
      break;
    case 1:
      RequestGroundFault = 1;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x40) >> 6)
    {
    case 0:
      RelayStatus = 0;
      break;
    case 1:
      RelayStatus = 1;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x80) >> 7)
    {
    case 0:
      WorkingMode = 0;
      break;
    case 1:
      WorkingMode = 1;
      break;
    default:
      break;
    }

    switch (((msg->data[5]) & 0x02))
    {
    case 0:
      PreCharge = 0;
      break;
    case 1:
      PreCharge = 1;
      break;
    default:
      break;
    }

    break;

  case 0x00000202:

    RequestChargeVoltage_CCS = (msg->data[2] * 256 + msg->data[1]) * 0.1;
    RequestChargeCurrent_CCS = (msg->data[4] * 256 + msg->data[3]) * 0.1;
    PreChargeVoltage_CCS = (msg->data[7] * 256 + msg->data[6]) * 0.1;

    switch (((msg->data[0]) & 0x03))
    {
    case 0:
      ChargerOnOffRequest_CCS = 0;
      break;
    case 1:
      ChargerOnOffRequest_CCS = 1;
      break;
    case 2:
      ChargerOnOffRequest_CCS = 2;
      break;
    case 3:
      ChargerOnOffRequest_CCS = 3;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0xC) >> 2)
    {
    case 0:
      RequestInsulationTest_CCS = 0;
      break;
    case 1:
      RequestInsulationTest_CCS = 1;
      break;
    case 2:
      RequestInsulationTest_CCS = 2;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x10) >> 4)
    {
    case 0:
      RequestChargerRest_CCS = 0;
      break;
    case 1:
      RequestChargerRest_CCS = 1;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x20) >> 5)
    {
    case 0:
      RequestGroundFault_CCS = 0;
      break;
    case 1:
      RequestGroundFault_CCS = 1;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x40) >> 6)
    {
    case 0:
      RelayStatus_CCS = 0;
      break;
    case 1:
      RelayStatus_CCS = 1;
      break;
    default:
      break;
    }

    switch (((msg->data[0]) & 0x80) >> 7)
    {
    case 0:
      WorkingMode_CCS = 0;
      break;
    case 1:
      WorkingMode_CCS = 1;
      break;
    default:
      break;
    }

    switch (((msg->data[5]) & 0x02))
    {
    case 0:
      PreCharge_CCS = 0;
      break;
    case 1:
      PreCharge_CCS = 1;
      break;
    default:
      break;
    }
  break;

  default:
    break;
  }
}

void MPSBMSInterface::MPSBMSonIntial()
{
  msg_401.data.at(0) = 0x00; // 1st 8 bits 00000000
  // msg_401.data.at(1) = 0x02; // 2nd 8 bits 0x1E      Channel A Max current
  msg_401.data.at(1) = 0x00;
  msg_401.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_401.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_401.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_401.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_401.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_401.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_208.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_208.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_208.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_208.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_208.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_208.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_208.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_208.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_209.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_209.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_209.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_209.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_209.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_209.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_209.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_209.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  battery_system_msg.header.stamp = this->now();
  battery_system_msg.bs_state = 0;
  battery_system_msg.charging_time = 0;
  battery_system_msg.error_code = 0;
}

void MPSBMSInterface::publishMPSBMSState()
{
  mpsbms_state_msg.header.stamp = this->now();
  mpsbms_state_msg.outputvoltage = 0;
  mpsbms_state_msg.present_voltage = Present_voltage;
  mpsbms_state_msg.present_current = Present_current;
  mpsbms_state_msg.present_soc = Present_SOC;

  mpsbms_state_msg.precharge_voltage = PreChargeVoltage;

  mpsbms_state_msg.request_charge_voltage = RequestChargeVoltage;
  mpsbms_state_msg.request_charge_current = RequestChargeCurrent;

  mpsbms_state_msg.mpsbms_plug_type = PlugType;
  mpsbms_state_msg.mpsbms_plug_state = PlugState;
  mpsbms_state_msg.mpsbms_chademo = CHAdeMO;
  mpsbms_state_msg.mpsbms_charger_fault = Charger_Fault;
  mpsbms_state_msg.mpsbms_vehicle_fault = Vehicle_Fault;
  mpsbms_state_msg.mpsbms_charger_status_ext = ChargerStatusExt;
  mpsbms_state_msg.mpsbms_charger_on_off_request = ChargerOnOffRequest;
  mpsbms_state_msg.mpsbms_request_insulation_test = RequestInsulationTest;
  mpsbms_state_msg.mpsbms_request_charger_rest = RequestChargerRest;
  mpsbms_state_msg.mpsbms_request_ground_fault = RequestGroundFault;
  mpsbms_state_msg.mpsbms_relay_status = RelayStatus;
  mpsbms_state_msg.mpsbms_pre_charge = PreCharge;

  mpsbms_state_pub_->publish(mpsbms_state_msg);
}

void MPSBMSInterface::MPSBMSMeasure()
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = false;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x208;
  msg.data.at(0) = 0x00; // 1st 8 bits Measured voltage
  msg.data.at(1) = 0x00; // 2nd 8 bits Measured output current
  msg.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
}

void MPSBMSInterface::MPSBMSonChAWake()
{
  msg_401.data.at(0) = 0x01; // 1st 8 bits 00000000
  msg_401.data.at(1) = 0x0C; // 2nd 8 bits 0x1E      Channel A Max current
  // msg_401.data.at(1) = 0x14;
  msg_401.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_401.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_401.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_401.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_401.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_401.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_208.data.at(0) = 0x00; // 1st 8 bits 00000000  measured voltage
  msg_208.data.at(1) = 0x00; // 2nd 8 bits 0x1E      
  msg_208.data.at(2) = 0x00; // 3rd 8 bits 00000000  measured current
  // msg_208.data.at(2) = 0x00;
  msg_208.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_208.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_208.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_208.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_208.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  // msg_209.data.at(0) = 0x00; // 1st 8 bits 00000000
  // msg_209.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  // msg_209.data.at(2) = 0x00; // 3rd 8 bits 00000000
  // msg_209.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  // msg_209.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  // msg_209.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  // msg_209.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  // msg_209.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
}

void MPSBMSInterface::MPSBMSonChAStart()
{
  msg_401.data.at(0) = 0x02; // 1st 8 bits 00000000
  // msg_401.data.at(1) = 0x02; // 2nd 8 bits 0x1E      Channel A Max current
  // msg_401.data.at(1) = 0xC8;
  msg_401.data.at(2) = 0x00; // 3rd 8 bits 00000000
  // msg_401.data.at(3) = 0x05; // 4th 8 bits 0x1E      Channel B Max current
  msg_401.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_401.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_401.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_401.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_208.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_208.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_208.data.at(2) = 0x00; // 3rd 8 bits 00000000
  // msg_208.data.at(2) = 0x00;
  msg_208.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_208.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_208.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_208.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_208.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_209.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_209.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_209.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_209.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_209.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_209.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_209.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_209.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
}

void MPSBMSInterface::MPSBMSonStop()
{
  msg_401.data.at(0) = 0x03; // 1st 8 bits 00000000
  msg_401.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_401.data.at(2) = 0x03; // 3rd 8 bits 00000000
  msg_401.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_401.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_401.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_401.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_401.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_208.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_208.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_208.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_208.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_208.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_208.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_208.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_208.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_209.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_209.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_209.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_209.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_209.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_209.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_209.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_209.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
}
// eric
void MPSBMSInterface::MPSBMSonChargerStatusStandby()
{
  msg_401.data.at(0) = 0x02; // 1st 8 bits 00000000
  // msg_401.data.at(1) = 0x02; // 2nd 8 bits 0x1E      Channel A Max current
  // msg_401.data.at(1) = 0xC8;
  msg_401.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_401.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_401.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_401.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_401.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_401.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_208.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_208.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_208.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_208.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_208.data.at(4) = 0x60; // 5th 8 bits 0110 0000  charger status = stand by
  msg_208.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_208.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_208.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_209.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_209.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_209.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_209.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_209.data.at(4) = 0x60; // 5th 8 bits 00000000  CAN Data format version
  msg_209.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_209.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_209.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
}

void MPSBMSInterface::MPSBMSonInsulationTestInProgress()
{
  msg_401.data.at(0) = 0x02; // 1st 8 bits 00000000
  // msg_401.data.at(1) = 0x02; // 2nd 8 bits 0x1E      Channel A Max current
  // msg_401.data.at(1) = 0xC8;
  msg_401.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_401.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_401.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_401.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_401.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_401.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_208.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_208.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_208.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_208.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_208.data.at(4) = 0x68; // 5th 8 bits 0 11 01 0 00  charger status = stand by insulation = in progress
  msg_208.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_208.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_208.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_209.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_209.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_209.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_209.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_209.data.at(4) = 0x68; // 5th 8 bits 00000000  CAN Data format version
  msg_209.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_209.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_209.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
}

void MPSBMSInterface::MPSBMSonInsulationTestPass()
{
  msg_401.data.at(0) = 0x02; // 1st 8 bits 00000000
  // msg_401.data.at(1) = 0x02; // 2nd 8 bits 0x1E      Channel A Max current
  // msg_401.data.at(1) = 0xC8;
  msg_401.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_401.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_401.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_401.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_401.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_401.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_208.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_208.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_208.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_208.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_208.data.at(4) = 0x70; // 5th 8 bits 0 11 10 0 00  charger status = stand by insulation = pass
  msg_208.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_208.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_208.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_209.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_209.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_209.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_209.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_209.data.at(4) = 0x70; // 5th 8 bits 00000000  CAN Data format version
  msg_209.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_209.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_209.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
}

void MPSBMSInterface::MPSBMSonChargerStatusOn()
{
  msg_401.data.at(0) = 0x02; // 1st 8 bits 00000000
  // msg_401.data.at(1) = 0x02; // 2nd 8 bits 0x1E      Channel A Max current
  // msg_401.data.at(1) = 0xC8;
  msg_401.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_401.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_401.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_401.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_401.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_401.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_208.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_208.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_208.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_208.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_208.data.at(4) = 0x30; // 5th 8 bits 0 01 10 0 00  charger status = on insulation = pass
  msg_208.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_208.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_208.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_209.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_209.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_209.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_209.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_209.data.at(4) = 0x30; // 5th 8 bits 00000000  CAN Data format version
  msg_209.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_209.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_209.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
}

void MPSBMSInterface::MPSBMSonInsulationTestOff()
{
  msg_401.data.at(0) = 0x02; // 1st 8 bits 00000000
  // msg_401.data.at(1) = 0x02; // 2nd 8 bits 0x1E      Channel A Max current
  // msg_401.data.at(1) = 0xC8;
  msg_401.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_401.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_401.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_401.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_401.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_401.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_208.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_208.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_208.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_208.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_208.data.at(4) = 0x30; // 5th 8 bits 0 01 00 0 00  charger status = on insulation = normal
  msg_208.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_208.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_208.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_209.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg_209.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg_209.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_209.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_209.data.at(4) = 0x30; // 5th 8 bits 00000000  CAN Data format version
  msg_209.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_209.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_209.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
}

void MPSBMSInterface::MPSBMSonMeasuredVoltageCurrent()
{
  msg_401.data.at(0) = 0x02; // 1st 8 bits 00000000
  if(inverter_state_msg.outputcurrent < 9){
    msg_401.data.at(1) = 0x0C;
  } else {
    msg_401.data.at(1) = 0x14;
  }
  // msg_401.data.at(1) = 0x02; // 2nd 8 bits 0x1E      Channel A Max current
  // msg_401.data.at(1) = 0xC8;
  msg_401.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_401.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_401.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg_401.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_401.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_401.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  U_PCB_DATA inverter_output_voltage;
  inverter_output_voltage.PCB_DATA = inverter_state_msg.outputvoltage * 10;  
  msg_208.data.at(0) = inverter_output_voltage.S_PCB_DATA.a;
  msg_208.data.at(1) = inverter_output_voltage.S_PCB_DATA.b;
  // msg_208.data.at(0) = 0xD8; // 
  // msg_208.data.at(1) = 0x0E; //  measured voltage

  U_PCB_DATA inverter_output_current;
  inverter_output_current.PCB_DATA = inverter_state_msg.outputcurrent * 10;  
  msg_208.data.at(2) = inverter_output_current.S_PCB_DATA.a;
  msg_208.data.at(3) = inverter_output_current.S_PCB_DATA.b;
  // msg_208.data.at(2) = 0x00; // 
  // msg_208.data.at(3) = 0x00; // measured current
  msg_208.data.at(4) = 0x30; // 5th 8 bits 00000000  CAN Data format version
  // msg_208.data.at(4) = 0x30;
  msg_208.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_208.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_208.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  msg_209.data.at(0) = 0xD8; // 1st 8 bits 00000000
  msg_209.data.at(1) = 0x0E; // 2nd 8 bits 0x1E      Channel A Max current
  msg_209.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg_209.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg_209.data.at(4) = 0x30; // 5th 8 bits 00000000  CAN Data format version
  msg_209.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg_209.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg_209.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
}

/*-------------------Channel A-------------------*/
void MPSBMSInterface::MPSBMSonChASlp()
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = false;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x00000401;
  msg.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  can_mpsbms_pub_->publish(msg);
}

void MPSBMSInterface::MPSBMSReset()
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x00000401;
  msg.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg.data.at(2) = 0x05; // 3rd 8 bits 00000000
  msg.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  can_mpsbms_pub_->publish(msg);

  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x00000208;
  msg.data.at(0) = 0x00; // 1st 8 bits 00000000
  msg.data.at(1) = 0x00; // 2nd 8 bits 0x1E      Channel A Max current
  msg.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  can_mpsbms_pub_->publish(msg);
}

/*-------------------Measured MSG-------------------*/

void MPSBMSInterface::MPSBMSMeasureTest()
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = false;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x00000208;
  msg.data.at(0) = 0x0DE; // 1st 8 bits Measured voltage
  msg.data.at(1) = 0x01;  // 2nd 8 bits Measured output current
  msg.data.at(2) = 0x02;  // 3rd 8 bits 00000000
  msg.data.at(3) = 0x00;  // 4th 8 bits 0x1E      Channel B Max current

  if (insulation_self_test_flag == 0)
  {
    template_date = 0x00 + template_date;
  }
  else if (insulation_self_test_flag == 1)
  {
    template_date = 0x01 + template_date;
    insulation_self_test_flag = 0;
  }
  else if (insulation_self_test_flag == 2)
  {
    template_date = 0x02 + template_date;
    insulation_self_test_flag = 0;
  }
  else if (insulation_self_test_flag == 3)
  {
    template_date = 0x03 + template_date;
    insulation_self_test_flag = 0;
  }

  if (Fault_Flag == 0)
  {
    template_date = 0x00 + template_date;
    Fault_Flag = 1;
  }
  else if (Fault_Flag == 1)
  {
    template_date = 0x04 + template_date;
    Fault_Flag = 1;
  }

  if (insulation_flag == 0)
  {
    template_date = 0x00 + template_date;
  }
  else if (insulation_flag == 1)
  {
    template_date = 0x08 + template_date;
    insulation_flag = 0;
  }
  else if (insulation_flag == 2)
  {
    template_date = 0x10 + template_date;
    insulation_flag = 0;
  }
  else if (insulation_flag == 3)
  {
    template_date = 0x18 + template_date;
    insulation_flag = 0;
  }

  if (charger_status_flag == 0)
  {
    template_date = 0x00 + template_date;
  }
  else if (charger_status_flag == 1)
  {
    template_date = 0x20 + template_date;
    charger_status_flag = 0;
  }
  else if (charger_status_flag == 2)
  {
    template_date = 0x40 + template_date;
    charger_status_flag = 0;
  }
  else if (charger_status_flag == 3)
  {
    template_date = 0x60 + template_date;
    charger_status_flag = 0;
  }

  if (charger_enable == 0)
  {
    template_date = 0x00 + template_date;
  }
  else if (charger_enable == 1)
  {
    template_date = 0x80 + template_date;
    charger_enable = 0;
  }

  msg.data.at(4) = template_date; // 5th 8 bits 00000000  CAN Data format version
  template_date = 0;

  if (Charger_Fault == 0)
  {
    template_date = 0x00 + template_date;
  }
  else if (Charger_Fault == 1)
  {
    template_date = 0x01 + template_date;
    Charger_Fault = 0;
  }
  else if (Charger_Fault == 2)
  {
    template_date = 0x02 + template_date;
    Charger_Fault = 0;
  }
  else if (Charger_Fault == 3)
  {
    template_date = 0x04 + template_date;
    Charger_Fault = 0;
  }
  else if (Charger_Fault == 4)
  {
    template_date = 0x08 + template_date;
    Charger_Fault = 0;
  }
  else if (Charger_Fault == 5)
  {
    template_date = 0x10 + template_date;
    Charger_Fault = 0;
  }
  else if (Charger_Fault == 6)
  {
    template_date = 0x20 + template_date;
    Charger_Fault = 0;
  }
  else if (Charger_Fault == 7)
  {
    template_date = 0x40 + template_date;
    Charger_Fault = 0;
  }
  else if (Charger_Fault == 8)
  {
    template_date = 0x80 + template_date;
    Charger_Fault = 0;
  }
  msg.data.at(5) = template_date; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg.data.at(6) = 0x00;          // 7th 8 bits 00000000  Reserved
  msg.data.at(7) = 0x00;          // 8th 8 bits 00000000  Packet sequence number
}

/*-------------------Check Flag-------------------*/
void MPSBMSInterface::MPSBMSCheckFault()
{
  if (Charger_Fault != 1 || Charger_Fault == 3 || Charger_Fault == 5 ||
      Charger_Fault == 7 || Charger_Fault == 9 || Charger_Fault == 11 || Charger_Fault == 13 || Charger_Fault == 17 ||
      Vehicle_Fault == 1 || Vehicle_Fault == 3 || Vehicle_Fault == 5 || Vehicle_Fault == 7 ||
      Vehicle_Fault == 9 || Vehicle_Fault == 11 || Vehicle_Fault == 13 || Vehicle_Fault == 15)
  {
    Fault_Flag = 1;
    return;
  }
  Fault_Flag = 0;
}

void MPSBMSInterface::MPSBMSInverterReading()
{
}

void MPSBMSInterface::MPSBMTesting()
{
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x00000401;
  msg.data.at(0) = 0x02; // 1st 8 bits 00000000
  msg.data.at(1) = 0x02; // 2nd 8 bits 0x1E      Channel A Max current
  msg.data.at(2) = 0x00; // 3rd 8 bits 00000000
  msg.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
  msg.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  can_mpsbms_pub_->publish(msg);

  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x00000208;
  msg.data.at(0) = 0xD8; // 1st 8 bits 00000000
  msg.data.at(1) = 0x0E; // 2nd 8 bits 0x1E      Channel A Max current
  msg.data.at(2) = 0x14; // 3rd 8 bits 00000000
  msg.data.at(3) = 0x00; // 4th 8 bits 0x1E      Channel B Max current
  msg.data.at(4) = 0x70; // 5th 8 bits 00000000  CAN Data format version
  msg.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
  msg.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
  msg.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

  can_mpsbms_pub_->publish(msg);
}

bool MPSBMSInterface::checkProtocolFault(){
  if (ChargerOnOffRequest==2 ||ChargerStatusExt == 0x0B || ChargerStatusExt == 0x0A||ChargerStatusExt == 0x05||ChargerStatusExt == 6 || RequestChargerRest == 1){
    return false;
  } else {
    return true;
  }
}

bool MPSBMSInterface::checkInverterFault(){
  if (inverter_state_msg.ac_input_failure || inverter_state_msg.ac_module_protection || inverter_state_msg.pfc_bus_overvoltage || inverter_state_msg.pfc_bus_undervoltage || inverter_state_msg.pfc_bus_unbalance || inverter_state_msg.dc_output_voltage || inverter_state_msg.dc_module_protection || inverter_state_msg.dc_output_undervoltage || inverter_state_msg.fan_failure || inverter_state_msg.fan_driven_circuit_damaged || inverter_state_msg.pfc_not_run) {
    return false;
  } else {
    return true;
  }
}

void MPSBMSInterface::serviceControl(const charging_robot_system_msgs::srv::PCBSwitch::Request::SharedPtr request, const charging_robot_system_msgs::srv::PCBSwitch::Response::SharedPtr response){
  if(request->enable){
    charging_flag_ = true;
  } else {
    charging_flag_ = false;
  }
  response->success = true;
  response->message = "charging flag updated";
}

void MPSBMSInterface::mpsbmscontrol(){
  // battery feedback
  battery_system_msg.header.stamp = this->now();
  battery_system_msg.bs_state = 0;
  battery_system_msg.charging_time = 0;
  battery_system_msg.soc = bms_state_msg.soc;
  battery_system_msg.current = inverter_state_msg.outputcurrent;
  battery_system_msg.error_code = 0;

  //inverter cmd
  inverter_cmd_msg.header.stamp = this->now();
  inverter_cmd_msg.inverter_enable = 0;

  recharge_switch_ = 0;

  if(!start_counting_flag_){
    start_time_ = this->now();
    // std::cout << "start time = " << start_time_.seconds() << std::endl;
  }
// std::cout << "bms_state_msg.bms_ready_signal = " << bms_state_msg.bms_ready_signal << std::endl;
  // bms_state_msg.bms_ready_signal = 1; // test
  std::cout << "recv = " << task_request_msg.charging_robot_state << std::endl;
if(charging_flag_){

    if(bms_state_msg.bms_ready_signal == 1){
      if(checkProtocolFault()){
        if(task_request_msg.charging_robot_state == 9){
		std::cout << "###############" << std::endl;
          if(ChargerStatusExt == 0){
            start_flag = 0;
          }
          if(start_flag == 0){
            MPSBMSonChAWake();
            start_flag = 1;
            std::cout<< "wake"<<endl;
          }
          // std::cout << "ChargerStatusExt = " << ChargerStatusExt << std::endl;
          if(start_flag == 1 && ChargerStatusExt ==1){
            MPSBMSonChAStart();
            start_flag = 2;  
            std::cout<< "start" <<endl;
          }
          if (start_flag == 2 /*&& ChargerOnOffRequest == 3*/ && ChargerStatusExt == 7){
            MPSBMSonChargerStatusStandby();
            start_flag = 3;
            std::cout<< "charger standby" <<endl;
          }
          if (start_flag == 3 && ChargerOnOffRequest == 3 && ChargerStatusExt ==7 && RequestInsulationTest == 1 && RequestChargeVoltage !=0){
            MPSBMSonInsulationTestInProgress();
            start_flag = 4;
            std::cout<< "InsulationTestInProgress" <<endl;
          }
          if (start_flag == 4 && ChargerOnOffRequest == 3 &&  ChargerStatusExt==7 && RequestInsulationTest ==1 && RequestChargeVoltage !=0 ){
            MPSBMSonInsulationTestPass();
            std::cout<< "InsulationTestPass" <<endl;
	    std::cout << "chargeronoffrequest = " << ChargerOnOffRequest << "	chargerStatusExt = " << ChargerStatusExt << "	RelayStatus = " << RelayStatus << std::endl; 
            start_flag = 5;
          }

	    std::cout << "chargeronoffrequest = " << ChargerOnOffRequest << "	chargerStatusExt = " << ChargerStatusExt << "	RelayStatus = " << RelayStatus << std::endl; 
          if (start_flag == 5 && ChargerStatusExt == 2 && ChargerOnOffRequest == 1 && RelayStatus == 1 && RequestChargeVoltage !=0 ){ 
            MPSBMSonChargerStatusOn();
            start_flag = 6;
            std::cout<< "Charger On" <<endl;
            // std::cout << PlugState << PlugType << ChargerStatusExt << ChargerOnOffRequest << RelayStatus << RequestChargeVoltage << RequestChargeCurrent << std::endl;
          }
          if (start_flag == 6) {
            battery_system_msg.bs_state = 2;
            battery_system_msg.charging_time = 0;
            battery_system_msg.error_code = 0;
          }
        } else if(task_request_msg.charging_robot_state == 10 && start_flag == 6) {
          //battery feedback
          battery_system_msg.bs_state = 2;
          battery_system_msg.charging_time = 0;
          battery_system_msg.error_code = 0;
          //inverter cmd
          inverter_cmd_msg.inverter_enable = 1;

          MPSBMSonMeasuredVoltageCurrent();

          if (checkInverterFault()){
            start_counting_flag_ = true;
            rclcpp::Time current_time = this->now();
            battery_system_msg.charging_time = current_time.seconds() - start_time_.seconds(); 
            RCLCPP_INFO(this->get_logger(), "start time counting");
          } else {
            //battery feedback
            battery_system_msg.bs_state = 1;
            battery_system_msg.charging_time = 0;
            battery_system_msg.error_code = 3;
            //inverter cmd
            inverter_cmd_msg.inverter_enable = 0;
            start_counting_flag_ = false;
            MPSBMSonStop();
          }  
        } else if(task_request_msg.charging_robot_state == 17 || task_request_msg.charging_robot_state == 18 || task_request_msg.charging_robot_state == 19 ){
	  recharge_switch_ = 1;
	  std::cout << "recharge_switch_ = " << recharge_switch_ << "	recharging entered " << std::endl;

          //battery feedback
          battery_system_msg.bs_state = 0;
          battery_system_msg.charging_time = 0;
          battery_system_msg.error_code = 0;
          //inverter cmd
          inverter_cmd_msg.inverter_enable = 0;
          start_counting_flag_ = false;
          MPSBMSonStop();
	}  else {
		std::cout << "idle" << std::endl;
		recharge_switch_ = 0;
          //battery feedback
          battery_system_msg.bs_state = 0;
          battery_system_msg.charging_time = 0;
          battery_system_msg.error_code = 0;
          //inverter cmd
          inverter_cmd_msg.inverter_enable = 0;
          start_counting_flag_ = false;
          MPSBMSonStop();
        }
      } else {
        battery_system_msg.bs_state = 1;
        battery_system_msg.charging_time = 0;
        battery_system_msg.error_code = 2;
        start_flag = 0;
        // MPSBMSonStop();
        inverter_cmd_msg.inverter_enable = 0;
        std::cout << "protocol fault" << std::endl; 
      }
    } else {
      //battery feedback
      battery_system_msg.bs_state = 1;
      battery_system_msg.charging_time = 0;
      battery_system_msg.error_code = 1;

      //inverter cmd
      inverter_cmd_msg.inverter_enable = 0;

      start_counting_flag_ = false;
      
      MPSBMSonStop();
    }
  } else {
    std::cout << "mannual stop" << std::endl;
    start_counting_flag_ = false;
    MPSBMSonStop();

  }
  can_mpsbms_pub_->publish(msg_401);
  can_mpsbms_pub_->publish(msg_208);
  // can_mpsbms_pub_->publish(msg_209);
  battery_system_pub_->publish(battery_system_msg);
  inverter_cmd_pub_->publish(inverter_cmd_msg);
  publishMPSBMSState();
  std::cout << "switch before = " << recharge_switch_ << std::endl;
  if (recharge_switch_backup_ != recharge_switch_){
	  modbus_write_register(ctx, 0, recharge_switch_);
	  std::cout << "#	MODBUS WRITE	#" << std::endl;
  }
//  modbus_write_register(ctx, 0, recharge_switch_);
  std::cout << "haoka = " << recharge_switch_ << std::endl;
  recharge_switch_backup_ = recharge_switch_;
  running_count++;
}
