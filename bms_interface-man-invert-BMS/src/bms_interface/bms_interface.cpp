#include <bms_interface/bms_interface.hpp>
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

BMSInterface::BMSInterface()
    : Node("bms_interface")
{
    //yaml
  std::string package_path = ament_index_cpp::get_package_share_directory("bms_interface");
  std::string config_path = package_path + "/config/bs.yaml";
  YAML::Node config = YAML::LoadFile(config_path);

  callback_group_organization = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);  

  //service
  mannual_control_service_ = this->create_service<charging_robot_system_msgs::srv::BMSSwitch>(config["bms_interface"]["service_name"]["BMSSwitch"].as<std::string>(), 
  std::bind(&BMSInterface::mannualControl, this, _1,_2), rmw_qos_profile_services_default, callback_group_organization);


  // Subscriber
  can_bms_sub_ = this->create_subscription<can_msgs::msg::Frame>(config["bms_interface"]["sub_topic_name"]["from_can"].as<std::string>(), rclcpp::QoS(500), std::bind(&BMSInterface::onCanFrame, this, _1));

  // can_inverter_sub_ = this->create_subscription<can_msgs::msg::Frame>(
  //     "/inverter/from_can_bus", rclcpp::QoS(500), std::bind(&BMSInterface::invertermsg, this, _1));

  // Publisher
  can_bms_pub_ = this->create_publisher<can_msgs::msg::Frame>(config["bms_interface"]["pub_topic_name"]["to_can"].as<std::string>(), rclcpp::QoS(500));
  // can_inverter_pub_ = this->create_publisher<can_msgs::msg::Frame>("inverter/to_can_bus", rclcpp::QoS(500));

  battery_state_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/charging_robot/status/battery_state", rclcpp::QoS(10));

  bms_state_pub_ = this->create_publisher<charging_robot_system_msgs::msg::BmsState>(config["bms_interface"]["pub_topic_name"]["bms_state"].as<std::string>(), rclcpp::QoS(10));

  // Timer

  const auto period_ns = rclcpp::Rate(config["bms_interface"]["loop_rate"].as<double>()).period();

  timer_ = rclcpp::create_timer(this, get_clock(), period_ns, [this]()
                                {
                                  std::bind(&BMSInterface::publishBatteryState, this)();
                                  std::bind(&BMSInterface::publishBmsState, this)();
                                  std::bind(&BMSInterface::controller, this)();

                                  // cout<<"inverter===================================================================";
                                  // std::bind(&BMSInterface::inverterOn, this)();
                                });

  battery_state_msg.cell_voltage.resize(96);
  battery_state_msg.cell_temperature.resize(12);

  // initial
  velocity_status_received_time_ = this->now();


}

//---------BMS On/Off---------
void BMSInterface::powerOn()
{
  /*
  if (BMS_systemFlag)
  {
    return;
  } */
  // if false, excuate following
  // sleep(10); //1s for linux
  // Onflag = true;
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = false;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x160;
  msg.data.at(0) = 0x1;

  can_bms_pub_->publish(msg);
}

void BMSInterface::powerOff()
{
  // Onflag = false;
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = false;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x160;
  msg.data.at(0) = 0x00;

  can_bms_pub_->publish(msg);
}

//---------BMS Estop---------
void BMSInterface::eStop()
{
  // eStopFlag = true;
  can_msgs::msg::Frame msg;
  msg.header.stamp = this->now();
  msg.is_error = false;
  msg.is_extended = false;
  msg.is_rtr = false;
  msg.dlc = 8;

  msg.id = 0x160;
  msg.data.at(0) = 0x2;

  can_bms_pub_->publish(msg);
}

//---------BMS publish Data---------
void BMSInterface::publishBatteryState()
{
	// std::cout << "publish battery state" << std::endl;
  battery_state_msg.header.stamp = this->now();

  battery_state_msg.charge = pack_leftcap;
  battery_state_msg.capacity = pack_leftcap;
  battery_state_msg.current = current_;
  battery_state_msg.voltage = voltage_;
  battery_state_msg.design_capacity = pack_totalcap;
  // battery_state_msg.power_supply_health = SOH_;
  battery_state_msg.percentage = soc_;
  battery_state_msg.temperature = TempHMax;
  // battery_state_msg.=POWER_SUPPLY_TECHNOLOGY_UNKNOWN
  battery_state_pub_->publish(battery_state_msg);
}

void BMSInterface::publishBmsState()
{
	// std::cout << "publish bms state" << std::endl;
  bms_state_msg.header.stamp = this->now();
  /*===============Flag===============*/
  bms_state_msg.onoffstate = Onflag;
  bms_state_msg.estopstate = eStopFlag;
  /*===============Data===============*/
  bms_state_msg.voltage = voltage_;
  bms_state_msg.current = current_;
  bms_state_msg.charge = pack_leftcap;
  bms_state_msg.capacity = pack_leftcap;
  bms_state_msg.design_capacity = pack_totalcap;
  bms_state_msg.soc = soc_;
  bms_state_msg.soh = SOH_;
  bms_state_msg.totalcapacity = pack_totalcap;
  bms_state_msg.leftcapacity = pack_leftcap;
  bms_state_msg.maxmihightemp = TempHMax;
  bms_state_msg.maxmilowtemp = TempLMax;
  bms_state_msg.peakpowerdischarge = powerDischargepeak;
  bms_state_msg.countinepowerdischarge = powerDischargecountine;
  bms_state_msg.peakpowercharge = powerChargepeak;
  bms_state_msg.countinepowercharge = powerChargecountine;
  bms_state_msg.maxlimitvoltagecharge = VolChargeMaxLimit;
  bms_state_msg.maxlimitcurcharge = CurChargeMaxLimit;
  bms_state_msg.isolatedresistor = isolatedResistor;
  bms_state_msg.batterypower = batteryPower;
  bms_state_msg.consumptioncapdischarge = cumCap_Discharge;
  bms_state_msg.consumptioncapcharge = cumCap_charge;
  bms_state_msg.statisticaveragevoltage = StatisticAverageVoltage;
  bms_state_msg.statisticaveragetemperature = StatisticAverageTemperature;
  bms_state_msg.statisticdeltavoltage = StatisticDeltaVoltage;
  bms_state_msg.statisticdeltatemperature = StatisticDeltaTemperature;
  bms_state_msg.statisticchargetimes = StatisticChargeTimes;
  bms_state_msg.numberofrelays = NumberofRelays;
  /*===============Mode===============*/
  bms_state_msg.bms_charging_mode = BMS_ElCurFlow;
  bms_state_msg.bms_charging_status = BMS_Status_of_charging;
  bms_state_msg.bms_alarm_level = BMS_alarm_Lvl;
  bms_state_msg.bms_ready_signal = BMS_readySignal;
  bms_state_msg.bms_warning_alarm = BMS_warningAlarm;
  bms_state_pub_->publish(bms_state_msg);
}

void BMSInterface::onCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
	// std::cout << "can phaser" << std::endl;
  switch (msg->id)
  {
  /*The status of Voltage, Current and SoC*/
  case 0x1e1:
  {
	  // std::cout <<"1e1" << std::endl;
    soc_ = (msg->data[6] * 256 + msg->data[7]) * 0.1;
    current_ = (msg->data[2] * 256 + msg->data[3]) * 0.1 - 1000;
    voltage_ = (msg->data[0] * 256 + msg->data[1]) * 0.1;
    break;
  }

  /*=========================================================================*/
  /*BMS output status and charge mode*/
  case 0x1e4:
  {
	  // std::cout <<"1e4" << std::endl;
    BMS_chargeMode = (msg->data[0] << 4);
    // BMShealth_status = (msg->data[7]);
    BMS_chargeMode = reverseBit(BMS_chargeMode);

    BMS_current_status = BMS_charge_status & 0x03;

    BMS_charge_status = (BMS_chargeMode & 0x0F) - BMS_current_status;

    BMS_alarm = (msg->data[6]);

    Data_readySignal = (msg->data[7]);

    readySignal = Data_readySignal & 0x03;

    switch (BMS_current_status)
    {
    case 0: // NOT CHARGING
      BMS_ElCurFlow = 0;
      break;
    case 1: // DC
      BMS_ElCurFlow = 1;
      break;
    case 2: // AC
      BMS_ElCurFlow = 2;
      break;
    case 3: // Unknown
      BMS_ElCurFlow = 3;
      break;
    default:
      break;
    }

    switch (BMS_charge_status)
    {
    case 4: // CHARGING
      BMScharge_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
      BMS_Status_of_charging = 1;
      break;
    case 0: // NOT CHARGING
      BMScharge_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
      BMS_Status_of_charging = 3;
      break;
    case 8: // POWER_SUPPLY_STATUS_FULL
      BMScharge_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
      BMS_Status_of_charging = 4;
      break;
    case 12: // POWER_SUPPLY_STATUS_UNKNOWN
      BMScharge_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
      BMS_Status_of_charging = 0;
      break;
    default:
      break;
    }

    /*BMS Alarm Level*/
    switch (BMS_alarm)
    {
    case 0: // No Fault
      BMS_alarm_Lvl = 0;
      break;
    case 1: // Lv1 Fault, does not affect normal operation.
      BMS_alarm_Lvl = 1;
      break;
    case 2: // Lv2 Fault, requiring the operator to restrict.
      BMS_alarm_Lvl = 2;
      break;
    case 3: // Lv3 Fault, please stop immediately or request rescue.
      BMS_alarm_Lvl = 3;
      break;
    case 4: // Lv4 Fault, stop
      BMS_alarm_Lvl = 4;
      break;
    case 0xFE: // exception
      BMS_alarm_Lvl = 5;
      break;
    case 0xFF: // invalid
      BMS_alarm_Lvl = 6;
      break;
    default:
      break;
    }

    /*BMS Ready Signal*/
    switch (readySignal)
    {
    case 0: // Not Ready
      BMS_readySignal = 0;
      break;
    case 1: // Ready
      BMS_readySignal = 1;
      break;
    case 2: // Error
      BMS_readySignal = 2;
      break;
    case 3: // Reserve
      BMS_readySignal = 3;
      break;
    default:
      break;
    }
    break;
  }

  /*=========================================================================*/
  /*The capacity and SOH */
  case 0x205:
  {
	  // std::cout <<"205" << std::endl;
    pack_totalcap = (msg->data[0] * 256 + msg->data[1]) * 0.1; // TotalCapacity
    pack_leftcap = (msg->data[2] * 256 + msg->data[3]) * 0.1;  // LeftCapacity
    SOH_ = (msg->data[6] * 256 + msg->data[7]) * 0.1;
    break;
  }

  /*=========================================================================*/
  /*Temperature*/
  case 0x353:
  {
	  // std::cout <<"353" << std::endl;
    TempHMax = (msg->data[0]) - 50;
    TempLMax = (msg->data[5]) - 50;
    break;
  }

  /*=========================================================================*/
  /*BMS Power Control*/
  case 0x1F3:
  {
	  // std::cout <<"1f3" << std::endl;
    powerDischargepeak = (msg->data[0] * 256 + msg->data[1]) * 0.1;
    powerDischargecountine = (msg->data[2] * 256 + msg->data[3]) * 0.1;
    powerChargepeak = (msg->data[4] * 256 + msg->data[5]) * 0.1;
    powerChargecountine = (msg->data[6] * 256 + msg->data[7]) * 0.1;
    break;
  }

  /*=========================================================================*/
  /*Voltage and Current charging limit*/
  case 0x1F4:
  {
	  // std::cout <<"1f4" << std::endl;
    VolChargeMaxLimit = (msg->data[0] * 256 + msg->data[1]) * 0.1 - 1000;
    CurChargeMaxLimit = (msg->data[2] * 256 + msg->data[3]) * 0.1 - 1000;
    break;
  }

  /*=========================================================================*/
  /*BMS Warning status*/
  case 0x1F5:
  {
	  // std::cout <<"1f5" << std::endl;
    BMS_warningStatus = (msg->data[1]);

    switch (BMS_warningStatus)
    {
    case 0: // No Fault
      BMS_warningAlarm = 0;
      break;
    case 1: // Lvl 1 Fault
      BMS_warningAlarm = 1;
      break;
    case 2: // Lvl 2 Fault
      BMS_warningAlarm = 2;
      break;
    case 3: // Lvl 3 Fault
      BMS_warningAlarm = 3;
      break;
    case 4: // Lvl 4 Fault
      BMS_warningAlarm = 4;
      break;
    case 0xEF: // exception
      BMS_warningAlarm = 5;
      break;
    case 0XFF: // invalid
      BMS_warningAlarm = 6;
      break;
    default:
      break;
    }
    break;
  }

  /*=========================================================================*/
  /*BMS Power Infor*/
  case 0x211:
  {
	  // std::cout <<"211" << std::endl;
    isolatedResistor = (msg->data[0] * 256 + msg->data[1]);
    batteryPower = (msg->data[2] * 256 + msg->data[3]) * 0.01 - 100;
    break;
  }

  /*=========================================================================*/
  /*BMS CumCap Infor*/
  case 0x3F5:
  {
	  // std::cout <<"3f5" << std::endl;
    cumCap_Discharge = (msg->data[2] * 256 + msg->data[3]);
    cumCap_charge = (msg->data[6] * 256 + msg->data[7]);
    break;
  }

  /*=========================================================================*/
  case 0x3f6:
  {
	  // std::cout <<"3f6" << std::endl;
    StatisticAverageVoltage = (msg->data[0] * 256 + msg->data[1]) * 0.01;
    StatisticAverageTemperature = (msg->data[2]) * 1 - 50;
    StatisticDeltaVoltage = (msg->data[3] * 256 + msg->data[4]) * 0.01;
    StatisticDeltaTemperature = (msg->data[5]) * 1 - 50;
    StatisticChargeTimes = (msg->data[6] * 256 + msg->data[7]);
    break;
  }

  /*=========================================================================*/
  /*cell voltage*/
  case 0x356:
  {
#if 0
	  // std::cout <<"356" << std::endl;
    battery_state_msg.cell_voltage[msg->data[1]] = ((msg->data[2] << 8) + msg->data[3]) * 0.01;
    battery_state_msg.cell_voltage[msg->data[1] + 1] = ((msg->data[4] << 8) + msg->data[5]) * 0.01;
    battery_state_msg.cell_voltage[msg->data[1] + 2] = ((msg->data[6] << 8) + msg->data[7]) * 0.01;
#endif
    break;

  }

  /*=========================================================================*/
  /*Relay Status*/
  case 0x360:
  {
	  // std::cout <<"360" << std::endl;
    // Number of relays
    NumberofRelays = (msg->data[0]);

    No1_relay = (msg->data[1]);
    No2_relay = (msg->data[2]);
    No3_relay = (msg->data[3]);
    No4_relay = (msg->data[4]);

    No1_relayStatus = (No1_relay >> 1);
    No2_relayStatus = (No2_relay >> 1);
    No3_relayStatus = (No3_relay >> 1);
    No4_relayStatus = (No4_relay >> 1);
    No5_relayStatus = (No5_relay >> 1);

    No1_relayFlagstatus = ((msg->data[1]) & 0x06);
    No2_relayFlagstatus = ((msg->data[2]) & 0x06);
    No3_relayFlagstatus = ((msg->data[3]) & 0x06);
    No4_relayFlagstatus = ((msg->data[4]) & 0x06);
    No5_relayFlagstatus = ((msg->data[5]) & 0x06);

    /*=============================Relay1=============================*/
    switch (No1_relayStatus)
    {
    case 0:
      No1relayStatus = 0;
      break;
    case 1:
      No1relayStatus = 1;
      break;
    default:
      break;
    }

    switch (No1_relayFlagstatus)
    {
    case 2:
      No1relayFlagstatus = 0;
      break;
    case 3:
      No1relayFlagstatus = 1;
      break;
    case 4:
      No1relayFlagstatus = 2;
      break;
    case 5:
      No1relayFlagstatus = 3;
      break;
    default:
      break;
    }

    /*=============================Relay2=============================*/
    switch (No2_relayStatus)
    {
    case 0:
      No2relayStatus = 0;
      break;
    case 1:
      No2relayStatus = 1;
      break;
    default:
      break;
    }

    switch (No2_relayFlagstatus)
    {
    case 2:
      No2relayFlagstatus = 0;
      break;
    case 3:
      No2relayFlagstatus = 1;
      break;
    case 4:
      No2relayFlagstatus = 2;
      break;
    case 5:
      No2relayFlagstatus = 3;
      break;
    default:
      break;
    }

    /*=============================Relay3=============================*/
    switch (No3_relayStatus)
    {
    case 0:
      No3relayStatus = 0;
      break;
    case 1:
      No3relayStatus = 1;
      break;
    default:
      break;
    }

    switch (No3_relayFlagstatus)
    {
    case 2:
      No3relayFlagstatus = 0;
      break;
    case 3:
      No3relayFlagstatus = 1;
      break;
    case 4:
      No3relayFlagstatus = 2;
      break;
    case 5:
      No3relayFlagstatus = 3;
      break;
    default:
      break;
    }

    /*=============================Relay4=============================*/
    switch (No4_relayStatus)
    {
    case 0:
      No4relayStatus = 0;
      break;
    case 1:
      No4relayStatus = 1;
      break;
    default:
      break;
    }

    switch (No4_relayFlagstatus)
    {
    case 2:
      No4relayFlagstatus = 0;
      break;
    case 3:
      No4relayFlagstatus = 1;
      break;
    case 4:
      No4relayFlagstatus = 2;
      break;
    case 5:
      No4relayFlagstatus = 3;
      break;
    default:
      break;
    }

    /*=============================Relay5=============================*/
    switch (No5_relayStatus)
    {
    case 0:
      No5relayStatus = 0;
      break;
    case 1:
      No5relayStatus = 1;
      break;
    default:
      break;
    }

    switch (No5_relayFlagstatus)
    {
    case 2:
      No5relayFlagstatus = 0;
      break;
    case 3:
      No5relayFlagstatus = 1;
      break;
    case 4:
      No5relayFlagstatus = 2;
      break;
    case 5:
      No5relayFlagstatus = 3;
      break;
    default:
      break;
    }

    break;
  }

  /*=========================================================================*/
  /*cell temperature*/
  case 0x357:
  {
	  // std::cout <<"357" << std::endl;
#if 0
    battery_state_msg.cell_temperature[msg->data[1]] = (msg->data[2]) - 50;
    battery_state_msg.cell_temperature[msg->data[1] + 1] = (msg->data[3]) - 50;
    battery_state_msg.cell_temperature[msg->data[1] + 2] = (msg->data[4]) - 50;
    battery_state_msg.cell_temperature[msg->data[1] + 3] = (msg->data[5]) - 50;
    battery_state_msg.cell_temperature[msg->data[1] + 4] = (msg->data[6]) - 50;
    battery_state_msg.cell_temperature[msg->data[1] + 5] = (msg->data[7]) - 50;
#endif 
    break;
  }
  default:
  {
    break;
  }
  }
}

bool BMSInterface::eStopCheck()
{
  // if ((BMS_alarm_Lvl < 3 || BMS_alarm_Lvl == 6) && (BMS_readySignal != 1) && (BMS_readySignal != 3))
  // {
  //   eStopFlag = true;
  // }
  // else if (resetFlag == true)
  // {
  //   eStopFlag = false;
  // }
  return (BMS_alarm_Lvl > 3) ? true : false;
}

bool BMSInterface::readyCheck()
{
  return (BMS_readySignal == 1) ? true : false;
}

//---------Reverse Bit---------
uint8_t BMSInterface::reverseBit(uint8_t value)
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

void BMSInterface::mannualControl(const charging_robot_system_msgs::srv::BMSSwitch::Request::SharedPtr request, const charging_robot_system_msgs::srv::BMSSwitch::Response::SharedPtr response)
{
  if(request->enable){
    charging_flag_ = true;
  } else {
    charging_flag_ = false;
  }
  response->success = true;
  response->message = "charging flag updated";
}

void BMSInterface::controller()
{
  std::cout << count ++ << std::endl;
  if(charging_flag_){
    std::cout << "charging on" << std::endl;
    std::cout << "estop = " << BMS_alarm_Lvl << " ready = " << BMS_readySignal << std::endl;
    if(!eStopCheck()){ //ready is a feedback when battery on
      std::cout << "battery status ok" << std::endl;
      if(soc_ > 10.0 && SOH_ > 80.0){
        powerOn();
        std::cout << "power on" << std::endl;
      } else {
        powerOff();
        std::cout << "power off bc of soc & soh" << std::endl;
      }
    } else {
      eStop();
      std::cout << "estop" << std::endl;
    }

  } else {
    powerOff();
    std::cout << "power off bc of request" << std::endl;
  }

  // if (startFlag == true)
  // {
  //   if ((soc_ > 10.0) && (SOH_ > 80.0) && (checkFlag == true))
  //   {

  //     if (chargingFlag == false)
  //     {
  //       chargingFlag = true;
  //     }
  //     else if (BMSdurationOnFlag == false)
  //     {
  //       powerOff();
  //       checkFlag = false;
  //       chargingFlag = false;
  //       return;
  //     }
  //     else if (eStopFlag == true)
  //     {
  //       eStop();
  //       checkFlag = false;
  //       chargingFlag = false;
  //       eStopFlag = false;
  //     }
  //     /*--------------------------*/
  //     //std::cout << "readySignal" << readySignal << std::endl;
  //     //std::cout << "BMS_readySignal" << BMS_readySignal << std::endl;
  //     /*--------------------------*/

  //     // std::cout << " On "<< std::endl;

  //     /*--------------------------*/
  //     //powerOn();
  //     /*--------------------------*/
  //   }
  // }

  /* const rclcpp::Time current_time = get_clock()->now();
  std::time_t time = current_time.seconds();
  //std::cout << "Current time: " << std::ctime(&time);

  //std::cout << "soc_ = " << soc_ <<  "   SOH_ = " << SOH_
  //          << "  checkFlag = " << checkFlag <<"BMShealth_status = "<< BMShealth_status << std::endl;

  if(startFlag == true)
  {
    //const double velocity_status_delta_time_ms = (current_time - velocity_status_received_time_).seconds() * 1000.0;
    if ( (soc_>10.0) && (SOH_>80.0) && (checkFlag == true))
    {
    //  std::cout << " Stop Charging " <<"BMShealth_status = "<< BMShealth_status<<std::endl;
    //  std::cout << "soc_>10.0 && SOH_>80.0 && (checkFlag == true)  satisfied " << std::endl;
      if (chargingFlag == false){
        // start time definition
        start_time_BMS = get_clock()->now();
        chargingFlag = true;
      } else if ((time-start_time_BMS.seconds())>=BMSdurationOn)
      { // if chargingFlag=True  AND  duration >= 10sec then:
        eStop();
        checkFlag = false;
        chargingFlag = false;
        BMShealth_status = 0;
      //  std::cout << " ==================================== " <<std::endl;
      //  std::cout << " Stop Charging " <<"BMShealth_status = "<< BMShealth_status<<std::endl;
        return;
      }


      powerOn();

    //  std::cout << "Time elapsed [s] = " << (time-start_time_BMS.seconds());
    //  std::cout << "Time elapsed [s] = " << time << "-"<<(start_time_BMS.seconds())<< std::endl;
    }
  } */
}
