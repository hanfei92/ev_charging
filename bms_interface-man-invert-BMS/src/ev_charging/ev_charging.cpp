#include "ev_charging.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include <rclcpp_components/register_node_macro.hpp>

#include "chademo_interface.hpp"
#include "ccs_interface.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace HKPC{

EVCharging::EVCharging(const rclcpp::NodeOptions & options) : rclcpp::Node("ev_charge", options){
    //yaml
    std::string package_path = ament_index_cpp::get_package_share_directory("bms_interface");
    std::string config_path = package_path + "/config/bs.yaml";
    YAML::Node config = YAML::LoadFile(config_path);

    const double frequency = config["mpsbms_interface"]["loop_rate"].as<double>();
    
    const auto gun_type = GetChargingGunType(config["gun_type"].as<std::string>());
    switch (gun_type){
        case ChargingGunType::CHADEMO:{
            charging_type_ = std::make_shared<CHAdeMO>(*this);
            break;
        }
        case ChargingGunType::CCS:{
            charging_type_ = std::make_shared<CombinedCS>(*this);
            break;
        }
        default:
            throw std::domain_error("[EVCharging] invalid charging mode");
    }

    /*parameter*/
    param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&EVCharging::ParametersCallback, this, _1));

    /*service*/
    charging_control_service_ = this->create_service<charging_robot_system_msgs::srv::PCBSwitch>(
        config["mpsbms_interface"]["service_name"]["PCBSwitch"].as<std::string>(), 
        std::bind(&EVCharging::OnOffService, this, _1,_2), rmw_qos_profile_services_default);

    /*Subscriber*/
    can_msgs_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        config["mpsbms_interface"]["sub_topic_name"]["from_can"].as<std::string>(), rclcpp::QoS(500), std::bind(&EVCharging::CANMsgPharserCallback, this, _1));
    bat_state_sub_ = this->create_subscription<charging_robot_system_msgs::msg::BmsState>(
        config["mpsbms_interface"]["sub_topic_name"]["bms_state_sub_"].as<std::string>(), rclcpp::QoS(1), std::bind(&EVCharging::BatStsCallback, this, _1));
    inv_state_sub_ = this->create_subscription<charging_robot_system_msgs::msg::InverterState>(
        config["mpsbms_interface"]["sub_topic_name"]["inverter_state_sub_"].as<std::string>(), rclcpp::QoS(1), std::bind(&EVCharging::InvStsCallback, this, _1));
    task_sub_ = this->create_subscription<charging_robot_system_msgs::msg::ChargingRobotState>(
        config["mpsbms_interface"]["sub_topic_name"]["task_sub_"].as<std::string>(), rclcpp::QoS(1), std::bind(&EVCharging::TaskReqCallback, this, _1));

    /*Publisher*/
    can_msgs_pub_ = this->create_publisher<can_msgs::msg::Frame>(config["mpsbms_interface"]["pub_topic_name"]["to_can"].as<std::string>(), rclcpp::QoS(500));
    charging_report_pub_ = this->create_publisher<charging_robot_system_msgs::msg::BatterySystem>(config["mpsbms_interface"]["pub_topic_name"]["battery_system_pub_"].as<std::string>(), rclcpp::QoS(10));
    charging_state_pub_ = this->create_publisher<charging_robot_system_msgs::msg::MpsbmsState>(config["mpsbms_interface"]["pub_topic_name"]["mpsbms_state_pub_"].as<std::string>(), rclcpp::QoS(10));
    inv_cmd_pub_ = this->create_publisher<charging_robot_system_msgs::msg::InverterCmd>(config["mpsbms_interface"]["pub_topic_name"]["inverter_cmd_pub_"].as<std::string>(), rclcpp::QoS(10));

    /*modbus*/
    {
    modbus_ptr_ = std::make_unique<ModbusInterface>(config["mpsbms_interface"]["modbus_para"]["port"].as<string>().c_str(), config["mpsbms_interface"]["modbus_para"]["baudrate"].as<int>(), 
            config["mpsbms_interface"]["modbus_para"]["parity"].as<char>(), config["mpsbms_interface"]["modbus_para"]["size"].as<int>(), config["mpsbms_interface"]["modbus_para"]["stop_bit"].as<int>());
    ctx = modbus_ptr_->createModbusRtuContext();
    modbus_ptr_->setSlaveAddress(ctx, config["mpsbms_interface"]["modbus_para"]["slave_addr"].as<int>());
    modbus_ptr_->setRS485Mode(ctx);
    modbus_ptr_->setRTSUp(ctx);
    modbus_ptr_->setRTSDelay(ctx, config["mpsbms_interface"]["modbus_para"]["delay"].as<int>());
    modbus_ptr_->getRTSDelay(ctx);
    modbus_ptr_->connectModbus(ctx);
    }

    /*Timer*/
    {
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1/frequency));
    timer_control_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&EVCharging::CallbackTimerControl, this));   
    }
}

ChargingGunType EVCharging::GetChargingGunType(const std::string &gun_type) const{
    if (gun_type == "chademo") return ChargingGunType::CHADEMO;
    if (gun_type == "ccs") return ChargingGunType::CCS;
    return ChargingGunType::INVALID;
}

rcl_interfaces::msg::SetParametersResult EVCharging::ParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  auto print_status = [this](const rclcpp::Parameter & parameter) {
    RCLCPP_INFO(this->get_logger(), "%s", parameter.get_name().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", parameter.value_to_string().c_str());
  };

  for (const auto & param : parameters) {
    if (param.get_name() == "fps") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        if (param.as_int() >= 1 && param.as_int() <= 20) {
          fps_ = param.as_int();
          result.successful = true;
          print_status(param);
        }
      }
    }
  }
  return result;    
}

void EVCharging::CANMsgPharserCallback(const can_msgs::msg::Frame::SharedPtr msg){
    can_msgs_ptr_ = msg;
}
void EVCharging::BatStsCallback(const charging_robot_system_msgs::msg::BmsState::SharedPtr msg){
    bms_state_ptr_ = msg;
}
void EVCharging::InvStsCallback(const charging_robot_system_msgs::msg::InverterState::SharedPtr msg){
    inverter_state_ptr_ = msg;
}
void EVCharging::TaskReqCallback(const charging_robot_system_msgs::msg::ChargingRobotState::SharedPtr msg){
    task_request_ptr_ = msg;
}
void EVCharging::OnOffService(const charging_robot_system_msgs::srv::PCBSwitch::Request::SharedPtr request, 
    const charging_robot_system_msgs::srv::PCBSwitch::Response::SharedPtr response)
{
    if (request->enable){
        charging_flag_ = true;
    } else {
        charging_flag_ = false;
    }
    response->success = true;
    response->message = "charging flag updated";
}

bool EVCharging::IsTimeOut(){
    return true;
}

std::optional<InputData> EVCharging::CreatInputData(rclcpp::Clock &clock) const{
    if (!can_msgs_ptr_) {
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 5000, "Waiting for can msgs.");
        return {};
    }
    if (!bms_state_ptr_) {
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 5000, "Waiting for battery msgs.");
        return {};
    }
    if (!inverter_state_ptr_) {
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 5000, "Waiting for inverter msgs.");
        return {};
    }
    if (!task_request_ptr_) {
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 5000, "Waiting for main controller msgs.");
        return {};
    }

    InputData input_data;
    input_data.can_msg = *can_msgs_ptr_;
    input_data.task_request_msg = *task_request_ptr_;
    input_data.bms_state_msg = *bms_state_ptr_;
    input_data.inverter_state_msg = *inverter_state_ptr_;
    return input_data;
}

void EVCharging::CallbackTimerControl(){
    if (!rclcpp::ok()) {
        RCLCPP_WARN(this->get_logger(), "ROS failed.");
        rclcpp::shutdown();
    }
    const auto input_data = CreatInputData(*get_clock());
    if (!input_data){
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, 
            "Charging is skipped since input data is not ready.");
        return;
    }

    // const bool is_charging_ready = charging_type_->IsReady(*input_data);
    // if (!is_charging_ready) {
    //     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    //         "Charging is skipped since pcb controllers are not ready to run.");
    //     return;
    // }

    if (!charging_flag_){
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "Charging is skipped since off service received.");
        return;
    }
    // stop_watch_.tic("charging") todo
    // auto state = charging_type_->GetCurrentState();
    // RCLCPP_INFO(this->get_logger(), "Current charging state: %s.", charging_type_->ToString(state).c_str());

    if (input_data->task_request_msg.charging_robot_state == 9) {
        charging_type_->Update(ChargingState::WAKE);
    } else if (input_data->task_request_msg.charging_robot_state == 10) {

    } else if (input_data->task_request_msg.charging_robot_state == 11) {

    } else if (input_data->task_request_msg.charging_robot_state == 17 || 
        input_data->task_request_msg.charging_robot_state ==  18 || 
        input_data->task_request_msg.charging_robot_state == 19) 
    {
        charging_type_->Update(ChargingState::IDLE);
    } else {
        charging_type_->Update(ChargingState::IDLE);
    }
    
    auto state = charging_type_->GetCurrentState();
    
    const auto charging_out = charging_type_->run(state, input_data->inverter_state_msg);

    can_msgs_pub_->publish(std::move(charging_out.can1_msgs));
    can_msgs_pub_->publish(std::move(charging_out.can2_msgs));
    // autoware_
}


RCLCPP_COMPONENTS_REGISTER_NODE(EVCharging)

}//