#ifndef _CHADEMO_OUTPUT_DATA_HPP_
#define _CHADEMO_OUTPUT_DATA_HPP_


#include "input_data.hpp"
#include "sync_data.hpp"

#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>

namespace HKPC{

union U_PCB_DATA {
  unsigned short PCB_DATA;
  struct{
    unsigned short a:8;
    unsigned short b:8;
  }S_PCB_DATA;
};

struct OutputData{
    can_msgs::msg::Frame can1_msgs;
    can_msgs::msg::Frame can2_msgs;
};

enum class ChargingState {
    IDLE = 0,
    WAKE = 1,
    START = 2,
    STANDBY = 3,
    INSULATIONTESTINPROGRESS = 4,
    INSULATIONTESTPASS = 5,
    CHARGERSTATIONON = 6,
    CHARGING = 7,
    STOP = 8,
};

using StateTransition = std::function<ChargingState()>;

class ChargingModeBase {
public:
    explicit ChargingModeBase(rclcpp::Node &node);
    virtual OutputData run(ChargingState const &state, 
        charging_robot_system_msgs::msg::InverterState const &inv_msg) = 0;
    void sync(SyncData const &sync_data);
    void SetTransitions();
    void Update(const ChargingState state);
    ChargingState GetCurrentState() const;
    std::string ToString(ChargingState state);
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_;
protected:
    SyncData sync_data_;
    
    ChargingState current_state_;
    std::unordered_map<ChargingState, std::unordered_map<ChargingState, 
        StateTransition>> transitions;
private:

};

} //
#endif