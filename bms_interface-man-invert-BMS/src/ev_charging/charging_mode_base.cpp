#include "charging_mode_base.hpp"

namespace HKPC {
ChargingModeBase::ChargingModeBase(rclcpp::Node &node) 
:   clock_(node.get_clock()),
    logger_(node.get_logger()),
    current_state_(ChargingState::IDLE)
{
    SetTransitions();
}

void ChargingModeBase::sync(SyncData const & sync_data) {
    sync_data_ = sync_data;
}

void ChargingModeBase::SetTransitions() {
    transitions[ChargingState::IDLE] = {
        {ChargingState::IDLE, []() { return ChargingState::IDLE; }},
        {ChargingState::WAKE, []() { return ChargingState::WAKE; }},
        {ChargingState::STOP, []() { return ChargingState::STOP; }}
    };
    transitions[ChargingState::WAKE] = {
        {ChargingState::START, []() { return ChargingState::START; }},
        {ChargingState::STOP,  []() { return ChargingState::STOP; }}
    };
    transitions[ChargingState::START] = {
        {ChargingState::STANDBY, []() { return ChargingState::STANDBY; }},
        {ChargingState::STOP,  []() { return ChargingState::STOP; }}
    };
    transitions[ChargingState::STANDBY] = {
        {ChargingState::INSULATIONTESTINPROGRESS, []() { return ChargingState::INSULATIONTESTINPROGRESS; }},
        {ChargingState::STOP, []() { return ChargingState::STOP; }}
    };
    transitions[ChargingState::INSULATIONTESTINPROGRESS] = {
        {ChargingState::INSULATIONTESTPASS, []() { return ChargingState::INSULATIONTESTPASS; }},
        {ChargingState::STOP, []() { return ChargingState::STOP; }}
    };
    transitions[ChargingState::INSULATIONTESTPASS] = {
        {ChargingState::CHARGERSTATIONON, []() { return ChargingState::CHARGERSTATIONON; }},
        {ChargingState::STOP, []() { return ChargingState::STOP; }}
    };
    transitions[ChargingState::CHARGERSTATIONON] = {
        {ChargingState::CHARGING, []() { return ChargingState::CHARGING; }},
        {ChargingState::STOP, []() { return ChargingState::STOP; }}
    };
    transitions[ChargingState::CHARGING] = {
        {ChargingState::IDLE, []() { return ChargingState::IDLE; }},
        {ChargingState::STOP, []() { return ChargingState::STOP; }}
    };
    transitions[ChargingState::STOP] = {
        {ChargingState::IDLE, []() { return ChargingState::IDLE; }}
    };
}

void ChargingModeBase::Update(const ChargingState state) {
    if (transitions.find(current_state_) != transitions.end()) {
        auto& cmds = transitions[current_state_];
        if (cmds.find(state) != cmds.end()) {
            current_state_ = cmds[state]();
            RCLCPP_INFO(logger_, 
                "state %s exec. New state: %s", ToString(state).c_str(), ToString(current_state_).c_str());
        } else {
            RCLCPP_INFO(logger_, "Next state cmd is not valid.");
            return;
        }
    } else {
        RCLCPP_INFO(logger_, "Charging state %s update failed.", ToString(current_state_).c_str());
        return;
    }
}

ChargingState ChargingModeBase::GetCurrentState() const {
    return current_state_; 
}

std::string ChargingModeBase::ToString(ChargingState state){
    switch (state) {  
        case ChargingState::IDLE: return "IDLE";  
        case ChargingState::WAKE: return "WAKE";  
        case ChargingState::START: return "START";  
        case ChargingState::STANDBY: return "STANDBY";  
        case ChargingState::INSULATIONTESTINPROGRESS: return "INSULATIONTESTINPROGRESS";  
        case ChargingState::INSULATIONTESTPASS: return "INSULATIONTESTPASS";  
        case ChargingState::CHARGERSTATIONON: return "CHARGERSTATIONON";
        case ChargingState::CHARGING: return "CHARGING";
        case ChargingState::STOP: return "STOP";  
    }  
    return "UNKNOWN"; 
}

}//