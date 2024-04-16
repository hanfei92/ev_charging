#include "ccs_interface.hpp"

namespace HKPC {

OutputData CombinedCS::run(ChargingState const &state, charging_robot_system_msgs::msg::InverterState const &inv_msg) {
    output_data_.can1_msgs.header.stamp = this->clock_->now();
    output_data_.can1_msgs.is_error = false;
    output_data_.can1_msgs.is_extended = true;
    output_data_.can1_msgs.is_rtr = false;
    output_data_.can1_msgs.dlc = 8;
    output_data_.can1_msgs.id = 0x00000401;

    output_data_.can2_msgs.header.stamp = this->clock_->now();
    output_data_.can2_msgs.is_error = false;
    output_data_.can2_msgs.is_extended = true;
    output_data_.can2_msgs.is_rtr = false;
    output_data_.can2_msgs.dlc = 8;
    output_data_.can2_msgs.id = 0x00000209;

    switch (state) {
        case ChargingState::IDLE:
            output_data_.can1_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000
            output_data_.can1_msgs.data.at(1) = 0x00; // Channel A Max Current      Channel A Max current
            output_data_.can1_msgs.data.at(2) = 0x00; // 3rd 8 bits 00000000
            output_data_.can1_msgs.data.at(3) = 0x00; // Channel B Max Current     Channel B Max current
            output_data_.can1_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can1_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can1_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can1_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

            output_data_.can2_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000  measured voltage
            output_data_.can2_msgs.data.at(1) = 0x00; // Channel A Max Current      
            output_data_.can2_msgs.data.at(2) = 0x00; // 3rd 8 bits 00000000  measured current
            output_data_.can2_msgs.data.at(3) = 0x00; // Channel B Max Current     Channel B Max current
            output_data_.can2_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can2_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can2_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can2_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
            return output_data_;

        case ChargingState::WAKE:
            output_data_.can1_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000
            output_data_.can1_msgs.data.at(1) = 0x00; // Channel A Max Current      Channel A Max current
            output_data_.can1_msgs.data.at(2) = 0x01; // 3rd 8 bits 00000000
            output_data_.can1_msgs.data.at(3) = 0x0C; // Channel B Max Current     Channel B Max current
            output_data_.can1_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can1_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can1_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can1_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

            output_data_.can2_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000  measured voltage
            output_data_.can2_msgs.data.at(1) = 0x00; // Channel A Max Current      
            output_data_.can2_msgs.data.at(2) = 0x00; // 3rd 8 bits 00000000  measured current
            output_data_.can2_msgs.data.at(3) = 0x00; // Channel B Max Current     Channel B Max current
            output_data_.can2_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can2_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can2_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can2_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
            return output_data_;

        case ChargingState::START:
            output_data_.can1_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000
            output_data_.can1_msgs.data.at(1) = 0x00; // Channel A Max Current      Channel A Max current
            output_data_.can1_msgs.data.at(2) = 0x02; // 3rd 8 bits 00000000
            output_data_.can1_msgs.data.at(3) = 0x0C; // Channel B Max Current     Channel B Max current
            output_data_.can1_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can1_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can1_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can1_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

            output_data_.can2_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000  measured voltage
            output_data_.can2_msgs.data.at(1) = 0x00; // Channel A Max Current      
            output_data_.can2_msgs.data.at(2) = 0x00; // 3rd 8 bits 00000000  measured current
            output_data_.can2_msgs.data.at(3) = 0x00; // Channel B Max Current     Channel B Max current
            output_data_.can2_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can2_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can2_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can2_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
            return output_data_;

        case ChargingState::STANDBY:
            output_data_.can1_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000
            output_data_.can1_msgs.data.at(1) = 0x00; // Channel A Max Current      Channel A Max current
            output_data_.can1_msgs.data.at(2) = 0x02; // 3rd 8 bits 00000000
            output_data_.can1_msgs.data.at(3) = 0x0C; // Channel B Max Current     Channel B Max current
            output_data_.can1_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can1_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can1_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can1_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

            output_data_.can2_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000  measured voltage
            output_data_.can2_msgs.data.at(1) = 0x00; // Channel A Max Current      
            output_data_.can2_msgs.data.at(2) = 0x00; // 3rd 8 bits 00000000  measured current
            output_data_.can2_msgs.data.at(3) = 0x00; // Channel B Max Current     Channel B Max current
            output_data_.can2_msgs.data.at(4) = 0x60; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can2_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can2_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can2_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
            return output_data_;

        case ChargingState::INSULATIONTESTINPROGRESS:
            output_data_.can1_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000
            output_data_.can1_msgs.data.at(1) = 0x00; // Channel A Max Current      Channel A Max current
            output_data_.can1_msgs.data.at(2) = 0x02; // 3rd 8 bits 00000000
            output_data_.can1_msgs.data.at(3) = 0x0C; // Channel B Max Current     Channel B Max current
            output_data_.can1_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can1_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can1_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can1_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

            output_data_.can2_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000  measured voltage
            output_data_.can2_msgs.data.at(1) = 0x00; // Channel A Max Current      
            output_data_.can2_msgs.data.at(2) = 0x00; // 3rd 8 bits 00000000  measured current
            output_data_.can2_msgs.data.at(3) = 0x00; // Channel B Max Current     Channel B Max current
            output_data_.can2_msgs.data.at(4) = 0x68; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can2_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can2_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can2_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
            return output_data_;

        case ChargingState::INSULATIONTESTPASS:
            output_data_.can1_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000
            output_data_.can1_msgs.data.at(1) = 0x00; // Channel A Max Current      Channel A Max current
            output_data_.can1_msgs.data.at(2) = 0x02; // 3rd 8 bits 00000000
            output_data_.can1_msgs.data.at(3) = 0x0C; // Channel B Max Current     Channel B Max current
            output_data_.can1_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can1_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can1_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can1_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

            output_data_.can2_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000  measured voltage
            output_data_.can2_msgs.data.at(1) = 0x00; // Channel A Max Current      
            output_data_.can2_msgs.data.at(2) = 0x00; // 3rd 8 bits 00000000  measured current
            output_data_.can2_msgs.data.at(3) = 0x00; // Channel B Max Current     Channel B Max current
            output_data_.can2_msgs.data.at(4) = 0x70; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can2_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can2_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can2_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
            return output_data_;

        case ChargingState::CHARGERSTATIONON:
            output_data_.can1_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000
            output_data_.can1_msgs.data.at(1) = 0x00; // Channel A Max Current      Channel A Max current
            output_data_.can1_msgs.data.at(2) = 0x02; // 3rd 8 bits 00000000
            output_data_.can1_msgs.data.at(3) = 0x0C; // Channel B Max Current     Channel B Max current
            output_data_.can1_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can1_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can1_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can1_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

            output_data_.can2_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000  measured voltage
            output_data_.can2_msgs.data.at(1) = 0x00; // Channel A Max Current      
            output_data_.can2_msgs.data.at(2) = 0x00; // 3rd 8 bits 00000000  measured current
            output_data_.can2_msgs.data.at(3) = 0x00; // Channel B Max Current     Channel B Max current
            output_data_.can2_msgs.data.at(4) = 0x30; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can2_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can2_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can2_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
            return output_data_;

        case ChargingState::CHARGING:
            output_data_.can1_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000
            output_data_.can1_msgs.data.at(1) = 0x00; // Channel A Max Current
            output_data_.can1_msgs.data.at(2) = 0x02; // 3rd 8 bits 00000000
            if (inv_msg.outputcurrent < 9) {
                output_data_.can1_msgs.data.at(3) = 0x0C;
            } else {
                output_data_.can1_msgs.data.at(3) = 0x14;
            }
            // Channel B Max Current
            output_data_.can1_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can1_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can1_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can1_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

            U_PCB_DATA inverter_output_voltage;
            inverter_output_voltage.PCB_DATA = inv_msg.outputvoltage * 10;  
            output_data_.can2_msgs.data.at(0) = inverter_output_voltage.S_PCB_DATA.a;
            output_data_.can2_msgs.data.at(1) = inverter_output_voltage.S_PCB_DATA.b;

            U_PCB_DATA inverter_output_current;
            inverter_output_current.PCB_DATA = inv_msg.outputcurrent * 10;  
            output_data_.can2_msgs.data.at(2) = inverter_output_current.S_PCB_DATA.a;
            output_data_.can2_msgs.data.at(3) = inverter_output_current.S_PCB_DATA.b;           

            output_data_.can2_msgs.data.at(4) = 0x30; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can2_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can2_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can2_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
            return output_data_;

        case ChargingState::STOP:
            output_data_.can1_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000
            output_data_.can1_msgs.data.at(1) = 0x00; // Channel A Max Current      Channel A Max current
            output_data_.can1_msgs.data.at(2) = 0x03; // 3rd 8 bits 00000000
            output_data_.can1_msgs.data.at(3) = 0x00; // Channel B Max Current     Channel B Max current
            output_data_.can1_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can1_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can1_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can1_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

            output_data_.can2_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000  measured voltage
            output_data_.can2_msgs.data.at(1) = 0x00; // Channel A Max Current      
            output_data_.can2_msgs.data.at(2) = 0x00; // 3rd 8 bits 00000000  measured current
            output_data_.can2_msgs.data.at(3) = 0x00; // Channel B Max Current     Channel B Max current
            output_data_.can2_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can2_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can2_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can2_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
            return output_data_;

        default:
            output_data_.can1_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000
            output_data_.can1_msgs.data.at(1) = 0x00; // Channel A Max Current      Channel A Max current
            output_data_.can1_msgs.data.at(2) = 0x00; // 3rd 8 bits 00000000
            output_data_.can1_msgs.data.at(3) = 0x00; // Channel B Max Current     Channel B Max current
            output_data_.can1_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can1_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can1_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can1_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number

            output_data_.can2_msgs.data.at(0) = 0x00; // 1st 8 bits 00000000  measured voltage
            output_data_.can2_msgs.data.at(1) = 0x00; // Channel A Max Current      
            output_data_.can2_msgs.data.at(2) = 0x00; // 3rd 8 bits 00000000  measured current
            output_data_.can2_msgs.data.at(3) = 0x00; // Channel B Max Current     Channel B Max current
            output_data_.can2_msgs.data.at(4) = 0x00; // 5th 8 bits 00000000  CAN Data format version
            output_data_.can2_msgs.data.at(5) = 0x00; // 6th 8 bits 00000001  01(CHAdeMO ON) 02(Combo_ON)
            output_data_.can2_msgs.data.at(6) = 0x00; // 7th 8 bits 00000000  Reserved
            output_data_.can2_msgs.data.at(7) = 0x00; // 8th 8 bits 00000000  Packet sequence number
            return output_data_;
    }       
}

}//