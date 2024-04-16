#include "modbus_interface.hpp"
#include <iostream>
#include <stdexcept> 

namespace HKPC {

ModbusPtr ModbusInterface::createModbusRtuContext(){
    modbus_t* ctx = modbus_new_rtu(port_.c_str(), baudrate_, parity_, size_, stop_bit_);  
    if (ctx == nullptr) { 
        std::cerr << "Failed to create Modbus RTU context." << std::endl;  
        return nullptr;  
    }  

    return ModbusPtr(ctx);
}

void ModbusInterface::setSlaveAddress(ModbusPtr& ctx, int address) {  
    if (!ctx) {  
        throw std::runtime_error("Modbus context is null");  
    }  
    if (modbus_set_slave(ctx.get(), address) == -1) {  
        throw std::runtime_error("Failed to set slave address");  
    }  
}  
  
void ModbusInterface::setRS485Mode(ModbusPtr& ctx) {  
    if (!ctx) {  
        throw std::runtime_error("Modbus context is null");  
    }  
    if (modbus_rtu_set_serial_mode(ctx.get(), MODBUS_RTU_RS485) == -1) {  
        throw std::runtime_error("Failed to set RS485 mode");  
    }  
}  

void ModbusInterface::setRTSUp(ModbusPtr& ctx) {  
    if (!ctx) {  
        throw std::runtime_error("Modbus context is null");  
    }  
    if (modbus_rtu_set_rts(ctx.get(), MODBUS_RTU_RTS_UP) == -1) {  
        throw std::runtime_error("Failed to set RTS up");  
    }  
}  
  
void ModbusInterface::setRTSDelay(ModbusPtr& ctx, int delay_ms) {  
    if (!ctx) {  
        throw std::runtime_error("Modbus context is null");  
    }  
    if (modbus_rtu_set_rts_delay(ctx.get(), delay_ms) == -1) {  
        throw std::runtime_error("Failed to set RTS delay");  
    }  
}  

int ModbusInterface::getRTSDelay(ModbusPtr& ctx) {  
    if (!ctx) {  
        throw std::runtime_error("Modbus context is null");  
    }  
    return modbus_rtu_get_rts_delay(ctx.get());  
}  
 
bool ModbusInterface::connectModbus(ModbusPtr& ctx) {  
    if (!ctx) {  
        throw std::runtime_error("Modbus context is null");  
    }  
    if (modbus_connect(ctx.get()) == -1) {  
        std::cerr << "Connection failed: " << modbus_strerror(errno) << std::endl;  
        return false;  
    }  
    return true;  
}

}//