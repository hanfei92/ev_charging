#ifndef _HKPC_MODBUS_HPP
#define _HKPC_MODBUS_HPP

#include <memory>
#include <modbus/modbus.h>
#include <string>

namespace HKPC{

#define MODBUS_RTU_RS232 0
#define MODBUS_RTU_RS485 1

#define MODBUS_RTU_RTS_NONE 0
#define MODBUS_RTU_RTS_UP 1
#define MODBUS_RTU_RTS_DOWN 2

const int REMOTE_ID = 1;

struct ModbusDeleter {  
    void operator()(modbus_t* ptr) const {  
        if (ptr != nullptr) {  
            modbus_close(ptr);
            modbus_free(ptr);  
        }  
    }  
};  
   
using ModbusPtr = std::unique_ptr<modbus_t, ModbusDeleter>;

class ModbusInterface{
public:
    explicit ModbusInterface(std::string port, int baudrate, char parity, int size, int stop_bit) : 
        port_(port), baudrate_(baudrate), parity_(parity), size_(size), stop_bit_(stop_bit){}
    ~ModbusInterface() = default;
    ModbusPtr createModbusRtuContext();
    void setSlaveAddress(ModbusPtr& ctx, int address);
    void setRS485Mode(ModbusPtr& ctx);
    void setRTSUp(ModbusPtr& ctx);
    void setRTSDelay(ModbusPtr& ctx, int delay_ms);
    int getRTSDelay(ModbusPtr& ctx);
    bool connectModbus(ModbusPtr& ctx);
private:
    std::string port_;
    int baudrate_;
    char parity_;
    int size_;
    int stop_bit_;
};
}//

#endif