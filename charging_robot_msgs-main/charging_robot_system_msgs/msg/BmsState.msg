#BMS Charging mode
uint8 BMS_CHARGING_MODE_NO_CHARGE = 0
uint8 BMS_CHARGING_MODE_DC= 1
uint8 BMS_CHARGING_MODE_AC = 2
uint8 BMS_CHARGING_MODE_UNKNOWN = 3

#BMS CHARGING STATUS
uint8 BMS_CHARGING_STATUS_UNKNOWN = 0
uint8 BMS_CHARGING_STATUS_CHARGING = 1
uint8 BMS_CHARGING_STATUS_DISCHARGING = 2
uint8 BMS_CHARGING_STATUS_NOT_CHARGING = 3
uint8 BMS_CHARGING_STATUS_FULL= 4

#BMS ALARM LEVEL
uint8 BMS_ALARM_NO_FAULT = 0
uint8 BMS_ALARM_LEVEL_1 = 1
uint8 BMS_ALARM_LEVEL_2 = 2
uint8 BMS_ALARM_LEVEL_3 = 3
uint8 BMS_ALARM_LEVEL_4 = 4
uint8 BMS_ALARM_EXCEPTION = 5
uint8 BMS_ALARM_INVALID = 6

#BMS READY SIGNAL
uint8 BMS_READY_SIGNAL_NOT_READY = 0
uint8 BMS_READY_SIGNAL_READY= 1
uint8 BMS_READY_SIGNAL_ERROR= 2
uint8 BMS_READY_SIGNAL_RESERVE = 3

#BMS WARNING ALARM
uint8 BMS_WARNING_ALARM_NO_FAULT = 0
uint8 BMS_WARNING_ALARM_LEVEL_1_FAULT = 1
uint8 BMS_WARNING_ALARM_LEVEL_2_FAULT = 2
uint8 BMS_WARNING_ALARM_LEVEL_3_FAULT = 3
uint8 BMS_WARNING_ALARM_LEVEL_4_FAULT = 4
uint8 BMS_WARNING_ALARM_EXCEPTION = 5
uint8 BMS_WARNING_ALARM_INVALID = 6


std_msgs/Header header
float32 voltage # Voltage in Volts (Mandatory)
float32 current # Negative when discharging (A) (If unmeasured NaN)
float32 charge # Current charge in Ah (If unmeasured NaN)
float32 capacity # Capacity in Ah (last full capacity) (If unmeasured NaN)
float32 design_capacity # Capacity in Ah (design capacity) (If unmeasured NaN)
float32 soc # Charge percentage on 0 to 1 range (If unmeasured NaN)
float32 soh
float32 totalcapacity
float32 leftcapacity
float32 maxmihightemp
float32 maxmilowtemp
float32 peakpowerdischarge
float32 countinepowerdischarge
float32 peakpowercharge
float32 countinepowercharge
float32 maxlimitvoltagecharge
float32 maxlimitcurcharge
float32 isolatedresistor
float32 batterypower
float32 consumptioncapdischarge
float32 consumptioncapcharge
float32 statisticaveragevoltage
float32 statisticaveragetemperature
float32 statisticdeltavoltage
float32 statisticdeltatemperature
float32 statisticchargetimes
float32 numberofrelays
uint8 bms_charging_mode # The charging status as reported. Values defined above
uint8 bms_charging_status # The battery health metric. Values defined above
uint8 bms_alarm_level
uint8 bms_ready_signal
uint8 bms_warning_alarm
bool onoffstate
bool estopstate