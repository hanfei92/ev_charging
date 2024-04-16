#Constants of current_state
uint8 CHASSIS_STATE_INIT = 0x00
uint8 CHASSIS_STATE_ATHOME = 0x01
uint8 CHASSIS_STATE_ATCHARGINGPOSE= 0x02
uint8 CHASSIS_STATE_ONTHEWAY = 0x03
uint8 CHASSIS_STATE_ERROR= 0xEE
#Constants of error_code
uint8 CHASSIS_ERROR_CODE_NOERROR=0x00
uint8 CHASSIS_ERROR_CODE_LOWBATTERY=0x01
uint8 CHASSIS_ERROR_CODE_LOSTLOCATION=0x02
uint8 CHASSIS_ERROR_CODE_CANNOTREACHGOAL=0x03
uint8 CHASSIS_ERROR_CODE_UNKNOWNERROR=0xEE

std_msgs/Header header
uint8   current_state
float32 current_pose_x
float32 current_pose_y
float32 current_pose_angle
float32 chassis_battery_v
uint8   error_code
