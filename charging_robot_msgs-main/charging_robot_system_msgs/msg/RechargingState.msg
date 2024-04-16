#Constants of bsm_state
uint8 RC_STATE_IDLING = 0x00
uint8 RC_STATE_NOT_READY = 0x01
uint8 RC_STATE_READY = 0x02


#Constants of error_code
uint8 RC_ERROR_CODE_NOERROR=0x00
uint8 RC_ERROR_CODE_BMSERROR=0x01
uint8 RC_ERROR_CODE_CHARGINGCOMMUNICATIONERROR=0x02
uint8 RC_ERROR_CODE_INVERTERERROR=0x03
uint8 RC_ERROR_CODE_UNKNOWNERROR=0xEE

std_msgs/Header header
uint8 rc_state
float32 soc
float32 current
uint8   error_code
