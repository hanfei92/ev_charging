#AC INPUT FAILURE
uint8 AC_INPUT_FAILURE_AC_OVER_VOLTAGE_NORMAL = 0
uint8 AC_INPUT_FAILURE_AC_OVER_VOLTAGE_ABNORMAL = 1
uint8 AC_INPUT_FAILURE_AC_UNDER_VOLTAGE_NORMAL = 2
uint8 AC_INPUT_FAILURE_AC_UNDER_VOLTAGE_ABNORMAL = 3

#Disconnects from AC (AC overvoltage shutdown)
uint8 AC_MODULE_PROTECTION_NORMAL = 0
uint8 AC_MODULE_PROTECTION_ABNORMAL = 1

#PFC Bus over voltage
uint8 PFC_BUS_OVERVOLTAGE_NORMAL = 0
uint8 PFC_BUS_OVERVOLTAGE_ABNORMAL = 1

#PFC Bus under voltage
uint8 PFC_BUS_UNDERVOLTAGE_NORMAL = 0
uint8 PFC_BUS_UNDERVOLTAGE_ABNORMAL = 1

#PFC Bus unbalance
uint8 PFC_BUS_UNBALANCE_NORMAL = 0
uint8 PFC_BUS_UNBALANCE_ABNORMAL = 1

#DC output overvoltage
uint8 DC_OUTPUT_VOLTAGE_NORMAL = 0
uint8 DC_OUTPUT_VOLTAGE_ABNORMAL = 1

#DC over voltage shutdown
uint8 DC_MODULE_PROTECTION_NORMAL = 0
uint8 DC_MODULE_PROTECTION_ABNORMAL = 1

#DC output undervoltage
uint8 DC_OUTPUT_UNDERVOLTAGE_NORMAL = 0
uint8 DC_OUTPUT_UNDERVOLTAGE_ABNORMAL = 1

#Fan stops operating
uint8 PFC_NOT_RUN_NORMAL = 0
uint8 PFC_NOT_RUN_ABNORMAL = 1

std_msgs/Header header
float32 outputvoltage
float32 outputcurrent
float32 referencevoltage
float32 limitoutputcurrent

uint8 ac_input_failure
uint8 ac_module_protection
uint8 pfc_bus_overvoltage
uint8 pfc_bus_undervoltage
uint8 pfc_bus_unbalance
uint8 dc_output_voltage
uint8 dc_module_protection
uint8 dc_output_undervoltage
uint8 fan_failure
uint8 fan_driven_circuit_damaged
uint8 pfc_not_run
