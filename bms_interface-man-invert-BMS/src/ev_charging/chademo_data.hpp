#ifndef _DATA_STRUCT_HPP_
#define _DATA_STRUCT_HPP_

namespace HKPC{

struct CHAdeMO_401{
  int cha_req = 0;
  int cha_imix = 0;
  int chb_req = 0;
  int chb_imix = 0;

  int measured_voltage = 0;
  int measured_output_current =0;
  int ground_fault_self_test =0;
  int ground_fault_status = 0;
  int insulation_test_status = 0;
  int charger_status =0;
  int charger_enable_display = 0;
  int charger_fault_status =0;
};

struct CHAdeMO_406{
  int PlugType = 0;
  int PlugState = 0;
  int CHAdeMO = 0;
  int Combo = 0;
  int Charger_Fault = 0;
  int Vehicle_Fault = 0;
  int ProtocolNumber = 0;
  int VersionInfoMinor = 0;
  int ChargerStatusExt = 0;
  int SequenceNo = 0;
  int Present_SOC = 0;
  int Remaining_charginf_time = 0;
  int Version_Info_Major = 0;
  int Remaining_charging_time = 0;
  int Sequence_no = 0;
};

struct CHAdeMO_408{
  int PlugType_CCS = 0;
  int PlugState_CCS = 0;
  int CHAdeMO_CCS = 0;
  int Combo_CCS = 0;
  int Charger_Fault_CCS = 0;
  int Vehicle_Fault_CCS = 0;
  int ProtocolNumber_CCS = 0;
  int VersionInfoMinor_CCS = 0;
  int ChargerStatusExt_CCS = 0;
  int SequenceNo_CCS = 0;
  int Present_SOC_CCS = 0;
  int Remaining_charginf_time_CCS = 0;
  int Version_Info_Major_CCS = 0;
  int Remaining_charging_time_CCS = 0;
  int Sequence_no_CCS = 0; 
};

struct CHAdeMO_201{
  int ChargerOnOffRequest = 0;
  int RequestInsulationTest = 0;
  int RequestChargerRest = 0;
  int RequestGroundFault = 0;
  int RelayStatus = 0;
  int WorkingMode = 0;
  int PreCharge = 0; 
};

struct CHAdeMO_202{
  int ChargerOnOffRequest_CCS = 0;  
  int RequestInsulationTest_CCS = 0;
  int RequestChargerRest_CCS = 0;
  int RequestGroundFault_CCS = 0;
  int RelayStatus_CCS = 0;
  int WorkingMode_CCS = 0;
  int PreCharge_CCS = 0;

  int template_date = 0;

  int insulation_self_test_flag = 0;
  int Fault_Flag = 0;
  int insulation_flag = 0;
  int charger_status_flag = 0;
  int charger_enable = 0;
};

}//

#endif