// #include <stdio>
#include <iostream>

union U_INVERTER_DATA {
  unsigned int INVERTER_DATA;
  struct{
    unsigned int d:8;
    unsigned int c:8;
    unsigned int b:8;
    unsigned int a:8;
  }S_INVERTER_DATA;
};

union U_PCB_DATA {
  unsigned short PCB_DATA;
  struct{
    unsigned short a:8;
    unsigned short b:8;
  }S_PCB_DATA;
};

int main(){
    #if 0
    int a = 380000;
    U_INVERTER_DATA request_current;
    request_current.INVERTER_DATA = a;
    std::cout << std::hex << request_current.S_INVERTER_DATA.a << std::endl;
    std::cout << std::hex << request_current.S_INVERTER_DATA.b << std::endl;
    std::cout << std::hex << request_current.S_INVERTER_DATA.c << std::endl;
    std::cout << std::hex << request_current.S_INVERTER_DATA.d << std::endl;
    #endif

    int a = 3800;
    U_PCB_DATA vol;
    vol.PCB_DATA = a;
    std::cout << std::hex << vol.S_PCB_DATA.a << std::endl;
    std::cout << std::hex << vol.S_PCB_DATA.b << std::endl;



    return 0;
}