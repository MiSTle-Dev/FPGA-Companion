/*
  jtag.c
*/

#include "jtag.h"
#include "debug.h"
#include "mcu_hw.h"

uint32_t jtag_identify(void) {
  // send a bunch of 1's to return into Test-Logic-Reset state
  mcu_hw_jtag_tms(1, 0b111111, 6);
  
  // send TMS 0/1/0/0
   mcu_hw_jtag_tms(1, 0b0100, 4);
  
  uint8_t rx_data[4];
  mcu_hw_jtag_data(NULL, rx_data, 32);
  hexdump(rx_data, 4);

  // send TMS 1/1/1/1/1
  mcu_hw_jtag_tms(1, 0b11111, 5);
  
  return *(uint32_t*)rx_data;
}

void jtag_test(void) {
  jtag_debugf("=== JTAG TEST ===");
  
  // read FPGA idcode via JTAG. Should be 0x81b for the GW2AR-LV18
  uint32_t idcode = jtag_identify();
  jtag_debugf("IDCODE 0x%lx", idcode);  
  
  // trigger FPGA reconfiguration
  //  gpio_put(PIN_nCFG, 0); gpio_put(PIN_nCFG, 1);  
}
