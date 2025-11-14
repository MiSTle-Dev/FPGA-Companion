/*
  jtag.c
*/

#include "jtag.h"
#include "debug.h"
#include "mcu_hw.h"

uint32_t jtag_identify(void) {
  // send a bunch of 1's to return into Test-Logic-Reset state
  mcu_hw_jtag_tms(1, 0b11111, 5);
  
  // send TMS 0/1/0/0
  mcu_hw_jtag_tms(1, 0b0010, 4);
  
  uint8_t rx_data[4];
  mcu_hw_jtag_data(NULL, rx_data, 32);
  hexdump(rx_data, 4);

  // send TMS 1/1/1/1/1
  mcu_hw_jtag_tms(1, 0b11111, 5);
  
  return *(uint32_t*)rx_data;
}

void jtag_test(void) {
  jtag_debugf("=== JTAG TEST ===");

  // configure pins for JTAG operation
  mcu_hw_jtag_set_pins(0x0b, 0x0f);

  // read FPGA idcode via JTAG. Should be 0x81b for the GW2AR-LV18
  uint32_t idcode = jtag_identify();
  jtag_debugf("IDCODE 0x%lx", idcode);  
  
  // trigger FPGA reconfiguration
  //  gpio_put(PIN_nCFG, 0); gpio_put(PIN_nCFG, 1);  

  // return all pins to input state to not interfere with
  // e.g. an externally connected JTAG programmer
  mcu_hw_jtag_set_pins(0x00, 0x00);
}
