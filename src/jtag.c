/*
  jtag.c

  Send JTAG commands.
*/

#include "jtag.h"
#include "debug.h"
#include "mcu_hw.h"

#include <FreeRTOS.h>
#include <timers.h>

void jtag_command_u08(uint8_t cmd) {
  // stay in RUN-TEST/IDLE like openFPGAloader
  mcu_hw_jtag_tms(1, 0b000000, 6);

  // send TMS 1/1/0/0 to get from RUN-TEST/IDLE into SHIFT-IR state
  mcu_hw_jtag_tms(1, 0b0011, 4);

  // shift command into IR
  mcu_hw_jtag_data(&cmd, NULL, 7);

  // send TMS 1/1/0 to return into RUN-TEST/IDLE
  mcu_hw_jtag_tms((cmd&0x80)?1:0, 0b1, 1);
  mcu_hw_jtag_tms(1, 0b01, 2);
}

// go into shift dr state
static inline void jtag_enter_shiftDR(void) {
  // stay in RUN-TEST/IDLE like openFPGAloader
  mcu_hw_jtag_tms(1, 0b000000, 6);
  
  // send TMS 1/0/0 to get from RUN-TEST/IDLE into SHIFT-DR state  
  mcu_hw_jtag_tms(1, 0b001, 3); 
}

// Shift data into and out of the JTAG data register (DR)
void jtag_shiftDR(uint8_t *tx, uint8_t *rx, uint16_t len) {
  // This currently assumes that len is a multiple of 8
  if(len & 7) jtag_highlight_debugf("Warning, shiftDR len not a multiple of 8");

  jtag_enter_shiftDR();
  
  // shift data out of DR, the last data bit will be transferred
  // with the begin of the state change
  mcu_hw_jtag_data(tx, rx, len-1);
  
  // send TMS 1/1/0 to return into RUN-TEST/IDLE state, the first
  // transaction carries the last payload bit
  uint8_t lrx = mcu_hw_jtag_tms(tx?((tx[(len-1)/8]&0x80)?1:0):1, 0b1, 1);
  if(rx) rx[(len-1)/8] = (rx[(len-1)/8] & 0x7f) | (lrx & 0x80);   

  mcu_hw_jtag_tms(1, 0b01, 2);
}

// Shift data into and out of the JTAG data register (DR). Unlike jtag_shiftDR, this
// can work on multiple data chunks with the first one being flagged with
// JTAG_FLAG_BEGIN and the last one JTAG_FLAG_END. If both flags are set, then
// jtag_shiftDR_part() will behave like jtag_shiftDR()

void jtag_shiftDR_part(uint8_t *tx, uint8_t *rx, uint16_t len, uint8_t flags) {
  // This currently assumes that len is a multiple of 8
  if(len & 7) jtag_highlight_debugf("Warning, shiftDR_part len not a multiple of 8");
  
  if(flags & JTAG_FLAG_BEGIN)
    jtag_enter_shiftDR();

  if(flags & JTAG_FLAG_END) {
    mcu_hw_jtag_data(tx, rx, len-1);

    // send TMS 1/1/0 to return into RUN-TEST/IDLE state, the first
    // transaction carries the last payload bit
    uint8_t lrx = mcu_hw_jtag_tms(tx?((tx[(len-1)/8]&0x80)?1:0):1, 0b1, 1);
    if(rx) rx[(len-1)/8] = (rx[(len-1)/8] & 0x7f) | (lrx & 0x80);   
    mcu_hw_jtag_tms(1, 0b01, 2);
  } else
    mcu_hw_jtag_data(tx, rx, len);  
}

uint32_t jtag_command_u08_read32(uint8_t cmd) {
  uint32_t retval = 0;
  
  jtag_command_u08(cmd);
  jtag_shiftDR(NULL, (uint8_t*)&retval, 32);

  return retval;
}

uint32_t jtag_identify(void) {
  // send a bunch of 1's to return into Test-Logic-Reset state.
  mcu_hw_jtag_tms(1, 0b11111, 5);
  
  // send TMS 0/1/0/0 to get into SHIFT-DR state
  mcu_hw_jtag_tms(1, 0b0010, 4);

  // shift data into DR
  uint32_t lidcode;
  mcu_hw_jtag_data(NULL, (uint8_t*)&lidcode, 32);

  // send TMS 1/1/0 to go into Run-Test-Idle state
  mcu_hw_jtag_tms(1, 0b011, 3);
  
  return lidcode;
}

uint32_t jtag_open(void) {
  // configure pins for JTAG operation
  mcu_hw_jtag_set_pins(0x0b, 0x08);

  // read FPGA idcode via JTAG
 uint32_t idcode = jtag_identify();

  // return into Test-Logic-Reset state
  mcu_hw_jtag_tms(1, 0b11111, 5);
  
  return idcode;
}
  
void jtag_close(void) {
  // send a bunch of 1's to return into Test-Logic-Reset state.
  mcu_hw_jtag_tms(1, 0b11111, 5);  
  
  // return all pins to input state to not interfere with
  // e.g. an externally connected JTAG programmer
  mcu_hw_jtag_set_pins(0x00, 0x00);
}
