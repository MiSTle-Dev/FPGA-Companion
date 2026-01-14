/*
  jtag.c

  Send JTAG commands.
*/

#include "jtag.h"
#include "debug.h"
#include "mcu_hw.h"

#include <FreeRTOS.h>
#include <timers.h>

#define IDCODE_GW2AR18  0x81b
#define IDCODE_GW5AT60  0x1481b
#define IDCODE_GW5AST138  0x1081b
#define IDCODE_GW5A25 0x1281b

static void jtag_gowin_command(uint8_t cmd) {
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

static void jtag_gowin_shiftDR(uint8_t *tx, uint8_t *rx, uint16_t len) {
  // This currently assumes that len is a multiple of 8
  if(len & 7) jtag_highlight_debugf("Warning, shiftDR len not a multiple of 8");

  // stay in RUN-TEST/IDLE like openFPGAloader
  mcu_hw_jtag_tms(1, 0b000000, 6);

  // send TMS 1/0/0 to get from RUN-TEST/IDLE into SHIFT-DR state  
  mcu_hw_jtag_tms(1, 0b001, 3);

  // shift data out of DR, the last data bit will be transferred
  // with the begin of the state change
  mcu_hw_jtag_data(tx, rx, len-1);
  
  // send TMS 1/1/0 to return into RUN-TEST/IDLE state, the first
  // transaction carries the last payload bit
  uint8_t lrx = mcu_hw_jtag_tms(tx?((tx[(len-1)/8]&0x80)?1:0):1, 0b1, 1);
  if(rx) rx[(len-1)/8] = (rx[(len-1)/8] & 0x7f) | (lrx & 0x80);   

  mcu_hw_jtag_tms(1, 0b01, 2);
}

#define JTAG_FLAG_BEGIN 1   // get from RUN-TEST/IDLE into SHIFT-DR before transfer
#define JTAG_FLAG_END   2   // return into RUN-TEST/IDLE after transfer

static void jtag_gowin_shiftDR_part(uint8_t *tx, uint8_t *rx, uint16_t len, uint8_t flags) {
  // This currently assumes that len is a multiple of 8
  if(len & 7) jtag_highlight_debugf("Warning, shiftDR_part len not a multiple of 8");
  
  if(flags & JTAG_FLAG_BEGIN) {
    // stay in RUN-TEST/IDLE like openFPGAloader, without this, the core won't start
    mcu_hw_jtag_tms(1, 0b000000, 6);
    
    // send TMS 1/0/0 to get from RUN-TEST/IDLE into SHIFT-DR state  
    mcu_hw_jtag_tms(1, 0b001, 3);
  }

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

static uint32_t jtag_gowin_command_read32(uint8_t cmd) {
  uint32_t retval = 0;
  
  jtag_gowin_command(cmd);
  jtag_gowin_shiftDR(NULL, (uint8_t*)&retval, 32);

  return retval;
}

uint32_t jtag_identify(void) {
  // send a bunch of 1's to return into Test-Logic-Reset state.
  mcu_hw_jtag_tms(1, 0b11111, 5);
  
  // send TMS 0/1/0/0 to get into SHIFT-DR state
  mcu_hw_jtag_tms(1, 0b0010, 4);

  // shift data into DR
  uint32_t idcode;
  mcu_hw_jtag_data(NULL, (uint8_t*)&idcode, 32);

  // send TMS 1/1/0 to go into Run-Test-Idle state
  mcu_hw_jtag_tms(1, 0b011, 3);
  
  return idcode;
}

bool jtag_open(void) {
  // configure pins for JTAG operation
  mcu_hw_jtag_set_pins(0x0b, 0x08);

  // read FPGA idcode via JTAG. Should be 0x81b for the GW2AR-18
  uint32_t idcode = jtag_identify();

  // return into Test-Logic-Reset state
  mcu_hw_jtag_tms(1, 0b11111, 5);
  
  return(idcode == IDCODE_GW2AR18  || 
         idcode == IDCODE_GW5AT60  || 
         idcode == IDCODE_GW5A25  ||
         idcode == IDCODE_GW5AST138);
}

void jtag_close(void) {
  // send a bunch of 1's to return into Test-Logic-Reset state.
  mcu_hw_jtag_tms(1, 0b11111, 5);  
  
  // return all pins to input state to not interfere with
  // e.g. an externally connected JTAG programmer
  mcu_hw_jtag_set_pins(0x00, 0x00);
}

static uint32_t jtag_gowin_readStatusReg(void) {
  uint32_t status = jtag_gowin_command_read32(JTAG_COMMAND_GOWIN_STATUS);

#if 0
  jtag_debugf("Status: %08lx", status);
  if(status & JTAG_GOWIN_STATUS_CRC_ERROR)           jtag_debugf(" CRC ERROR");
  if(status & JTAG_GOWIN_STATUS_BAD_COMMAND)         jtag_debugf(" BAD COMMAND");
  if(status & JTAG_GOWIN_STATUS_ID_VERIFY_FAILED)    jtag_debugf(" ID VERIFY FAILED");
  if(status & JTAG_GOWIN_STATUS_TIMEOUT)             jtag_debugf(" TIMEOUT");
  if(status & JTAG_GOWIN_STATUS_MEMORY_ERASE)        jtag_debugf(" MEMORY ERASE");
  if(status & JTAG_GOWIN_STATUS_PREAMBLE)            jtag_debugf(" PREAMBLE");
  if(status & JTAG_GOWIN_STATUS_SYSTEM_EDIT_MODE)    jtag_debugf(" SYSTEM EDIT MODE");
  if(status & JTAG_GOWIN_STATUS_PRG_SPIFLASH_DIRECT) jtag_debugf(" PRG SPIFLASH DIRECT");
  if(status & JTAG_GOWIN_STATUS_NON_JTAG_CNF_ACTIVE) jtag_debugf(" NON JTAG CNF ACTIVE");
  if(status & JTAG_GOWIN_STATUS_BYPASS)              jtag_debugf(" BYPASS");
  if(status & JTAG_GOWIN_STATUS_GOWIN_VLD)           jtag_debugf(" GOWIN VLD");
  if(status & JTAG_GOWIN_STATUS_DONE_FINAL)          jtag_debugf(" DONE FINAL");
  if(status & JTAG_GOWIN_STATUS_SECURITY_FINAL)      jtag_debugf(" SECURITY FINAL");
  if(status & JTAG_GOWIN_STATUS_READY)               jtag_debugf(" READY");
  if(status & JTAG_GOWIN_STATUS_POR)                 jtag_debugf(" POR");
  if(status & JTAG_GOWIN_STATUS_FLASH_LOCK)          jtag_debugf(" FLASH LOCK");
#endif
  
  return status;
}

static void jtag_gowin_gw2a_force_state(void) {
  /* undocumented sequence but required when
   * flash failure
   */
  uint32_t state =  jtag_gowin_readStatusReg();
  if ((state & JTAG_GOWIN_STATUS_CRC_ERROR) == 0)
    return;
  jtag_gowin_command(JTAG_COMMAND_GOWIN_CONFIG_DISABLE);
  jtag_gowin_command(0);
  jtag_gowin_command_read32(JTAG_COMMAND_GOWIN_IDCODE);
  state = jtag_gowin_readStatusReg();
  jtag_gowin_command(JTAG_COMMAND_GOWIN_CONFIG_DISABLE);
  jtag_gowin_command(0);
  state = jtag_gowin_readStatusReg();
  jtag_gowin_command_read32(JTAG_COMMAND_GOWIN_IDCODE);
  jtag_gowin_command(JTAG_COMMAND_GOWIN_CONFIG_ENABLE);
  jtag_gowin_command(JTAG_COMMAND_GOWIN_CONFIG_DISABLE);
  jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP);
  jtag_gowin_command_read32(JTAG_COMMAND_GOWIN_IDCODE);
  jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP);
  jtag_gowin_command_read32(JTAG_COMMAND_GOWIN_IDCODE);
}

static bool jtag_gowin_pollFlag(uint32_t mask, uint32_t value) {
  uint32_t status;
  int timeout = 0;
  do {
    status = jtag_gowin_readStatusReg();
    if (timeout == 1000){  // TODO: was 100000000
      jtag_debugf("timeout");
      return false;
    }
    timeout++;
  } while ((status & mask) != value);
  
  return true;
}

static bool jtag_gowin_enableCfg(void) {
  jtag_gowin_command(JTAG_COMMAND_GOWIN_CONFIG_ENABLE);
  return jtag_gowin_pollFlag(JTAG_GOWIN_STATUS_SYSTEM_EDIT_MODE, JTAG_GOWIN_STATUS_SYSTEM_EDIT_MODE);
}

static bool jtag_gowin_disableCfg(void) {
  jtag_gowin_command(JTAG_COMMAND_GOWIN_CONFIG_DISABLE);
  jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP);
  return jtag_gowin_pollFlag(JTAG_GOWIN_STATUS_SYSTEM_EDIT_MODE, 0);
}

// prepare SRAM upload. Designed after openFPGAloader
bool jtag_gowin_eraseSRAM(void) {
  jtag_gowin_readStatusReg();

  jtag_gowin_gw2a_force_state();
  if(!jtag_gowin_enableCfg()) {
    jtag_debugf("Failed to enable config");
    return false;
  }
    
  jtag_gowin_command(JTAG_COMMAND_GOWIN_ERASE_SRAM);
  jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP);
  if(!jtag_gowin_pollFlag(JTAG_GOWIN_STATUS_MEMORY_ERASE, JTAG_GOWIN_STATUS_MEMORY_ERASE)) {
    jtag_debugf("Failed to trigger SRAM erase");
    return false;
  }

  jtag_gowin_command(JTAG_COMMAND_GOWIN_XFER_DONE);
  jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP);
  if(!jtag_gowin_disableCfg()) {
    jtag_debugf("Failed to disable config");
    return false;
  }

  return true;
}

bool jtag_gowin_writeSRAM_prepare(void) {
  jtag_gowin_command(JTAG_COMMAND_GOWIN_CONFIG_ENABLE); // config enable

  /* UG704 3.4.3 */
  jtag_gowin_command(JTAG_COMMAND_GOWIN_INIT_ADDR); // address initialize

  /* 2.2.6.4 */
  jtag_gowin_command(JTAG_COMMAND_GOWIN_XFER_WRITE); // transfer configuration data

  return true;
}

bool jtag_gowin_writeSRAM_transfer(uint8_t *data, uint16_t len, bool first, bool last) {

  jtag_gowin_shiftDR_part(data, NULL, len, (first?JTAG_FLAG_BEGIN:0) | (last?JTAG_FLAG_END:0));
  
  return true;
}

bool jtag_gowin_writeSRAM_postproc(uint32_t checksum) {
  // The following is being implemented by openFPGAloader. But it doesn't seem to be
  // necessary and it's also not mentioned in the Gowin JTAG programming guide TN653
  if(checksum != 0xffffffff) {
    jtag_gowin_command(0x0a);
    jtag_gowin_shiftDR((uint8_t *)&checksum, NULL, 32);
    jtag_gowin_command(0x08);
  }
  
  jtag_gowin_command(JTAG_COMMAND_GOWIN_CONFIG_DISABLE); // config disable
  jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP); // noop
  
  uint32_t status_reg = jtag_gowin_readStatusReg();  
  if(!(status_reg & JTAG_GOWIN_STATUS_DONE_FINAL)) {
    fatal_debugf("Failed to write SRAM, status = 0x%04lx", status_reg);
    return false;
  }

  jtag_debugf("SRAM successfully written, status = 0x%04lx", status_reg);
  return true;
}

void jtag_gowin_fpgaReset(void) {
    jtag_debugf("FPGA RECONFIG");

    jtag_gowin_command(JTAG_COMMAND_GOWIN_RECONFIG);
    jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP);
    // send TMS 1/1/0 to go into Run-Test-Idle state
    mcu_hw_jtag_tms(1, 0b011, 3);
    jtag_toggleClk(1000000);
}
