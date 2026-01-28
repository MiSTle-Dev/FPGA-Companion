/*
  jtag.c

  Send JTAG commands.
*/

#include "jtag.h"
#include "debug.h"
#include "mcu_hw.h"

#include <FreeRTOS.h>
#include <timers.h>
static bool is_gw2a = false;

volatile uint32_t idcode = 0;

void printStatusReg(uint32_t status);

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
  uint32_t lidcode;
  mcu_hw_jtag_data(NULL, (uint8_t*)&lidcode, 32);

  // send TMS 1/1/0 to go into Run-Test-Idle state
  mcu_hw_jtag_tms(1, 0b011, 3);
  
  return lidcode;
}

bool jtag_open(void) {
  // configure pins for JTAG operation
  mcu_hw_jtag_set_pins(0x0b, 0x08);

  // read FPGA idcode via JTAG
  idcode = jtag_identify();

  if (idcode == IDCODE_GW2AR18) {
    is_gw2a = true;
    jtag_debugf("GW2AR detected");
  }

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
  if(status & JTAG_GOWIN_STATUS_ENCRYPTED_FORMAT)    jtag_debugf(" ENCRYPTED_FORMAT");
  if(status & JTAG_GOWIN_STATUS_KEY_IS_RIGHT)        jtag_debugf(" KEY_IS_RIGHT");
  if(status & JTAG_GOWIN_STATUS_SSPI_MODE)           jtag_debugf(" SSPI_MODE");
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
      printStatusReg(status);
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

void sendClkUs(uint32_t us)
{
  uint64_t clocks = 15000000;
  clocks *= us;
  clocks /= 1000000;
  jtag_toggleClk(clocks);
}

void printStatusReg(uint32_t status) {
    if (status & JTAG_GOWIN_STATUS_CRC_ERROR)        jtag_debugf("Bit 0: CRC ERROR detected");
    if (status & JTAG_GOWIN_STATUS_BAD_COMMAND)      jtag_debugf("Bit 1: Bad command received");
    if (status & JTAG_GOWIN_STATUS_ID_VERIFY_FAILED) jtag_debugf("Bit 2: ID verification failed");
    if (status & JTAG_GOWIN_STATUS_TIMEOUT)          jtag_debugf("Bit 3: Timeout occurred");
    if (status & JTAG_GOWIN_STATUS_AUTO_BOOT_2ND_FAIL) jtag_debugf("Bit 4: Auto boot 2nd failed");
    if (status & JTAG_GOWIN_STATUS_MEMORY_ERASE)     jtag_debugf("Bit 5: Memory erase in progress");
    if (status & JTAG_GOWIN_STATUS_PREAMBLE)         jtag_debugf("Bit 6: Preamble detected");
    if (status & JTAG_GOWIN_STATUS_SYSTEM_EDIT_MODE) jtag_debugf("Bit 7: System edit mode active");
    if (status & JTAG_GOWIN_STATUS_PRG_SPIFLASH_DIRECT) jtag_debugf("Bit 8: Programming SPI flash directly");
    if (status & JTAG_GOWIN_STATUS_AUTO_BOOT_1ST_FAILED) jtag_debugf("Bit 9: Auto boot 1st failed");
    if (status & JTAG_GOWIN_STATUS_NON_JTAG_CNF_ACTIVE) jtag_debugf("Bit 10: Non-JTAG configuration active");
    if (status & JTAG_GOWIN_STATUS_BYPASS)           jtag_debugf("Bit 11: Bypass mode enabled");
    if (status & JTAG_GOWIN_STATUS_I2C_SRAM_F)       jtag_debugf("Bit 12: I2C_SRAM_F");
    if (status & JTAG_GOWIN_STATUS_DONE_FINAL)       jtag_debugf("Bit 13: Done final");
    if (status & JTAG_GOWIN_STATUS_SECURITY_FINAL)   jtag_debugf("Bit 14: Security final");
    if (status & JTAG_GOWIN_STATUS_ENCRYPTED_FORMAT) jtag_debugf("Bit 15: ENCRYPTED_FORMAT");
    if (status & JTAG_GOWIN_STATUS_KEY_IS_RIGHT)     jtag_debugf("Bit 16: KEY_IS_RIGHT");
    if (status & JTAG_GOWIN_STATUS_SSPI_MODE)        jtag_debugf("Bit 17: SSPI_MODE");
    if (status & JTAG_GOWIN_STATUS_SER_CRC_DONE)     jtag_debugf("Bit 18: Serial CRC done");
    if (status & JTAG_GOWIN_STATUS_SER_CRC_ERR)      jtag_debugf("Bit 19: Serial CRC error");
    if (status & JTAG_GOWIN_STATUS_SER_ECC_CORR)     jtag_debugf("Bit 20: ECC corrected");
    if (status & JTAG_GOWIN_STATUS_SER_ECC_UNCORR)   jtag_debugf("Bit 21: ECC uncorrectable");
    if (status & JTAG_GOWIN_STATUS_SER_RUNNING)      jtag_debugf("Bit 22: Serial running");
    if (status & JTAG_GOWIN_STATUS_CPU_BUS_WIDTH_0)  jtag_debugf("Bit 23: CPU_BUS_WIDTH_0");
    if (status & JTAG_GOWIN_STATUS_CPU_BUS_WIDTH_1)  jtag_debugf("Bit 24: CPU_BUS_WIDTH_1");
#ifndef TANG_MEGA138KPRO
    if (status & JTAG_GOWIN_STATUS_SYNC_DET_TERY_0)  jtag_debugf("Bit 25: SYNC_DET_TERY_0");
    if (status & JTAG_GOWIN_STATUS_SYNC_DET_TERY_1)  jtag_debugf("Bit 26: SYNC_DET_TERY_1");
    if (status & JTAG_GOWIN_STATUS_DECOMP_FAIL)      jtag_debugf("Bit 27: Decompression failed");
    if (status & JTAG_GOWIN_STATUS_MFG_DONE)         jtag_debugf("Bit 28: Manufacturing done");
    if (status & JTAG_GOWIN_STATUS_INIT)             jtag_debugf("Bit 29: Initialization complete");
    if (status & JTAG_GOWIN_STATUS_WAKEUP)           jtag_debugf("Bit 30: Wakeup signal");
    if (status & JTAG_GOWIN_STATUS_AUTO_ERASE)       jtag_debugf("Bit 31: Auto erase enabled");
#endif
}

// prepare SRAM upload
bool jtag_gowin_eraseSRAM(void) {
  uint32_t status;
  jtag_gowin_command_read32(JTAG_COMMAND_GOWIN_USERCODE);

  // Clearing if failed loading
  status = jtag_gowin_readStatusReg();

  if ((idcode == IDCODE_GW5AST138)||(idcode == IDCODE_GW5A25)) {
    if ((status & JTAG_GOWIN_STATUS_DONE_FINAL) == 0) {
      jtag_debugf("FPGA REINIT");
      jtag_gowin_command(JTAG_COMMAND_GOWIN_REINIT);
      sendClkUs(10000);
    }
  }
  // mandatory for GW5A-60
  if (idcode == IDCODE_GW5AT60) {
      jtag_debugf("FPGA REINIT");
      jtag_gowin_command(JTAG_COMMAND_GOWIN_REINIT);
      sendClkUs(10000);
    }

  // Clearing Status Code Errors
  if (!is_gw2a) {
    status = jtag_gowin_readStatusReg();
    bool auto_boot_2nd_fail = (status & JTAG_GOWIN_STATUS_AUTO_BOOT_2ND_FAIL) == JTAG_GOWIN_STATUS_AUTO_BOOT_2ND_FAIL;
    bool is_timeout = (status & JTAG_GOWIN_STATUS_TIMEOUT) == JTAG_GOWIN_STATUS_TIMEOUT;
    bool bad_cmd = (status & JTAG_GOWIN_STATUS_BAD_COMMAND) == JTAG_GOWIN_STATUS_BAD_COMMAND;
    bool id_verify_failed = (status & JTAG_GOWIN_STATUS_ID_VERIFY_FAILED) ==  JTAG_GOWIN_STATUS_ID_VERIFY_FAILED;
    if (is_timeout || auto_boot_2nd_fail || bad_cmd || id_verify_failed) {
    jtag_debugf("Clearing status errors by FPGA RELOAD");
    printStatusReg(status);
    jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP);
    jtag_gowin_command(JTAG_COMMAND_GOWIN_CONFIG_ENABLE);
    jtag_gowin_command(JTAG_COMMAND_GOWIN_RECONFIG);
    jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP);
    sendClkUs(100000);
    jtag_gowin_command(JTAG_COMMAND_GOWIN_CONFIG_DISABLE);
    jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP);
    sendClkUs(100000);
    }
  }

  if (is_gw2a) {
    jtag_gowin_gw2a_force_state();

    if(!jtag_gowin_enableCfg()) {
      status = jtag_gowin_readStatusReg();
      printStatusReg(status);
      jtag_debugf("Failed to enable config");
      return false;
     }
  } else {
      jtag_gowin_command(JTAG_COMMAND_GOWIN_CONFIG_ENABLE);
      // no status polling !
   }

  jtag_gowin_command(JTAG_COMMAND_GOWIN_ERASE_SRAM);
  jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP);
  sendClkUs(10000); // wait for erase to complete
#ifdef TANG_MEGA138KPRO
  sendClkUs(40000);
#endif

  if(!jtag_gowin_pollFlag(JTAG_GOWIN_STATUS_MEMORY_ERASE, JTAG_GOWIN_STATUS_MEMORY_ERASE)) {
    status = jtag_gowin_readStatusReg();
    printStatusReg(status);
    jtag_debugf("Failed to trigger SRAM erase");
    return false;
  }

  jtag_gowin_command(JTAG_COMMAND_GOWIN_XFER_DONE);
  jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP);
  if(!jtag_gowin_disableCfg()) {
    status = jtag_gowin_readStatusReg();
    printStatusReg(status);
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

uint32_t readUserCode(void)
{
	return jtag_gowin_command_read32(JTAG_COMMAND_GOWIN_USERCODE);
}

bool jtag_gowin_writeSRAM_postproc(uint32_t checksum) {
  jtag_gowin_command(JTAG_COMMAND_GOWIN_CONFIG_DISABLE);
  jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP);

  uint32_t status_reg = jtag_gowin_readStatusReg();
  status_reg = jtag_gowin_readStatusReg(); // read twice to get updated status
  if(!(status_reg & JTAG_GOWIN_STATUS_DONE_FINAL)) {
    printStatusReg(status_reg);
    fatal_debugf("Failed to write SRAM");
    return false;
  }
  uint32_t usercode = readUserCode();
  jtag_debugf("SRAM successfully written, Usercode=0x%08x", usercode);
  return true;
}

void jtag_gowin_fpgaReset(void) {
    jtag_debugf("FPGA RECONFIG");

    jtag_gowin_command(JTAG_COMMAND_GOWIN_RECONFIG);
    jtag_gowin_command(JTAG_COMMAND_GOWIN_NOOP);
    sendClkUs(100000);
}

