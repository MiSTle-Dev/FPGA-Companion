/*
  gowin.c

*/


#include <string.h>   // for memXXX()

#include "gowin.h"
#include "../debug.h"
#include "../jtag.h"
#include "../spi.h"     // for freetos includes

#ifdef ENABLE_JTAG
#include <ff.h>

// idcode is stored globally so the FPGA routines can address the right FPGA
static uint32_t idcode = 0;

static void gowin_printStatusReg(uint32_t status) {
    if (status & GOWIN_STATUS_CRC_ERROR)            gowin_debugf("Bit 0: CRC ERROR detected");
    if (status & GOWIN_STATUS_BAD_COMMAND)          gowin_debugf("Bit 1: Bad command received");
    if (status & GOWIN_STATUS_ID_VERIFY_FAILED)     gowin_debugf("Bit 2: ID verification failed");
    if (status & GOWIN_STATUS_TIMEOUT)              gowin_debugf("Bit 3: Timeout occurred");
    if (status & GOWIN_STATUS_AUTO_BOOT_2ND_FAIL)   gowin_debugf("Bit 4: Auto boot 2nd failed");
    if (status & GOWIN_STATUS_MEMORY_ERASE)         gowin_debugf("Bit 5: Memory erase in progress");
    if (status & GOWIN_STATUS_PREAMBLE)             gowin_debugf("Bit 6: Preamble detected");
    if (status & GOWIN_STATUS_SYSTEM_EDIT_MODE)     gowin_debugf("Bit 7: System edit mode active");
    if (status & GOWIN_STATUS_PRG_SPIFLASH_DIRECT)  gowin_debugf("Bit 8: Programming SPI flash directly");
    if (status & GOWIN_STATUS_AUTO_BOOT_1ST_FAILED) gowin_debugf("Bit 9: Auto boot 1st failed");
    if (status & GOWIN_STATUS_NON_JTAG_CNF_ACTIVE)  gowin_debugf("Bit 10: Non-JTAG configuration active");
    if (status & GOWIN_STATUS_BYPASS)               gowin_debugf("Bit 11: Bypass mode enabled");
    if (status & GOWIN_STATUS_I2C_SRAM_F)           gowin_debugf("Bit 12: I2C_SRAM_F");
    if (status & GOWIN_STATUS_DONE_FINAL)           gowin_debugf("Bit 13: Done final");
    if (status & GOWIN_STATUS_SECURITY_FINAL)       gowin_debugf("Bit 14: Security final");
    if (status & GOWIN_STATUS_ENCRYPTED_FORMAT)     gowin_debugf("Bit 15: ENCRYPTED_FORMAT");
    if (status & GOWIN_STATUS_KEY_IS_RIGHT)         gowin_debugf("Bit 16: KEY_IS_RIGHT");
    if (status & GOWIN_STATUS_SSPI_MODE)            gowin_debugf("Bit 17: SSPI_MODE");
    if (status & GOWIN_STATUS_SER_CRC_DONE)         gowin_debugf("Bit 18: Serial CRC done");
    if (status & GOWIN_STATUS_SER_CRC_ERR)          gowin_debugf("Bit 19: Serial CRC error");
    if (status & GOWIN_STATUS_SER_ECC_CORR)         gowin_debugf("Bit 20: ECC corrected");
    if (status & GOWIN_STATUS_SER_ECC_UNCORR)       gowin_debugf("Bit 21: ECC uncorrectable");
    if (status & GOWIN_STATUS_SER_RUNNING)          gowin_debugf("Bit 22: Serial running");
    if (status & GOWIN_STATUS_CPU_BUS_WIDTH_0)      gowin_debugf("Bit 23: CPU_BUS_WIDTH_0");
    if (status & GOWIN_STATUS_CPU_BUS_WIDTH_1)      gowin_debugf("Bit 24: CPU_BUS_WIDTH_1");
#ifndef TANG_MEGA138KPRO
    if (status & GOWIN_STATUS_SYNC_DET_TERY_0)      gowin_debugf("Bit 25: SYNC_DET_TERY_0");
    if (status & GOWIN_STATUS_SYNC_DET_TERY_1)      gowin_debugf("Bit 26: SYNC_DET_TERY_1");
    if (status & GOWIN_STATUS_DECOMP_FAIL)          gowin_debugf("Bit 27: Decompression failed");
    if (status & GOWIN_STATUS_MFG_DONE)             gowin_debugf("Bit 28: Manufacturing done");
    if (status & GOWIN_STATUS_INIT)                 gowin_debugf("Bit 29: Initialization complete");
    if (status & GOWIN_STATUS_WAKEUP)               gowin_debugf("Bit 30: Wakeup signal");
    if (status & GOWIN_STATUS_AUTO_ERASE)           gowin_debugf("Bit 31: Auto erase enabled");
#endif
}

static uint32_t gowin_readStatusReg(void) {
  uint32_t status = jtag_command_u08_read32(GOWIN_COMMAND_STATUS);
  // gowin_printStatusReg(status);
  return status;
}

static void gowin_gw2a_force_state(void) {
  /* undocumented sequence but required when
   * flash failure
   */
  uint32_t state =  gowin_readStatusReg();
  if ((state & GOWIN_STATUS_CRC_ERROR) == 0)
    return;
  jtag_command_u08(GOWIN_COMMAND_CONFIG_DISABLE);
  jtag_command_u08(0);
  jtag_command_u08_read32(GOWIN_COMMAND_IDCODE);
  state = gowin_readStatusReg();
  jtag_command_u08(GOWIN_COMMAND_CONFIG_DISABLE);
  jtag_command_u08(0);
  state = gowin_readStatusReg();
  jtag_command_u08_read32(GOWIN_COMMAND_IDCODE);
  jtag_command_u08(GOWIN_COMMAND_CONFIG_ENABLE);
  jtag_command_u08(GOWIN_COMMAND_CONFIG_DISABLE);
  jtag_command_u08(GOWIN_COMMAND_NOOP);
  jtag_command_u08_read32(GOWIN_COMMAND_IDCODE);
  jtag_command_u08(GOWIN_COMMAND_NOOP);
  jtag_command_u08_read32(GOWIN_COMMAND_IDCODE);
}

static bool gowin_pollFlag(uint32_t mask, uint32_t value) {
  uint32_t status;
  int timeout = 0;
  do {
    status = gowin_readStatusReg();
    if (timeout == 1000){  // TODO: was 100000000
      gowin_debugf("pollFlag(): timeout");
      gowin_printStatusReg(status);
      return false;
    }
    timeout++;
  } while ((status & mask) != value);
  
  return true;
}

static bool gowin_enableCfg(void) {
  jtag_command_u08(GOWIN_COMMAND_CONFIG_ENABLE);
  return gowin_pollFlag(GOWIN_STATUS_SYSTEM_EDIT_MODE, GOWIN_STATUS_SYSTEM_EDIT_MODE);
}

static bool gowin_disableCfg(void) {
  jtag_command_u08(GOWIN_COMMAND_CONFIG_DISABLE);
  jtag_command_u08(GOWIN_COMMAND_NOOP);
  return gowin_pollFlag(GOWIN_STATUS_SYSTEM_EDIT_MODE, 0);
}

// prepare SRAM upload
GOWIN_STATIC bool gowin_eraseSRAM(void) {
  uint32_t status;
  jtag_command_u08_read32(GOWIN_COMMAND_USERCODE);

  // Clearing if failed loading
  status = gowin_readStatusReg();

  if ((idcode == IDCODE_GW5AST138)||(idcode == IDCODE_GW5A25)) {
    if ((status & GOWIN_STATUS_DONE_FINAL) == 0) {
      gowin_debugf("FPGA REINIT");
      jtag_command_u08(GOWIN_COMMAND_REINIT);
      jtag_clk_us(10000);
    }
  }
  // mandatory for GW5A-60
  if (idcode == IDCODE_GW5AT60) {
      gowin_debugf("FPGA REINIT");
      jtag_command_u08(GOWIN_COMMAND_REINIT);
      jtag_clk_us(10000);
    }

  // Clearing Status Code Errors
  if (idcode != IDCODE_GW2AR18) {
    status = gowin_readStatusReg();
    bool auto_boot_2nd_fail = (status & GOWIN_STATUS_AUTO_BOOT_2ND_FAIL) == GOWIN_STATUS_AUTO_BOOT_2ND_FAIL;
    bool is_timeout = (status & GOWIN_STATUS_TIMEOUT) == GOWIN_STATUS_TIMEOUT;
    bool bad_cmd = (status & GOWIN_STATUS_BAD_COMMAND) == GOWIN_STATUS_BAD_COMMAND;
    bool id_verify_failed = (status & GOWIN_STATUS_ID_VERIFY_FAILED) ==  GOWIN_STATUS_ID_VERIFY_FAILED;
    if (is_timeout || auto_boot_2nd_fail || bad_cmd || id_verify_failed) {
    gowin_debugf("Clearing status errors by FPGA RELOAD");
    gowin_printStatusReg(status);
    jtag_command_u08(GOWIN_COMMAND_NOOP);
    jtag_command_u08(GOWIN_COMMAND_CONFIG_ENABLE);
    jtag_command_u08(GOWIN_COMMAND_RECONFIG);
    jtag_command_u08(GOWIN_COMMAND_NOOP);
    jtag_clk_us(100000);
    jtag_command_u08(GOWIN_COMMAND_CONFIG_DISABLE);
    jtag_command_u08(GOWIN_COMMAND_NOOP);
    jtag_clk_us(100000);
    }
  }

  if(idcode == IDCODE_GW2AR18) {
    gowin_gw2a_force_state();

    if(!gowin_enableCfg()) {
      status = gowin_readStatusReg();
      gowin_printStatusReg(status);
      gowin_debugf("Failed to enable config");
      return false;
     }
  } else {
      jtag_command_u08(GOWIN_COMMAND_CONFIG_ENABLE);
      // no status polling !
   }

  jtag_command_u08(GOWIN_COMMAND_ERASE_SRAM);
  jtag_command_u08(GOWIN_COMMAND_NOOP);
  jtag_clk_us(10000); // wait for erase to complete
#ifdef TANG_MEGA138KPRO
  jtag_clk_us(40000);
#endif

  if(!gowin_pollFlag(GOWIN_STATUS_MEMORY_ERASE, GOWIN_STATUS_MEMORY_ERASE)) {
    status = gowin_readStatusReg();
    gowin_printStatusReg(status);
    gowin_debugf("Failed to trigger SRAM erase");
    return false;
  }

  jtag_command_u08(GOWIN_COMMAND_XFER_DONE);
  jtag_command_u08(GOWIN_COMMAND_NOOP);
  if(!gowin_disableCfg()) {
    status = gowin_readStatusReg();
    gowin_printStatusReg(status);
    gowin_debugf("Failed to disable config");
    return false;
  }

  return true;
}

GOWIN_STATIC bool gowin_writeSRAM_prepare(void) {
  jtag_command_u08(GOWIN_COMMAND_CONFIG_ENABLE); // config enable

  /* UG704 3.4.3 */
  jtag_command_u08(GOWIN_COMMAND_INIT_ADDR); // address initialize

  /* 2.2.6.4 */
  jtag_command_u08(GOWIN_COMMAND_XFER_WRITE); // transfer configuration data

  return true;
}

GOWIN_STATIC bool gowin_writeSRAM_transfer(uint8_t *data, uint16_t len, bool first, bool last) {
  jtag_shiftDR_part(data, NULL, 8*len, (first?JTAG_FLAG_BEGIN:0) | (last?JTAG_FLAG_END:0));  
  return true;
}

GOWIN_STATIC bool gowin_writeSRAM_postproc(__attribute__((unused)) uint32_t checksum) {
  jtag_command_u08(GOWIN_COMMAND_CONFIG_DISABLE);
  jtag_command_u08(GOWIN_COMMAND_NOOP);

  uint32_t status_reg = gowin_readStatusReg();
  status_reg = gowin_readStatusReg(); // read twice to get updated status
  if(!(status_reg & GOWIN_STATUS_DONE_FINAL)) {
    gowin_printStatusReg(status_reg);
    fatal_debugf("Failed to write SRAM");
    return false;
  }
  uint32_t usercode = jtag_command_u08_read32(GOWIN_COMMAND_USERCODE);
  gowin_debugf("SRAM successfully written, Usercode=0x%08lx", usercode);
  return true;
}

GOWIN_STATIC void gowin_fpgaReset(void) {
    gowin_debugf("FPGA RECONFIG");

    jtag_command_u08(GOWIN_COMMAND_RECONFIG);
    jtag_command_u08(GOWIN_COMMAND_NOOP);
    jtag_clk_us(100000);
}

static inline uint8_t reverse_byte(uint8_t byte) {
  byte = ((byte & 0x55) << 1) | ((byte & 0xaa) >> 1);
  byte = ((byte & 0x33) << 2) | ((byte & 0xcc) >> 2);
  return ((byte & 0x0f) << 4) | ((byte & 0xf0) >> 4);
}

GOWIN_STATIC bool gowin_open(void) {
  uint32_t idcode = jtag_open();  
  gowin_debugf("FPGA detected: %08lx", idcode);

  return(idcode == IDCODE_GW2AR18  || 
         idcode == IDCODE_GW5AT60  || 
         idcode == IDCODE_GW5A25  ||
         idcode == IDCODE_GW5AST138);
}

// Make idcode accessible externally. This should be
// removed one day as all gowin specific code should
// go here.
uint32_t gowin_idcode(void) {
  return idcode;
}

// try to open "core.bin" on the sd card and try to transfer it via JTAG to
// the FPGA
bool gowin_upload_core_bin(const char *name) {
  // try to open core.bin
  FIL fil;
  if(f_open(&fil, name, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
    gowin_debugf("%s not found", name);
    return false;
  }
    
  if(!gowin_open()) {
    gowin_debugf("FPGA not detected");
    jtag_close();
    f_close(&fil);
    return false;
  }
  
  // measure total download time
  TickType_t ticks = xTaskGetTickCount();
  
  gowin_debugf("=== Erase SRAM ===");
  if(!gowin_eraseSRAM()) {
    gowin_debugf("Failed to erase SRAM");
    jtag_close();
    f_close(&fil);
    return false;
  }

  if (idcode == IDCODE_GW5AST138) {
    gowin_debugf("=== 2nd Erase SRAM ===");
    if(!gowin_eraseSRAM()) {
      gowin_debugf("Failed to erase SRAM");
      jtag_close();
      f_close(&fil);
      return false;
    }
  }
 
  gowin_debugf("=== Load SRAM ===");
  gowin_writeSRAM_prepare();
    
  FRESULT fr;
  uint8_t buffer[512];
  UINT bytesRead;
  uint32_t total = 0;
  
  // using the table instead of reversing each payload byte through the
  // algorithm saves ~0.3 sec for the full download
  uint8_t rev_table[256];
  for(int i=0;i<256;i++) rev_table[i] = reverse_byte(i);    
  
  // gw2ar-18 core size is 907418 bytes. This is not a multiple of 64, so when
  // loading in 64 byte chunks, the last chunk is smaller      
  do {
    if((fr = f_read(&fil, buffer, sizeof(buffer), &bytesRead)) == FR_OK) {
      // the binary data has the wrong endianess
      for(UINT i=0;i<bytesRead;i++) buffer[i] = rev_table[buffer[i]];
      gowin_writeSRAM_transfer(buffer, bytesRead, !total, bytesRead != sizeof(buffer));
      total += bytesRead;
    }
  } while(fr == FR_OK && bytesRead == sizeof(buffer));
  
  if(fr != FR_OK) {
    fatal_debugf("Binary download failed after %lu bytes", total);
    jtag_close();
    f_close(&fil);
    return false;	
  }

  // don't set checksum
  gowin_writeSRAM_postproc(0xffffffff);
  
  gowin_debugf("Read %lu bytes", total);
  
  ticks = xTaskGetTickCount() - ticks;
  gowin_debugf("Download time: %lu.%03lu seconds", ticks/1000, ticks%1000);
  
  jtag_close();
  f_close(&fil);

  return true;
}

// try to open on the sd card and try to transfer it via JTAG into the FPGA
bool gowin_upload_core_fs(const char *name) {  
  // try to open core.fs
  FIL fil;
  if(f_open(&fil, name, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
    gowin_debugf("%s not found", name);
    return false;
  }
    
  // transmit core via JTAG
  if(!gowin_open()) {
    gowin_debugf("FPGA not detected");    
    jtag_close();
    f_close(&fil);
    return false;
  }
  
  // measure total download time
  TickType_t ticks = xTaskGetTickCount();
  
  gowin_debugf("=== Erase SRAM ===");
  if(!gowin_eraseSRAM()) {
    gowin_debugf("Failed to erase SRAM");
    jtag_close();
    f_close(&fil);
    return false;
  }
  
  gowin_debugf("=== Load SRAM ===");
  gowin_writeSRAM_prepare();
  
  // =================== parse the fs file =================
  // these are rather huge (>7MB for the GW2AR-18) and mainly contain
  // ASCII encoded binary data with some text header
  uint8_t buffer[128];
  UINT bytesRead;
  FRESULT fr;
  uint32_t total = 0;
  bool header_done = false;
  bool parse_error = false;
  uint16_t used = 0;
  
  // --------- parse the .fs header -----------
  do {
    if((fr = f_read(&fil, buffer+used, sizeof(buffer)-used, &bytesRead)) == FR_OK) {
      total += bytesRead;
      
      // check if the line starts with a binary digit
      if(buffer[0] == '0' || buffer[0] == '1') {
	gowin_debugf("header done");
	used += bytesRead;
	header_done = true;
      } else {
	// the buffer should start with "//" and there should be a newline in this buffer
	if(buffer[0] != '/' || buffer[1] != '/')
	  parse_error = true;
	else {	    
	  uint8_t *nl = memchr(buffer, '\n', sizeof(buffer));
	  if(!nl) parse_error = true;  // that would not happen with a valid fs file
	  else {
	    // there's a complete header line in the buffer. Terminate it
	    *nl = '\0';
	    // line may have DOS line endings. In that case terminate earlier
	    if(*(nl-1) == '\r') *(nl-1) = '\0';
	    
	    gowin_debugf("header: %s", buffer);
	    
	    // shift unused part of buffer down
	    memmove(buffer, nl+1, sizeof(buffer)-(nl-buffer+1));
	    used = sizeof(buffer)-(nl-buffer+1);
	  }
	}	    
      }
    }
  } while(fr == FR_OK && bytesRead && !header_done && !parse_error);
  
  // TODO: check header values to e.g. make sure the bitstream is for the
  // correct FPGA type
    
  if(fr != FR_OK || !header_done || parse_error) {
    gowin_debugf("Header parsing failed");
    jtag_close();
    f_close(&fil);
    return false;
  }
  
  // --------- parse the .fs binary data -----------
  
  gowin_debugf("Parsing and uploading binary bitstream");
    
  uint8_t data[512];  // one line holds up to 430 bytes
  uint8_t byte = 0, bit = 0;
  uint32_t total_data = 0, line_data = 0;
  uint16_t line = 0;
  
  do {
    // collect all 0/1 bits
    for(int i=0;i<used && !parse_error;i++) {
      if(buffer[i] == '0' || buffer[i] == '1') {
	byte = (byte >> 1)|((buffer[i] == '1')?0x80:0);
	bit = (bit+1)&7;
	if(!bit) {
	  // got a complete byte
	  data[line_data++] = byte;
	  total_data++;
	  byte = 0;
	}
      } else {
	// the digits should always come in multiple of 8
	if(bit) {
	  fatal_debugf("incomplete byte");
	  parse_error = true;
	} else if(buffer[i] == '\n') {
	  
	  // gowin_debugf("line %d: %ld (total %ld, used %d)", line, line_data, total_data, used);
	  // hexdump(data, line_data);
	  
	  gowin_writeSRAM_transfer(data, line_data, !line, line>100 && line_data == 2);
	  
	  line_data = 0;
	  line++;
	}
      }
    }
    
    if(!parse_error) {
      // refill the buffer
      if((fr = f_read(&fil, buffer, sizeof(buffer), &bytesRead)) == FR_OK) {
	total += bytesRead;
	used = bytesRead;
      }
    }
  } while(fr == FR_OK && bytesRead && !parse_error);
  
  if((fr != FR_OK) || parse_error) {
    fatal_debugf("Binary download failed after %lu bytes", total);
    jtag_close();
    f_close(&fil);
    return false;	
  }
  
  gowin_debugf("Total data transferred: %ld bytes", total_data);
  
  // TODO: Figure out where the checksum is supposed to come from. The checksum openFPGAloader
  // downloads is not the one mentioned in the .fs header.
  gowin_writeSRAM_postproc(0xffffffff);
  
  gowin_debugf("Read %lu bytes", total);
  
  ticks = xTaskGetTickCount() - ticks;
  gowin_debugf("Download time: %lu.%03lu seconds", ticks/1000, ticks%1000);
  
  jtag_close();
  f_close(&fil);
  return true;
}
#endif // ENABLE_JTAG
