/*
  sdc_direct.c - access the sd card directly from the  BL616

  SD card access for Console 60K and 138k where the
  card is connected to the FPGA _and_ the BL616  

  uses the BL616 SDH SD Host controller (SDH) to access the card or read from an USB Mass Storage (MSC).
*/

#include "stdlib.h"
#include "bflb_spi.h"
#include <FreeRTOS.h>
#include <timers.h>
#include <string.h>
#include "bflb_gpio.h"
#include "board.h"
#include "ff.h"
#include "fatfs_diskio_register.h"
#include "sdc_direct.h"
#include "../debug.h"
#include "../mcu_hw.h"
#include "../jtag.h"
#include <sys/stat.h>

static struct bflb_device_s *gpio;
static bool sdc_direct_active = false;

#ifndef USB_NOCACHE_RAM_SECTION
#define USB_NOCACHE_RAM_SECTION __attribute__((section(".noncacheable")))
#endif

#define BLOCK_SIZE (8*1024)
USB_NOCACHE_RAM_SECTION BYTE __attribute__((aligned(64))) fbuf[BLOCK_SIZE];

uint32_t get_file_size(const char *fname) {
    FILINFO fno;
    FRESULT r = f_stat(fname, &fno);
    if (r != FR_OK) return 0;
    return fno.fsize;
}

bool sdc_direct_init(void) {  
  gpio = bflb_device_get_by_name("gpio");

#ifdef TANG_CONSOLE60K
  /* Console 60k/138k SD Card bus to BL616 TF_SDIO_SEL*/
  bflb_gpio_set(gpio, PIN_TF_SDIO_SEL);
#endif
bflb_mtimer_delay_ms(250);

  board_sdh_gpio_init();

  fatfs_sdh_driver_register();
  sdc_direct_active = true;

  return true;
}

void sdc_direct_release(void) {

  gpio = bflb_device_get_by_name("gpio");
  sdc_direct_active = false;
#ifdef TANG_CONSOLE60K
  /* Console 60k/138k SD Card bus back to FPGA TF_SDIO_SEL*/
  bflb_gpio_reset(gpio, PIN_TF_SDIO_SEL);
#endif
}

static FATFS fs;

static inline uint8_t reverse_byte(uint8_t byte) {
  byte = ((byte & 0x55) << 1) | ((byte & 0xaa) >> 1);
  byte = ((byte & 0x33) << 2) | ((byte & 0xcc) >> 2);
  return ((byte & 0x0f) << 4) | ((byte & 0xf0) >> 4);
}

// try to open "core.bin" on the sd card and try to transfer it via JTAG to
// the FPGA
bool sdc_direct_upload_core_bin(const char *name) {
  // try to open core.bin
  FIL fil;
  if(f_open(&fil, name, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
    sdc_debugf("%s not found", name);
    return false;
  }
    
  if(!jtag_open()) {
    jtag_debugf("FPGA not detected");
    jtag_close();
    f_close(&fil);
    return false;
  }
  
  sdc_debugf("JTAG FPGA detected :%08lx", idcode);
    
  // measure total download time
  TickType_t ticks = xTaskGetTickCount();
  
  jtag_debugf("=== Erase SRAM ===");
  if(!jtag_gowin_eraseSRAM()) {
    jtag_debugf("Failed to erase SRAM");
    jtag_close();
    f_close(&fil);
    return false;
  }

  if (idcode == IDCODE_GW5AST138) {
    jtag_debugf("=== 2nd Erase SRAM ===");
    if(!jtag_gowin_eraseSRAM()) {
      jtag_debugf("Failed to erase SRAM");
      jtag_close();
      f_close(&fil);
      return false;
    }
  }
  jtag_debugf("=== Load SRAM ===");
  jtag_gowin_writeSRAM_prepare();

  FRESULT fr;
  UINT bytes = 0, total = 0;
  uint32_t len = 0;

  len = get_file_size(name);

  BYTE *fbuf_cached;
  fbuf_cached = (BYTE*)malloc(BLOCK_SIZE);
  if (!fbuf_cached) {
    fatal_debugf("Cannot malloc buffer\r\n");
    jtag_close();
    f_close(&fil);
    return false;	
  }

  // RUN-TEST/IDLE
  mcu_hw_jtag_tms(1, 0b000000, 6);
  // send TMS 1/0/0 to get from RUN-TEST/IDLE into SHIFT-DR state  
  mcu_hw_jtag_tms(1, 0b001, 3);

  jtag_enter_gpio_out_mode();

  for (;;) {
      fr = f_read(&fil, fbuf, BLOCK_SIZE, &bytes);
      if (bytes == 0) break;
      total += bytes;
      taskENTER_CRITICAL();
      jtag_writeTDI_msb_first_gpio_out_mode(fbuf, bytes, total >= len);
      taskEXIT_CRITICAL();
      if (bytes < BLOCK_SIZE) break;
  }
  jtag_exit_gpio_out_mode();

  if(fr != FR_OK) {
    fatal_debugf("Binary download failed after %lu bytes", total);
    free(fbuf_cached);
    jtag_close();
    f_close(&fil);
    return false;	
  }

  // send TMS 1/0 to return into RUN-TEST/IDLE
   if (idcode == IDCODE_GW2AR18) {
    mcu_hw_jtag_tms(1, 0b01, 2);
  }
  free(fbuf_cached);
// don't set checksum
  jtag_gowin_writeSRAM_postproc(0xffffffff);
  
  sdc_debugf("Read %lu bytes", total);
  
  ticks = xTaskGetTickCount() - ticks;
  sdc_debugf("Download time: %lu.%03lu seconds", ticks/1000, ticks%1000);
  
  jtag_close();
  f_close(&fil);

  return true;
}

// try to open on the sd card and try to transfer it via JTAG into the FPGA
bool sdc_direct_upload_core_fs(const char *name) {  
  // try to open core.fs
  FIL fil;
  if(f_open(&fil, name, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
    sdc_debugf("%s not found", name);
    return false;
  }
    
  // transmit core via JTAG
  if(!jtag_open()) {
    jtag_debugf("FPGA not detected");    
    jtag_close();
    f_close(&fil);
    return false;
  }
  
  jtag_debugf("JTAG FPGA detected");

  // measure total download time
  TickType_t ticks = xTaskGetTickCount();
  
  jtag_debugf("=== Erase SRAM ===");
  if(!jtag_gowin_eraseSRAM()) {
    jtag_debugf("Failed to erase SRAM");
    jtag_close();
    f_close(&fil);
    return false;
  }
  
  if (idcode == IDCODE_GW5AST138) {
    jtag_debugf("=== 2nd Erase SRAM ===");
    if(!jtag_gowin_eraseSRAM()) {
      jtag_debugf("Failed to erase SRAM");
      jtag_close();
      f_close(&fil);
      return false;
    }
  }

  jtag_debugf("=== Load SRAM ===");
  jtag_gowin_writeSRAM_prepare();
  
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
	sdc_debugf("header done");
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
	    
	    sdc_debugf("header: %s", buffer);
	    
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
    sdc_debugf("Header parsing failed");
    jtag_close();
    f_close(&fil);
    return false;
  }
  
  // --------- parse the .fs binary data -----------
  
  sdc_debugf("Parsing and uploading binary bitstream");
    
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
	  
	  // sdc_debugf("line %d: %ld (total %ld, used %d)", line, line_data, total_data, used);
	  // hexdump(data, line_data);
	  
	  jtag_gowin_writeSRAM_transfer(data, 8*line_data, !line, line>100 && line_data == 2);
	  
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
  
  sdc_debugf("Total data transferred: %ld bytes", total_data);
  
  // TODO: Figure out where the checksum is supposed to come from. The checksum openFPGAloader
  // downloads is not the one mentioned in the .fs header.
  jtag_gowin_writeSRAM_postproc(0xffffffff);
  
  sdc_debugf("Read %lu bytes", total);
  
  ticks = xTaskGetTickCount() - ticks;
  sdc_debugf("Download time: %lu.%03lu seconds", ticks/1000, ticks%1000);
  
  jtag_close();
  f_close(&fil);
  return true;
}

void sdc_boot(void) {
  // before doing anything, check if an SD card is inserted
  // the FPGA is not ready, but it may still drive the SD card.
  // So reconfigure the FPGA without allowing it to boot from flash
  FRESULT res = FR_NOT_READY;
  uint64_t start;
  bool upload_ok = false;

#if defined(TANG_CONSOLE60K) || defined(TANG_MEGA60K)
    // try to boot the FPGA from SD card
    sdc_debugf("Attempting direct SD card boot ...");
    sdc_direct_init();
    start = bflb_mtimer_get_time_ms();
    sdc_debugf("Mounting sd card ...\r\n");
    while ((res = f_mount(&fs, "/sd", 1)) != FR_OK && bflb_mtimer_get_time_ms() - start < 500)
      bflb_mtimer_delay_ms(100);
    if (res == FR_OK) {
        sdc_debugf("SD card mounted in %d ms", bflb_mtimer_get_time_ms() - start);
        upload_ok = sdc_direct_upload_core_bin("/sd/core.bin");
        if(!upload_ok) upload_ok = sdc_direct_upload_core_fs("/sd/core.fs");
        f_mount(NULL, "/sd", 1);
      } else {
        sdc_debugf("SD card not found...\r\n");
    }
    sdc_direct_release();
#endif

  if(!upload_ok) {
    sdc_debugf("Mounting USB drive...");
    start = bflb_mtimer_get_time_ms();
    while ((res = f_mount(&fs, "/usb", 1)) != FR_OK && bflb_mtimer_get_time_ms() - start < 1000)
      bflb_mtimer_delay_ms(100);
    if (res != FR_OK) {
        sdc_debugf("Failed to mount USB drive\r\n");
    } else {
        sdc_debugf("USB drive mounted in %d ms", bflb_mtimer_get_time_ms() - start);
        // try to load core.bin and if that doesn't work core.fs
        if(!upload_ok) upload_ok = sdc_direct_upload_core_bin("/usb/core.bin");
        if(!upload_ok) upload_ok = sdc_direct_upload_core_fs("/usb/core.fs");
        f_mount(NULL, "/usb", 1);
      }
  }
  // on upload failure reconfig the FPGA and allow it to (re-)boot from flash
  if(!upload_ok) {
    sdc_debugf("Upload failed");
    jtag_gowin_fpgaReset();
  } else {
    sdc_debugf("Upload successful, FPGA now running new core");
  }
}
