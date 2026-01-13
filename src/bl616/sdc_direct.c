/*
  sdc_direct.c - access the sd card directly from the rp2040 / BL616

  This only works on the dev20k board and Console 60K and 138k where the
  SD card is connected to the FPGA _and_ the rp2040 / BL616  

  This implements software driven 1-bit SD mode even
  though SPI would be faster due to HW support. But it turned
  out that the SD card cannot return from SPI mode into SD
  mode which the FPGA uses once it has booted and takes
  over the SD card access.
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

#define PIN_TF_SDIO_SEL GPIO_PIN_16  // 0 = FPGA , 1 = BL616

static struct bflb_device_s *gpio;

static bool sdc_direct_active = false;

bool sdc_direct_init(void) {  
  gpio = bflb_device_get_by_name("gpio");

  /* Console 60k/138k SD Card bus to BL616 TF_SDIO_SEL*/
  bflb_gpio_init(gpio, PIN_TF_SDIO_SEL, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_set(gpio, PIN_TF_SDIO_SEL);

  bflb_mtimer_delay_ms(250);

  board_sdh_gpio_init();

  fatfs_sdh_driver_register();

 return true;
}

bool sdc_direct_write(__attribute__((unused)) uint32_t lba, __attribute__((unused)) const uint8_t *buffer) {
  // return true when direct access is being used to tell the
  // sdc.c that the request has been handled directly
  return sdc_direct_active;
}

extern struct sd_card_s sd_card;

// Read single 512-byte block
bool sdc_direct_read(uint32_t lba, uint8_t *buffer) {
  // return false when direct access is not being used to tell the
  // sdc.c that it needs to handle the access through the FPGA
  if(!sdc_direct_active) return false;

  if (sdh_sd_read_blocks(&sd_card, buffer, lba, 1) < 0) {
    sdc_debugf("sdc_direct_read failed");
    return false;
  }

  return true;
}

void sdc_direct_release(void) {

  gpio = bflb_device_get_by_name("gpio");
  sdc_direct_active = false;
  
  /* Console 60k/138k SD Card bus back to FPGA TF_SDIO_SEL*/
  bflb_gpio_reset(gpio, PIN_TF_SDIO_SEL);
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
  
  jtag_debugf("FPGA detected");
    
  // measure total download time
  TickType_t ticks = xTaskGetTickCount();
  
  jtag_debugf("=== Erase SRAM ===");
  if(!jtag_gowin_eraseSRAM()) {
    jtag_debugf("Failed to erase SRAM");
    jtag_close();
    f_close(&fil);
    return false;
  }
  
  jtag_debugf("=== Load SRAM ===");
  jtag_gowin_writeSRAM_prepare();
    
  FRESULT fr;
  uint8_t buffer[256];
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
      jtag_gowin_writeSRAM_transfer(buffer, 8*bytesRead, !total, bytesRead != sizeof(buffer));
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
  
  jtag_debugf("FPGA detected");

  // measure total download time
  TickType_t ticks = xTaskGetTickCount();
  
  jtag_debugf("=== Erase SRAM ===");
  if(!jtag_gowin_eraseSRAM()) {
    jtag_debugf("Failed to erase SRAM");
    jtag_close();
    f_close(&fil);
    return false;
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
  gpio = bflb_device_get_by_name("gpio");

  //mcu_hw_fpga_reconfig(false);  // TODO: reenable this
  
  // try to boot the FPGA from SD card
  sdc_debugf("Attempting direct SD card boot ...");
  
    sdc_direct_init();

    uint64_t start = bflb_mtimer_get_time_ms();
    FRESULT res;
    sdc_debugf("Mounting sd card ...\r\n");

    while ((res = f_mount(&fs, "/sd", 1)) != FR_OK && bflb_mtimer_get_time_ms() - start < 500)
    bflb_mtimer_delay_ms(100);
  
    if (res == FR_OK) {
        sdc_debugf("SD card mounted in %d ms", bflb_mtimer_get_time_ms() - start);
    } else  {
        sdc_debugf("SD card not found...\r\n");
        sdc_debugf("try to mount USB drive...");
        start = bflb_mtimer_get_time_ms();
        while ((res = f_mount(&fs, "/usb", 1)) != FR_OK && bflb_mtimer_get_time_ms() - start < 2000)
          bflb_mtimer_delay_ms(100);
        if (res != FR_OK) {
            sdc_debugf("Failed to mount USB drive\r\n");
            sdc_direct_release();
            mcu_hw_fpga_reconfig(true);
            return;
          } else {
            sdc_debugf("USB drive mounted in %d ms", bflb_mtimer_get_time_ms() - start);
   }
  }
  
  sdc_debugf("Card mounted directly");
  // we can now try to load a core from SD card

  // try to load core.bin and if that doesn't work core.fs
  bool upload_ok = sdc_direct_upload_core_bin("/sd/core.bin");
  if(!upload_ok) upload_ok = sdc_direct_upload_core_fs("/sd/core.fs");
  if(!upload_ok) upload_ok = sdc_direct_upload_core_fs("/usb/core.bin");
  if(!upload_ok) upload_ok = sdc_direct_upload_core_fs("/usb/core.fs");

  // unmount the fs
  f_mount(NULL, "/sd", 1);
  f_mount(NULL, "/usb", 1);
  // release the sd card
  sdc_direct_release();
  
  // on upload failure reconfig the FPGA and allow it to (re-)boot from flash
  if(!upload_ok) {
    sdc_debugf("Upload failed, triggering FPGA reconfig from flash");
    mcu_hw_fpga_reconfig(true);
  }
}
