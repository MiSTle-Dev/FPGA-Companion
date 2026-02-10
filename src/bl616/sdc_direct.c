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
#include "../gowin.h"
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

#ifdef JTAG_ENABLE

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
    
  if(!gowin_open()) {
    jtag_debugf("FPGA not detected");
    jtag_close();
    f_close(&fil);
    return false;
  }
  
  // measure total download time
  TickType_t ticks = xTaskGetTickCount();
  
  jtag_debugf("=== Erase SRAM ===");
  if(!gowin_eraseSRAM()) {
    jtag_debugf("Failed to erase SRAM");
    jtag_close();
    f_close(&fil);
    return false;
  }

  if (gowin_idcode() == IDCODE_GW5AST138) {
    jtag_debugf("=== 2nd Erase SRAM ===");
    if(!gowin_eraseSRAM()) {
      jtag_debugf("Failed to erase SRAM");
      jtag_close();
      f_close(&fil);
      return false;
    }
  }
  jtag_debugf("=== Load SRAM ===");
  gowin_writeSRAM_prepare();

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
  if (gowin_idcode() == IDCODE_GW2AR18) {
    mcu_hw_jtag_tms(1, 0b01, 2);
  }
  free(fbuf_cached);
// don't set checksum
  gowin_writeSRAM_postproc(0xffffffff);
  
  sdc_debugf("Read %lu bytes", total);
  
  ticks = xTaskGetTickCount() - ticks;
  sdc_debugf("Download time: %lu.%03lu seconds", ticks/1000, ticks%1000);
  
  jtag_close();
  f_close(&fil);

  return true;
}

#endif // JTAG_ENABLE
