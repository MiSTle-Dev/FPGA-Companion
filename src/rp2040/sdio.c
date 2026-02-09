/*
  sdio.c - sdio for direct sd card access (not though FPGA)

  This e.g. works on the dev20k board and similar where the
  SD card is connected to the FPGA _and_ the rp2040  

  This implements software driven 1-bit SDIO mode even
  though SPI would be faster due to HW support. But it turned
  out that the SD card cannot return from SPI mode into SD
  mode which the FPGA uses once it has booted and takes
  over the SD card access.
*/

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <FreeRTOS.h>
#include <timers.h>
#include <string.h>

#include "sdio.h"
#include "../debug.h"
#include "../mcu_hw.h"
#include "../jtag.h"

#include "../gowin.h"   // TODO remove gowin dependency

#if MISTLE_BOARD != 4
#error "Direct SDIO access only works for the Dev20k board"
#endif

#define SD_VERIFY_CRC   0

// direct SD card pins
#define PIN_SD_DET      7   // pin 9, gpio 7 (not at FPGA)

#define PIN_SD_CLK     10   // pin 13, gpio 10
#define PIN_SD_CMD     11   // pin 14, gpio 11
#define PIN_SD_DAT0     8   // pin 11, gpio 8
#define PIN_SD_DAT3     9   // pin 12, gpio 9, actually unsed

static uint8_t crc7(uint8_t crcIn, uint8_t data) {
  crcIn ^= data;
  for (int i = 0; i < 8; i++) {
    if (crcIn & 0x80) crcIn ^= 0x89;
    crcIn <<= 1;
  } 
  return crcIn;
}

static void sdio_clock(uint16_t n) {
  while(n--) {
    // Slightly reduced clock. Most cards won't work
    // with a faster clock. TODO: Measure what clock
    // this actually results in.
    gpio_put(PIN_SD_CLK, 0); gpio_put(PIN_SD_CLK, 0);
    gpio_put(PIN_SD_CLK, 0); gpio_put(PIN_SD_CLK, 0);

    gpio_put(PIN_SD_CLK, 1); gpio_put(PIN_SD_CLK, 1);
    gpio_put(PIN_SD_CLK, 1); gpio_put(PIN_SD_CLK, 1);
  }
}

static void sdio_cmd_byte(uint8_t data) {
  for(int i=0;i<8;i++) {
    gpio_put(PIN_SD_CMD, (data&0x80)?1:0);
    data <<= 1;
    sdio_clock(1);
  }
}

static uint8_t sdio_cmd_byte_crc(uint8_t data, uint8_t crc) {
  sdio_cmd_byte(data);
  return crc7(crc, data);
}

static uint8_t sdio_cmd_bit_get(void) {
  char bit = gpio_get(PIN_SD_CMD)?1:0;
  sdio_clock(1);  
  return bit;  
}

static uint32_t sdio_reply(int len) {
  uint8_t buffer[(len-16)/8];
  
  gpio_set_dir(PIN_SD_CMD, GPIO_IN);

  // wait for startbit
  int i;
  for(i=0;i<32 && sdio_cmd_bit_get();i++);
  if(i == 32) return 0;

  uint8_t cmd = 0;
  for(i=0;i<7;i++)
    cmd = (cmd << 1)|sdio_cmd_bit_get();
  
  // get data bits
  uint32_t arg = 0;
  for(i=0;i<len-16;i++) {
    arg = (arg << 1)|sdio_cmd_bit_get();
    if((i&7) == 7) buffer[i/8] = arg & 0xff;
  }

  uint8_t crc = 0;
  for(i=0;i<8;i++)
    crc = (crc << 1)|sdio_cmd_bit_get();

  uint8_t crc_calc = crc7(0, cmd);
  if(cmd == 63) crc_calc = 0;
  for (int i=0; i<(len-16)/8; i++)
    crc_calc = crc7(crc_calc, buffer[i]);

  if((cmd != 63 || crc != 0xff) && (crc&0xfe) != crc_calc) {
    sdc_debugf(" rx %2d, 0x%08lx, 0x%02x", cmd & 0x3f, arg, crc);    
    sdc_debugf(" ---> CRC error: is %02x, expected %02x <----", crc&0xfe, crc_calc);
  }
    
  return arg;
}

uint32_t sdio_cmd(uint16_t pre_clocks, uint8_t cmd, uint32_t arg, uint8_t response_len) {
  sdio_clock(pre_clocks);
  
  gpio_set_dir(PIN_SD_CMD, GPIO_OUT);

  // send command
  uint8_t crc = sdio_cmd_byte_crc(cmd|0x40, 0);
  
  // send arg
  crc = sdio_cmd_byte_crc(arg >> 24, crc);
  crc = sdio_cmd_byte_crc(arg >> 16, crc);
  crc = sdio_cmd_byte_crc(arg >> 8, crc);
  crc = sdio_cmd_byte_crc(arg, crc);

  // sdc_debugf("sdio_cmd(%2d, 0x%08lx, 0x%02x)", cmd & 0x3f, arg, crc);
  
  sdio_cmd_byte(crc|1);

  // don't wait for response if none is expected (CMD0)
  if(!response_len) return 0;
  
  return sdio_reply(response_len);
}

static uint8_t sdio_data_bit_get(void) {
  uint8_t bit = gpio_get(PIN_SD_DAT0)?1:0;
  sdio_clock(1);  
  return bit;  
}

#if SD_VERIFY_CRC
// Calculate CRC16 CCITT
// It's a 16 bit CRC with polynomial x^16 + x^12 + x^5 + 1
// input:
//   crcIn - the CRC before (0 for rist step)
//   data - byte for CRC calculation
// return: the CRC16 value

// this CRC computation is expensive and currently slows the
// .fs loading down from 10 to 15 seconds
  
static uint16_t CRC16_one(uint16_t crcIn, uint8_t data) {
  crcIn  = (uint8_t)(crcIn >> 8)|(crcIn << 8);
  crcIn ^=  data;
  crcIn ^= (uint8_t)(crcIn & 0xff) >> 4;
  crcIn ^= (crcIn << 8) << 4;
  crcIn ^= ((crcIn & 0xff) << 4) << 1;
  
  return crcIn;
}
#endif

uint8_t sdio_read_data(uint8_t *buffer) {
  // wait for dat0 to go low
  // TODO: Implement proper timeout
  int i;
  for(i=0;i<65535 && (sdio_data_bit_get()&1);i++);
  if(i == 65535) {
    sdc_debugf("Data timeout!");
    return 1;
  }

  // read 512 bytes
  for(int i=0;i<512;i++) {
    buffer[i] =
      (sdio_data_bit_get()?0x80:0x00) |
      (sdio_data_bit_get()?0x40:0x00) |
      (sdio_data_bit_get()?0x20:0x00) |
      (sdio_data_bit_get()?0x10:0x00) |
      (sdio_data_bit_get()?0x08:0x00) |
      (sdio_data_bit_get()?0x04:0x00) |
      (sdio_data_bit_get()?0x02:0x00) |
      (sdio_data_bit_get()?0x01:0x00);
  }

  // read crc
  uint16_t crc = 0;
  for(int b=0;b<16;b++)
    crc = (crc << 1)|sdio_data_bit_get();

#if SD_VERIFY_CRC
  // CRC verification slows things down significantly and we
  // basically have no real way to deal with them nicely ...
  
  // calculate the crc
  uint16_t crc_calc = 0;
  for(int i=0;i<512;i++)
    crc_calc = CRC16_one(crc_calc, buffer[i]);

  if(crc_calc != crc) {
    sdc_debugf("CRC error, is %04x, expected  %04x", crc_calc, crc);
    return 1;
  }
#endif
  
  return 0;
}

static bool sdio_active = false;

static bool sdio_init(void) {  
  sdc_debugf("=== Direct SD card via SW SD 1-bit ===");
  sdc_debugf("  DET  = %2d", PIN_SD_DET);
  sdc_debugf("  SCK  = %2d", PIN_SD_CLK);
  sdc_debugf("  CMD  = %2d", PIN_SD_CMD);
  sdc_debugf("  DAT0 = %2d", PIN_SD_DAT0);
  sdc_debugf("  DAT1 = na");
  sdc_debugf("  DAT2 = na");
  sdc_debugf("  DAT3 = %2d", PIN_SD_DAT3);
  
  // -------- init SD control pins ---------
  gpio_init(PIN_SD_DET);
  gpio_set_dir(PIN_SD_DET, GPIO_IN);
  if(gpio_get(PIN_SD_DET)) {
    sdc_debugf("No card inserted");
    return false;
  }
  
  // we should only drive these signals if we can be sure that the FPGA does
  // not drive them
  gpio_init(PIN_SD_CLK); gpio_put(PIN_SD_CLK, 1);
  gpio_set_dir(PIN_SD_CLK, GPIO_OUT);

  // in SD mode the DI pin acts as CMD
  gpio_init(PIN_SD_CMD); gpio_put(PIN_SD_CMD, 1);
  gpio_set_dir(PIN_SD_CMD, GPIO_OUT);

  // in SD mode the DO pin acts as DAT0
  gpio_init(PIN_SD_DAT0);
  gpio_set_dir(PIN_SD_DAT0, GPIO_IN);

  // hw is setup, card is inserted. Try to set the card up.  
  uint32_t r=0;

  // start clocking
  sdio_cmd(64000, 0, 0x00000000, 0);   // go idle
  r = sdio_cmd(512, 8, 0x000001aa, 48);  // interface condition command
  if(r != 0x1aa) {
    sdc_debugf("interface condition failed");
    return false;
  }
  
  // wait for sd card to become ready
  do {
    sdio_cmd(512, 55, 0x00000000, 48);      // prepare ACMD
    r = sdio_cmd(256, 41, 0x40100000, 48);  // read OCR
  } while(!(r & 0x80000000));    // check busy bit
  
  // we only support sdhc cards
  if(!(r & 0x40000000)) {
    sdc_debugf("Not a SDHCv2 card");
    return false;
  }

  sdio_cmd(256, 2, 0x00000000, 136);        // read CID
  r = sdio_cmd(256, 3, 0x00000000, 48);     // get rca
  sdc_debugf("RCA = %04lx", r>>16);
  sdio_cmd(256, 9, r, 136);      // get CSD
  sdio_cmd(256, 7, r, 48);       // select card 
  sdio_cmd(512, 55, r, 48);      // prepare ACMD
  sdio_cmd(256, 16, 512, 48);    // select block len 512

  sdio_active = true;
  return true;
}

// Writing is not implemented, yet as it's not needed to load a core
bool sdio_sector_write(__attribute__((unused)) uint32_t lba, __attribute__((unused)) const uint8_t *buffer, __attribute__((unused))int count) {
  // return true when direct access is being used to tell the
  // sdc.c that the request has been handled directly
  return sdio_active;
}

// Read 512-byte blocks
bool sdio_sector_read(uint32_t lba, uint8_t *buffer, int count) {
  // return false when direct access is not being used to tell the
  // sdc.c that it needs to handle the access through the FPGA
  if(!sdio_active) return false;

  while(count--) {  
    // sdc_debugf("sdio_sector_read(%lu)", lba);
    sdio_cmd(96, 17, lba, 48);   // returns 0x2304
    if(sdio_read_data(buffer))
      sdc_debugf("read failed");

    lba++;
    buffer += 512;
  } 
    
  return true;
}

void sdio_release(void) {
  sdio_active = false;
  
  // No need to deinit the SD detection pin as that is not connected
  // to the FPGA at all.  
  sdc_debugf("De-initialize SW SD 1-bit");
  gpio_deinit(PIN_SD_CLK);
  gpio_deinit(PIN_SD_CMD);
  gpio_deinit(PIN_SD_DAT0);
}

// gain direct access to the sd card
void sdio_take_over(void) {
  mcu_hw_fpga_reconfig(false);
  sdio_init();
}

/* TODO: Move this into mcu_hw.c as it's quite hardware specific */
#include "ff.h"

void sdc_boot(void) {
  FATFS sdio_fs;

  // before doing anything, check if an SD card is inserted
  
  // -------- init SD control pins ---------
  gpio_init(PIN_SD_DET);
  gpio_set_dir(PIN_SD_DET, GPIO_IN);
  if(gpio_get(PIN_SD_DET)) {
    sdc_debugf("No card inserted");
    return;
  }
  
  // The FPGA is not ready, but it may still drive the SD card.
  // So reconfigure the FPGA without allowing it to boot from flash
  mcu_hw_fpga_reconfig(false);
  
  // try to boot the FPGA from SD card
  sdc_debugf("Attempting direct SD card boot ...");
  
  if(!sdio_init()) {
    sdc_debugf("No usable card found. Stopping boot attempt");
    sdio_release();
    mcu_hw_fpga_reconfig(true);
    return;
  }

  if(f_mount(&sdio_fs, "/sd", 1) != FR_OK) {
    sdc_debugf("direct mount failed");
    sdio_release();
    mcu_hw_fpga_reconfig(true);
    return;
  }
  
  sdc_debugf("Card mounted directly");
  // we can now try to load a core from SD card

  // try to load core.bin and if that doesn't work core.fs
  bool upload_ok = gowin_upload_core_bin("core.bin");
  if(!upload_ok) upload_ok = gowin_upload_core_fs("core.fs");  
    
  // unmount the fs
  f_unmount("/sd");
  
  // release the sd card
  sdio_release();
  
  // on upload failure reconfig the FPGA and allow it to (re-)boot from flash
  if(!upload_ok) {
    sdc_debugf("Upload failed, triggering FPGA reconfig from flash");
    mcu_hw_fpga_reconfig(true);
  }

  mcu_hw_reset();
  
}
