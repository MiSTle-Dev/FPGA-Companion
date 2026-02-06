/*
  sdc_direct.c - access the sd card directly from the rp2040

  This only works on the dev20k board and similar where the
  SD card is connected to the FPGA _and_ the rp2040  

  This implements software driven 1-bit SD mode even
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

#include "sdc_direct.h"
#include "../debug.h"
#include "../mcu_hw.h"
#include "../jtag.h"

#if MISTLE_BOARD != 4
#error "SDC direct access only works for the Dev20k board"
#endif

#define SD_BITBANG

// direct SD card pins
#define PIN_SD_DET      7   // pin 9, gpio 7 (not at FPGA)

#ifdef SD_BITBANG
#define PIN_SD_CLK     10   // pin 13, gpio 10
#define PIN_SD_CMD     11   // pin 14, gpio 11
#define PIN_SD_DAT0     8   // pin 11, gpio 8
#define PIN_SD_DAT3     9   // pin 12, gpio 9, actually unsed
#else
#define PIN_SD_nCS      9   // pin 12, gpio 9
#define PIN_SPI_SCK    10   // pin 13, gpio 10
#define PIN_SPI_DI     11   // pin 14, gpio 11
#define PIN_SPI_DO      8   // pin 11, gpio 8
#endif

static uint8_t spi_direct_crc7(uint8_t crcIn, uint8_t data) {
  crcIn ^= data;
  for (int i = 0; i < 8; i++) {
    if (crcIn & 0x80) crcIn ^= 0x89;
    crcIn <<= 1;
  } 
  return crcIn;
}

#ifdef SD_BITBANG
static void sdc_direct_sd_clock(uint16_t n) {
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

static void sdc_direct_sd_cmd_byte(uint8_t data) {
  for(int i=0;i<8;i++) {
    gpio_put(PIN_SD_CMD, (data&0x80)?1:0);
    data <<= 1;
    sdc_direct_sd_clock(1);
  }
}

static uint8_t sdc_direct_sd_cmd_byte_crc(uint8_t data, uint8_t crc) {
  sdc_direct_sd_cmd_byte(data);
  return spi_direct_crc7(crc, data);
}

static uint8_t sdc_direct_sd_cmd_bit_get(void) {
  char bit = gpio_get(PIN_SD_CMD)?1:0;
  sdc_direct_sd_clock(1);  
  return bit;  
}

static uint32_t sdc_direct_sd_reply(int len) {
  uint8_t buffer[(len-16)/8];
  
  gpio_set_dir(PIN_SD_CMD, GPIO_IN);

  // wait for startbit
  int i;
  for(i=0;i<32 && sdc_direct_sd_cmd_bit_get();i++);
  if(i == 32) return 0;

  uint8_t cmd = 0;
  for(i=0;i<7;i++)
    cmd = (cmd << 1)|sdc_direct_sd_cmd_bit_get();
  
  // get data bits
  uint32_t arg = 0;
  for(i=0;i<len-16;i++) {
    arg = (arg << 1)|sdc_direct_sd_cmd_bit_get();
    if((i&7) == 7) buffer[i/8] = arg & 0xff;
  }

  uint8_t crc = 0;
  for(i=0;i<8;i++)
    crc = (crc << 1)|sdc_direct_sd_cmd_bit_get();

  uint8_t crc_calc = spi_direct_crc7(0, cmd);
  if(cmd == 63) crc_calc = 0;
  for (int i=0; i<(len-16)/8; i++)
    crc_calc = spi_direct_crc7(crc_calc, buffer[i]);

  if((cmd != 63 || crc != 0xff) && (crc&0xfe) != crc_calc) {
    sdc_debugf(" rx %2d, 0x%08lx, 0x%02x", cmd & 0x3f, arg, crc);    
    sdc_debugf(" ---> CRC error: is %02x, expected %02x <----", crc&0xfe, crc_calc);
  }
    
  return arg;
}

uint32_t sdc_direct_sd_cmd(uint16_t pre_clocks, uint8_t cmd, uint32_t arg, uint8_t response_len) {
  sdc_direct_sd_clock(pre_clocks);
  
  gpio_set_dir(PIN_SD_CMD, GPIO_OUT);

  // send command
  uint8_t crc = sdc_direct_sd_cmd_byte_crc(cmd|0x40, 0);
  
  // send arg
  crc = sdc_direct_sd_cmd_byte_crc(arg >> 24, crc);
  crc = sdc_direct_sd_cmd_byte_crc(arg >> 16, crc);
  crc = sdc_direct_sd_cmd_byte_crc(arg >> 8, crc);
  crc = sdc_direct_sd_cmd_byte_crc(arg, crc);

  // sdc_debugf("sdc_direct_sd_cmd(%2d, 0x%08lx, 0x%02x)", cmd & 0x3f, arg, crc);
  
  sdc_direct_sd_cmd_byte(crc|1);

  // don't wait for response if none is expected (CMD0)
  if(!response_len) return 0;
  
  return sdc_direct_sd_reply(response_len);
}

static uint8_t sdc_direct_sd_data_bit_get(void) {
  uint8_t bit = gpio_get(PIN_SD_DAT0)?1:0;
  sdc_direct_sd_clock(1);  
  return bit;  
}

#if 0
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

uint8_t sdc_direct_sd_read_data(uint8_t *buffer) {
  // wait for dat0 to go low
  // TODO: Implement proper timeout
  int i;
  for(i=0;i<65535 && (sdc_direct_sd_data_bit_get()&1);i++);
  if(i == 65535) {
    sdc_debugf("Data timeout!");
    return 1;
  }

  // read 512 bytes
  for(int i=0;i<512;i++) {
#if 0
    uint8_t byte = 0;
    for(int bit=0;bit<8;bit++)
      byte = (byte << 1)|sdc_direct_sd_data_bit_get();
    buffer[i] = byte;
#else
    buffer[i] =
      (sdc_direct_sd_data_bit_get()?0x80:0x00) |
      (sdc_direct_sd_data_bit_get()?0x40:0x00) |
      (sdc_direct_sd_data_bit_get()?0x20:0x00) |
      (sdc_direct_sd_data_bit_get()?0x10:0x00) |
      (sdc_direct_sd_data_bit_get()?0x08:0x00) |
      (sdc_direct_sd_data_bit_get()?0x04:0x00) |
      (sdc_direct_sd_data_bit_get()?0x02:0x00) |
      (sdc_direct_sd_data_bit_get()?0x01:0x00);
    #endif
  }

  // read crc
  uint16_t crc = 0;
  for(int b=0;b<16;b++)
    crc = (crc << 1)|sdc_direct_sd_data_bit_get();

#if 0
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
#else

uint8_t sdc_direct_spi_transfer(uint8_t data) {
  uint8_t retval = 0;
  spi_write_read_blocking(spi1, &data, &retval, 1);
  return retval;
}

static uint8_t sdc_direct_spi_transfer_crc(uint8_t byte, uint8_t crc) {
  sdc_direct_spi_transfer(byte);
  return spi_direct_crc7(byte, crc);
}

void sdc_direct_spi_cmd(uint8_t cmd, uint32_t arg) {
  // transmit command and argument to sd card
  uint8_t crc = sdc_direct_spi_transfer_crc(cmd|0x40, 0);
  
  crc = sdc_direct_spi_transfer_crc((uint8_t)(arg >> 24), crc);
  crc = sdc_direct_spi_transfer_crc((uint8_t)(arg >> 16), crc);
  crc = sdc_direct_spi_transfer_crc((uint8_t)(arg >> 8), crc);
  crc = sdc_direct_spi_transfer_crc((uint8_t)(arg), crc);

  // transmit crc
  sdc_direct_spi_transfer(crc | 0x01);
}

uint8_t sdc_direct_spi_read_result(uint8_t *buffer, uint16_t bsize) {
  // in SD mode the CMD reply comes on the CMD pin
  uint8_t i = 0, res = 0;

  // keep polling until actual data received
  while((res = sdc_direct_spi_transfer(0xff)) == 0xff) {
    i++;
    
    // if no data received for 8 bytes, break
    if(i > 8) break;
  }

  // parse the response code  
  if(res & 0x80) sdc_debugf("Invalid R1 response");
  else {
    if(res & 0x40) sdc_debugf("Parameter Error");
    if(res & 0x20) sdc_debugf("Address Error");
    if(res & 0x10) sdc_debugf("Erase Sequence Error");
    if(res & 0x08) sdc_debugf("Command CRC Error");
    if(res & 0x04) sdc_debugf("Illegal Command");
    if(res & 0x02) sdc_debugf("Erase Reset");
    // if(res & 0x01) sdc_debugf("In Idle State");
  }
    
  if(buffer && bsize) {
    // if bsize == 512, we are reading a sector, else just read
    // the given number of bytes
    if(bsize != 512) {
      while(bsize--)
	*buffer++ = sdc_direct_spi_transfer(0xff);
    } else {
      // now we are waiting for data token, it takes around 300us
      int timeout = 0;
      while(sdc_direct_spi_transfer(0xff) != 0xfe) {
        if (timeout++ >= 1000000) { // we can't wait forever
	  sdc_debugf("CMD17/18 (READ_BLOCK): no data token!");
	  return(0xff);
        }
      }

      spi_read_blocking(spi1, 0xff, buffer, 512);

      sdc_direct_spi_transfer(0xff); // read CRC lo byte
      sdc_direct_spi_transfer(0xff); // read CRC hi byte
    }
  }
  
  return res;
}

static void sdc_direct_spi_select(bool select) {
  // (de)assert chip select
  sdc_direct_spi_transfer(0xff);
  gpio_put(PIN_SD_nCS, select?0:1);  
  sdc_direct_spi_transfer(0xff);
}

static uint8_t sdc_direct_send_cmd(uint8_t cmd, uint32_t arg,
				  uint8_t *buffer, uint16_t blen) {
  sdc_direct_spi_select(true);
  sdc_direct_spi_cmd(cmd, arg);
  uint8_t res = sdc_direct_spi_read_result(buffer, blen);
  sdc_direct_spi_select(false);
  return res;
}
#endif

static bool sdc_direct_active = false;

bool sdc_direct_init(void) {  
#ifndef SD_BITBANG
  sdc_debugf("=== Direct SD card via HW SPI ===");
  sdc_debugf("  DET = %2d", PIN_SD_DET);
  sdc_debugf("  nCS = %2d", PIN_SD_nCS);
  sdc_debugf("  SCK = %2d", PIN_SPI_SCK);
  sdc_debugf("  DI  = %2d", PIN_SPI_DI);
  sdc_debugf("  DO  = %2d", PIN_SPI_DO);
#else  
  sdc_debugf("=== Direct SD card via SW SD 1-bit ===");
  sdc_debugf("  DET  = %2d", PIN_SD_DET);
  sdc_debugf("  SCK  = %2d", PIN_SD_CLK);
  sdc_debugf("  CMD  = %2d", PIN_SD_CMD);
  sdc_debugf("  DAT0 = %2d", PIN_SD_DAT0);
  sdc_debugf("  DAT1 = na");
  sdc_debugf("  DAT2 = na");
  sdc_debugf("  DAT3 = %2d", PIN_SD_DAT3);
#endif  
  
  // -------- init SD control pins ---------
  gpio_init(PIN_SD_DET);
  gpio_set_dir(PIN_SD_DET, GPIO_IN);
  if(gpio_get(PIN_SD_DET)) {
    sdc_debugf("No card inserted");
    return false;
  }
  
#ifndef SD_BITBANG
  // init spi1 at 100kHz, mode 0
  spi_init(spi1, 100000);
  spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

  gpio_set_function(PIN_SPI_DO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SPI_DI, GPIO_FUNC_SPI);
#else  
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
#endif
  
  // hw is setup, card is inserted. Try to set the card up.  
#ifdef SD_BITBANG
  uint32_t r=0;

  // start clocking
  sdc_direct_sd_cmd(64000, 0, 0x00000000, 0);   // go idle
  r = sdc_direct_sd_cmd(512, 8, 0x000001aa, 48);  // interface condition command
  if(r != 0x1aa) {
    sdc_debugf("interface condition failed");
    return false;
  }
  
  // wait for sd card to become ready
  do {
    sdc_direct_sd_cmd(512, 55, 0x00000000, 48);      // prepare ACMD
    r = sdc_direct_sd_cmd(256, 41, 0x40100000, 48);  // read OCR
  } while(!(r & 0x80000000));    // check busy bit
  
  // we only support sdhc cards
  if(!(r & 0x40000000)) {
    sdc_debugf("Not a SDHCv2 card");
    return false;
  }

  sdc_direct_sd_cmd(256, 2, 0x00000000, 136);        // read CID
  r = sdc_direct_sd_cmd(256, 3, 0x00000000, 48);     // get rca
  sdc_debugf("RCA = %04lx", r>>16);
  sdc_direct_sd_cmd(256, 9, r, 136);      // get CSD
  sdc_direct_sd_cmd(256, 7, r, 48);       // select card 
  sdc_direct_sd_cmd(512, 55, r, 48);      // prepare ACMD
  sdc_direct_sd_cmd(256, 16, 512, 48);    // select block len 512

  sdc_direct_active = true;
  return true;
#else
  gpio_init(PIN_SD_nCS); gpio_put(PIN_SD_nCS, 1);
  gpio_set_dir(PIN_SD_nCS, GPIO_OUT);

  // send 80 clocks
  for(int i=0;i<10;i++) sdc_direct_spi_transfer(0xff);

  // try a few times to reset the card
  uint8_t retries;
  for(retries=0; retries<16; retries++) {
    vTaskDelay(pdMS_TO_TICKS(1));
    if(sdc_direct_send_cmd(0, 0, NULL, 0) == 0x01) break;
  }

  if(retries < 16) {
    uint8_t ocr[4];
    if(sdc_direct_send_cmd(8, 0x1AA, ocr, sizeof(ocr)) == 1) {
      if(ocr[2] == 0x01 && ocr[3] == 0xAA) {
	// the card can work at 2.7-3.6V
	sdc_debugf("SD/SDHC card detected");
	// now we must wait until CMD41 returns 0 (or timeout elapses)
	TickType_t timeout = xTaskGetTickCount();
	while((xTaskGetTickCount() - timeout) < pdMS_TO_TICKS(2000)) {	
	  if (sdc_direct_send_cmd(55, 0, NULL, 0) == 0x01) {
	    // CMD55 must precede any ACMD command
	    if (sdc_direct_send_cmd(41, 1 << 30, NULL, 0) == 0x00) { // ACMD41 with HCS bit
	      // initialization completed
	      if (sdc_direct_send_cmd(58, 0, ocr, sizeof(ocr)) == 0x00) {
		// check CCS (Card Capacity Status) bit in the OCR
		// if CCS set then the card is SDHC compatible
		if(ocr[0] & 0x40) {
		  sdc_debugf("SDHC card initialized");
		  // increase SPI speed to 25MHz
		  spi_set_baudrate(spi1, 25000000);
		  sdc_direct_active = true;
		  return true;
		}
		sdc_debugf("Regular SD card not supported");
	      } else
		sdc_debugf("cmd58 failed");
	    } else
	      sdc_debugf("cmd41 failed");
	  } else
	    sdc_debugf("cmd55 failed");
	}
	sdc_debugf("Card initialization timed out!");
      }
    } else
      sdc_debugf("MMC card not supported");
  } else
    sdc_debugf("Unable to reset card");

#endif

  return false;
}

bool sdc_direct_write(__attribute__((unused)) uint32_t lba, __attribute__((unused)) const uint8_t *buffer) {
  // return true when direct access is being used to tell the
  // sdc.c that the request has been handled directly
  return sdc_direct_active;
}

// Read single 512-byte block
bool sdc_direct_read(uint32_t lba, uint8_t *buffer) {
  // return false when direct access is not being used to tell the
  // sdc.c that it needs to handle the access through the FPGA
  if(!sdc_direct_active) return false;

#ifdef SD_BITBANG
  // sdc_debugf("sdc_direct_read(%lu)", lba);
  sdc_direct_sd_cmd(96, 17, lba, 48);   // returns 0x2304
  if(sdc_direct_sd_read_data(buffer))
    sdc_debugf("read failed");
#else
  // spi
  if(sdc_direct_send_cmd(17, lba, buffer, 512))
    sdc_debugf("read failed");
#endif
  
  return true;
}

void sdc_direct_release(void) {
  sdc_direct_active = false;
  
  // No need to deinit the SD detection pin as that is not connected
  // to the FPGA at all.
  
#ifndef SD_BITBANG
  // from Part1PhysicalLayerSimplifiedSpecification page 287:
  // The only way to return to the SD mode is by entering the power cycle.
  // TODO: Wire pin20 of the MCU to the SD cards power input.
    
  sdc_debugf("De-initialize HW SPI");
  gpio_deinit(PIN_SD_nCS);
  spi_deinit(spi1);
  gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_NULL);
  gpio_set_function(PIN_SPI_DI, GPIO_FUNC_NULL);
  gpio_set_function(PIN_SPI_DO, GPIO_FUNC_NULL);

#warning "================================================="  
#warning "SPI needs the ability to power cycle the SD card!"  
#warning "The FPGA will not be able to use the SD card!!!!!"  
#warning "================================================="  
#else  
  sdc_debugf("De-initialize SW SD 1-bit");
  gpio_deinit(PIN_SD_CLK);
  gpio_deinit(PIN_SD_CMD);
  gpio_deinit(PIN_SD_DAT0);
#endif
}

#include <ff.h>
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
  
  jtag_debugf("GW2AR-18 detected");

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
  
  // -------- init SD control pins ---------
  gpio_init(PIN_SD_DET);
  gpio_set_dir(PIN_SD_DET, GPIO_IN);
  if(gpio_get(PIN_SD_DET)) {
    sdc_debugf("No card inserted");
    return;
  }
  
  // the FPGA is not ready, but it may still drive the SD card.
  // So reconfigure the FPGA without allowing it to boot from flash
  mcu_hw_fpga_reconfig(false);  // TODO: reenable this
  
  // try to boot the FPGA from SD card
  sdc_debugf("Attempting direct SD card boot ...");
  
  if(!sdc_direct_init()) {
    sdc_debugf("No usable card found. Stopping boot attempt");
    sdc_direct_release();
    mcu_hw_fpga_reconfig(true);
    return;
  }

  if(f_mount(&fs, "/sd", 1) != FR_OK) {
    sdc_debugf("direct mount failed");
    sdc_direct_release();
    mcu_hw_fpga_reconfig(true);
    return;
  }
  
  sdc_debugf("Card mounted directly");
  // we can now try to load a core from SD card

  // try to load core.bin and if that doesn't work core.fs
  bool upload_ok = sdc_direct_upload_core_bin("core.bin");
  if(!upload_ok) upload_ok = sdc_direct_upload_core_fs("core.fs");  
    
  // unmount the fs
  f_unmount("/sd");
  
  // release the sd card
  sdc_direct_release();
  
  // on upload failure reconfig the FPGA and allow it to (re-)boot from flash
  if(!upload_ok) {
    sdc_debugf("Upload failed, triggering FPGA reconfig from flash");
    mcu_hw_fpga_reconfig(true);
  }
}
