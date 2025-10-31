/*
  sdc_local.c - access the sd card directly from the rp2040

  This only works on the dev20k board and similar where the
  SD card is connected to the FPGA _and_ the rp2040  
*/

#include "pico/stdlib.h"
#include "sdc_local.h"
#include "../debug.h"
#include "hardware/spi.h"

#if MISTLE_BOARD != 4
#error "SDC local access only works for the Dev20k board"
#endif

#if MISTLE_BOARD == 4
// direct SD card pins
#define PIN_SD_DET     7   // pin 9, gpio 7
#define PIN_SD_DO      8   // pin 11, gpio 8
#define PIN_SD_nCS     9   // pin 12, gpio 9
#define PIN_SD_CLK    10   // pin 13, gpio 10
#define PIN_SD_DI     11   // pin 14, gpio 11
#else
// other boards ...
#endif

// #define SPI_BITBANG

// spi bit bang a single byte
uint8_t sdc_local_transfer(uint8_t data) {
  uint8_t retval = 0;
#ifndef SPI_BITBANG
  spi_write_read_blocking(spi1, &data, &retval, 1);
#else
  for(int i=0;i<8;i++) {
    gpio_put(PIN_SD_DI, (data&0x80)?1:0);
    data <<= 1;
    
    retval <<= 1;
    if(gpio_get(PIN_SD_DO)) retval |= 0x01;

    gpio_put(PIN_SD_CLK, 1); sleep_us(100);
    gpio_put(PIN_SD_CLK, 0); sleep_us(100);
  }
#endif
  return retval;
}

void sdc_local_cmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
  // transmit command to sd card
  sdc_local_transfer(cmd|0x40);

  // transmit argument
  sdc_local_transfer((uint8_t)(arg >> 24));
  sdc_local_transfer((uint8_t)(arg >> 16));
  sdc_local_transfer((uint8_t)(arg >> 8));
  sdc_local_transfer((uint8_t)(arg));
  
  // transmit crc
  sdc_local_transfer(crc|0x01);
}

#define CMD0        0
#define CMD0_ARG    0x00000000
#define CMD0_CRC    0x94

uint8_t sdc_local_readRes1() {
  uint8_t i = 0, res1;
  
  // keep polling until actual data received
  while((res1 = sdc_local_transfer(0xFF)) == 0xFF) {
    i++;
    
    // if no data received for 8 bytes, break
    if(i > 8) break;
  }

  // parse the response code
  
  if(res1 & 0x80) sdc_debugf("Invalid R1 response");
  if(res1 & 0x40) sdc_debugf("Parameter Error");
  if(res1 & 0x20) sdc_debugf("Address Error");
  if(res1 & 0x10) sdc_debugf("Erase Sequence Error");
  if(res1 & 0x08) sdc_debugf("Command CRC Error");
  if(res1 & 0x04) sdc_debugf("Illegal Command");
  if(res1 & 0x02) sdc_debugf("Erase Reset");
  if(res1 & 0x01) sdc_debugf("In Idle State");
  
  return res1;
}

uint8_t sdc_local_goIdleState() {
  // assert chip select
  sdc_local_transfer(0xFF);
  gpio_put(PIN_SD_nCS, 0);
  sdc_local_transfer(0xFF);

  // send CMD0
  sdc_local_cmd(CMD0, CMD0_ARG, CMD0_CRC);

  // read response
  uint8_t res1 = sdc_local_readRes1();

  // deassert chip select
  sdc_local_transfer(0xFF);
  gpio_put(PIN_SD_nCS, 1);
  sdc_local_transfer(0xFF);
  
  return res1;
}

void sdc_local_test(void) {
  // send a software reset command and check for valid r1 reply code
  uint8_t res = sdc_local_goIdleState();
  sdc_debugf("CMD0 result 0x%02x (should be 0x01)", res);
}
  
void sdc_local_init(void) {
  
  sdc_debugf("=== Local SD card ===");
  sdc_debugf("  DET = %d", PIN_SD_DET);
  sdc_debugf("  DO  = %d", PIN_SD_DO);
  sdc_debugf("  nCS = %d", PIN_SD_nCS);
  sdc_debugf("  SCK = %d", PIN_SD_CLK);
  sdc_debugf("  DI  = %d", PIN_SD_DI);

  // -------- init SD control pins ---------
  gpio_init(PIN_SD_DET);
  gpio_set_dir(PIN_SD_DET, GPIO_IN);
  sdc_debugf("SD detect: %scard inserted", gpio_get(PIN_SD_DET)?"no ":"");

#ifndef SPI_BITBANG
  sdc_debugf("Using hw spi");
  
  // init spi1 at 100kHz, mode 1
  spi_init(spi1, 100000);
  spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

  gpio_set_function(PIN_SD_DO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SD_CLK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SD_DI, GPIO_FUNC_SPI);
#else  
  sdc_debugf("Using sw spi");

  // we should only drive these signals if we can be sure that the FPGA does
  // not drive them
  gpio_init(PIN_SD_CLK); gpio_put(PIN_SD_CLK, 0);
  gpio_set_dir(PIN_SD_CLK, GPIO_OUT);
  gpio_init(PIN_SD_DI); gpio_put(PIN_SD_DI, 0);
  gpio_set_dir(PIN_SD_DI, GPIO_OUT);
  gpio_init(PIN_SD_DO);
  gpio_set_dir(PIN_SD_DO, GPIO_IN);
#endif
  
  gpio_init(PIN_SD_nCS); gpio_put(PIN_SD_nCS, 1);
  gpio_set_dir(PIN_SD_nCS, GPIO_OUT);
  
  for(int i=0;i<10;i++)
    sdc_local_transfer(0xff);
}

void sdc_local_release(void) {
  sdc_debugf("Releasing local access");

  for(int i=0;i<32;i++) sdc_local_transfer(0xff);
  
  gpio_deinit(PIN_SD_DET);
  gpio_deinit(PIN_SD_nCS);

#ifndef SPI_BITBANG
  spi_deinit(spi1);
  gpio_set_function(PIN_SD_CLK, GPIO_FUNC_NULL);
  gpio_set_function(PIN_SD_DI, GPIO_FUNC_NULL);
  gpio_set_function(PIN_SD_DO, GPIO_FUNC_NULL);
#else  
  gpio_deinit(PIN_SD_CLK);
  gpio_deinit(PIN_SD_DI);
  gpio_deinit(PIN_SD_DO);
#endif
}
