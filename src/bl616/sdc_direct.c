/*
  sdc_direct.c - access the sd card directly from the  BL616

  SD card access for Console 60K and 138k where the
  card is connected to the FPGA _and_ the BL616  

  uses the BL616 SDH SD Host controller (SDH) to access the card or read from an USB Mass Storage (MSC).
*/

#include "bflb_gpio.h"
#include "board.h"
#include "sdc_direct.h"

static struct bflb_device_s *gpio;
static bool sdc_direct_active = false;

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
