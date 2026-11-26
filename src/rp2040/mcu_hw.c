/*
  mcu_hw.c - MiSTeryNano FPGA companion hardware driver for rp2040
*/

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>
#include <malloc.h>

#include "pico/stdlib.h"

#include <stdio.h>
#include <strings.h>
#include "tusb.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/flash.h"
#include "../usb_controller_maps_tools.h"

#include "../debug.h"
#include "../config.h"
#include "../spi.h"
#include "../sysctrl.h"
#include "../inifile.h"
#include "../at_wifi.h"
#include "../inifile.h"
#include "../menu.h"
#include "../gowin.h"

#include "../mcu_hw.h"

#include "board.h"

// return info/version strings
const char *mcu_get_info(int index) {
  static const char *board_name = BOARD_NAME;
  static const char *sdk_version = "Pico SDK " PICO_SDK_VERSION_STRING;
  static char gcc_version[32];
  
  if(index == 0)      return board_name;
  else if(index == 1) return sdk_version;
  else if(index == 2) {
    sprintf(gcc_version, "GCC %d.%d.%d", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
    return gcc_version;
  }

  return NULL;
}

/* ======================================================================== */
/* ===============                USB                        ============== */
/* ======================================================================== */

CFG_TUH_MEM_SECTION struct {
  TUH_EPBUF_TYPE_DEF(tusb_desc_device_t, device);
  TUH_EPBUF_DEF(serial, 64*sizeof(uint16_t));
  TUH_EPBUF_DEF(buf, 128*sizeof(uint16_t));
} desc;

TaskHandle_t pio_usb_task_handle = NULL;
extern void usb_jtag_poll(void);
static void pio_usb_task(__attribute__((unused)) void *parms) {
#ifdef ENABLE_USB_HID
  usb_hid_init();
#endif
  
#ifdef ENABLE_USB_XBOX
  usb_xbox_init();
#endif
  
  while(1) {
    for(int i=0;i<100;i++) {
      tuh_task();
#ifdef ENABLE_JTAG
#ifdef ENABLE_USB_DEVICE
      tud_task();
      usb_jtag_poll();
#endif
#endif
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

bool mcu_hw_hid_present(void) {
#ifdef ENABLE_USB_HID
  if(usb_hid_present(USB_HID_KEYBOARD | USB_HID_JOYSTICK))
    return true;
#endif

#ifdef ENABLE_USB_XBOX
  if(usb_xbox_present())
    return true;
#endif
  
  return false;
}

// check for presence of usb devices and drive leds accordingly
void mcu_hw_usb_check_devices(void) {
#ifdef LED_MOUSE_PIN
  int mice = 0;
#endif
#ifdef LED_KEYBOARD_PIN
  int keyboards = 0;
#endif
#ifdef LED_JOYSTICK_PIN
  int joysticks = 0;
#endif

#ifdef ENABLE_USB_HID
#ifdef LED_MOUSE_PIN
  mice += usb_hid_present(USB_HID_MOUSE);
#endif
#ifdef LED_JOYSTICK_PIN
  joysticks += usb_hid_present(USB_HID_JOYSTICK);
#endif
#ifdef LED_KEYBOARD_PIN
  keyboards += usb_hid_present(USB_HID_KEYBOARD);
#endif
#endif
  
#if defined(LED_JOYSTICK_PIN) && defined(ENABLE_USB_XBOX)
  joysticks += usb_xbox_present();
#endif
  
#ifdef LED_MOUSE_PIN
  gpio_put(LED_MOUSE_PIN, mice);
#endif
  
#ifdef LED_KEYBOARD_PIN
  gpio_put(LED_KEYBOARD_PIN, keyboards);
#endif
  
#ifdef LED_JOYSTICK_PIN
  gpio_put(LED_JOYSTICK_PIN, joysticks);
#endif  

  // Redraw OSD whenever the USB LEDs are being updated
  // as the presence of a keyboard may have chaned which in
  // turn is reflected by the 'X' icon on the main OSD title
  menu_notify(MENU_EVENT_NONE);
}

/* ========================================================================= */
/* =======                          SPI                                ===== */
/* ========================================================================= */
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "queue.h"

extern TaskHandle_t com_task_handle;
static SemaphoreHandle_t sem;

static void irq_handler(void) {
  if(gpio_get_irq_event_mask(SPI_IRQ_PIN) & GPIO_IRQ_LEVEL_LOW) {
    gpio_acknowledge_irq(SPI_IRQ_PIN, GPIO_IRQ_LEVEL_LOW);

    // Disable interrupt. It will be re-enabled by the com task
    gpio_set_irq_enabled(SPI_IRQ_PIN, GPIO_IRQ_LEVEL_LOW, false);
 
    if(com_task_handle) {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR( com_task_handle, &xHigherPriorityTaskWoken );
      portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
  }
}

void mcu_hw_spi_init(void) {
  debugf("Initializing SPI");

  sem = xSemaphoreCreateMutex();

  // init SPI at 20Mhz, mode 1
  spi_init(SPI_BUS, 20000000);
  spi_set_format(SPI_BUS, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
  
  debugf("  MISO = %d", SPI_RX_PIN);
  gpio_set_function(SPI_RX_PIN, GPIO_FUNC_SPI);
  debugf("  SCK  = %d", SPI_SCK_PIN);
  gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
  debugf("  MOSI = %d", SPI_TX_PIN);
  gpio_set_function(SPI_TX_PIN, GPIO_FUNC_SPI);
  
  // Chip select is active-low, so we'll initialise it to a driven-high state
  debugf("  CSn  = %d", SPI_CSN_PIN);
  gpio_init(SPI_CSN_PIN);
  gpio_set_dir(SPI_CSN_PIN, GPIO_OUT);
  gpio_put(SPI_CSN_PIN, 1);

  // The interruput input isn't strictly part of the SPI
  // The interrupt is active low on GP22
  debugf("  IRQn = %d", SPI_IRQ_PIN);

  // set handler but not enable yet as the main task may not be ready
  gpio_init(SPI_IRQ_PIN);
  gpio_set_dir(SPI_IRQ_PIN, GPIO_IN);
  gpio_pull_up(SPI_IRQ_PIN);
  gpio_add_raw_irq_handler(SPI_IRQ_PIN, irq_handler);  
}

void mcu_hw_irq_ack(void) {
  static bool first = true;

  if(first) {
    debugf("enable IRQ");
    irq_set_enabled(IO_IRQ_BANK0, true);
    first = false;
  }
  //  else debugf("re-enable IRQ");
  
  // re-enable the interrupt since it was now serviced outside the irq handler
  gpio_set_irq_enabled(SPI_IRQ_PIN, GPIO_IRQ_LEVEL_LOW, 1); 
}

void mcu_hw_spi_begin() {
  xSemaphoreTake(sem, 0xffffffffUL);      // wait forever
  gpio_put(SPI_CSN_PIN, 0);  // Active low
}

void mcu_hw_spi_end() {
  gpio_put(SPI_CSN_PIN, 1);
  xSemaphoreGive(sem);
}

unsigned char mcu_hw_spi_tx_u08(unsigned char b) {
  unsigned char retval;  spi_write_read_blocking(SPI_BUS, &b, &retval, 1);
  return retval;
}

/* ======================================================================= */
/* ======                   custom usb host drivers              ========= */
/* ======================================================================= */

#include "xinput_host.h"
#include "asix_host.h"

usbh_class_driver_t const* usbh_app_driver_get_cb(uint8_t* driver_count){
  static usbh_class_driver_t drivers[2];

  *driver_count = 2;
  memcpy(drivers+0, &usbh_xinput_driver, sizeof(usbh_class_driver_t));
  memcpy(drivers+1, &usbh_asix_driver, sizeof(usbh_class_driver_t));

  return drivers;
}

#include "hardware/watchdog.h"

void mcu_hw_reset(void) {
  debugf("HW reset");
#ifdef ENABLE_WIFI
  mcu_hw_wifi_drop();
#endif
  watchdog_reboot(0, 0, 10);
  while(1);
}

#ifndef WS2812_PIN
// the LED PIN is not defined if we build for a pico-w. But we
// detect the wireless chip and know when running on the regular pico
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

static void led_timer(__attribute__((unused)) TimerHandle_t pxTimer) {
  switch(inifile_config_get_int("system", "led", DEFAULT_LED_MODE)) {
  case 0:    
    gpio_xor_mask( 1u << PICO_DEFAULT_LED_PIN );
    break;
  case 1:    
    gpio_set_mask( 1u << PICO_DEFAULT_LED_PIN );
    break;
  case 2:    
    gpio_clr_mask( 1u << PICO_DEFAULT_LED_PIN );
    break;
  }
}
#endif

void mcu_hw_main_loop(void) {
  /* Start the tasks and timer running. */  
  vTaskStartScheduler();
  
  /* If all is well, the scheduler will now be running, and the following
     line will never be reached.  If the following line does execute, then
     there was insufficient FreeRTOS heap memory available for the Idle and/or
     timer tasks to be created.  See the memory management section on the
     FreeRTOS web site for more details on the FreeRTOS heap
     http://www.freertos.org/a00111.html. */

  for( ;; );
}

#ifdef ENABLE_WIFI
// the adc is used to determine the Pico type (W or not)
#include "hardware/adc.h"

static void wifi_task(__attribute__((unused)) void *parms) {
  debugf("WiFi init task ...");

  // wifi init only returns if bluetooth is not enabled. Otherwise
  // it will run the bluetooth main loop
  mcu_hw_wifi_init();

  // only used for init
  vTaskDelete(NULL);
}
#endif

#ifdef WS2812_PIN
#define WS2812_COLOR 0x40000000  // GRBX: 25% green

static void ws_led_timer(__attribute__((unused)) TimerHandle_t pxTimer) {
  static char state = 0;
  switch(inifile_config_get_int("system", "led", DEFAULT_LED_MODE)) {
  case 0:    
    pio_sm_put_blocking(pio0, 0, state?WS2812_COLOR:0);
    break;
  case 1:    
    pio_sm_put_blocking(pio0, 0, WS2812_COLOR);
    break;
  case 2:    
    pio_sm_put_blocking(pio0, 0, 0);
    break;
  }    
  state = !state;
}
#endif

// keep track of one single device only, by now
static int msc_dev_addr = -1;
static volatile bool msc_busy = 0;

static bool disk_io_complete(uint8_t dev_addr, tuh_msc_complete_data_t const * cb_data) {
  // usb_debugf("disk_io_complete(%d)", dev_addr);
  (void) dev_addr; (void) cb_data;
  msc_busy = 0;
  return true;
}

// This should never be called from the USB task itself
void mcu_hw_usb_sector_read(void *buffer, int sector, int count) {
  usb_debugf("mcu_hw_usb_sector_read(%d, %p, %d, %d)", msc_dev_addr, buffer, sector, count);
  if(msc_dev_addr < 0 || msc_busy) return;
  
  msc_busy = 1;
  tuh_msc_read10(msc_dev_addr, 0, buffer, sector, count, disk_io_complete, 0);

  // wait for transfer to be done. If the pio task is not running, then
  // actively call the usb host task handler
  while(msc_busy)
    if(!pio_usb_task_handle)
      tuh_task();
}

bool mcu_hw_usb_msc_present(void) {
  return msc_dev_addr >= 0;
}

// Invoked when a device with MassStorage interface is mounted
void tuh_msc_mount_cb(uint8_t dev_addr) {
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);
  usb_debugf("[%04x:%04x][%u] MSC mounted", vid, pid, dev_addr);

  if(msc_dev_addr >= 0) {
    usb_debugf("Skipping additional device");
    return;
  }

  if(tuh_msc_get_block_size(dev_addr, 0) != 512) {
    usb_debugf("Unsupported block size %lu", tuh_msc_get_block_size(dev_addr, 0));
    return;
  }
  
  // Get capacity of device
  usb_debugf("Block count: %lu", tuh_msc_get_block_count(dev_addr, 0));
  msc_dev_addr = dev_addr;

  menu_notify(MENU_EVENT_USB_MOUNTED);
}


extern char __StackLimit, __bss_end__;   
uint32_t getTotalHeap(void) {
   return &__StackLimit  - &__bss_end__;
}

uint32_t getFreeHeap(void) {
   struct mallinfo m = mallinfo();
   return getTotalHeap() - m.uordblks;
}

void mcu_hw_fpga_reconfig(bool run) {
  // alternally the FPGA may be put into a mode != 00 to
  // suppress MSPI loading
#ifdef PIN_nCFG  
  if(!run) {
    gpio_put(PIN_MODE0, 1); gpio_set_dir(PIN_MODE0, GPIO_OUT);
    gpio_put(PIN_MODE1, 0); gpio_set_dir(PIN_MODE1, GPIO_OUT);
  }

  // trigger FPGA reconfiguration
  gpio_put(PIN_nCFG, 0);  vTaskDelay(pdMS_TO_TICKS(1));
  gpio_put(PIN_nCFG, 1);  vTaskDelay(pdMS_TO_TICKS(100));
  
  // make mode pins input, so the buttons S1/S2 connected to them work
  gpio_set_dir(PIN_MODE0, GPIO_IN);
  gpio_set_dir(PIN_MODE1, GPIO_IN);  
#endif
}

#ifdef PIN_3WAY_SELECT
static void button_irq_handler(void) {
  if(gpio_get_irq_event_mask(PIN_3WAY_CLICK) & GPIO_IRQ_EDGE_FALL) {
    gpio_acknowledge_irq(PIN_3WAY_CLICK, GPIO_IRQ_EDGE_FALL);
    menu_notify(MENU_EVENT_TOGGLE);
  }
}
#endif

void mcu_hw_init(void) {
  // default 125MHz is not appropriate for PIO USB. Sysclock should be multiple of 12MHz.
  // some devices won't enumerate propery below ~16*12Mhz
  set_sys_clock_khz(16*12000, true);
  
  stdio_init_all();    // ... so stdio can adjust its bit rate
#if MISTLE_BOARD == 2
  // the waveshare mini does not support SWD and we thus use a simpler (slower) UART
  uart_set_baudrate(uart0, 460800);  
#else
  uart_set_baudrate(uart0, 921600);
#endif
  
#ifdef PICO_RP2350
  debugf( LOGO "        FPGA Companion for RP2350\r\n");
#else
  debugf( LOGO "        FPGA Companion for RP2040\r\n");
#endif

  uint8_t txbuf[4] = {0x9f};
  uint8_t rxbuf[4] = {0};
  flash_do_cmd(txbuf, rxbuf, 4);
  debugf("Flash manufacturer ID: %02x", rxbuf[1]);
  debugf("Flash memory type: %02x", rxbuf[2]);
  if(rxbuf[3] < 10)      debugf("Flash size: %d", 1 << rxbuf[3]);
  else if(rxbuf[3] < 20) debugf("Flash size: %dKB", 1 << (rxbuf[3]-10));
  else                   debugf("Flash size: %dMB", 1 << (rxbuf[3]-20));
  
#if CFG_TUH_RPI_PIO_USB == 0
  debugf("Using native USB");
#else
  debugf("USB D+/D- on GP%d and GP%d", PIO_USB_DP_PIN_DEFAULT, PIO_USB_DP_PIN_DEFAULT+1);
#endif

  mcu_hw_spi_init();

#ifdef PIN_3WAY_SELECT
  // Initialize 3WAY button as input and use it as a menu
  // button during runtime. This would be extended to use the
  // 3way on the Mini20K to control the menu
  gpio_init(PIN_3WAY_SELECT);
  gpio_set_dir(PIN_3WAY_SELECT, GPIO_IN);
  gpio_pull_up(PIN_3WAY_SELECT);
  gpio_set_irq_enabled(PIN_3WAY_SELECT, GPIO_IRQ_EDGE_FALL, true);
  gpio_add_raw_irq_handler(PIN_3WAY_SELECT, button_irq_handler);  
#endif
  
  // initialize the LED gpios
#ifdef LED_MOUSE_PIN
  debugf("LED MOUSE    = %d", LED_MOUSE_PIN);
  gpio_init(LED_MOUSE_PIN);
  gpio_set_dir(LED_MOUSE_PIN, GPIO_OUT);
  gpio_put(LED_MOUSE_PIN, 0);
#endif
#ifdef LED_KEYBOARD_PIN
  debugf("LED KEYBOARD = %d", LED_KEYBOARD_PIN);
  gpio_init(LED_KEYBOARD_PIN);
  gpio_set_dir(LED_KEYBOARD_PIN, GPIO_OUT);
  gpio_put(LED_KEYBOARD_PIN, 0);
#endif
#ifdef LED_JOYSTICK_PIN
  debugf("LED JOYSTICK = %d", LED_JOYSTICK_PIN);
  gpio_init(LED_JOYSTICK_PIN);
  gpio_set_dir(LED_JOYSTICK_PIN, GPIO_OUT);
  gpio_put(LED_JOYSTICK_PIN, 0);
#endif
  
  tuh_hid_set_default_protocol(HID_PROTOCOL_REPORT);
  //  tuh_init(BOARD_TUH_RHPORT);
  tusb_rhport_init_t host_init = {
    .role = TUSB_ROLE_HOST,
    .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUH_RHPORT, &host_init);
  
#ifdef ENABLE_USB_DEVICE
  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);
#endif

  xTaskCreate(pio_usb_task, "usb_task", 2048, NULL, configMAX_PRIORITIES, &pio_usb_task_handle);

#ifdef WS2812_PIN
  uint offset = pio_add_program(pio0, &ws2812_program);  
  ws2812_program_init(pio0, 0, offset, WS2812_PIN, 800000, 0);

  TimerHandle_t led_timer_handle =
    xTimerCreate("LED timer", pdMS_TO_TICKS(200), pdTRUE, NULL, ws_led_timer);
  xTimerStart(led_timer_handle, 0);
#endif

#if !defined(WS2812_PIN) || defined(ENABLE_WIFI)
  // a regular non-ws2812 led or the wifi detection needs gpio25  
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif
  
#ifdef ENABLE_WIFI
  // gpio25 on the pico-w/pico2-w is cs for the wifi module but also disconnects
  // the vsys input. On a regular non-w pico it controls the on-board led and vsys
  // monitoring is permanently enabled. So if we can read a sane VSYS voltage with
  // gpio25 being low, then we are on a regular non-w pico
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
  
  adc_init();
  adc_gpio_init(29);
  adc_select_input(3);
  sleep_ms(10);  // wait a few ms to get a stable value

  uint16_t result = adc_read();
  debugf("ADC3 value: 0x%03x/%d -> VSYS = %.2fV", result, result, result * 3.3 / 65536 * 33);	 
  bool is_pico_w = result < 0x100;
  
  if(!is_pico_w)
#endif
    {
#ifndef WS2812_PIN
      // the LED pin has already been setup to detect a PICO-W
      gpio_put(PICO_DEFAULT_LED_PIN, !PICO_DEFAULT_LED_PIN_INVERTED);
      
      TimerHandle_t led_timer_handle =
	xTimerCreate("LED timer", pdMS_TO_TICKS(200), pdTRUE, NULL, led_timer);
      xTimerStart(led_timer_handle, 0);
#endif
      
      mcu_hw_asix_start();
    }
#ifdef ENABLE_WIFI
  else {
    xTaskCreate(wifi_task, (char *)"wifi_task", 2048, NULL, configMAX_PRIORITIES-10, NULL);  
  }
#endif

  debugf("Running on core %d", get_core_num());
  debugf("SDK heap total: %ld, free: %ld", getTotalHeap() ,getFreeHeap());
  debugf("FreeRTOS heap total: %u, free: %u", configTOTAL_HEAP_SIZE, xPortGetFreeHeapSize());

#ifdef ENABLE_JTAG
  mcu_hw_jtag_init();
#endif
}

extern TaskHandle_t com_task_handle;
extern TaskHandle_t menu_handle;

void mcu_hw_upload_core(char *name) {
  debugf("Request to upload core %s", name);  
#ifdef ENABLE_JTAG

  // if the core is to be loaded from sd card, then the hw may need
  // to hand card access over from FPGA to the MCU
#if MISTLE_BOARD == 4 || MISTLE_BOARD == 6   // DEV20k/DEV25K
  if(strncasecmp(name, "/sd", 3) == 0)
    sdio_take_over();
#endif
  
  // stop various tasks so they don't interfere with the
  // download. We don't have to care about any consequences
  // as we'll reboot afterwards, anyways
  vTaskDelete(com_task_handle);
  com_task_handle = NULL;

  // disable SPI interrupts to prevent main IRQ handler from running
  gpio_set_irq_enabled(SPI_IRQ_PIN, GPIO_IRQ_LEVEL_LOW, false);

  // stop the USB task. This requires the sector read routine to
  // call the tuh_task while waiting
  vTaskDelete(pio_usb_task_handle);
  pio_usb_task_handle = NULL;

  // TODO: Check why this locks up ...
  //  vTaskDelete(menu_handle);
  //  menu_handle = NULL;
  gowin_upload_core(name);

  // restart companion to cope with new core
  mcu_hw_reset();
#endif
}
