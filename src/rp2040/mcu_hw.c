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
#include "tusb.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/flash.h"

#include "../debug.h"
#include "../config.h"
#include "../spi.h"
#include "../sysctrl.h"
#include "../at_wifi.h"
#include "../inifile.h"

#include "../mcu_hw.h"

#ifndef MISTLE_BOARD
#error "No MiSTle board type specified!"
#endif

#if MISTLE_BOARD == 0
#warning "Building for Raspberry Pi Pico or Pico-W"
#define ENABLE_WIFI
#elif MISTLE_BOARD == 1
#warning "Building for Raspberry Pi Pico2 or Pico2-W"
#define ENABLE_WIFI
#elif MISTLE_BOARD == 2
#warning "Building for Waveshare RP2040-Zero"
#elif MISTLE_BOARD == 3
#warning "Building for MiSTeryShield20k-Lite"
#elif MISTLE_BOARD == 4
#warning "Building for MiSTeryDev20k"
#include "../jtag.h"
#include "./sdc_direct.h"
#define ENABLE_JTAG

// FPGA JTAG pins
#define PIN_JTAG_TDI  12   // pin 15, gpio 12
#define PIN_JTAG_TMS  13   // pin 16, gpio 13
#define PIN_JTAG_TDO  14   // pin 17, gpio 14
#define PIN_JTAG_TCK  15   // pin 18, gpio 15

// special FPGA configuration pins
#define PIN_nCFG      21   // pin 32, PIO21
#define PIN_MODE0     23   // pin 35, PIO23
#define PIN_MODE1     24   // pin 36, PIO24

#else
#error "Not a supported MiSTle board!"
#endif

#if MISTLE_BOARD == 2
// the waveshare mini does not expose the default spi0 pins, so we need
// to specify them
#define SPI_RX_PIN     4
#define SPI_SCK_PIN    6
#define SPI_TX_PIN     7
#define SPI_CSN_PIN    5
#define SPI_IRQ_PIN    8
#define SPI_BUS     spi0
#define WS2812_PIN    16
#else
// the regular pi pico uses spi0 by default
#define SPI_RX_PIN   PICO_DEFAULT_SPI_RX_PIN
#define SPI_SCK_PIN  PICO_DEFAULT_SPI_SCK_PIN
#define SPI_TX_PIN   PICO_DEFAULT_SPI_TX_PIN
#define SPI_CSN_PIN  PICO_DEFAULT_SPI_CSN_PIN
#define SPI_IRQ_PIN  22
#define SPI_BUS      spi_default

// the resular pi pico uses GPIO4, 5 and 6 for status
// indicator leds. These are e.g. present on the
// PiPico shield

#define LED_MOUSE_PIN    4
#define LED_KEYBOARD_PIN 5
#define LED_JOYSTICK_PIN 6
#endif

#ifdef ENABLE_WIFI
#warning "WiFi support enabled"
#else
#warning "WiFi support disabled"
#endif

#ifdef WS2812_PIN
#include "ws2812.pio.h"
#endif

/* ======================================================================== */
/* ===============                USB                        ============== */
/* ======================================================================== */

#include "tusb.h"
#include "../hid.h"
#include "../hidparser.h"

#include "tusb_option.h"
#ifndef TUSB_VERSION_NUMBER
#error "Cannot determine TinyUSB version!"
#endif

#if TUSB_VERSION_NUMBER < 1700
#error "Please update your TinyUSB installation!"
#endif

#include "tusb_config.h"
#if defined(WS2812_PIN) && CFG_TUH_RPI_PIO_USB == 1
#error "WS2812B and PIO USB cannot be used simultaneously!"
#endif

static struct {
  uint8_t dev_addr;
  uint8_t instance;
  hid_state_t state;
  hid_report_t rep;
} hid_device[MAX_HID_DEVICES];

static struct {
  uint8_t dev_addr;
  uint8_t instance;
  uint8_t js_index;
  uint8_t state;
  int16_t state_x;
  int16_t state_y;
  uint8_t state_btn_extra;
} xbox_state[MAX_XBOX_DEVICES];

extern void usb_jtag_poll(void);
static void pio_usb_task(__attribute__((unused)) void *parms) {
  // mark all hid and xbox entries as unused
  for(int i=0;i<MAX_HID_DEVICES;i++)
    hid_device[i].dev_addr = 0xff;

  for(int i=0;i<MAX_XBOX_DEVICES;i++)
    xbox_state[i].dev_addr = 0xff;
    
  while(1) {
    for(int i=0;i<100;i++) {
      tuh_task();
#if MISTLE_BOARD == 4
      tud_task();
      usb_jtag_poll();
#endif
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

uint8_t byteScaleAnalog(int16_t xbox_val)
{
  // Scale the xbox value from [-32768, 32767] to [1, 255]
  // Offset by 32768 to get in range [0, 65536], then divide by 256 to get in range [1, 255]
  uint8_t scale_val = (xbox_val + 32768) / 256;
  if (scale_val == 0) return 1;
  return scale_val;
}

// check for presence of usb devices and drive leds accordingly
static void usb_check_devices(void) {
#ifdef LED_MOUSE_PIN
  int mice = 0;
#endif
#ifdef LED_KEYBOARD_PIN
  int keyboards = 0;
#endif
#ifdef LED_JOYSTICK_PIN
  int joysticks = 0;
#endif
  
  for(int idx=0;idx<MAX_HID_DEVICES;idx++) {
    if(hid_device[idx].dev_addr != 0xff) {    
#ifdef LED_MOUSE_PIN
      if(hid_device[idx].rep.type == REPORT_TYPE_MOUSE)    mice++;
#endif
#ifdef LED_KEYBOARD_PIN
      if(hid_device[idx].rep.type == REPORT_TYPE_KEYBOARD) keyboards++;
#endif
#ifdef LED_JOYSTICK_PIN
      if(hid_device[idx].rep.type == REPORT_TYPE_JOYSTICK) joysticks++;
#endif
    }
  }
    
#ifdef LED_JOYSTICK_PIN
  for(int idx=0;idx<MAX_XBOX_DEVICES;idx++)
    if(xbox_state[idx].dev_addr != 0xff)
      joysticks++;
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
}

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len) {
  // Interface protocol (hid_interface_protocol_enum_t)
  const char* protocol_str[] = { "None", "Keyboard", "Mouse" };
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);
  usb_debugf("[%04x:%04x][%u] HID Interface%u, Protocol = %s",
	     vid, pid, dev_addr, instance, protocol_str[itf_protocol]);

  // search for a free hid entry
  int idx;
  for(idx=0;idx<MAX_HID_DEVICES && (hid_device[idx].dev_addr != 0xff);idx++);
  if(idx != MAX_HID_DEVICES) {
    usb_debugf("Using HID entry %d", idx);
    
    if(parse_report_descriptor(desc_report, desc_len, &hid_device[idx].rep, NULL)) {
      hid_device[idx].dev_addr = dev_addr;
      hid_device[idx].instance = instance;
      if(hid_device[idx].rep.type == REPORT_TYPE_JOYSTICK)
	      hid_device[idx].state.joystick.js_index = hid_allocate_joystick();
    } else
      usb_debugf("Ignoring device");
  } else
    usb_debugf("Error, no more free HID entries");
  
  // tuh_hid_report_received_cb() will be invoked when report is available
  if (!tuh_hid_receive_report(dev_addr, instance) ) 
    usb_debugf("Error: cannot request report");

  usb_check_devices();
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  usb_debugf("[%u] HID Interface%u is unmounted", dev_addr, instance);

  // find matching hid report
  for(int idx=0;idx<MAX_HID_DEVICES;idx++) {
    if(hid_device[idx].dev_addr == dev_addr && hid_device[idx].instance == instance) {
      usb_debugf("releasing %d", idx);
      hid_device[idx].dev_addr = 0xff;
      if(hid_device[idx].rep.type == REPORT_TYPE_JOYSTICK)
	hid_release_joystick(hid_device[idx].state.joystick.js_index);
    }
  }
  usb_check_devices();
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
  //  usb_debugf("[%u] HID Interface%u %p/%d", dev_addr, instance, report, len);

  // find matching hid report
  for(int idx=0;idx<MAX_HID_DEVICES;idx++)
    if(hid_device[idx].dev_addr == dev_addr && hid_device[idx].instance == instance)
      hid_parse(&hid_device[idx].rep, &hid_device[idx].state, report, len);
  
  // continue to request to receive report
  if ( report && !tuh_hid_receive_report(dev_addr, instance) )
    usb_debugf("Error: cannot request report");
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
  unsigned char retval;
  spi_write_read_blocking(SPI_BUS, &b, &retval, 1);
  return retval;
}

/* ======================================================================= */
/* ======                   XBOX controllers                     ========= */
/* ======================================================================= */

#include "xinput_host.h"

usbh_class_driver_t const* usbh_app_driver_get_cb(uint8_t* driver_count){
  *driver_count = 1;
  return &usbh_xinput_driver;
}

void tuh_xinput_report_received_cb(uint8_t dev_addr, uint8_t instance, xinputh_interface_t const* xid_itf, __attribute__((unused)) uint16_t len) {
  const xinput_gamepad_t *p = &xid_itf->pad;
  
  if (xid_itf->last_xfer_result == XFER_RESULT_SUCCESS) {
    if (xid_itf->connected && xid_itf->new_pad_data) {

      // find matching hid report
      for(int idx=0;idx<MAX_XBOX_DEVICES;idx++) {
	if(xbox_state[idx].dev_addr == dev_addr && xbox_state[idx].instance == instance) {
      
	  // build new state
	  unsigned char state =
	    ((p->wButtons & XINPUT_GAMEPAD_DPAD_UP   )?0x08:0x00) |
	    ((p->wButtons & XINPUT_GAMEPAD_DPAD_DOWN )?0x04:0x00) |
	    ((p->wButtons & XINPUT_GAMEPAD_DPAD_LEFT )?0x02:0x00) |
	    ((p->wButtons & XINPUT_GAMEPAD_DPAD_RIGHT)?0x01:0x00) |
	    ((p->wButtons & 0xf000) >> 8);

    // build extra button new state
    unsigned char state_btn_extra =
	    ((p->wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER  )?0x01:0x00) |
	    ((p->wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER )?0x02:0x00) |
	    ((p->wButtons & XINPUT_GAMEPAD_BACK           )?0x10:0x00) | // Rumblepad 2 / Dual Action compatibility
	    ((p->wButtons & XINPUT_GAMEPAD_START          )?0x20:0x00);

	  // build analog stick x,y state
      int16_t sThumbLX = p->sThumbLX;
      int16_t sThumbLY = p->sThumbLY;
      uint8_t ax = byteScaleAnalog(sThumbLX);
      uint8_t ay = ~byteScaleAnalog(sThumbLY);

    // map analog stick directions to digital
    if(ax > (uint8_t) 0xc0) state |= 0x01;
    if(ax < (uint8_t) 0x40) state |= 0x02;
    if(ay > (uint8_t) 0xc0) state |= 0x04;
    if(ay < (uint8_t) 0x40) state |= 0x08;

    // submit if state has changed
    if((state != xbox_state[idx].state) ||
      (state_btn_extra != xbox_state[idx].state_btn_extra) ||
      (ax != xbox_state[idx].state_x) ||
      (ay != xbox_state[idx].state_y)) {

      xbox_state[idx].state = state;
      xbox_state[idx].state_btn_extra = state_btn_extra;
      xbox_state[idx].state_x = sThumbLX;
      xbox_state[idx].state_y = sThumbLY;
      usb_debugf("XBOX Joy%d: B %02x EB %02x X %02x Y %02x", xbox_state[idx].js_index, state, state_btn_extra, byteScaleAnalog(ax), byteScaleAnalog(ay));

	    mcu_hw_spi_begin();
	    mcu_hw_spi_tx_u08(SPI_TARGET_HID);
	    mcu_hw_spi_tx_u08(SPI_HID_JOYSTICK);
	    mcu_hw_spi_tx_u08(xbox_state[idx].js_index);
	    mcu_hw_spi_tx_u08(state);
	    mcu_hw_spi_tx_u08(ax); // gamepad analog X
	    mcu_hw_spi_tx_u08(ay); // gamepad analog Y
	    mcu_hw_spi_tx_u08(state_btn_extra); // gamepad extra buttons
	    mcu_hw_spi_end();
	  }
	}
      }
    }
    tuh_xinput_receive_report(dev_addr, instance);
  }
}

void tuh_xinput_mount_cb(uint8_t dev_addr, uint8_t instance, const xinputh_interface_t *xinput_itf) {
  usb_debugf("xbox mounted %d/%d", dev_addr, instance);

  // search for a free xbox entry
  int idx;
  for(idx=0;idx<MAX_XBOX_DEVICES && (xbox_state[idx].dev_addr != 0xff);idx++);
  if(idx != MAX_XBOX_DEVICES) {
    usb_debugf("Using XBOX entry %d", idx);
    xbox_state[idx].dev_addr = dev_addr;
    xbox_state[idx].instance = instance;
    xbox_state[idx].state = 0;
    xbox_state[idx].state_btn_extra = 0;
    xbox_state[idx].state_x = 0;
    xbox_state[idx].state_y = 0;
    xbox_state[idx].js_index = hid_allocate_joystick();
  } else
    usb_debugf("Error, no more free XBOX entries");

  // If this is a Xbox 360 Wireless controller we need to wait for a connection packet
  // on the in pipe before setting LEDs etc. So just start getting data until a controller is connected.
  if (xinput_itf->type == XBOX360_WIRELESS && xinput_itf->connected == false) {
    tuh_xinput_receive_report(dev_addr, instance);
    return;
  }
  tuh_xinput_set_led(dev_addr, instance, 0, true);
  tuh_xinput_set_led(dev_addr, instance, 1, true);
  tuh_xinput_set_rumble(dev_addr, instance, 0, 0, true);
  tuh_xinput_receive_report(dev_addr, instance);

  usb_check_devices();
}

void tuh_xinput_umount_cb(uint8_t dev_addr, uint8_t instance) {
  usb_debugf("xbox unmounted %d/%d", dev_addr, instance);

  // find matching hid report
  for(int idx=0;idx<MAX_XBOX_DEVICES;idx++) {
    if(xbox_state[idx].dev_addr == dev_addr && xbox_state[idx].instance == instance) {
      usb_debugf("releasing %d/%d", idx, xbox_state[idx].js_index);
      xbox_state[idx].dev_addr = 0xff;
      hid_release_joystick(xbox_state[idx].js_index);
    }
  }
  usb_check_devices();
}

#include "hardware/watchdog.h"

void mcu_hw_reset(void) {
  debugf("HW reset");
  watchdog_reboot(0, 0, 10);
  while(1);
}

/* ========================================================================= */
/* ======                              WiFi                           ====== */
/* ========================================================================= */

#ifndef ENABLE_WIFI
void mcu_hw_wifi_scan(void) { }
void mcu_hw_wifi_connect(__attribute__((unused)) char *ssid, __attribute__((unused)) char *key) { }
void mcu_hw_tcp_connect(__attribute__((unused)) char *host, __attribute__((unused)) int port) { }
void mcu_hw_tcp_disconnect(void) { }
bool mcu_hw_tcp_data(__attribute__((unused)) unsigned char byte) { return false; }
#else  
static bool is_pico_w = false;
#include "pico/cyw43_arch.h"

static void led_timer_w(__attribute__((unused)) TimerHandle_t pxTimer) {
  static char state = 0;
  switch(inifile_option_get(INIFILE_OPTION_LED)) {
  case 0:    
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state & 1);
    break;
  case 1:    
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    break;
  case 2:    
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    break;
  }
    
  state = !state;
}

// the wifi connection state
#define WIFI_STATE_UNKNOWN      0
#define WIFI_STATE_DISCONNECTED 1
#define WIFI_STATE_CONNECTING   2
#define WIFI_STATE_CONNECTED    3

static int wifi_state = WIFI_STATE_UNKNOWN;

static void mcu_hw_wifi_init(void) {
#ifdef PICO_RP2350
  debugf("Detected Pico2-W");
#else
  debugf("Detected Pico-W");
#endif
  
  if(cyw43_arch_init_with_country(CYW43_COUNTRY_GERMANY)) {
    debugf("WiFi failed to initialised");
    return;
  }
  debugf("WiFi initialised");
  wifi_state = WIFI_STATE_DISCONNECTED;
  
  cyw43_arch_enable_sta_mode();
  debugf("STA mode enabled");

  cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);

  TimerHandle_t led_timer_handle =
    xTimerCreate("LED timer (W)", pdMS_TO_TICKS(200), pdTRUE,
		 NULL, led_timer_w);
  xTimerStart(led_timer_handle, 0);
}

static const char *auth_mode_str(int authmode) {
  static const struct { int mode; char *str; } mode_str[] = {
    { 0, "OPEN" },
    { 1, "WEP"  },
    { 2, "WPA2 PSK"  },
    { 3, "WPA WPA2 PSK"  },
    { 4, "WPA PSK"  },
    { 5, "ENTERPRISE"  },
    { 6, "WPA3 PSK"  },
    { 7, "WPA2 WPA3 PSK"  },
    { -1, "<unknown>" }
  };

  int i;
  for(i=0;mode_str[i].mode != -1;i++)
    if(mode_str[i].mode == authmode || mode_str[i].mode == -1)
      return mode_str[i].str;

  return mode_str[i].str;
}

static int scan_result(__attribute__((unused)) void *env, const cyw43_ev_scan_result_t *result) {
  if (result) {
    char str[74];
    
    debugf("ssid: %s rssi: %d chan: %d sec: %u",
	   result->ssid, result->rssi, result->channel,
	   result->auth_mode);

    snprintf(str, sizeof(str), "SSID %s, RSSI %d, CH %d, %s\r\n",
	     result->ssid, result->rssi, result->channel,
	     auth_mode_str(result->auth_mode));

    at_wifi_puts(str);
  }
  return 0;
}

void mcu_hw_wifi_scan(void) {
  debugf("WiFi: Performing scan");
    
  cyw43_wifi_scan_options_t scan_options = {0};
  int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result);
  if(err) {
    at_wifi_puts("Scan failed\r\n");
    return;
  }

  at_wifi_puts("Scanning...\r\n");  

  while(cyw43_wifi_scan_active(&cyw43_state))
    vTaskDelay(pdMS_TO_TICKS(10));
  }

void mcu_hw_wifi_connect(char *ssid, char *key) {

  debugf("WiFI: connect to %s/%s", ssid, key);
  
  at_wifi_puts("Connecting...");
  if(cyw43_arch_wifi_connect_timeout_ms(ssid, key, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
    at_wifi_puts("\r\nConnection failed!\r\n");
  } else {
    at_wifi_puts("\r\nConnected\r\n");
  }  
}

#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

static struct tcp_pcb *tcp_pcb = NULL;

static err_t mcu_tcp_connected( __attribute__((unused)) void *arg, __attribute__((unused)) struct tcp_pcb *tpcb, err_t err) {
  if (err != ERR_OK) {
    debugf("connect failed %d\n", err);
    return ERR_OK;
  }
  
  debugf("Connected");
  at_wifi_puts("Connected\r\n");
  wifi_state = WIFI_STATE_CONNECTED;  // connected
  return ERR_OK;
}

static void mcu_tcp_err(__attribute__((unused)) void *arg, err_t err) {
  if( err == ERR_RST) {
    debugf("tcp connection reset");
    at_wifi_puts("\r\nNO CARRIER\r\n");
    wifi_state = WIFI_STATE_DISCONNECTED;      
  } else if (err == ERR_ABRT) {
    debugf("err abort");
    at_wifi_puts("Connection failed\r\n");
    wifi_state = WIFI_STATE_DISCONNECTED;    
  } else {
    debugf("tcp_err %d", err);
  }
}

err_t mcu_tcp_recv(__attribute__((unused)) void *arg, struct tcp_pcb *tpcb, struct pbuf *p, __attribute__((unused)) err_t err) {
  if (!p) {
    debugf("No data, disconnected?");
    at_wifi_puts("\r\nNO CARRIER\r\n");
    wifi_state = WIFI_STATE_DISCONNECTED;    
    return ERR_OK;
  }

  // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
  // can use this method to cause an assertion in debug mode, if this method is called when
  // cyw43_arch_lwip_begin IS needed
  cyw43_arch_lwip_check();
  if (p->tot_len > 0) {
    // debugf("recv %d err %d", p->tot_len, err);

    for (struct pbuf *q = p; q != NULL; q = q->next)
      at_wifi_puts_n(q->payload, q->len);
    
    tcp_recved(tpcb, p->tot_len);
  }
  pbuf_free(p);
  
  return ERR_OK;
}

static void mcu_tcp_connect(const ip_addr_t *ipaddr, int port) {
  debugf("Connecting to IP %s %d", ipaddr_ntoa(ipaddr), port);
  
  // the address was resolved and we can connect
  tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(ipaddr));
  if (!tcp_pcb) {    
    debugf("Unable to create pcb");
    at_wifi_puts("Connection failed!\r\n");
  }

  tcp_recv(tcp_pcb, mcu_tcp_recv);
  tcp_err(tcp_pcb, mcu_tcp_err);
  
  cyw43_arch_lwip_begin();
  err_t err = tcp_connect(tcp_pcb, ipaddr, port, mcu_tcp_connected);
  cyw43_arch_lwip_end();

  if(err) {
    debugf("tcp_connect() failed"); 
    at_wifi_puts("Connection failed!\r\n");
  } else
    wifi_state = WIFI_STATE_CONNECTING;    
}

void mcu_hw_tcp_disconnect(void) {
  if(wifi_state == WIFI_STATE_CONNECTED)
    tcp_close(tcp_pcb);
}

// Call back with a DNS result
static void dns_found(__attribute__((unused)) const char *hostname, const ip_addr_t *ipaddr, void *arg) {
  if (ipaddr) {
    // state->ntp_server_address = *ipaddr;
    at_wifi_puts("Using address ");
    at_wifi_puts(ipaddr_ntoa(ipaddr));
    at_wifi_puts("\r\n");

    mcu_tcp_connect(ipaddr, *(int*)arg);
  } else
    at_wifi_puts("Cannot resolve host\r\n");
}

void mcu_hw_tcp_connect(char *host, int port) {
  static int lport;
  static ip_addr_t address;

  lport = port;
  debugf("connecting to %s %d", host, lport);
  
  cyw43_arch_lwip_begin();
  int err = dns_gethostbyname(host, &address, dns_found, &lport);
  cyw43_arch_lwip_end();

  if(err != ERR_OK && err != ERR_INPROGRESS) {
    debugf("DNS error");
    at_wifi_puts("Cannot resolve host\r\n");
    return;
  }

  if(err == ERR_OK)
    mcu_tcp_connect(&address, port);

  else if(err == ERR_INPROGRESS) 
    debugf("DNS in progress");
}

bool mcu_hw_tcp_data(unsigned char byte) {
  if(wifi_state == WIFI_STATE_CONNECTED) {
    cyw43_arch_lwip_begin();
    err_t err = tcp_write(tcp_pcb, &byte, 1, TCP_WRITE_FLAG_COPY);
    cyw43_arch_lwip_end();
    if (err != ERR_OK) debugf("Failed to write data %d", err);

    return true;
  }
    
  return false;  // data has not been processed (we are not connected)
}
#endif

#ifndef WS2812_PIN
// the LED PIN is not defined if we build for a pico-w. But we
// detect the wireless chip and know when running on the regular pico
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

static void led_timer(__attribute__((unused)) TimerHandle_t pxTimer) {
  switch(inifile_option_get(INIFILE_OPTION_LED)) {
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

  mcu_hw_wifi_init();

  // only used for init
  vTaskDelete(NULL);
}
#endif

#ifdef WS2812_PIN
#define WS2812_COLOR 0x40000000  // GRBX: 25% green

static void ws_led_timer(__attribute__((unused)) TimerHandle_t pxTimer) {
  static char state = 0;
  switch(inifile_option_get(INIFILE_OPTION_LED)) {
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

// Invoked when a device with MassStorage interface is mounted
void tuh_msc_mount_cb(uint8_t dev_addr) {
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);
  usb_debugf("[%04x:%04x][%u] MSC mounted", vid, pid, dev_addr);
}

// Invoked when a device with MassStorage interface is unmounted
void tuh_msc_umount_cb(__attribute__((unused)) uint8_t dev_addr) {
  usb_debugf("MSC unmounted");
}

extern char __StackLimit, __bss_end__;   
uint32_t getTotalHeap(void) {
   return &__StackLimit  - &__bss_end__;
}

uint32_t getFreeHeap(void) {
   struct mallinfo m = mallinfo();
   return getTotalHeap() - m.uordblks;
}

/* ========================================================================= */
/* ======                              JTAG                           ====== */
/* ========================================================================= */

#ifdef ENABLE_JTAG

static bool jtag_is_active = false;

//#define DEBUG_JTAG    // debug raw JTAG IO
//#define DEBUG_TAP     // set to follow the JTAG state machine

#ifdef DEBUG_TAP
#define JTAG_STATE_TEST_LOGIC_RESET  0
#define JTAG_STATE_RUN_TEST_IDLE     1
#define JTAG_STATE_SELECT_DR_SCAN    2
#define JTAG_STATE_CAPTURE_DR        3
#define JTAG_STATE_SHIFT_DR          4
#define JTAG_STATE_EXIT1_DR          5
#define JTAG_STATE_PAUSE_DR          6
#define JTAG_STATE_EXIT2_DR          7
#define JTAG_STATE_UPDATE_DR         8
#define JTAG_STATE_SELECT_IR_SCAN    9
#define JTAG_STATE_CAPTURE_IR       10
#define JTAG_STATE_SHIFT_IR         11
#define JTAG_STATE_EXIT1_IR         12
#define JTAG_STATE_PAUSE_IR         13
#define JTAG_STATE_EXIT2_IR         14
#define JTAG_STATE_UPDATE_IR        15

// state flow table, telling which state follows onto which state depending on TMS
const struct state_flow_S {
  uint8_t tms[2];
  const char *name;  
} state_flow[] = {
  //  next state when TMS == 0    next state when TMS == 1      state name
  { {JTAG_STATE_RUN_TEST_IDLE, JTAG_STATE_TEST_LOGIC_RESET }, "Test-Logic-Reset" }, // 0
  { {JTAG_STATE_RUN_TEST_IDLE, JTAG_STATE_SELECT_DR_SCAN   }, "Run-Test/Idle"    }, // 1
  
  { {JTAG_STATE_CAPTURE_DR,    JTAG_STATE_SELECT_IR_SCAN   }, "Select-DR-Scan"   }, // 2
  { {JTAG_STATE_SHIFT_DR,      JTAG_STATE_EXIT1_DR         }, "Capture-DR"       }, // 3
  { {JTAG_STATE_SHIFT_DR,      JTAG_STATE_EXIT1_DR         }, "Shift-DR"         }, // 4
  { {JTAG_STATE_PAUSE_DR,      JTAG_STATE_UPDATE_DR        }, "Exit1-DR"         }, // 5
  { {JTAG_STATE_PAUSE_DR,      JTAG_STATE_EXIT2_DR         }, "Pause-DR"         }, // 6
  { {JTAG_STATE_SHIFT_DR,      JTAG_STATE_UPDATE_DR        }, "Exit2-DR"         }, // 7
  { {JTAG_STATE_RUN_TEST_IDLE, JTAG_STATE_SELECT_DR_SCAN   }, "Update-DR"        }, // 8
  
  { {JTAG_STATE_CAPTURE_IR,    JTAG_STATE_TEST_LOGIC_RESET }, "Select-IR-Scan"   }, // 9
  { {JTAG_STATE_SHIFT_IR,      JTAG_STATE_EXIT1_IR         }, "Capture-IR"       }, // 10
  { {JTAG_STATE_SHIFT_IR,      JTAG_STATE_EXIT1_IR         }, "Shift-IR"         }, // 11
  { {JTAG_STATE_PAUSE_IR,      JTAG_STATE_UPDATE_IR        }, "Exit1-IR"         }, // 12
  { {JTAG_STATE_PAUSE_IR,      JTAG_STATE_EXIT2_IR         }, "Pause-IR"         }, // 13
  { {JTAG_STATE_SHIFT_IR,      JTAG_STATE_UPDATE_IR        }, "Exit2-IR"         }, // 14
  { {JTAG_STATE_RUN_TEST_IDLE, JTAG_STATE_SELECT_DR_SCAN   }, "Update-IR"        }  // 15
};

static uint8_t tap_state = JTAG_STATE_TEST_LOGIC_RESET;
static uint8_t tap_ir_bits;
static uint32_t tap_ir;
static uint32_t tap_dr_bits;
static uint8_t tap_dr[4], tap_dr_byte;  // we capture only the first 32 bits
static uint32_t tap_dr_sum;

const struct gowin_ir_S {
  int ir;
  const char *name;  
} gowin_ir[] = {
  { 0x00, "Bypass" },
  { 0x02, "Noop" },
  { 0x03, "Read SRAM" },
  { 0x05, "Erase SRAM" },
  { 0x09, "XFER Done" },
  { 0x11, "IDCode" },
  { 0x12, "Address Init" },
  { 0x13, "UserCode" },
  { 0x15, "ConfigEnable" },
  { 0x16, "Transfer SPI" },
  { 0x17, "Transfer Bitstream" },
  { 0x21, "Program Key" },  
  { 0x23, "Security" },  
  { 0x24, "Program EFuse" },  
  { 0x25, "Read Key" },
  { 0x29, "Program Key" },  
  { 0x3a, "ConfigDisable" },
  { 0x3c, "Reconfig" },
  { 0x3d, "BSCAN 2 SPI" },
  { 0x41, "Status" },
  { 0x42, "GAO#1" },
  { 0x43, "GAO#2" },
  { 0x71, "EFlash Program" },  
  { 0x75, "EFlash Erase" },  
  { 0x7a, "Switch to MCU JTAG" },  
  { 0xff, "Bypass" },
  {   -1, "<unknown command>" }  
};

static void jtag_tap_advance_state(uint8_t tms, uint8_t tdi) {
  // capture instruction register write
  if(tap_state == JTAG_STATE_SHIFT_IR) {
    if(tdi) tap_ir |= (1<<tap_ir_bits);
    tap_ir_bits++;
  }
  
  // capture data register write
  if(tap_state == JTAG_STATE_SHIFT_DR) {
    if(tdi && ((tap_dr_bits/8)<sizeof(tap_dr)))
      tap_dr[tap_dr_bits/8] |= (1<<(tap_dr_bits&7));

    // update sum, whenever the last bit of a byte
    // has been written
    if(tdi) tap_dr_byte |= (1<<(tap_dr_bits&7));
    if((tap_dr_bits&7) == 7) {
      tap_dr_sum += tap_dr_byte;
      tap_dr_byte = 0;
    }
    
    tap_dr_bits++;
  }

  // check if we'd do into TEST_LOGIC_RESET state
  if(tap_state != JTAG_STATE_TEST_LOGIC_RESET &&
     state_flow[tap_state].tms[tms] == JTAG_STATE_TEST_LOGIC_RESET) {
    jtag_highlight_debugf("TEST LOGIC RESET");
  }
    
  tap_state = state_flow[tap_state].tms[tms];

  // clear IR if we just entered the capture IR state
  if(tap_state == JTAG_STATE_CAPTURE_IR) {
    tap_ir_bits = 0;  
    tap_ir = 0;
  }

  if(tap_state == JTAG_STATE_CAPTURE_DR) {
    tap_dr_bits = 0;
    for(unsigned int i=0;i<sizeof(tap_dr);i++) tap_dr[i] = 0;
    tap_dr_sum = 0;
    tap_dr_byte = 0;
  }
    
  // display IR if we just entered the update IR state
  if(tap_state == JTAG_STATE_UPDATE_IR) {
    // since we know which FPGA we are dealing with, we can disect
    // this even further
    int i;
    for(i=0;gowin_ir[i].ir != -1 && gowin_ir[i].ir != (int)tap_ir;i++);
    jtag_highlight_debugf("IR %02lx/%d: GOWIN %s", tap_ir, tap_ir_bits, gowin_ir[i].name);
  }

  if(tap_state == JTAG_STATE_UPDATE_DR) {
    if(!(tap_dr_bits&7))  jtag_highlight_debugf("DR %lu bytes, sum %ld", tap_dr_bits/8, tap_dr_sum);
    else jtag_highlight_debugf("DR %lu bytes + %lu bits, sum %ld", tap_dr_bits/8, tap_dr_bits&7, tap_dr_sum);
    hexdump(tap_dr, sizeof(tap_dr));
  }
}

#ifdef DEBUG_JTAG
static const char *jtag_tap_state_name(void) {
  return state_flow[tap_state].name;
}
#endif
#endif

bool mcu_hw_jtag_is_active(void) {
  return jtag_is_active;
}

void mcu_hw_jtag_set_pins(uint8_t dir, uint8_t data) {
  // bit order is TMS/TDO/TDI/TCK, for JTAG this will be IOII  
  uint8_t pins[] = { PIN_JTAG_TCK, PIN_JTAG_TDI, PIN_JTAG_TDO, PIN_JTAG_TMS };

#ifdef DEBUG_JTAG
  jtag_debugf("PIN DIR 0x%02x, DATA 0x%02x", dir, data);
#endif

  // only the lowest four bits are actually implemented
  // TODO: consider another bit for RECONF
  for(int i=0;i<4;i++) {
    if(dir & (1<<i))  gpio_put(pins[i], (data & (1<<i))?1:0);
    gpio_set_dir(pins[i], (dir & (1<<i))?GPIO_OUT:GPIO_IN);
  }  

  // check if the pin direction pattern matches JTAG
  if((dir & 0x0f) == 0x0b) {
    jtag_is_active = true;
    
#ifdef DEBUG_JTAG
    jtag_debugf("DIR pattern matches JTAG");
#endif
    // JTAG may actually be disabled on the FPGA. Try to detect the
    // FPGA and if none is detected, try to reconfigure it

    // send a bunch of 1's to return into Test-Logic-Reset state.
    mcu_hw_jtag_tms(1, 0b11111, 5);
  
    // send TMS 0/1/0/0 to get into SHIFT-DR state
    mcu_hw_jtag_tms(1, 0b0010, 4);

    // shift data into DR
    uint32_t idcode;
    mcu_hw_jtag_data(NULL, (uint8_t*)&idcode, 32);
    
    // finally return into Test-Logic-Reset state.
    mcu_hw_jtag_tms(1, 0b11111, 5);
  
    jtag_debugf("IDCODE = %08lx", idcode);

    // anything bit all 1's or all 0's indicates that JTAG seems to
    // be working
    if((idcode == 0xffffffff) || (idcode == 0x00000000)) {
      jtag_highlight_debugf("JTAG doesn't seem to work. Forcing non-flash reconfig");
      mcu_hw_fpga_reconfig(false);
    }
  } else
    jtag_is_active = false;
}

// send up to 8 TMS bits with a given fixed TDI state
uint8_t mcu_hw_jtag_tms(uint8_t tdi, uint8_t data, int len) {
  int dlen = len & 7;
  uint8_t mask = 1;
  uint8_t rx = 0;

  gpio_put(PIN_JTAG_TDI, tdi);  

  while(len--) {
    gpio_put(PIN_JTAG_TMS, (data & mask)?1:0);  
    gpio_put(PIN_JTAG_TCK, 1);

#ifdef DEBUG_TAP
    jtag_tap_advance_state((data & mask)?1:0, tdi);
#ifdef DEBUG_JTAG
    jtag_debugf("TMS %d TDI %d TDO %d -> %s", (data & mask)?1:0, tdi, gpio_get(PIN_JTAG_TDO),
		jtag_tap_state_name());
#endif
#else
#ifdef DEBUG_JTAG
    jtag_debugf("TMS %d TDI %d TDO %d", (data & mask)?1:0, tdi, gpio_get(PIN_JTAG_TDO));
#endif
#endif
    
    if(gpio_get(PIN_JTAG_TDO)) rx |= mask;
    
    gpio_put(PIN_JTAG_TCK, 0);

    mask <<= 1;
  }

  // adjust for the fact that we aren't really shifting
  rx <<= 8-dlen;
  return rx;
}

void mcu_hw_jtag_data(uint8_t *txd, uint8_t *rxd, int len) {
  uint8_t mask = 1;
  int dlen = len & 7;

#ifdef DEBUG_TAP
  // data transmissions are only expected in states SHIFT_DR and SHIFT_IR
  if((tap_state != JTAG_STATE_SHIFT_IR) && (tap_state != JTAG_STATE_SHIFT_DR))
    jtag_debugf("Warning: data i/o in non-shifting state %d!", tap_state);
#endif

  // data transmission always keeps TMS at zero
  gpio_put(PIN_JTAG_TMS, 0);

  // special version for txd-only with a multiple of 8 bits
  // as that's the most common case
  if(txd && !rxd && !(len&7)) {
#ifdef DEBUG_TAP
    for(int i=0;i<len;i++)
      jtag_tap_advance_state(0, txd[i>>3] & (1<<(i&7)));
#endif

    len >>= 3;
    while(len--) {
      // set data bit and clock tck at once
      gpio_put(PIN_JTAG_TDI, *txd & 0x01); gpio_put(PIN_JTAG_TCK, 1); gpio_put(PIN_JTAG_TCK, 0);
      gpio_put(PIN_JTAG_TDI, *txd & 0x02); gpio_put(PIN_JTAG_TCK, 1); gpio_put(PIN_JTAG_TCK, 0);
      gpio_put(PIN_JTAG_TDI, *txd & 0x04); gpio_put(PIN_JTAG_TCK, 1); gpio_put(PIN_JTAG_TCK, 0);
      gpio_put(PIN_JTAG_TDI, *txd & 0x08); gpio_put(PIN_JTAG_TCK, 1); gpio_put(PIN_JTAG_TCK, 0);
      gpio_put(PIN_JTAG_TDI, *txd & 0x10); gpio_put(PIN_JTAG_TCK, 1); gpio_put(PIN_JTAG_TCK, 0);
      gpio_put(PIN_JTAG_TDI, *txd & 0x20); gpio_put(PIN_JTAG_TCK, 1); gpio_put(PIN_JTAG_TCK, 0);
      gpio_put(PIN_JTAG_TDI, *txd & 0x40); gpio_put(PIN_JTAG_TCK, 1); gpio_put(PIN_JTAG_TCK, 0);
      gpio_put(PIN_JTAG_TDI, *txd & 0x80); gpio_put(PIN_JTAG_TCK, 1); gpio_put(PIN_JTAG_TCK, 0);
      txd++;
    }
  } else {  
    while(len) {
      // send 1 of nothing was given
      int tx_bit = txd?((*txd & mask)?1:0):1;
      
      // set data bit and clock tck at once
      gpio_put(PIN_JTAG_TDI, tx_bit);  
      gpio_put(PIN_JTAG_TCK, 1);
      
#ifdef DEBUG_TAP
      jtag_tap_advance_state(0, tx_bit);
#endif
      
#ifdef DEBUG_JTAG
      jtag_debugf("TMS 0 TDI %d TDO %d", tx_bit, gpio_get(PIN_JTAG_TDO));
#endif
      
      if(rxd) {
	// shift in from lsb
	if(gpio_get(PIN_JTAG_TDO)) *rxd |=  mask;
	else                       *rxd &= ~mask;
      }
      
      gpio_put(PIN_JTAG_TCK, 0);
      
      // advance bit mask
      mask <<= 1;
      if(!mask) {
	mask = 0x01;
	if(rxd) rxd++;      
      if(txd) txd++;
      }
      len--;
    }    
  }
    
  // We aren't really shifting, but instead setting bits
  // via mask. This makes a difference for the last byte
  // when not reading all 8 bits
  if(dlen) {
    // jtag_highlight_debugf("last byte %02x, rshift = %d", *rxd, dlen);
    *rxd <<= 8-dlen;
  }
}

static void mcu_hw_jtag_init(void) {
  // -------- init FPGA control pins ---------

  // FPGA mode pins. Init as inputs, so the buttons work
  gpio_init(PIN_MODE0); // gpio_put(PIN_MODE0, 0);
  gpio_set_dir(PIN_MODE0, GPIO_IN);
  gpio_init(PIN_MODE1); // gpio_put(PIN_MODE1, 0);
  gpio_set_dir(PIN_MODE1, GPIO_IN);

  // FPGA reconfig pin, active low
  gpio_init(PIN_nCFG); gpio_put(PIN_nCFG, 1);
  gpio_set_dir(PIN_nCFG, GPIO_OUT);
  
  // -------- init FPGA JTAG pins ---------
  gpio_init(PIN_JTAG_TCK); gpio_init(PIN_JTAG_TDI);
  gpio_init(PIN_JTAG_TMS); gpio_init(PIN_JTAG_TDO);
  // init to all input, the JTAG engine will reconfigure
  // them if required
  mcu_hw_jtag_set_pins(0x00, 0x00);
}

void mcu_hw_fpga_reconfig(bool run) {
  // alternally the FPGA may be put into a mode != 00 to
  // suppress MSPI loading
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
  
#if MISTLE_BOARD == 4
  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);
#endif

  xTaskCreate(pio_usb_task, "usb_task", 2048, NULL, configMAX_PRIORITIES, NULL);

#ifdef WS2812_PIN
  uint offset = pio_add_program(pio0, &ws2812_program);  
  ws2812_program_init(pio0, 0, offset, WS2812_PIN, 800000, 0);

  TimerHandle_t led_timer_handle =
    xTimerCreate("LED timer", pdMS_TO_TICKS(200), pdTRUE, NULL, ws_led_timer);
  xTimerStart(led_timer_handle, 0);
#endif
  
#ifdef ENABLE_WIFI
  adc_init();
  adc_gpio_init(29);
  adc_select_input(3);

  uint16_t result = adc_read();
  debugf("ADC3 value: 0x%03x", result);
  is_pico_w = result < 0x100;

  if(!is_pico_w)
#endif
    {
#ifndef WS2812_PIN
      // prepare for using a regular led
      gpio_init(PICO_DEFAULT_LED_PIN);
      gpio_set_dir(PICO_DEFAULT_LED_PIN, 1);
      gpio_put(PICO_DEFAULT_LED_PIN, !PICO_DEFAULT_LED_PIN_INVERTED);
      
      TimerHandle_t led_timer_handle =
	xTimerCreate("LED timer", pdMS_TO_TICKS(200), pdTRUE, NULL, led_timer);
      xTimerStart(led_timer_handle, 0);
#endif
    }
#ifdef ENABLE_WIFI
  else
    xTaskCreate(wifi_task, (char *)"wifi_task", 2048, NULL, configMAX_PRIORITIES-10, NULL);  
#endif

  debugf("SDK heap total: %ld, free: %ld", getTotalHeap() ,getFreeHeap());
  debugf("FreeRTOS heap total: %u, free: %u", configTOTAL_HEAP_SIZE, xPortGetFreeHeapSize());

#ifdef ENABLE_JTAG
  mcu_hw_jtag_init();
#endif
}
