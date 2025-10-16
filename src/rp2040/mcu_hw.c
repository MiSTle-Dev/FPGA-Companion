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

#include "../debug.h"
#include "../config.h"
#include "../spi.h"
#include "../sysctrl.h"
#include "../at_wifi.h"
#include "../inifile.h"

#include "../mcu_hw.h"

#ifdef WAVESHARE_RP2040_ZERO
#warning "Building for Waveshare RP2040-Zero mini board"

// the waveshare mini does not expose the default spi0 pins, so we need
// to specify them
#define SPI_RX_PIN    4
#define SPI_SCK_PIN   6
#define SPI_TX_PIN    7
#define SPI_CSN_PIN   5
#define SPI_IRQ_PIN   8
#define SPI_BUS  spi0
#define WS2812_PIN    16
#else
#ifdef PICO2
#warning "Building for Pi Pico2 and Pico2(W)"
#define ENABLE_WIFI
#else
#ifdef SH20KLITE
#warning "Building for MiSTeryShield20K-Lite"
#else
#warning "Building for Pi Pico and Pico(W)"
#define ENABLE_WIFI
#endif
#endif

// the regular pi pico uses spi0 by default
#define SPI_RX_PIN   PICO_DEFAULT_SPI_RX_PIN
#define SPI_SCK_PIN  PICO_DEFAULT_SPI_SCK_PIN
#define SPI_TX_PIN   PICO_DEFAULT_SPI_TX_PIN
#define SPI_CSN_PIN  PICO_DEFAULT_SPI_CSN_PIN
#define SPI_IRQ_PIN  22
#define SPI_BUS  spi_default

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
  
static void pio_usb_task(__attribute__((unused)) void *parms) {
  // mark all hid and xbox entries as unused
  for(int i=0;i<MAX_HID_DEVICES;i++)
    hid_device[i].dev_addr = 0xff;

  for(int i=0;i<MAX_XBOX_DEVICES;i++)
    xbox_state[i].dev_addr = 0xff;
    
  while(1) {
    tuh_task();
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
  usb_debugf("[%u] HID Interface%u %p/%d", dev_addr, instance, report, len);

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
  // Disable interrupt. It will be re-enabled by the com task
  gpio_set_irq_enabled(SPI_IRQ_PIN, GPIO_IRQ_LEVEL_LOW, false);

  if(com_task_handle) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR( com_task_handle, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
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

  // The interruput input isn't strictly part of the SPi
  // The interrupt is active low on GP22
  debugf("  IRQn = %d", SPI_IRQ_PIN);
  // set handler but not enable yet as the main task may not be ready
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
unsigned int petsc2;

static void mcu_hw_wifi_init(void) {
#ifdef PICO2
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
  if (petsc2 == 1) {
    int len = strlen(ssid);
    for (int i = 0; i < len; i++) {
        ssid[i] = pet2asc(ssid[i]);
    }
    len = strlen(key);
    for (int i = 0; i < len; i++) {
        key[i] = pet2asc(key[i]);
    }
  }

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

  if (petsc2 == 1) {
    int len = strlen(host);
    for (int i = 0; i < len; i++) {
        host[i] = pet2asc(host[i]);
    }
  }

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

  petsc2 = 0;  // default ASC-2 mode
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

extern char __StackLimit, __bss_end__;   
uint32_t getTotalHeap(void) {
   return &__StackLimit  - &__bss_end__;
}

uint32_t getFreeHeap(void) {
   struct mallinfo m = mallinfo();
   return getTotalHeap() - m.uordblks;
}

void mcu_hw_init(void) {
  // default 125MHz is not appropriate for PIO USB. Sysclock should be multiple of 12MHz.
  // some devices won't enumerate propery below ~16*12Mhz
  set_sys_clock_khz(16*12000, true);
  
  stdio_init_all();    // ... so stdio can adjust its bit rate
#ifdef WAVESHARE_RP2040_ZERO
  // the waveshare mini does not support SWD and we thus use a simpler (slower) UART
  uart_set_baudrate(uart0, 460800);  
#else
  uart_set_baudrate(uart0, 921600);
#endif
  
  printf("\r\n\r\n" LOGO "        FPGA Companion for RP2040/RP2350\r\n\r\n");
#if CFG_TUH_RPI_PIO_USB == 0
  printf("Using native USB\r\n");
#else
  printf("USB D+/D- on GP%d and GP%d\r\n", PIO_USB_DP_PIN_DEFAULT, PIO_USB_DP_PIN_DEFAULT+1);
#endif
  
  mcu_hw_spi_init();

  // initialize the LED gpios
#ifdef LED_MOUSE_PIN
  debugf("LED MOUSE =    GP%d", LED_MOUSE_PIN);
  gpio_init(LED_MOUSE_PIN);
  gpio_set_dir(LED_MOUSE_PIN, GPIO_OUT);
  gpio_put(LED_MOUSE_PIN, 0);
#endif
#ifdef LED_KEYBOARD_PIN
  debugf("LED KEYBOARD = GP%d", LED_KEYBOARD_PIN);
  gpio_init(LED_KEYBOARD_PIN);
  gpio_set_dir(LED_KEYBOARD_PIN, GPIO_OUT);
  gpio_put(LED_KEYBOARD_PIN, 0);
#endif
#ifdef LED_JOYSTICK_PIN
  debugf("LED JOYSTICK = GP%d", LED_JOYSTICK_PIN);
  gpio_init(LED_JOYSTICK_PIN);
  gpio_set_dir(LED_JOYSTICK_PIN, GPIO_OUT);
  gpio_put(LED_JOYSTICK_PIN, 0);
#endif
  
  tuh_hid_set_default_protocol(HID_PROTOCOL_REPORT);
  tuh_init(BOARD_TUH_RHPORT);
  
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
  is_pico_w = adc_read() < 0x100;

  if(!is_pico_w)
#endif
    {
#ifndef WAVESHARE_RP2040_ZERO
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

  printf("Total heap: %ld\n", getTotalHeap());
  printf("Free heap: %ld\n", getFreeHeap());
}
