/*
  mcu_hw.c - bl616 hardware driver
*/

#include <FreeRTOS.h>
#include "mm.h"
#include "semphr.h"
#include "async_event.h"

#include "usbh_core.h"
#include "usbh_hid.h"
#include "usbh_xbox.h"
#include "usbh_msc.h"
#include "usbd_core.h"
#include "bflb_name.h"
#include "../usb_controller_maps.h"
#include "ff.h"
#include "fatfs_diskio_register.h"

#include "../spi.h"
#include "../hid.h"
#include "../sdc.h"
#include "../sysctrl.h"
#include "../debug.h"
#include "../mcu_hw.h"
#include "../inifile.h"
#include "../menu.h"
#include "../gowin.h"

#include <bl616_hbn.h>
#include "bl616_glb.h"
#include "bflb_mtimer.h"
#include "bflb_spi.h"
#include "bflb_dma.h"
#include "bflb_gpio.h"
#include "bflb_wdg.h"
#ifdef CONFIG_CONSOLE_WO
#include "bflb_wo.h"
#else
#include "shell.h"
#include "bflb_uart.h"
#endif
#include "bflb_clock.h"
#include "bflb_flash.h"
#include "bflb_xip_sflash.h"
#include "bflb_sf_ctrl.h"
#include "board_flash_psram.h"

#include <lwip/tcpip.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/pbuf.h>
#include <lwip/tcp.h>
#include "fhost_api.h"
#include "wifi_mgmr_ext.h"
#include "wifi_mgmr.h"
#include "rfparam_adapter.h"

#include "bflb_rtc.h" 
#include "bflb_acomp.h"
#include "bflb_efuse.h"
#include "board.h"
#include "bl616_tzc_sec.h"
#include "task.h"
#include "timers.h"
#include "bflb_irq.h"
#include "../at_wifi.h"

//#define DEBUG_JTAG
//#define DEBUG_TAP

#ifdef TANG_CONSOLE60K
#warning "Building for TANG_CONSOLE60K internal BL616"
#include "../jtag.h"
#include "./sdc_direct.h"
#define PIN_JTAGSEL  GPIO_PIN_28
#define PIN_UART_TX  GPIO_PIN_30
#define PIN_UART_RX  GPIO_PIN_22
#elif TANG_NANO20K
#warning "Building for TANG_NANO20K internal BL616"
#include "../jtag.h"
#include "./sdc_direct.h"
#define PIN_UART_TX  GPIO_PIN_11
#define PIN_UART_RX  GPIO_PIN_22
#elif M0S_DOCK
#warning "Building for M0S DOCK BL616"
#define PIN_UART_TX  GPIO_PIN_21
#define PIN_UART_RX  GPIO_PIN_22
#elif TANG_MEGA138KPRO
#warning "Building for TANG_MEGA138KPRO internal BL616"
#include "../jtag.h"
#include "./sdc_direct.h"
#define PIN_JTAGSEL  GPIO_PIN_10
#define PIN_UART_TX  GPIO_PIN_28
#define PIN_UART_RX  GPIO_PIN_22
#elif TANG_MEGA60K
#warning "Building for TANG_MEGA60K internal BL616"
#include "../jtag.h"
#include "./sdc_direct.h"
#define PIN_JTAGSEL  GPIO_PIN_28
#define PIN_UART_TX  GPIO_PIN_30
#define PIN_UART_RX  GPIO_PIN_22
#elif TANG_PRIMER25K
#warning "Building for TANG_PRIMER25K internal BL616"
#include "../jtag.h"
#include "./sdc_direct.h"
#define PIN_JTAGSEL  GPIO_PIN_11
#define PIN_UART_TX  GPIO_PIN_12
#define PIN_UART_RX  GPIO_PIN_22
#else
#error "No valid TANG_BOARD specified!"
#endif

static struct bflb_device_s *gpio;

#if defined(TANG_NANO20K)
#define PIN_JTAG_TMS GPIO_PIN_16
#define PIN_JTAG_TCK GPIO_PIN_10
#define PIN_JTAG_TDI GPIO_PIN_12
#define PIN_JTAG_TDO GPIO_PIN_14
volatile uint32_t *reg_gpio_tms = (volatile uint32_t *)0x20000904; // gpio_cfg16
volatile uint32_t *reg_gpio_tck = (volatile uint32_t *)0x200008ec; // gpio_cfg10
volatile uint32_t *reg_gpio_tdo = (volatile uint32_t *)0x200008fc; // gpio_cfg14
volatile uint32_t *reg_gpio_tdi = (volatile uint32_t *)0x200008f4; // gpio_cfg12
#else
#define PIN_JTAG_TMS GPIO_PIN_0
#define PIN_JTAG_TCK GPIO_PIN_1
#define PIN_JTAG_TDI GPIO_PIN_3
#define PIN_JTAG_TDO GPIO_PIN_2
volatile uint32_t *reg_gpio_tms = (volatile uint32_t *)0x200008c4; // gpio_cfg0
volatile uint32_t *reg_gpio_tck = (volatile uint32_t *)0x200008c8; // gpio_cfg1
volatile uint32_t *reg_gpio_tdo = (volatile uint32_t *)0x200008cc; // gpio_cfg2
volatile uint32_t *reg_gpio_tdi = (volatile uint32_t *)0x200008d0; // gpio_cfg3
#endif
volatile uint32_t *reg_gpio0_31 = (volatile uint32_t *)0x20000ae4; // gpio_cfg136，Register Controlled GPIO Output Value

#define xGPIO_INT_MASK    (1<<22) // GLB_REG_GPIO_0_INT_MASK
#define xGPIO_FUNC_SWGPIO (11<<8) // SWGPIO function definition
#define xGPIO_OUTPUT_EN   (1<<6)  // GLB_REG_GPIO_0_OE
#define xGPIO_SCHMITT_EN  (1<<1)  // GLB_REG_GPIO_0_SMT
#define xGPIO_DRV_3       (3<<2)  // GLB_REG_GPIO_0_DRV_MASK GLB_REG_GPIO_0_DRV_SHIFT

uint32_t jtag_tms_cfg, jtag_tck_cfg, jtag_tdi_cfg;

// set GPIO0 (TMS), GPIO1 (TCK) and GPIO3 (TDI) as direct output mode
void mcu_hw_jtag_enter_gpio_out_mode(void) {
	jtag_tms_cfg = *reg_gpio_tms;
	jtag_tck_cfg = *reg_gpio_tck;
	jtag_tdi_cfg = *reg_gpio_tdi;
	*reg_gpio_tms = xGPIO_INT_MASK | xGPIO_FUNC_SWGPIO | xGPIO_OUTPUT_EN | xGPIO_SCHMITT_EN | xGPIO_DRV_3;
	*reg_gpio_tck = xGPIO_INT_MASK | xGPIO_FUNC_SWGPIO | xGPIO_OUTPUT_EN | xGPIO_SCHMITT_EN | xGPIO_DRV_3;
	*reg_gpio_tdi = xGPIO_INT_MASK | xGPIO_FUNC_SWGPIO | xGPIO_OUTPUT_EN | xGPIO_SCHMITT_EN | xGPIO_DRV_3;
}

// restore GPIO0 (TMS), GPIO1 (TCK) and GPIO3 (TDI) settings
void mcu_hw_jtag_exit_gpio_out_mode(void) {
	*reg_gpio_tms = jtag_tms_cfg;
	*reg_gpio_tck = jtag_tck_cfg;
	*reg_gpio_tdi = jtag_tdi_cfg;
}

static void mcu_hw_jtag_init(void);

/* ============================================================================================= */
/* ===============                          USB                                   ============== */
/* ============================================================================================= */

// usb_host.c

#include <queue.h>
#include <hardware/bl616.h>

#define MAX_REPORT_SIZE   8
#define XBOX_REPORT_SIZE 20

#define STATE_NONE      0 
#define STATE_DETECTED  1 
#define STATE_RUNNING   2
#define STATE_FAILED    3
#define NO_JOYSTICK 255

#define XINPUT_GAMEPAD_DPAD_UP 0x0001
#define XINPUT_GAMEPAD_DPAD_DOWN 0x0002
#define XINPUT_GAMEPAD_DPAD_LEFT 0x0004
#define XINPUT_GAMEPAD_DPAD_RIGHT 0x0008
#define XINPUT_GAMEPAD_START 0x0010
#define XINPUT_GAMEPAD_BACK 0x0020
#define XINPUT_GAMEPAD_LEFT_SHOULDER 0x0100
#define XINPUT_GAMEPAD_RIGHT_SHOULDER 0x0200

extern void shell_init_with_task(struct bflb_device_s *shell);

// Lookup if there is a map for current gamepad
const UsbGamepadMap *find_usb_gamepad_map(uint16_t vid,
                                          uint16_t pid,
                                          int version_optional)
{
  const UsbGamepadMap *fallback = NULL;

  for (size_t i = 0; i < kUsbGamepadMapsCount; i++)
  {
    const UsbGamepadMap *m = &kUsbGamepadMaps[i];

    if (m->vid == vid && m->pid == pid)
    {
      if (version_optional >= 0)
      {
        if (m->version == (uint16_t)version_optional)
        {
          return m; // version hit
        }
      }
      else
      {
        if (!fallback)
          fallback = m;
      }
    }
  }

  return fallback;
}

void set_led(int pin, int on) {
#ifdef M0S_DOCK
  // only M0S dock has those leds
  if(on) bflb_gpio_reset(gpio, pin);
  else   bflb_gpio_set(gpio, pin);
#endif
}

static struct usb_config {
  struct xbox_info_S {
    int index;
    int state;
    struct usbh_hid *class;
    uint8_t *buffer;
    int nbytes;
    hid_report_t report;

    unsigned char last_state;
    unsigned char js_index;
    unsigned char last_state_btn_extra;
    int16_t last_state_x;
    int16_t last_state_y;

    struct usb_config *usb;
    SemaphoreHandle_t sem;
    TaskHandle_t task_handle;
#ifdef RATE_CHECK
    TickType_t rate_start;
    unsigned long rate_events;
#endif
    hid_state_t xbox_state;
    volatile uint8_t stop;
} xbox_info[CONFIG_USBHOST_MAX_XBOX_CLASS];
    
  struct hid_info_S {
    int index;
    int state;
    struct usbh_hid *class;
    uint8_t *buffer;
    int nbytes;
    hid_report_t report;
    struct usb_config *usb;
    SemaphoreHandle_t sem;
    TaskHandle_t task_handle;
#ifdef RATE_CHECK
    TickType_t rate_start;
    unsigned long rate_events;
#endif
    hid_state_t hid_state;
    volatile uint8_t stop;
  } hid_info[CONFIG_USBHOST_MAX_HID_CLASS];
} usb_config;

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t hid_buffer[CONFIG_USBHOST_MAX_HID_CLASS][MAX_REPORT_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t xbox_buffer[CONFIG_USBHOST_MAX_XBOX_CLASS][XBOX_REPORT_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t report_desc[CONFIG_USBHOST_MAX_HID_CLASS][128];

uint8_t byteScaleAnalog(int16_t xbox_val)
{
  // Scale the xbox value from [-32768, 32767] to [1, 255]
  // Offset by 32768 to get in range [0, 65536], then divide by 256 to get in range [1, 255]
  uint8_t scale_val = (xbox_val + 32768) / 256;
  if (scale_val == 0) return 1;
  return scale_val;
}

void usbh_hid_callback(void *arg, int nbytes) {
  struct hid_info_S *hid = (struct hid_info_S *)arg;

  xSemaphoreGiveFromISR(hid->sem, NULL);
  hid->nbytes = nbytes;
}  

void usbh_xbox_callback(void *arg, int nbytes) {
  struct xbox_info_S *xbox = (struct xbox_info_S *)arg;
    xSemaphoreGiveFromISR(xbox->sem, NULL);
    xbox->nbytes = nbytes;
}  

bool mcu_hw_hid_present(void) {
  struct usb_config *usb = &usb_config;

  for(int i=0;i<CONFIG_USBHOST_MAX_HID_CLASS;i++) {
      if(usb->hid_info[i].state == STATE_DETECTED)
      return true;
    }

  for(int i=0;i<CONFIG_USBHOST_MAX_XBOX_CLASS;i++) {
    if(usb->xbox_info[i].state == STATE_DETECTED)
      return true;
    }
  return false;
  }  


static void xbox_parse(struct xbox_info_S *xbox) {

  // verify length field
  if(xbox->buffer[0] != 0 || xbox->buffer[1] != 20)
    return;

  uint16_t wButtons = xbox->buffer[3] << 8 | xbox->buffer[2];

  // build new state
  unsigned char state =
    ((wButtons & XINPUT_GAMEPAD_DPAD_UP   )?0x08:0x00) |
    ((wButtons & XINPUT_GAMEPAD_DPAD_DOWN )?0x04:0x00) |
    ((wButtons & XINPUT_GAMEPAD_DPAD_LEFT )?0x02:0x00) |
    ((wButtons & XINPUT_GAMEPAD_DPAD_RIGHT)?0x01:0x00) |
    ((wButtons & 0xf000) >> 8); // Y, X, B, A

  // build extra button new state
  unsigned char state_btn_extra =
    ((wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER  )?0x01:0x00) |
    ((wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER )?0x02:0x00) |
    ((wButtons & XINPUT_GAMEPAD_BACK           )?0x10:0x00) | // Rumblepad 2 / Dual Action compatibility
    ((wButtons & XINPUT_GAMEPAD_START          )?0x20:0x00);

  // build analog stick x,y state
  int16_t sThumbLX = xbox->buffer[7] << 8 | xbox->buffer[6];
  int16_t sThumbLY = xbox->buffer[9] << 8 | xbox->buffer[8];
  uint8_t ax = byteScaleAnalog(sThumbLX);
  uint8_t ay = ~byteScaleAnalog(sThumbLY);

  // map analog stick directions to digital
  if(ax > (uint8_t) 0xc0) state |= 0x01;
  if(ax < (uint8_t) 0x40) state |= 0x02;
  if(ay > (uint8_t) 0xc0) state |= 0x04;
  if(ay < (uint8_t) 0x40) state |= 0x08;

  // submit if state has changed
  if(state != xbox->last_state ||
    state_btn_extra != xbox->last_state_btn_extra ||
    sThumbLX != xbox->last_state_x ||
    sThumbLY != xbox->last_state_y) {

    xbox->last_state = state;
    xbox->last_state_btn_extra = state_btn_extra;
    xbox->last_state_x = sThumbLX;
    xbox->last_state_y = sThumbLY;
    usb_debugf("XBOX Joy%d: B %02x EB %02x X %02x Y %02x", xbox->js_index, state, state_btn_extra, ax, ay);

    if(osd_is_visible()) {	       
      // if OSD is visible, then process events locally
      menu_joystick_state(state);
    } else {
    mcu_hw_spi_begin();
    mcu_hw_spi_tx_u08(SPI_TARGET_HID);
    mcu_hw_spi_tx_u08(SPI_HID_JOYSTICK);
    mcu_hw_spi_tx_u08(xbox->js_index);
    mcu_hw_spi_tx_u08(state);
    mcu_hw_spi_tx_u08(ax); // gamepad analog X
    mcu_hw_spi_tx_u08(ay); // gamepad analog Y
    mcu_hw_spi_tx_u08(state_btn_extra); // gamepad extra buttons
    mcu_hw_spi_end();
    }
  }
}

// each HID client gets itws own thread which submits urbs
// and waits for the interrupt to succeed
static void usbh_hid_client_thread(void *argument) {
  struct hid_info_S *hid = (struct hid_info_S *)argument;

  usb_debugf("HID client #%d: thread started", hid->index);

  uint16_t vendor_id = hid->class->hport->device_desc.idVendor;
  uint16_t product_id = hid->class->hport->device_desc.idProduct;
  uint16_t version_id = hid->class->hport->device_desc.bcdDevice;

  usb_debugf("idVendor: 0x%04x          ", vendor_id);
  usb_debugf("idProduct: 0x%04x         ", product_id);
  usb_debugf("bcdDevice(version): 0x%04x         ", version_id);
  usb_debugf("report type: 0x%04x          ", hid->report.type);

  const UsbGamepadMap *map;

  if (hid->report.type == REPORT_TYPE_JOYSTICK) {
    usb_debugf("HID client #%d: trying to allocate joystick", hid->index);
    hid->hid_state.joystick.js_index = hid_allocate_joystick();
    usb_debugf("  -> joystick %d", hid->hid_state.joystick.js_index);
  }

  
  if (hid->report.type == REPORT_TYPE_JOYSTICK)
  {

    // lookup for VID/PID/Version
    map = find_usb_gamepad_map(vendor_id, product_id, version_id);

    if (map)
    {
      usb_debugf("Found gamepad map: %s (VID=%04x PID=%04x VER=%04x)",
                 map->name, map->vid, map->pid, map->version);

      hid->report.map = map;
      hid->report.map_found = 1;
      hid->report.map_checked = 1;
    }
    else
    {
      usb_debugf("No map for VID=%04x PID=%04x, VERSION=%04x", vendor_id, product_id, version_id);
      hid->report.map = NULL;
      hid->report.map_found = 0;
      hid->report.map_checked = 1;
    }
  }

  size_t len = hid->report.report_size + (hid->report.report_id_present ? 1 : 0);

  uint32_t hid_interval_ms;
  if (hid->class->hport->speed == USB_SPEED_HIGH) {
      /* HS */
      hid_interval_ms = (0x01 << (hid->class->intin->bInterval - 1)) * 125 / 1000;
  } else {
      /* LS/FS */
      hid_interval_ms = hid->class->intin->bInterval;
  }
  usb_debugf("USB XBOX hid_interval: %dms", hid->class->intin->bInterval);

  usbh_int_urb_fill(&hid->class->intin_urb,
                    hid->class->hport,
                    hid->class->intin,
                    hid->buffer, len,
                    hid_interval_ms,
                    usbh_hid_callback, hid);

  if (hid->class->hport->device_desc.idVendor == 0x2563 && hid->class->hport->device_desc.idProduct == 0x0575) {
    usb_debugf("SHANWAN gamepad detected in bootstrap mode - triggering switch to HID mode");

    // Sending GET_REPORT to initiate 0079:181c
    uint8_t dummy_report[27];
    int ret = usbh_hid_get_report(hid->class,
                                  1, // INPUT report
                                  0, // Report ID = 0
                                  dummy_report,
                                  27); // report size

    usb_debugf("GET_REPORT ret=%d - device should reenumerate as 0079:181c", ret);
  }

  usb_debugf("HID client #%d: entering hid thread loop", hid->index);

  while (1) {
    if (hid->stop)
      break;
    if (!hid->class->hport || !hid->class->hport->connected)
      break;

    usbh_int_urb_fill(&hid->class->intin_urb,
                      hid->class->hport,
                      hid->class->intin,
                      hid->buffer, len,
                      hid_interval_ms,
                      usbh_hid_callback, hid);

    int ret = usbh_submit_urb(&hid->class->intin_urb);

    if (hid->stop)
      break;

    if (ret < 0) {
      if (ret == -USB_ERR_NODEV || ret == -USB_ERR_NOTCONN)
        break;

      if (ret != -USB_ERR_TIMEOUT)
        usb_debugf("HID client #%d: submit failed, %d", hid->index, ret);

      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (xSemaphoreTake(hid->sem, portMAX_DELAY) != pdTRUE) {
      if (hid->stop)
        break;

    usbh_kill_urb(&hid->class->intin_urb);
    continue;
    }

    if (hid->stop)
      break;

    if (hid->nbytes > 0) {
      hid_parse(&hid->report, &hid->hid_state, hid->buffer, hid->nbytes);
    }

    hid->nbytes = 0;

#ifdef RATE_CHECK
    hid->rate_events++;
    if (!(hid->rate_events % 100)) {
      float ms_since_start = (xTaskGetTickCount() - hid->rate_start) * portTICK_PERIOD_MS;
      usb_debugf("Rate = %f events/sec", 1000 * hid->rate_events / ms_since_start);
    }
#endif

  /* wait for next interval */
  vTaskDelay(pdMS_TO_TICKS(hid_interval_ms));
  }

  usb_debugf("HID client #%d: stopping", hid->index);
  if (hid->hid_state.joystick.js_index != NO_JOYSTICK) {
    hid_release_joystick(hid->hid_state.joystick.js_index);
    hid->hid_state.joystick.js_index = NO_JOYSTICK;
  }
  hid->task_handle = NULL;
  vTaskDelete(NULL);
}

// from fixcontroler.py: https://gist.github.com/adnanh/f60f069fc9185a48b73db9987b9e9108
struct usb_setup_packet xbox_init_packets[5] = {
  {0x80, 0x06, 0x0302, 0x0409, 2},        // get string descriptor (SN30pro needs this)
  {0x80, 0x06, 0x0302, 0x0409, 32},       // get string descriptor
  {0xC1, 0x01, 0x0100, 0x0000, 20},       // control transfer 1 (a lot of pads need this)
  {0xC1, 0x01, 0x0000, 0x0000, 8},        // control transfer 2
  /* {0xC0, 0x01, 0x0100, 0x0000, 4}*/};  // skipped as 8bitdo wireless adapter hangs on this

// four data packets to EP2
uint8_t xbox_ep2_packets[4][3] = {{0x01, 0x03, 0x02}, {0x02, 0x08, 0x03}, 
                                  {0x01, 0x03, 0x02}, {0x01, 0x03, 0x06}};

static void xbox_init(struct xbox_info_S *xbox) {
  for (int i = 0; i < 4; i++) {
      usbh_int_urb_fill(&xbox->class->intout_urb,
          xbox->class->hport,
          xbox->class->intout,
          xbox_ep2_packets[i], 3,
          0, usbh_xbox_callback, xbox);
      int ret = usbh_submit_urb(&xbox->class->intout_urb);
      if (ret < 0)
      usb_debugf("XBOX FATAL: submit EP2 failed %d", ret);
      else
          xSemaphoreTake(xbox->sem, 0xffffffffUL);    // wait for callback to finish
  }
}

// ... and XBOX clients as well
static void usbh_xbox_client_thread(void *argument) {
  struct xbox_info_S *xbox = (struct xbox_info_S *)argument;
  int ret = 0;
  const UsbGamepadMap *map;

  usb_debugf("XBOX client #%d: thread started", xbox->index);

  uint16_t vendor_id = xbox->class->hport->device_desc.idVendor;
  uint16_t product_id = xbox->class->hport->device_desc.idProduct;
  uint16_t version_id = xbox->class->hport->device_desc.bcdDevice;

  usb_debugf("idVendor: 0x%04x          ", vendor_id);
  usb_debugf("idProduct: 0x%04x         ", product_id);
  usb_debugf("bcdDevice(version): 0x%04x         ", version_id);
  usb_debugf("report type: 0x%04x          ", xbox->report.type);

  // Send initialization packets
  for (int i = 0; i < CONFIG_USBHOST_MAX_XBOX_CLASS; i++) {
    if ((ret = usbh_control_transfer(xbox->class->hport, &xbox_init_packets[i], xbox->buffer)) < 0) {
      usb_debugf("XBOX: init packet %d failed: %d", i, ret);
     }
   }
  xbox_init(xbox);
  usb_debugf("XBOX client #%d: all init packets sent", xbox->index);

  // lookup for VID/PID/Version
  map = find_usb_gamepad_map(vendor_id, product_id, version_id);

  if (map) {
    usb_debugf("Found gamepad map: %s (VID=%04x PID=%04x VER=%04x)",
                map->name, map->vid, map->pid, map->version);

    xbox->report.map = map;
    xbox->report.map_found = 1;
    xbox->report.map_checked = 1;
  } 
  else {
    usb_debugf("No map for VID=%04x PID=%04x, VERSION=%04x", vendor_id, product_id, version_id);
    xbox->report.map = NULL;
    xbox->report.map_found = 0;
    xbox->report.map_checked = 1;
  }

  uint32_t hid_interval_ms;
  if (xbox->class->hport->speed == USB_SPEED_HIGH) {
      /* HS */
      hid_interval_ms = (0x01 << (xbox->class->intin->bInterval - 1)) * 125 / 1000;
  } else {
      /* LS/FS */
      hid_interval_ms = xbox->class->intin->bInterval;
  }
  usb_debugf("USB XBOX hid_interval: %dms", xbox->class->intin->bInterval);

  while(1) {
    if (xbox->stop)
      break;

    if (!xbox->class->hport || !xbox->class->hport->connected)
      break;

    usbh_int_urb_fill(&xbox->class->intin_urb, 
                    xbox->class->hport, 
                    xbox->class->intin, 
                    xbox->buffer, XBOX_REPORT_SIZE,
                    hid_interval_ms,
                    usbh_xbox_callback, xbox);

    int ret = usbh_submit_urb(&xbox->class->intin_urb);

    if (xbox->stop)
      break;

    if (ret < 0) {
      if (ret == -USB_ERR_NODEV || ret == -USB_ERR_NOTCONN)
        break;

      if (ret != -USB_ERR_TIMEOUT)
        usb_debugf("HID client #%d: submit failed, %d", xbox->index, ret);

      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (xSemaphoreTake(xbox->sem, portMAX_DELAY) != pdTRUE) {
      if (xbox->stop)
        break;

    usbh_kill_urb(&xbox->class->intin_urb);
    continue;
    }

    if (xbox->stop)
      break;

    if(xbox->nbytes == XBOX_REPORT_SIZE)
      xbox_parse(xbox);

    xbox->nbytes = 0;

#ifdef RATE_CHECK
    xbox->rate_events++;
    if(!(xbox->rate_events % 100)) {
     float ms_since_start = (xTaskGetTickCount() - xbox->rate_start) * portTICK_PERIOD_MS;
     usb_debugf("Rate = %f events/sec",  1000 * xbox->rate_events /  ms_since_start);
    }    
#endif
  /* wait for next interval */
  vTaskDelay(pdMS_TO_TICKS(hid_interval_ms));
  }

  usb_debugf("XBOX client #%d: stopping", xbox->index);
  xbox->task_handle = NULL;
  vTaskDelete(NULL);
}

void usbh_hid_run(struct usbh_hid *hid_class)
{
  struct usb_config *usb = &usb_config;

  uint8_t i = hid_class->minor;

  const char *driver_name = hid_class->hport->config.intf[hid_class->intf].class_driver->driver_name;
  debugf("New Path - connected - Driver name: %s intf: %d minor: %u rep_size: %u", hid_class->hport->config.intf[hid_class->intf].class_driver->driver_name, i, hid_class->minor, hid_class->report_size);

  if (driver_name && strcmp(driver_name, "hid") == 0) {
    // request status (currently only dummy data, will return 0x5c, 0x42)
    // in the long term the core is supposed to return its HID demands
    // (keyboard matrix type, joystick type and number, ...)
    
    mcu_hw_spi_begin();
    mcu_hw_spi_tx_u08(SPI_TARGET_HID);
    mcu_hw_spi_tx_u08(SPI_HID_STATUS);
    mcu_hw_spi_tx_u08(0x00);
    usb_debugf("HID status #0: %02x", mcu_hw_spi_tx_u08(0x00));
    usb_debugf("HID status #1: %02x", mcu_hw_spi_tx_u08(0x00));
    mcu_hw_spi_end();

    usb->hid_info[i].class = hid_class;

    usb_debugf("NEW HID %d", i);
    uint8_t ifnt = usb->hid_info[i].class->intf;
    memset(&usb->hid_info[i].report, 0, sizeof(usb->hid_info[i].report));

    uint16_t rep_desc = usbh_hid_get_report_descriptor(hid_class, report_desc[i], 1024);
    if (rep_desc < 0)
    {
      usb_debugf("usbh_hid_get_report_descriptor issue");
      usb->hid_info[i].state = STATE_FAILED;
      return;
    }

    if (!parse_report_descriptor(report_desc[i], (uint16_t)rep_desc, &usb->hid_info[i].report, NULL))
    {
      usb->hid_info[i].state = STATE_FAILED;
      return;
    }

    uint8_t ifn = usb->hid_info[i].class->intf;
    const struct usb_interface_descriptor *id =
        &usb->hid_info[i].class->hport->config.intf[ifn].altsetting[0].intf_desc;

    usb->hid_info[i].stop = 0;
    usb->hid_info[i].state = STATE_DETECTED;
  }

  xTaskCreate(usbh_hid_client_thread, (char *)"hid_task", 1024,
              &usb->hid_info[i], configMAX_PRIORITIES - 3, &usb->hid_info[i].task_handle);
}

void usbh_hid_stop(struct usbh_hid *hid_class)
{
  uint8_t i = hid_class->minor;
  usb_config.hid_info[i].stop = 1;

  if (usb_config.hid_info[i].hid_state.joystick.js_index != NO_JOYSTICK)
  {
    hid_release_joystick(usb_config.hid_info[i].hid_state.joystick.js_index);
    usb_config.hid_info[i].hid_state.joystick.js_index = NO_JOYSTICK;
  }
}

void usbh_xbox_run(struct usbh_xbox *xbox_class) {
  struct usb_config *usb = &usb_config;

  uint8_t i = xbox_class->minor;

  const char *driver_name = xbox_class->hport->config.intf[xbox_class->intf].class_driver->driver_name;
  debugf("New Path - connected - XBOX Driver name: %s intf: %d minor: %u", xbox_class->hport->config.intf[xbox_class->intf].class_driver->driver_name, i, xbox_class->minor);

  if (driver_name && strcmp(driver_name, "xbox") == 0) {

    usb->xbox_info[i].class = xbox_class;

    usb_debugf("NEW XBOX HID %d", i);
    uint8_t ifnt = usb->xbox_info[i].class->intf;
    memset(&usb->xbox_info[i].report, 0, sizeof(usb->xbox_info[i].report));

    uint16_t rep_desc = usbh_hid_get_report_descriptor(xbox_class, report_desc[i], 1024);
    if (rep_desc < 0)
    {
      usb_debugf("usbh_hid_get_report_descriptor issue");
      usb->xbox_info[i].state = STATE_FAILED;
      return;
    }

    uint8_t ifn = usb->xbox_info[i].class->intf;
    const struct usb_interface_descriptor *id =
        &usb->xbox_info[i].class->hport->config.intf[ifn].altsetting[0].intf_desc;

    usb->xbox_info[i].stop = 0;
    usb->xbox_info[i].state = STATE_DETECTED;
  }

	xTaskCreate(usbh_xbox_client_thread, (char *)"xbox_task", 2048,
		    &usb->xbox_info[i], configMAX_PRIORITIES-3, &usb->xbox_info[i].task_handle );
}

void usbh_xbox_stop(struct usbh_xbox *xbox_class) {
  uint8_t i = xbox_class->minor;
  usb_config.xbox_info[i].stop = 1;
}

static struct bflb_device_s *usb_dev;

void usb_host(void) {

  usb_debugf("init usb hid host");

  usb_dev = bflb_device_get_by_name(BFLB_NAME_USB_V2);
  if (!usb_dev) {
      usb_debugf("usb device not found");
      return -1;
  }

  // initialize all HID info entries
  for(int i=0;i<CONFIG_USBHOST_MAX_HID_CLASS;i++) {
    usb_config.hid_info[i].index = i;
    usb_config.hid_info[i].state = 0;
    usb_config.hid_info[i].buffer = hid_buffer[i];      
    usb_config.hid_info[i].usb = &usb_config;
    usb_config.hid_info[i].sem = xSemaphoreCreateBinary();
  }
  
  // initialize all XBOX info entries
  for(int i=0;i<CONFIG_USBHOST_MAX_XBOX_CLASS;i++) {
    usb_config.xbox_info[i].index = i;
    usb_config.xbox_info[i].state = 0;
    usb_config.xbox_info[i].buffer = xbox_buffer[i];      
    usb_config.xbox_info[i].usb = &usb_config;
    usb_config.xbox_info[i].sem = xSemaphoreCreateBinary();
  }

  usbh_initialize(0, usb_dev->reg_base);
}

uint8_t usbh_get_hport_active_config_index(struct usbh_hubport *hport)
{
    struct usb_device_descriptor *dev_desc = &(hport->device_desc);
    return 0;
}

/* ============================================================================================= */
/* ===============                          SPI                                   ============== */
/* ============================================================================================= */

extern TaskHandle_t com_task_handle;
static SemaphoreHandle_t spi_sem;
static struct bflb_device_s *spi_dev;

#ifdef M0S_DOCK
  #define SPI_PIN_CSN   GPIO_PIN_12
  #define SPI_PIN_SCK   GPIO_PIN_13
  #define SPI_PIN_MISO  GPIO_PIN_10
  #define SPI_PIN_MOSI  GPIO_PIN_11
  #define SPI_PIN_IRQ   GPIO_PIN_14
#elif TANG_NANO20K
  #define SPI_PIN_CSN   GPIO_PIN_0  /* out SPI_CSn */
  #define SPI_PIN_SCK   GPIO_PIN_1  /* out SPI_SCLK */
  #define SPI_PIN_MISO  GPIO_PIN_2  /* in  SPI_DIR, CHIP_EN, 10K PD*/
  #define SPI_PIN_MOSI  GPIO_PIN_3  /* out SPI_DAT */
  #define SPI_PIN_IRQ   GPIO_PIN_13 /* in  UART RX, crossed */
#elif TANG_CONSOLE60K
  #define SPI_PIN_CSN   GPIO_PIN_0 /* out TMS */
  #define SPI_PIN_SCK   GPIO_PIN_1 /* out TCK 4K7 PD SOM */
  #define SPI_PIN_MISO  GPIO_PIN_2 /* in  TDO, CHIP_EN 3K3 PD */
  #define SPI_PIN_MOSI  GPIO_PIN_3 /* out TDI */
  #define SPI_PIN_IRQ   GPIO_PIN_27/* in  UART RX, crossed */
#elif TANG_MEGA138KPRO
  #define SPI_PIN_CSN   GPIO_PIN_0 /* out TMS */
  #define SPI_PIN_SCK   GPIO_PIN_1 /* out TCK */
  #define SPI_PIN_MISO  GPIO_PIN_2 /* in  TDO, CHIP_EN 4K7 PD */
  #define SPI_PIN_MOSI  GPIO_PIN_3 /* out TDI */
  #define SPI_PIN_IRQ   GPIO_PIN_11/* in  UART RX, crossed */
  // requesting 20Mhz on the Tang Mega actually results in 26.67MHz
  // which doesn't seem to work together with the 4k7 pulldown
  #define SPI_FREQUENCY 12000000   /* actually results in 13.3333MHz*/
#elif TANG_MEGA60K
  #define SPI_PIN_CSN   GPIO_PIN_0 /* out TMS */
  #define SPI_PIN_SCK   GPIO_PIN_1 /* out TCK */
  #define SPI_PIN_MISO  GPIO_PIN_2 /* in  TDO, CHIP_EN */
  #define SPI_PIN_MOSI  GPIO_PIN_3 /* out TDI */
  #define SPI_PIN_IRQ   GPIO_PIN_27/* in  UART RX, crossed */
#elif TANG_PRIMER25K
  #define SPI_PIN_CSN   GPIO_PIN_0 /* out TMS */
  #define SPI_PIN_SCK   GPIO_PIN_1 /* out TCK */
  #define SPI_PIN_MISO  GPIO_PIN_2 /* in  TDO, CHIP_EN */
  #define SPI_PIN_MOSI  GPIO_PIN_3 /* out TDI */
  #define SPI_PIN_IRQ   GPIO_PIN_10/* in  UART RX, crossed */
  #define SPI_FREQUENCY 12000000   /* actually results in 13.3333MHz*/
#endif

#ifndef SPI_FREQUENCY
#define SPI_FREQUENCY 20000000   /* default SPI clock is 20 MHz, actually 26Mhz ! */
#endif

void spi_isr(uint8_t pin) {
  if (pin == SPI_PIN_IRQ) {
    // disable further interrupts until thread has processed the current message
    bflb_irq_disable(gpio->irq_num);

    if(com_task_handle) {    
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR( com_task_handle, &xHigherPriorityTaskWoken );
      portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
  }
}

static void mcu_hw_spi_init(void) {
  // when FPGA sets data on rising edge:
  // stable with long cables up to 20Mhz
  // short cables up to 32MHz
  gpio = bflb_device_get_by_name("gpio");

  /* spi miso */
  bflb_gpio_init(gpio, SPI_PIN_MISO, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
  /* spi mosi */
  bflb_gpio_init(gpio, SPI_PIN_MOSI, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
  /* spi clk */
  bflb_gpio_init(gpio, SPI_PIN_SCK, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
  /* spi cs */
  bflb_gpio_init(gpio, SPI_PIN_CSN, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_set(gpio, SPI_PIN_CSN);

  struct bflb_spi_config_s spi_cfg = {
    .freq = SPI_FREQUENCY,   // 20MHz
    .role = SPI_ROLE_MASTER,
    .mode = SPI_MODE1,         // mode 1: idle state low, data sampled on falling edge
    .data_width = SPI_DATA_WIDTH_8BIT,
    .bit_order = SPI_BIT_MSB,
    .byte_order = SPI_BYTE_LSB,
    .tx_fifo_threshold = 0,
    .rx_fifo_threshold = 0,
  };
  
  spi_dev = bflb_device_get_by_name("spi0");
  bflb_spi_init(spi_dev, &spi_cfg);

  bflb_spi_feature_control(spi_dev, SPI_CMD_SET_DATA_WIDTH, SPI_DATA_WIDTH_8BIT);

  // display the actual spi frequency
  int freq = bflb_spi_feature_control(spi_dev, SPI_CMD_GET_FREQ, 0);
  debugf("SPI frequency is %d", freq);

  // semaphore to access the spi bus
  spi_sem = xSemaphoreCreateMutex();

  /* interrupt input */
  bflb_irq_disable(gpio->irq_num);
  bflb_gpio_init(gpio, SPI_PIN_IRQ, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN);
  bflb_gpio_int_init(gpio, SPI_PIN_IRQ, GPIO_INT_TRIG_MODE_SYNC_LOW_LEVEL);
  bflb_gpio_irq_attach(SPI_PIN_IRQ, spi_isr);
}

// spi may be used by different threads. Thus begin and end are using
// semaphores

void mcu_hw_spi_begin(void) {
  xSemaphoreTake(spi_sem, 0xffffffffUL); // wait forever
  bflb_gpio_reset(gpio, SPI_PIN_CSN);
}

unsigned char mcu_hw_spi_tx_u08(unsigned char b) {
  return bflb_spi_poll_send(spi_dev, b);
}

void mcu_hw_spi_end(void) {
  bflb_gpio_set(gpio, SPI_PIN_CSN);
  xSemaphoreGive(spi_sem);
}

void mcu_hw_irq_ack(void) {
  // re-enable the interrupt since it was now serviced outside the irq handeler
  bflb_irq_enable(gpio->irq_num);   // resume interrupt processing
}

/* ============================================================================================= */
/* ============================================================================================= */

extern void log_start(void);

extern uint32_t __HeapBase;
extern uint32_t __HeapLimit;

extern uint32_t _heap_wifi_start;
extern uint32_t _heap_wifi_size;

extern void bl_show_flashinfo(void);
extern void bl_show_log(void);
extern void bl_show_component_version(void);

#define CONSOLE_BAUDRATE 2000000

#ifdef CONFIG_CONSOLE_WO
extern void bflb_wo_set_console(struct bflb_device_s *dev);
static struct bflb_device_s *wo;
#else
extern void bflb_uart_set_console(struct bflb_device_s *dev);
static struct bflb_device_s *uart0;
#endif

#if (defined(CONFIG_LUA) || defined(CONFIG_BFLOG) || defined(CONFIG_FATFS))
static struct bflb_device_s *rtc;
#endif

static void ATTR_CLOCK_SECTION __attribute__((noinline)) system_clock_init(void) {
  /* wifipll/audiopll */

#if defined(TANG_MEGA138KPRO) || defined(TANG_PRIMER25K)|| defined(TANG_MEGA60K)
  GLB_Power_On_XTAL_And_PLL_CLK(GLB_XTAL_26M, GLB_PLL_WIFIPLL | GLB_PLL_AUPLL);
#else
  GLB_Power_On_XTAL_And_PLL_CLK(GLB_XTAL_40M, GLB_PLL_WIFIPLL | GLB_PLL_AUPLL);
#endif
  GLB_Set_MCU_System_CLK(GLB_MCU_SYS_CLK_TOP_WIFIPLL_320M);
  HBN_Set_MCU_XCLK_Sel(HBN_MCU_XCLK_XTAL);
  CPU_Set_MTimer_CLK(ENABLE, BL_MTIMER_SOURCE_CLOCK_MCU_XCLK, Clock_System_Clock_Get(BL_SYSTEM_CLOCK_XCLK) / 1000000 - 1);

#ifdef CONFIG_WIFI6
    /* enable wifi clock */
    GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_IP_WIFI_PHY | GLB_AHB_CLOCK_IP_WIFI_MAC_PHY | GLB_AHB_CLOCK_IP_WIFI_PLATFORM);
    GLB_AHB_MCU_Software_Reset(GLB_AHB_MCU_SW_WIFI);
#endif
}

/* TODO: disabled everything we don't need or use */
static void peripheral_clock_init(void) {
  PERIPHERAL_CLOCK_ADC_DAC_ENABLE();
  PERIPHERAL_CLOCK_SEC_ENABLE();
  PERIPHERAL_CLOCK_DMA0_ENABLE();
  PERIPHERAL_CLOCK_UART0_ENABLE();
  PERIPHERAL_CLOCK_UART1_ENABLE();
  PERIPHERAL_CLOCK_SPI0_ENABLE();
  PERIPHERAL_CLOCK_I2C0_ENABLE();
  PERIPHERAL_CLOCK_PWM0_ENABLE();
  PERIPHERAL_CLOCK_TIMER0_1_WDG_ENABLE();
  PERIPHERAL_CLOCK_IR_ENABLE();
  PERIPHERAL_CLOCK_I2S_ENABLE();
  PERIPHERAL_CLOCK_USB_ENABLE();
  PERIPHERAL_CLOCK_CAN_ENABLE();
  PERIPHERAL_CLOCK_AUDIO_ENABLE();
  PERIPHERAL_CLOCK_CKS_ENABLE();
    
  GLB_Set_UART_CLK(ENABLE, HBN_UART_CLK_XCLK, 0);
  GLB_Set_SPI_CLK(ENABLE, GLB_SPI_CLK_MCU_MUXPLL_160M, 0);
  GLB_Set_DBI_CLK(ENABLE, GLB_SPI_CLK_MCU_MUXPLL_160M, 0);
  GLB_Set_I2C_CLK(ENABLE, GLB_I2C_CLK_XCLK, 0);
  GLB_Set_ADC_CLK(ENABLE, GLB_ADC_CLK_XCLK, 1);
  GLB_Set_DIG_CLK_Sel(GLB_DIG_CLK_XCLK);
  GLB_Set_DIG_512K_CLK(ENABLE, ENABLE, 0x4E);
  GLB_Set_PWM1_IO_Sel(GLB_PWM1_IO_SINGLE_END);
  GLB_Set_IR_CLK(ENABLE, GLB_IR_CLK_SRC_XCLK, 19);
  GLB_Set_CAM_CLK(ENABLE, GLB_CAM_CLK_WIFIPLL_96M, 3);
  
  GLB_Set_PKA_CLK_Sel(GLB_PKA_CLK_MCU_MUXPLL_160M);
  
  GLB_Set_USB_CLK_From_WIFIPLL(1);
  GLB_Swap_MCU_SPI_0_MOSI_With_MISO(0);
}

static void console_init() {
  gpio = bflb_device_get_by_name("gpio");
  
#ifdef CONFIG_CONSOLE_WO
  wo = bflb_device_get_by_name("wo");
  bflb_wo_uart_init(wo, CONSOLE_BAUDRATE, PIN_UART_TX);
  bflb_wo_set_console(wo);
#else
  bflb_gpio_uart_init(gpio, PIN_UART_TX, GPIO_UART_FUNC_UART0_TX);
  bflb_gpio_uart_init(gpio, PIN_UART_RX, GPIO_UART_FUNC_UART0_RX);

  struct bflb_uart_config_s cfg;
  cfg.baudrate = CONSOLE_BAUDRATE;
  cfg.data_bits = UART_DATA_BITS_8;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.parity = UART_PARITY_NONE;
  cfg.flow_ctrl = 0;
  cfg.tx_fifo_threshold = 7;
  cfg.rx_fifo_threshold = 7;
  cfg.bit_order = UART_LSB_FIRST;
  
  uart0 = bflb_device_get_by_name("uart0");
  
  bflb_uart_init(uart0, &cfg);
  bflb_uart_set_console(uart0);
#endif
}

static void wifi_init(void);

// local board_init used as a replacemement for global board_init
static void mn_board_init(void) {
    int ret = -1;
    uintptr_t flag;
    size_t heap_len;
    uint32_t xtal_value = 0;

    /* lock */
    flag = bflb_irq_save();

    ret = bflb_flash_init();

    /* system clock */
    system_clock_init();

    peripheral_clock_init();

    /* irq init */
    bflb_irq_initialize();

#ifdef CONFIG_WIFI6
    extern void interrupt0_handler(void);
    bflb_irq_attach(WIFI_IRQn, (irq_callback)interrupt0_handler, NULL);
    bflb_irq_enable(WIFI_IRQn);
#endif

    gpio = bflb_device_get_by_name("gpio");
    // deinit all GPIOs
    bflb_gpio_deinit(gpio, GPIO_PIN_0);
    bflb_gpio_deinit(gpio, GPIO_PIN_1);
    bflb_gpio_deinit(gpio, GPIO_PIN_2);
    bflb_gpio_deinit(gpio, GPIO_PIN_3);

    bflb_gpio_deinit(gpio, GPIO_PIN_10);
    bflb_gpio_deinit(gpio, GPIO_PIN_11);
    bflb_gpio_deinit(gpio, GPIO_PIN_12);
    bflb_gpio_deinit(gpio, GPIO_PIN_13);
    bflb_gpio_deinit(gpio, GPIO_PIN_14);
    bflb_gpio_deinit(gpio, GPIO_PIN_15);
    bflb_gpio_deinit(gpio, GPIO_PIN_16);
    bflb_gpio_deinit(gpio, GPIO_PIN_17);

    bflb_gpio_deinit(gpio, GPIO_PIN_20);
    bflb_gpio_deinit(gpio, GPIO_PIN_21);
    bflb_gpio_deinit(gpio, GPIO_PIN_22);

    bflb_gpio_deinit(gpio, GPIO_PIN_27);
    bflb_gpio_deinit(gpio, GPIO_PIN_28);
    bflb_gpio_deinit(gpio, GPIO_PIN_29);
    bflb_gpio_deinit(gpio, GPIO_PIN_30);

    /* console init (uart or wo) */
    console_init();

    /* ram heap init */
    mem_manager_init();
    /* ocram heap init */
    heap_len = ((size_t)&__HeapLimit - (size_t)&__HeapBase);
    mm_register_heap(MM_HEAP_OCRAM_0, "OCRAM", MM_ALLOCATOR_TLSF, &__HeapBase, heap_len);
    mm_register_heap(MM_HEAP_WRAM_0, "WRAM", MM_ALLOCATOR_TLSF, &_heap_wifi_start, (uintptr_t)&_heap_wifi_size);

    debugf("\r\ndynamic memory init success");
    debugf("ocram heap size: %d Kbyte",((size_t)&__HeapLimit - (size_t)&__HeapBase) / 1024);

    /* boot info dump */
    bl_show_log();
    /* version info dump */
    bl_show_component_version();

    /* chip info dump */
    bl_show_chipinfo();

    /* flash info dump */
    bl_show_flashinfo();
    if (ret != 0) {
          debugf("flash init fail !!!");
        }

    debugf("uart  sig1:%08x, sig2:%08x", getreg32(GLB_BASE + GLB_UART_CFG1_OFFSET), getreg32(GLB_BASE + GLB_UART_CFG2_OFFSET));
    debugf("clock gen1:%08x, gen2:%08x", getreg32(GLB_BASE + GLB_CGEN_CFG1_OFFSET), getreg32(GLB_BASE + GLB_CGEN_CFG2_OFFSET));
    HBN_Get_Xtal_Value(&xtal_value);
    debugf("xtal:%dHz(%s)", xtal_value, ((getreg32(AON_BASE + AON_XTAL_CFG_OFFSET) >> 3) & 0x01) ? "oscillator" : "crystal");

    log_start();

#if (defined(CONFIG_LUA) || defined(CONFIG_BFLOG) || defined(CONFIG_FATFS))
    rtc = bflb_device_get_by_name("rtc");
#endif

#ifdef CONFIG_MBEDTLS
    extern void bflb_sec_mutex_init(void);
    bflb_sec_mutex_init();
#endif

    /* unlock */
    bflb_irq_restore(flag);

    debugf("board init done");
    debugf("===========================");
}

void mcu_hw_init(void) {
  mn_board_init();

  debugf("\r\n\r\n" LOGO "        FPGA Companion for BL616\r\n\r\n");

  gpio = bflb_device_get_by_name("gpio");

#ifdef M0S_DOCK
  // init on-board LEDs
  bflb_gpio_init(gpio, GPIO_PIN_27, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
  bflb_gpio_init(gpio, GPIO_PIN_28, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
  // both leds off
  bflb_gpio_set(gpio, GPIO_PIN_27);
  bflb_gpio_set(gpio, GPIO_PIN_28);
#elif TANG_NANO20K
  /* configure JTAGSEL_n */
#elif TANG_MEGA138KPRO
  /* LED6 enable */
  bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_reset(gpio, GPIO_PIN_20);
  /* configure JTAGSEL */
  bflb_gpio_init(gpio, PIN_JTAGSEL, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_reset(gpio, PIN_JTAGSEL);
#elif TANG_PRIMER25K
  /* LED5 enable 
  bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_reset(gpio, GPIO_PIN_20);
  */
  /* configure JTAGSEL_n */
  bflb_gpio_init(gpio, PIN_JTAGSEL, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_reset(gpio, PIN_JTAGSEL);
#elif TANG_CONSOLE60K
  /* configure JTAGSEL_n */
  bflb_gpio_init(gpio, PIN_JTAGSEL, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_reset(gpio, PIN_JTAGSEL);
  /* configure PIN_TF_SDIO_SEL to FPGA */
  bflb_gpio_init(gpio, PIN_TF_SDIO_SEL, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_reset(gpio, PIN_TF_SDIO_SEL);
#elif TANG_MEGA60K
  /* configure JTAGSEL_n */
  bflb_gpio_init(gpio, PIN_JTAGSEL, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_reset(gpio, PIN_JTAGSEL);
#endif
  mcu_hw_spi_init();

#ifndef CONFIG_CONSOLE_WO
  uart0 = bflb_device_get_by_name("uart0");
  shell_init_with_task(uart0);
#endif

#ifdef M0S_DOCK
  wifi_init();
#elif TANG_CONSOLE60K
  wifi_init();
#elif TANG_MEGA60K
  wifi_init();
#endif

#ifdef ENABLE_JTAG
  mcu_hw_jtag_init();
#endif
  usb_host();
}

void stop_hid(void) {
  struct usb_config *usb = &usb_config;

  for (int i = 0; i < CONFIG_USBHOST_MAX_HID_CLASS; i++) {
    if(usb->hid_info[i].state != STATE_NONE ) {
      usb_debugf("HID shutdown %d", i);
      usb->hid_info[i].stop = 1;
    }
  }

  for (int i = 0; i < CONFIG_USBHOST_MAX_XBOX_CLASS; i++) {
    if(usb->xbox_info[i].state != STATE_NONE ) {
      usb_debugf("HID xbox shutdown %d", i);
      usb->xbox_info[i].stop = 1;
    }
  }
 }

extern void hid_keyboard_init(uint8_t busid, uintptr_t reg_base);
extern int bl_sys_reset_por(void);

void mcu_hw_reset(void) {
  debugf("HW reset");

  struct bflb_device_s *wdg;
  struct bflb_wdg_config_s wdg_cfg;
  wdg_cfg.clock_source = WDG_CLKSRC_32K;
  wdg_cfg.clock_div = 31;
  wdg_cfg.comp_val = 1000;
  wdg_cfg.mode = WDG_MODE_RESET;

  wdg = bflb_device_get_by_name("watchdog0");
  bflb_wdg_init(wdg, &wdg_cfg);
  bflb_wdg_start(wdg);
  bflb_wdg_reset_countervalue(wdg);

  stop_hid();
  vTaskDelay(pdMS_TO_TICKS(250));

  gpio = bflb_device_get_by_name("gpio");
  bflb_irq_disable(gpio->irq_num);

  bflb_gpio_deinit(gpio, GPIO_PIN_0);
  bflb_gpio_deinit(gpio, GPIO_PIN_1);
  bflb_gpio_deinit(gpio, GPIO_PIN_2);
  bflb_gpio_deinit(gpio, GPIO_PIN_3);

  bflb_gpio_irq_detach(SPI_PIN_IRQ);
  bflb_spi_deinit(spi_dev);

  usbh_deinitialize(0);
#ifdef TANG_PRIMER25K
  bflb_mtimer_delay_ms(10);
  hid_keyboard_init(0,  usb_dev->reg_base);
  bflb_mtimer_delay_ms(100);
  debugf("deinit done and system POR reset");
  HBN_Set_User_Boot_Config(0); //HAL_REBOOT_AS_BOOTPIN
  bl_sys_reset_por();
#else
  debugf("deinit done and waiting for WDT POR reset");
#endif
  while (1) {
    /*empty dead loop*/
  }
}

void mcu_hw_port_byte(unsigned char byte) {
  debugf("port byte %d", byte);
}

#define WIFI_STACK_SIZE  (1536)
#define TASK_PRIORITY_FW (16)

// the wifi connection state
#define WIFI_STATE_UNKNOWN      0
#define WIFI_STATE_DISCONNECTED 1
#define WIFI_STATE_CONNECTING   2
#define WIFI_STATE_CONNECTED    3

static int wifi_state = WIFI_STATE_UNKNOWN;

static char *wifi_ssid = NULL;
static char *wifi_key = NULL;
static int s_retry_num = 0;
static wifi_conf_t conf = { .country_code = "EU" }; // "CN","US","JP","EU"
static QueueHandle_t wifi_event_queue = NULL;

void wifi_event_handler(async_input_event_t ev, void *priv)
{
  uint32_t code = ev->code;

  switch (code) {
  case CODE_WIFI_ON_INIT_DONE: {
    debugf("[APP] [EVT] %s, CODE_WIFI_ON_INIT_DONE", __func__);
    wifi_mgmr_init(&conf);
  } break;
  case CODE_WIFI_ON_MGMR_DONE: {
    debugf("[APP] [EVT] %s, CODE_WIFI_ON_MGMR_DONE", __func__);
  } break;
  case CODE_WIFI_ON_SCAN_DONE: {
    debugf("[APP] [EVT] %s, CODE_WIFI_ON_SCAN_DONE", __func__);
    wifi_mgmr_sta_scanlist();
    unsigned char evt = 1; 
    xQueueSendFromISR(wifi_event_queue, &evt, 0);
  } break;
  case CODE_WIFI_ON_CONNECTED: {
    debugf("[APP] [EVT] %s, CODE_WIFI_ON_CONNECTED", __func__);
    unsigned char evt = 3; 
    xQueueSendFromISR(wifi_event_queue, &evt, 0);
  } break;
  case CODE_WIFI_ON_GOT_IP: {
    debugf("[APP] [EVT] %s, CODE_WIFI_ON_GOT_IP", __func__);
    unsigned char evt = 4; 
    xQueueSendFromISR(wifi_event_queue, &evt, 0);
  } break;
  case CODE_WIFI_ON_DISCONNECT: {
    debugf("[APP] [EVT] %s, CODE_WIFI_ON_DISCONNECT", __func__);
    unsigned char evt = 2; 
    xQueueSendFromISR(wifi_event_queue, &evt, 0);
  } break;
  case CODE_WIFI_ON_AP_STARTED: {
    debugf("[APP] [EVT] %s, CODE_WIFI_ON_AP_STARTED", __func__);
  } break;
  case CODE_WIFI_ON_AP_STOPPED: {
    debugf("[APP] [EVT] %s, CODE_WIFI_ON_AP_STOPPED", __func__);
  } break;
  case CODE_WIFI_ON_AP_STA_ADD: {
    debugf("[APP] [EVT] [AP] [ADD] %lld", xTaskGetTickCount());
  } break;
  case CODE_WIFI_ON_AP_STA_DEL: {
    debugf("[APP] [EVT] [AP] [DEL] %lld", xTaskGetTickCount());
  } break;
  default:
    debugf("[APP] [EVT] Unknown code %u ", code);
  }
}

#define WIFI_STACK_SIZE  (1536)
#define TASK_PRIORITY_FW (16)

void wifi_start_firmware_task(void *param)
{
    /* network init */
    tcpip_init(NULL, NULL);
    debugf("Starting wifi ...");

    if (0 != rfparam_init(0, NULL, 0)) {
      debugf("PHY RF init failed!");
      return;
    }
    debugf("PHY RF init success!");

    async_register_event_filter(EV_WIFI, wifi_event_handler, NULL);
    wifi_task_create();

    debugf("Starting fhost ...");
    fhost_init();

    vTaskDelete(NULL);
}


static void wifi_init(void) {
  wifi_event_queue = xQueueCreate(10, sizeof(char));

  /* Enable wifi irq */
  extern void interrupt0_handler(void);
  bflb_irq_attach(WIFI_IRQn, (irq_callback)interrupt0_handler, NULL);
  bflb_irq_enable(WIFI_IRQn);

  xTaskCreate(wifi_start_firmware_task, "wifi init", 1024, NULL, 10, NULL);
}

static void wait4event(char code, char code2) {
  char evt = -1;
  for (int i = 0;i<10*30;i++) {
    if (code != evt && code2 != evt) {
      if(xQueueReceive(wifi_event_queue, &evt, pdMS_TO_TICKS(100))) {
        debugf("event: %d", evt);

        switch(evt) {
        case 1:
          debugf("  -> scan done");
          break;
        case 2:
          debugf("  -> disconnect");
          if (s_retry_num < 10) {
            // connect
            wifi_mgmr_sta_quickconnect(wifi_ssid, wifi_key, 0, 0);
            s_retry_num++;
            debugf("retry to connect to the AP");
            at_wifi_puts(".");
          } else {
            at_wifi_puts("\r\nConnection failed!\r\n");
            debugf("finally failed");
          }	
          break;
        case 3:
          debugf("  -> connect");
          break;
        case 4:
          debugf("  -> got ip");
          at_wifi_puts("\r\nConnected\r\n");
          break;	
        case 5:
          debugf("  -> init done");
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    }
 //   debugf("wait done");
  }
}

static const char *auth_mode_str(int authmode) {
  static const struct { int mode; char *str; } mode_str[] = {
    { WIFI_EVENT_BEACON_IND_AUTH_OPEN, "OPEN" },
    { WIFI_EVENT_BEACON_IND_AUTH_WEP, "WEP" },
    { WIFI_EVENT_BEACON_IND_AUTH_WPA_PSK, "WPA-PSK" },
    { WIFI_EVENT_BEACON_IND_AUTH_WPA2_PSK,"WPA2-PSK" },
    { WIFI_EVENT_BEACON_IND_AUTH_WPA_WPA2_PSK, "WPA-WPA2-PSK" },
    { WIFI_EVENT_BEACON_IND_AUTH_WPA_ENT, "ENTERPRISE" },
    { WIFI_EVENT_BEACON_IND_AUTH_WPA3_SAE, "WPA3-SAE" },
    { WIFI_EVENT_BEACON_IND_AUTH_WPA2_PSK_WPA3_SAE, "WPA2-PSK-WPA3-SAE" },
    { -1, "<unknown>" }
  };

  int i;
  for(i=0;mode_str[i].mode != -1;i++)
    if(mode_str[i].mode == authmode || mode_str[i].mode == -1)
      return mode_str[i].str;

  return mode_str[i].str;
}

static void wifi_scan_item_cb(void *env, void *arg, wifi_mgmr_scan_item_t *item) {
  debugf("scan item cb %s", item->ssid);

  char str[74];
  snprintf(str, sizeof(str), "SSID %s, RSSI %d, CH %d, %s\r\n", item->ssid, item->rssi,
	   item->channel, auth_mode_str(item->auth));

  at_wifi_puts(str);
}  

void mcu_hw_wifi_scan(void) {
  debugf("WiFi: Performing scan");

  static wifi_mgmr_scan_params_t config;
  /* duration in microseconds for which channel is scanned, default 220000 */
  config.duration = 220000;

  memset(&config, 0, sizeof(wifi_mgmr_scan_params_t));
  if (0 != wifi_mgmr_sta_scan(&config)) {
    at_wifi_puts("Scan failed\r\n");
    return;
  }

  at_wifi_puts("Scanning...\r\n"); 
  wait4event(1, 1);

  if (0 != wifi_mgmr_scan_ap_all(NULL, NULL, wifi_scan_item_cb)) {
    at_wifi_puts("Scan all failed\r\n");
  };
}

static void wifi_info()
{
    ip4_addr_t ip, gw, mask, dns;
    char str[64];
    char str_tmp[20];

    wifi_sta_ip4_addr_get(&ip.addr, &mask.addr, &gw.addr, &dns.addr);

    ip4addr_ntoa_r((ip4_addr_t *) &ip.addr, str_tmp, sizeof(str_tmp));
    debugf("IP  :%s", str_tmp);
    snprintf(str, sizeof(str), "IP  :%s \r\n", str_tmp);
    at_wifi_puts(str);

    ip4addr_ntoa_r((ip4_addr_t *) &mask.addr, str_tmp, sizeof(str_tmp));
    debugf("MASK:%s", str_tmp);
    snprintf(str, sizeof(str), "MASK:%s \r\n", str_tmp);
    at_wifi_puts(str);
    
    ip4addr_ntoa_r((ip4_addr_t *) &gw.addr, str_tmp, sizeof(str_tmp));
    debugf("GW  :%s", str_tmp);
    snprintf(str, sizeof(str), "GW  :%s \r\n", str_tmp);
    at_wifi_puts(str);

    ip4addr_ntoa_r((ip4_addr_t *) &dns.addr, str_tmp, sizeof(str_tmp));
    debugf("DNS  :%s", str_tmp);
    snprintf(str, sizeof(str), "DNS :%s \r\n", str_tmp);
    at_wifi_puts(str);

}

void mcu_hw_wifi_connect(char *ssid, char *key) {
  debugf("WiFI: connect to %s/%s", ssid, key);
  
  at_wifi_puts("WiFI: Connecting...");
  if(wifi_ssid) free(wifi_ssid);
  if(wifi_key) free(wifi_key);

  if (wifi_mgmr_sta_state_get() == 1) {
    wifi_sta_disconnect();
  }

  // store ssid/key for retry
  wifi_ssid = strdup(ssid);
  wifi_key = strdup(key);
  
  s_retry_num = 0;
  if (0 != wifi_mgmr_sta_quickconnect(wifi_ssid, wifi_key, 0, 0)) {
    debugf("\r\nWiFI: STA failed!");
  } else {
    wait4event(4, 4);
    if (wifi_mgmr_sta_state_get() == 1 ) {
      at_wifi_puts("\r\nWiFI: Connected\r\n");
      wifi_info();
    } else {
      debugf("\r\nWiFI: Connection failed!");
      at_wifi_puts("\r\nWiFI: Connection failed!\r\n");
      }
    }
}

static struct tcp_pcb *tcp_pcb = NULL;

static err_t mcu_tcp_connected( __attribute__((unused)) void *arg, __attribute__((unused)) struct tcp_pcb *tpcb, err_t err) {
  if (err != ERR_OK) {
    debugf("connect failed %d", err);
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

  if (p->tot_len > 0) {
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
  
  err_t err = tcp_connect(tcp_pcb, ipaddr, port, mcu_tcp_connected);

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
  
  int err = dns_gethostbyname(host, &address, dns_found, &lport);

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
    err_t err = tcp_write(tcp_pcb, &byte, 1, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) debugf("Failed to write data %d", err);

    return true;
  }
    
  return false;  // data has not been processed (we are not connected)
}

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

/* ========================================================================= */
/* ======                              JTAG                           ====== */
/* ========================================================================= */


static bool jtag_is_active = false;

#ifdef ENABLE_JTAG
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
  gpio = bflb_device_get_by_name("gpio");
  spi_dev = bflb_device_get_by_name("spi0");

  if((dir & 0x0f) == 0x00 && (data == 0x00)) {
    mcu_hw_fpga_resume_spi();  // release JTAG and resume SPI operation
  } else  
  // check if the pin direction pattern matches JTAG mode
  if((dir & 0x0f) == 0x0b) {
    debugf("SPI deinit and JTAG activation");

   stop_hid();
   vTaskDelay(pdMS_TO_TICKS(250));

  #ifndef TANG_NANO20K
    bflb_gpio_deinit(gpio, SPI_PIN_MISO);
    bflb_gpio_deinit(gpio, SPI_PIN_MOSI);
    bflb_gpio_deinit(gpio, SPI_PIN_SCK);
    bflb_gpio_deinit(gpio, SPI_PIN_CSN);
#endif
    bflb_gpio_irq_detach(SPI_PIN_IRQ);

    bflb_gpio_deinit(gpio, PIN_JTAG_TCK);
    bflb_gpio_deinit(gpio, PIN_JTAG_TDI);
    bflb_gpio_deinit(gpio, PIN_JTAG_TMS);
    bflb_gpio_deinit(gpio, PIN_JTAG_TDO);

    bflb_gpio_init(gpio, PIN_JTAG_TCK, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
    bflb_gpio_init(gpio, PIN_JTAG_TDO, GPIO_INPUT  | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
    bflb_gpio_init(gpio, PIN_JTAG_TDI, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
    bflb_gpio_init(gpio, PIN_JTAG_TMS, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
#ifndef TANG_NANO20K
    bflb_gpio_set(gpio, PIN_JTAGSEL); // select JTAG mode
#endif
    
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
    uint32_t lidcode;
    mcu_hw_jtag_data(NULL, (uint8_t*)&lidcode, 32);
    
    // finally return into Test-Logic-Reset state.
    mcu_hw_jtag_tms(1, 0b11111, 5);
  
    jtag_debugf("IDCODE = %08lx", lidcode);

    // anything bit all 1's or all 0's indicates that JTAG seems to
    // be working
    if((lidcode == 0xffffffff) || (lidcode == 0x00000000)) {
      jtag_highlight_debugf("JTAG doesn't seem to work. Forcing non-flash reconfig");
      mcu_hw_fpga_reconfig(false);
      return;
    }
    jtag_is_active = true;
  } else
    jtag_is_active = false;
}

// send up to 8 TMS bits with a given fixed TDI state
uint8_t mcu_hw_jtag_tms(uint8_t tdi, uint8_t data, int len) {
  int dlen = len & 7;
  uint8_t mask = 1;
  uint8_t rx = 0;

  if(tdi) bflb_gpio_set(gpio, PIN_JTAG_TDI); else bflb_gpio_reset(gpio, PIN_JTAG_TDI);

  while(len--) {
    if ((data & mask)?1:0) bflb_gpio_set(gpio, PIN_JTAG_TMS); else bflb_gpio_reset(gpio, PIN_JTAG_TMS);
    bflb_gpio_set(gpio, PIN_JTAG_TCK);

#ifdef DEBUG_TAP
    jtag_tap_advance_state((data & mask)?1:0, tdi);
#ifdef DEBUG_JTAG
    jtag_debugf("TMS %d TDI %d TDO %d -> %s", (data & mask)?1:0, tdi, bflb_gpio_read(gpio,PIN_JTAG_TDO),
    jtag_tap_state_name());
#endif
#else
#ifdef DEBUG_JTAG
    jtag_debugf("TMS %d TDI %d TDO %d", (data & mask)?1:0, tdi, bflb_gpio_read(gpio, PIN_JTAG_TDO));
#endif
#endif
    
    if(bflb_gpio_read(gpio, PIN_JTAG_TDO)) rx |= mask;
    bflb_gpio_reset(gpio, PIN_JTAG_TCK);

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
  bflb_gpio_reset(gpio, PIN_JTAG_TMS);
  
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
      if (*txd & 0x01) bflb_gpio_set(gpio, PIN_JTAG_TDI); else bflb_gpio_reset(gpio, PIN_JTAG_TDI);
      bflb_gpio_set(gpio, PIN_JTAG_TCK); bflb_gpio_reset(gpio, PIN_JTAG_TCK);
      if (*txd & 0x02) bflb_gpio_set(gpio, PIN_JTAG_TDI); else bflb_gpio_reset(gpio, PIN_JTAG_TDI);
      bflb_gpio_set(gpio, PIN_JTAG_TCK); bflb_gpio_reset(gpio, PIN_JTAG_TCK);
      if (*txd & 0x04) bflb_gpio_set(gpio, PIN_JTAG_TDI); else bflb_gpio_reset(gpio, PIN_JTAG_TDI);
      bflb_gpio_set(gpio, PIN_JTAG_TCK); bflb_gpio_reset(gpio, PIN_JTAG_TCK);
      if (*txd & 0x08) bflb_gpio_set(gpio, PIN_JTAG_TDI); else bflb_gpio_reset(gpio, PIN_JTAG_TDI);
      bflb_gpio_set(gpio, PIN_JTAG_TCK); bflb_gpio_reset(gpio, PIN_JTAG_TCK);
      if (*txd & 0x10) bflb_gpio_set(gpio, PIN_JTAG_TDI); else bflb_gpio_reset(gpio, PIN_JTAG_TDI);
      bflb_gpio_set(gpio, PIN_JTAG_TCK); bflb_gpio_reset(gpio, PIN_JTAG_TCK);
      if (*txd & 0x20) bflb_gpio_set(gpio, PIN_JTAG_TDI); else bflb_gpio_reset(gpio, PIN_JTAG_TDI);
      bflb_gpio_set(gpio, PIN_JTAG_TCK); bflb_gpio_reset(gpio, PIN_JTAG_TCK);
      if (*txd & 0x40) bflb_gpio_set(gpio, PIN_JTAG_TDI); else bflb_gpio_reset(gpio, PIN_JTAG_TDI);
      bflb_gpio_set(gpio, PIN_JTAG_TCK); bflb_gpio_reset(gpio, PIN_JTAG_TCK);
      if (*txd & 0x80) bflb_gpio_set(gpio, PIN_JTAG_TDI); else bflb_gpio_reset(gpio, PIN_JTAG_TDI);
      bflb_gpio_set(gpio, PIN_JTAG_TCK); bflb_gpio_reset(gpio, PIN_JTAG_TCK);
      txd++;
    }
  } else {  
    while(len) {
      // send 1 of nothing was given
      int tx_bit = txd?((*txd & mask)?1:0):1;
      // set data bit and clock tck at once
      if (tx_bit) bflb_gpio_set(gpio, PIN_JTAG_TDI); else bflb_gpio_reset(gpio, PIN_JTAG_TDI);
      bflb_gpio_set(gpio, PIN_JTAG_TCK);
      
#ifdef DEBUG_TAP
      jtag_tap_advance_state(0, tx_bit);
#endif
      
#ifdef DEBUG_JTAG
      jtag_debugf("TMS 0 TDI %d TDO %d", tx_bit, bflb_gpio_read(gpio, PIN_JTAG_TDO));
#endif

      if(rxd) {
	// shift in from lsb
	if(bflb_gpio_read(gpio, PIN_JTAG_TDO)) *rxd |=  mask;
  else                       *rxd &= ~mask;
      }
      bflb_gpio_reset(gpio, PIN_JTAG_TCK);

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
  if(dlen && rxd) {
    // jtag_highlight_debugf("last byte %02x, rshift = %d", *rxd, dlen);
    *rxd <<= 8-dlen;
  }
}

#ifdef TANG_NANO20K
#define JTAG_TDI_BIT  (12)
#define JTAG_TCK_BIT  (10)
#define JTAG_TMS_BIT  (16)
#else
#define JTAG_TDI_BIT  (3)
#define JTAG_TCK_BIT  (1)
#define JTAG_TMS_BIT  (0)
#endif

// These macros write TDI, TCK and TMS in one step
#define JTAG_DATA_BIT(byte, bit)  (((bit) > JTAG_TDI_BIT)?(((byte) & (1<<(bit))) >> ((bit)-JTAG_TDI_BIT)):(((byte) & (1<<(bit))) << (JTAG_TDI_BIT-(bit))))
#define JTAG_WRITE_BIT(byte, bit, tms) {				\
    *reg_gpio0_31 = JTAG_DATA_BIT((byte),(bit)) | ((tms)?(1<<JTAG_TMS_BIT):0);	\
    *reg_gpio0_31 = JTAG_DATA_BIT((byte),(bit)) | (1<<JTAG_TCK_BIT) | ((tms)?(1<<JTAG_TMS_BIT):0); }

void mcu_hw_jtag_writeTDI_msb_first_gpio_out_mode(uint8_t *tx, unsigned int bytes, bool end) {
	for (int i = 0; i < bytes; i++) {
		uint8_t byte = tx[i];		
		JTAG_WRITE_BIT(byte, 7, 0);
		JTAG_WRITE_BIT(byte, 6, 0);
		JTAG_WRITE_BIT(byte, 5, 0);
		JTAG_WRITE_BIT(byte, 4, 0);
		JTAG_WRITE_BIT(byte, 3, 0);
		JTAG_WRITE_BIT(byte, 2, 0);
		JTAG_WRITE_BIT(byte, 1, 0);
		JTAG_WRITE_BIT(byte, 0, i == bytes-1 && end);    // set TMS on last bit
	}
}

static void mcu_hw_jtag_init(void) {
  // -------- init FPGA control pins ---------
}

void mcu_hw_fpga_reconfig(bool run) {
  // trigger FPGA reconfiguration

  if(!jtag_open()) {
    jtag_debugf("FPGA not detected");
  } else {
    gowin_fpgaReset(); // JTAG-based reset/reconfig
   }
  jtag_close();
}

void mcu_hw_fpga_resume_spi(void) {
  gpio = bflb_device_get_by_name("gpio");
  debugf("disable JTAG and resume SPI");

  bflb_gpio_deinit(gpio, PIN_JTAG_TCK);
  bflb_gpio_deinit(gpio, PIN_JTAG_TDI);
  bflb_gpio_deinit(gpio, PIN_JTAG_TMS);
  bflb_gpio_deinit(gpio, PIN_JTAG_TDO);

#ifndef TANG_NANO20K
  bflb_gpio_init(gpio, SPI_PIN_MISO, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_init(gpio, SPI_PIN_MOSI, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_init(gpio, SPI_PIN_SCK, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_init(gpio, SPI_PIN_CSN, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_set(gpio, SPI_PIN_CSN);

  bflb_gpio_reset(gpio, PIN_JTAGSEL);
#endif

#ifdef TANG_CONSOLE60K
  struct bflb_device_s *sdh;
  sdh = bflb_device_get_by_name("sdh");

  bflb_sdh_sta_int_en(sdh, 0xffffffff, false);
  GLB_AHB_MCU_Software_Reset(GLB_AHB_MCU_SW_EXT_SDH);
#endif
  jtag_is_active = false;
  bflb_gpio_irq_attach(SPI_PIN_IRQ, spi_isr);
}

void mcu_hw_jtag_toggleClk(uint32_t clk_len)
{
  mcu_hw_jtag_enter_gpio_out_mode();

  for (uint32_t i = 0; i < clk_len; i++) {
    *reg_gpio0_31 = *reg_gpio0_31 & (0xffffffff ^ (1 << 10)); // TCK=0
    *reg_gpio0_31 = *reg_gpio0_31 | (1 << 10); // TCK=1
  }

  mcu_hw_jtag_exit_gpio_out_mode();
}

#endif

// USB host MSC support
static usb_osal_thread_t usbh_msc_handle = NULL;
static bool usb_msc_mounted = false;
extern void fatfs_sdh_driver_register(void);

static void usbh_msc_thread(CONFIG_USB_OSAL_THREAD_SET_ARGV)
{
  int ret;
  struct usbh_msc *msc_class = (struct usbh_msc *)CONFIG_USB_OSAL_THREAD_GET_ARGV;

  while ((msc_class = (struct usbh_msc *)usbh_find_class_instance("/dev/sda")) == NULL) {
      goto delete;
  }

  ret = usbh_msc_scsi_init(msc_class);
  if (ret < 0) {
      sdc_debugf("scsi_init error,ret:%d", ret);
      goto delete;
  }

  fatfs_usbh_driver_register(msc_class);

  usb_msc_mounted = true;
  menu_notify(MENU_EVENT_USB_MOUNTED);

    // clang-format off
delete: 
    usb_osal_thread_delete(NULL);
    // clang-format on
}

void usbh_msc_run(struct usbh_msc *msc_class)
{
  usbh_msc_handle = usb_osal_thread_create("usbh_msc", 2048, CONFIG_USBHOST_PSC_PRIO - 1, usbh_msc_thread, msc_class);
}

void usbh_msc_stop(struct usbh_msc *msc_class)
{
  menu_notify(MENU_EVENT_USB_UMOUNTED);
  usb_msc_mounted = false;
}

#ifdef CONFIG_BFLOG
__attribute__((weak)) uint64_t bflog_clock(void)
{
    return bflb_mtimer_get_time_us();
}

__attribute__((weak)) uint32_t bflog_time(void)
{
    return BFLB_RTC_TIME2SEC(bflb_rtc_get_time(rtc));
}

__attribute__((weak)) char *bflog_thread(void)
{
    return "";
}
#endif

#ifdef CONFIG_FATFS
#include "bflb_timestamp.h"
__attribute__((weak)) uint32_t get_fattime(void)
{
    bflb_timestamp_t tm;

    bflb_timestamp_utc2time(BFLB_RTC_TIME2SEC(bflb_rtc_get_time(rtc)), &tm);

    return ((uint32_t)(tm.year - 1980) << 25) /* Year 2015 */
           | ((uint32_t)tm.mon << 21)         /* Month 1 */
           | ((uint32_t)tm.mday << 16)        /* Mday 1 */
           | ((uint32_t)tm.hour << 11)        /* Hour 0 */
           | ((uint32_t)tm.min << 5)          /* Min 0 */
           | ((uint32_t)tm.sec >> 1);         /* Sec 0 */
}
#endif

#ifndef CONFIG_CONSOLE_WO
  SHELL_CMD_EXPORT_ALIAS(lsusb, lsusb, ls usb);
#endif

void mcu_hw_upload_core(char *name) {
  debugf("Request to upload core %s", name);
#ifdef ENABLE_JTAG
  uint64_t start;
  FATFS fs;
  FRESULT res = FR_NOT_READY;

  if (strstr(name, "/sd/") != NULL) {
#if defined(TANG_CONSOLE60K) || defined(TANG_MEGA60K)
    f_mount(NULL, "/sd", 1);
    sdc_direct_init();
    start = bflb_mtimer_get_time_ms();
    while ((res = f_mount(&fs, "/sd", 1)) != FR_OK && bflb_mtimer_get_time_ms() - start < 500)
      bflb_mtimer_delay_ms(100);
    if (res == FR_OK) {
        sdc_debugf("SD card mounted");
        gowin_upload_core(name);
        f_mount(NULL, "/sd", 1);
      } else {
        sdc_debugf("SD card not found...");
    }
    sdc_direct_release();
#endif
  } else {
      f_mount(NULL, "/usb", 1);
      start = bflb_mtimer_get_time_ms();
      while ((res = f_mount(&fs, "/usb", 1)) != FR_OK && bflb_mtimer_get_time_ms() - start < 1000)
        bflb_mtimer_delay_ms(100);
      if (res != FR_OK) {
          debugf("failed to mount USB drive");
      } else {
          debugf("USB drive mounted");
	  gowin_upload_core(name);
          f_mount(NULL, "/usb", 1);
        }
  }
#else
  fatal_debugf("JTAG not enabled!");
#endif // ENABLE_JTAG
}

bool mcu_hw_usb_msc_present(void) {
  return usb_msc_mounted;
}

// M0S_DOCK
/* GPIO 21 default UART TX */
/* GPIO 22 default UART RX */

// TANG_NANO20K
/* GPIO 11 default UART TX */
/* GPIO 13 default UART RX */

// TANG_CONSOLE60K
/* GPIO 27 default UART RX, FPGA U15 TX */
/* GPIO 28 default UART TX, FPGA V15 RX */
/* GPIO 29 default TWI.SDA, FPGA L13 DDC DAT */
/* GPIO 30 default TWI.SCL, FPGA M13 DDC CLK */
/* GPIO 21 USB-C SBU1 */
/* GPIO 22 USB-C SBU2 */

// TANG_MEGA138KPRO
/* GPIO 10 default UART TX, FPGA N16, RX */
/* GPIO 11 default UART RX, FPGA P15, TX */
/* GPIO 27 default PLL1_TWI SDA, FPGA K26, SDA */
/* GPIO 28 default PLL1_TWI SCL, FPGA K25, SCL */

// TANG_MEGA60K
/* GPIO 27 default UART RX, FPGA U15 TX */
/* GPIO 28 default UART TX, FPGA V15 RX */
/* GPIO 16 PWR_KEY, no FPGA connection, re-use possible */
/* GPIO 17 BL616_IO17_ModeSel, no FPGA connection, re-use possible */
/* GPIO 20 I2C INT, no FPGA connection */
/* GPIO 21 I2C SDA, no FPGA connection */
/* GPIO 22 I2C CLK, no FPGA connection, re-use possible */

// TANG_PRIMER25K
/* GPIO 11 default UART TX */
/* GPIO 10 default UART RX */
/* GPIO 12 access at button S3, Capacitor C22 need to be removed */
/* GPIO 20 access at LED6, not usable for UART, unknown reason */

