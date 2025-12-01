/*
  mcu_hw.c - bl616 hardware driver
*/

#include <FreeRTOS.h>
#include "mem.h"
#include "shell.h"
#include "semphr.h"

#include "usbh_core.h"
#include "usbh_hid.h"
#include "usbd_core.h"

#include "../spi.h"
#include "../hid.h"
#include "../sdc.h"
#include "../sysctrl.h"
#include "../debug.h"
#include "../mcu_hw.h"

#include "bl616_glb.h"

#include "bflb_mtimer.h"
#include "bflb_spi.h"
#include "bflb_dma.h"
#include "bflb_gpio.h"
#include "bflb_wdg.h"
#include "bflb_uart.h"
#include "bflb_clock.h"
#include "bflb_flash.h"
#include <lwip/tcpip.h>
#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#if __has_include("bl_fw_api.h")
#include "bl_fw_api.h"        // old SDK 2.0, buggy and WIFI not working
#else
#include "export/bl_fw_api.h" // SDK 2.0.1 WIFI working fine
#undef __INLINE
#undef __PACKED
#include <fhost.h>
#endif
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

extern uint32_t __HeapBase;
extern uint32_t __HeapLimit;

#ifdef TANG_CONSOLE60K
#warning "Building for TANG_CONSOLE60K internal BL616"
#elif TANG_NANO20K
#warning "Building for TANG_NANO20K internal BL616"
#elif M0S_DOCK
#warning "Building for M0S DOCK BL616"
#elif TANG_MEGA138KPRO
#warning "Building for TANG_MEGA138KPRO internal BL616"
#elif TANG_MEGA60K
#warning "Building for TANG_MEGA60K internal BL616"
#elif TANG_PRIMER25K
#warning "Building for TANG_PRIMER25K internal BL616"
#endif

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

#define XINPUT_GAMEPAD_DPAD_UP 0x0001
#define XINPUT_GAMEPAD_DPAD_DOWN 0x0002
#define XINPUT_GAMEPAD_DPAD_LEFT 0x0004
#define XINPUT_GAMEPAD_DPAD_RIGHT 0x0008
#define XINPUT_GAMEPAD_START 0x0010
#define XINPUT_GAMEPAD_BACK 0x0020
#define XINPUT_GAMEPAD_LEFT_SHOULDER 0x0100
#define XINPUT_GAMEPAD_RIGHT_SHOULDER 0x0200

static struct bflb_device_s *gpio;
extern void shell_init_with_task(struct bflb_device_s *shell);

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
    struct usb_config *usb;
    SemaphoreHandle_t sem;
    TaskHandle_t task_handle;    
    unsigned char last_state;
    unsigned char js_index;
    unsigned char last_state_btn_extra;
    int16_t last_state_x;
    int16_t last_state_y;
    #ifdef RATE_CHECK
    TickType_t rate_start;
    unsigned long rate_events;
#endif    
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

static void usbh_update(struct usb_config *usb) {
  // check for active hid devices
  for(int i=0;i<CONFIG_USBHOST_MAX_HID_CLASS;i++) {
    char *dev_str = "/dev/inputX";
    dev_str[10] = '0' + i;
    usb->hid_info[i].class = (struct usbh_hid *)usbh_find_class_instance(dev_str);
    
    if(usb->hid_info[i].class && usb->hid_info[i].state == STATE_NONE) {
      usb_debugf("NEW HID %d", i);

      usb_debugf("Interval: %d", usb->hid_info[i].class->hport->config.intf[i].altsetting[0].ep[0].ep_desc.bInterval);
	 
      usb_debugf("Interface %d", usb->hid_info[i].class->intf);
      usb_debugf("  class %d", usb->hid_info[i].class->hport->config.intf[i].altsetting[0].intf_desc.bInterfaceClass);
      usb_debugf("  subclass %d", usb->hid_info[i].class->hport->config.intf[i].altsetting[0].intf_desc.bInterfaceSubClass);
      usb_debugf("  protocol %d", usb->hid_info[i].class->hport->config.intf[i].altsetting[0].intf_desc.bInterfaceProtocol);
      int rep_desc = usbh_hid_get_report_descriptor(usb->hid_info[i].class, report_desc[i], 128);
      if (rep_desc < 0) {
        usb_debugf("usbh_hid_get_report_descriptor issue");}
      bool skip = false;
      uint16_t vendor_id = usb->hid_info[i].class->hport->device_desc.idVendor;
      uint16_t product_id = usb->hid_info[i].class->hport->device_desc.idProduct;
      if (vendor_id == 0x2dc8 && product_id == 0x3107) {  // 8bitdo wireless adapter
          skip = true;
      }
      // parse report descriptor ...
      usb_debugf("report descriptor: %p", report_desc[i]);
      if(skip || !parse_report_descriptor(report_desc[i], 128, &usb->hid_info[i].report, NULL)) {
	usb->hid_info[i].state = STATE_FAILED;   // parsing failed, don't use
	return;
      }
      
      usb->hid_info[i].state = STATE_DETECTED;
    }
    
    else if(!usb->hid_info[i].class && usb->hid_info[i].state != STATE_NONE) {
      usb_debugf("HID LOST %d", i);
      vTaskDelete( usb->hid_info[i].task_handle );
      usb->hid_info[i].state = STATE_NONE;

      if(usb->hid_info[i].report.type == REPORT_TYPE_JOYSTICK) {
	usb_debugf("Joystick %d gone", usb->hid_info[i].hid_state.joystick.js_index);
	hid_release_joystick(usb->hid_info[i].hid_state.joystick.js_index);
      }
    }
  }

  // check for active xbox devices
  for(int i=0;i<CONFIG_USBHOST_MAX_XBOX_CLASS;i++) {
    char *dev_str = "/dev/xboxX";
    dev_str[9] = '0' + i;
    usb->xbox_info[i].class = (struct usbh_hid *)usbh_find_class_instance(dev_str);
    
    if(usb->xbox_info[i].class && usb->xbox_info[i].state == STATE_NONE) {
      usb_debugf("NEW XBOX %d", i);

      usb_debugf("Interval: %d", usb->xbox_info[i].class->hport->config.intf[i].altsetting[0].ep[0].ep_desc.bInterval);
	 
      usb_debugf("Interface %d", usb->xbox_info[i].class->intf);
      usb_debugf("  class %d", usb->xbox_info[i].class->hport->config.intf[i].altsetting[0].intf_desc.bInterfaceClass);
      usb_debugf("  subclass %d", usb->xbox_info[i].class->hport->config.intf[i].altsetting[0].intf_desc.bInterfaceSubClass);
      usb_debugf("  protocol %d", usb->xbox_info[i].class->hport->config.intf[i].altsetting[0].intf_desc.bInterfaceProtocol);
	
      usb->xbox_info[i].state = STATE_DETECTED;
    }
    
    else if(!usb->xbox_info[i].class && usb->xbox_info[i].state != STATE_NONE) {
      usb_debugf("XBOX %d", i);
      vTaskDelete( usb->xbox_info[i].task_handle );
      usb->xbox_info[i].state = STATE_NONE;
      
      usb_debugf("Joystick %d gone", usb->xbox_info[i].js_index);
      hid_release_joystick(usb->xbox_info[i].js_index);
    }
  }

  // check for number of mice and keyboards and update leds
  int mice = 0, keyboards = 0;  
  for(int i=0;i<CONFIG_USBHOST_MAX_HID_CLASS;i++) {
    if(usb->hid_info[i].state == STATE_RUNNING) {
      if(usb->hid_info[i].report.type == REPORT_TYPE_MOUSE)    mice++;
      if(usb->hid_info[i].report.type == REPORT_TYPE_KEYBOARD) keyboards++;      
    }
  }

  extern void set_led(int pin, int on);
#ifdef M0S_DOCK
  set_led(GPIO_PIN_27, mice);
  set_led(GPIO_PIN_28, keyboards);
#endif
}

static void xbox_parse(struct xbox_info_S *xbox) {
#if 0
  USB_LOG_RAW("XBOX%d: ", xbox->index);
  
  // just dump the report
  for (size_t i = 0; i < 20; i++) 
    USB_LOG_RAW("0x%02x ", xbox->buffer[i]);
  USB_LOG_RAW("");
#endif

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

// each HID client gets itws own thread which submits urbs
// and waits for the interrupt to succeed
static void usbh_hid_client_thread(void *argument) {
  struct hid_info_S *hid = (struct hid_info_S *)argument;

  usb_debugf("HID client #%d: thread started", hid->index);

  while(1) {
    int ret = usbh_submit_urb(&hid->class->intin_urb);
    if (ret < 0)
      usb_debugf("HID client #%d: submit failed", hid->index);
    else {
      // Wait for result
      xSemaphoreTake(hid->sem, 0xffffffffUL);
      if(hid->nbytes > 0)
	hid_parse(&hid->report, &hid->hid_state, hid->buffer, hid->nbytes);
      
      hid->nbytes = 0;
    }      

#ifdef RATE_CHECK
    hid->rate_events++;
    if(!(hid->rate_events % 100)) {
     float ms_since_start = (xTaskGetTickCount() - hid->rate_start) * portTICK_PERIOD_MS;
     usb_debugf("Rate = %f events/sec",  1000 * hid->rate_events /  ms_since_start);
    }    
#endif
  }
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
      usbh_bulk_urb_fill(&xbox->class->intout_urb,
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

  usb_debugf("XBOX client #%d: thread started", xbox->index);

    // Send initialization packets
    for (int i = 0; i < CONFIG_USBHOST_MAX_XBOX_CLASS; i++) {
      if ((ret = usbh_control_transfer(xbox->class->hport, &xbox_init_packets[i], xbox->buffer)) < 0) {
        usb_debugf("XBOX: init packet %d failed: %d", i, ret);
      }
  }
  xbox_init(xbox);
  usb_debugf("XBOX client #%d: all init packets sent, entering main loop.\n", xbox->index);

    // setup urb
    usbh_int_urb_fill(&xbox->class->intin_urb, xbox->class->hport, 
      xbox->class->intin, xbox->buffer, XBOX_REPORT_SIZE,
      50, usbh_xbox_callback, xbox);

  while(1) {
    int ret = usbh_submit_urb(&xbox->class->intin_urb);
    if (ret < 0)
      usb_debugf("XBOX client #%d: submit failed", xbox->index);
    else {
      // Wait for result
      xSemaphoreTake(xbox->sem, 0xffffffffUL);
      if(xbox->nbytes == XBOX_REPORT_SIZE)
        xbox_parse(xbox);
      xbox->nbytes = 0;
    }      

#ifdef RATE_CHECK
    xbox->rate_events++;
    if(!(xbox->rate_events % 100)) {
     float ms_since_start = (xTaskGetTickCount() - xbox->rate_start) * portTICK_PERIOD_MS;
     usb_debugf("Rate = %f events/sec",  1000 * xbox->rate_events /  ms_since_start);
    }    
#endif
  }
}

static void usbh_hid_thread(void *argument) {
  usb_debugf("Starting usb host task...");

  struct usb_config *usb = (struct usb_config *)argument;

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

  while (1) {
    usbh_update(usb);

    for(int i=0;i<CONFIG_USBHOST_MAX_HID_CLASS;i++) {
      if(usb->hid_info[i].state == STATE_DETECTED) {
	usb_debugf("NEW HID device %d", i);
	usb->hid_info[i].state = STATE_RUNNING; 

	if( usb->hid_info[i].report.type == REPORT_TYPE_JOYSTICK ) {	
	  usb->hid_info[i].hid_state.joystick.js_index = hid_allocate_joystick();
	  usb_debugf("  -> joystick %d", usb->hid_info[i].hid_state.joystick.js_index);
	}
	  
#if 0
	// set report protocol 1 if subclass != BOOT_INTF
	// CherryUSB doesn't report the InterfaceSubClass (HID_BOOT_INTF_SUBCLASS)
	// we thus set boot protocol on keyboards
	if( usb->hid_info[i].report.type == REPORT_TYPE_KEYBOARD ) {	
	  // /* 0x0 = boot protocol, 0x1 = report protocol */
	  usb_debugf("setting boot protocol");
	  ret = usbh_hid_set_protocol(usb->hid_info[i].class, HID_PROTOCOL_BOOT);
	  if (ret < 0) {
	    usb_debugf("failed");
	    usb->hid_info[i].state = STATE_FAILED;  // failed
	    continue;
	  }
	}
#endif

	// setup urb
	usbh_int_urb_fill(&usb->hid_info[i].class->intin_urb,
			  usb->hid_info[i].class->hport,
			  usb->hid_info[i].class->intin, usb->hid_info[i].buffer,
			  usb->hid_info[i].report.report_size + (usb->hid_info[i].report.report_id_present ? 1:0),
			  0, usbh_hid_callback, &usb->hid_info[i]);

#ifdef RATE_CHECK
	usb->hid_info[i].rate_start = xTaskGetTickCount();
	usb->hid_info[i].rate_events = 0;
#endif
	
	// start a new thread for the new device
	xTaskCreate(usbh_hid_client_thread, (char *)"hid_task", 1024,
		    &usb->hid_info[i], configMAX_PRIORITIES-3, &usb->hid_info[i].task_handle );
      }
    }
    
    for(int i=0;i<CONFIG_USBHOST_MAX_XBOX_CLASS;i++) {
      if(usb->xbox_info[i].state == STATE_DETECTED) {
	usb_debugf("NEW XBOX device %d", i);
	usb->xbox_info[i].state = STATE_RUNNING; 

	// search for free joystick slot
	usb->xbox_info[i].js_index = hid_allocate_joystick();
	usb_debugf("  -> joystick %d", usb->xbox_info[i].js_index);
	
	// setup urb
	usbh_int_urb_fill(&usb->xbox_info[i].class->intin_urb,
			  usb->xbox_info[i].class->hport,
			  usb->xbox_info[i].class->intin, usb->xbox_info[i].buffer,
			  XBOX_REPORT_SIZE,
			  0, usbh_xbox_callback, &usb->xbox_info[i]);
	
#ifdef RATE_CHECK
	usb->xbox_info[i].rate_start = xTaskGetTickCount();
	usb->xbox_info[i].rate_events = 0;
#endif

	// start a new thread for the new device
	xTaskCreate(usbh_xbox_client_thread, (char *)"xbox_task", 2048,
		    &usb->xbox_info[i], configMAX_PRIORITIES-3, &usb->xbox_info[i].task_handle );
      }
    }

    // this thread only handles new devices and thus doesn't have to run very
    // often
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void usb_host(void) {
  TaskHandle_t usb_handle;

  usb_debugf("init usb hid host");

  usbh_initialize(0, USB_BASE, NULL);

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
  xTaskCreate(usbh_hid_thread, (char *)"usb_task", 2048, &usb_config, configMAX_PRIORITIES-3, &usb_handle);
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
  #define SPI_PIN_MISO  GPIO_PIN_2 /* in  TDO, CHIP_EN */
  #define SPI_PIN_MOSI  GPIO_PIN_3 /* out TDI */
  #define SPI_PIN_IRQ   GPIO_PIN_11/* in  UART RX, crossed */
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
    .freq = 20000000,   // 20MHz
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
extern void bl_show_flashinfo(void);
extern void bl_show_log(void);
extern void bl_show_component_version(void);
extern void bflb_uart_set_console(struct bflb_device_s *dev);

// the M0S uses max bitrate as I use another M0S for console debug which can
// deal with 2MBit/s
#define CONSOLE_BAUDRATE 2000000

static struct bflb_device_s *uart0;

#if (defined(CONFIG_LUA) || defined(CONFIG_BFLOG) || defined(CONFIG_FATFS))
static struct bflb_device_s *rtc;
#endif

static void system_clock_init(void) {
  /* wifipll/audiopll */

#if TANG_MEGA138KPRO
  GLB_Power_On_XTAL_And_PLL_CLK(GLB_XTAL_26M, GLB_PLL_WIFIPLL | GLB_PLL_AUPLL);
#elif TANG_PRIMER25K
  GLB_Power_On_XTAL_And_PLL_CLK(GLB_XTAL_26M, GLB_PLL_WIFIPLL | GLB_PLL_AUPLL);
#elif TANG_MEGA60K
  GLB_Power_On_XTAL_And_PLL_CLK(GLB_XTAL_26M, GLB_PLL_WIFIPLL | GLB_PLL_AUPLL);
#else
  GLB_Power_On_XTAL_And_PLL_CLK(GLB_XTAL_40M, GLB_PLL_WIFIPLL | GLB_PLL_AUPLL);
#endif
  GLB_Set_MCU_System_CLK(GLB_MCU_SYS_CLK_TOP_WIFIPLL_320M);
  HBN_Set_MCU_XCLK_Sel(HBN_MCU_XCLK_XTAL);
  CPU_Set_MTimer_CLK(ENABLE, BL_MTIMER_SOURCE_CLOCK_MCU_XCLK, Clock_System_Clock_Get(BL_SYSTEM_CLOCK_XCLK) / 1000000 - 1);
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

#if defined(CONFIG_ANTI_ROLLBACK) && !defined(CONFIG_BOOT2)
extern const blverinf_t app_ver;
uint8_t efuse_version = 0xFF;

static void bflb_check_anti_rollback(void)
{
    if (0 != bflb_get_app_version_from_efuse(&efuse_version)) {
        printf("error! can't read app version in efuse\r\n");
        while (1) {}
    } else {
        printf("app version in efuse is: %d\r\n", efuse_version);
    }

    if (app_ver.anti_rollback < efuse_version) {
        printf("app version in application is: %d, less than app version in efuse, the application should not run up\r\n", app_ver.anti_rollback);
    } else {
        printf("app version in application is: %d, not less than app version in efuse, the application should run up\r\n", app_ver.anti_rollback);
    }

    /* change app version in efuse to app_ver.anti_rollback, default is 0 */
    if (app_ver.anti_rollback > efuse_version) {
        bflb_set_app_version_to_efuse(app_ver.anti_rollback); //be attention! app version in efuse is incremental(from 0 to 128), and cannot be reduced forever
        printf("update app version in efuse to %d\r\n", app_ver.anti_rollback);

        /* check app version in efuse */
        if (0 != bflb_get_app_version_from_efuse(&efuse_version)) {
            printf("error! can't read app version in efuse\r\n");
            while (1) {}
        } else {
            printf("app version in efuse is: %d\r\n", efuse_version);
        }
    }
}
#endif

static void console_init() {
  gpio = bflb_device_get_by_name("gpio");
  
#ifdef M0S_DOCK
  /* M0S Dock has debug uart on default pins 21 and 22 */
  bflb_gpio_uart_init(gpio, GPIO_PIN_21, GPIO_UART_FUNC_UART0_TX);
  bflb_gpio_uart_init(gpio, GPIO_PIN_22, GPIO_UART_FUNC_UART0_RX);
#elif TANG_NANO20K
  bflb_gpio_uart_init(gpio, GPIO_PIN_11, GPIO_UART_FUNC_UART0_TX);
  bflb_gpio_uart_init(gpio, GPIO_PIN_14, GPIO_UART_FUNC_UART0_RX); /* 8 TDO */
  /* GPIO 14, FPGA 8 TDO */
#elif TANG_CONSOLE60K
  bflb_gpio_uart_init(gpio, GPIO_PIN_28, GPIO_UART_FUNC_UART0_TX);
  bflb_gpio_uart_init(gpio, GPIO_PIN_30, GPIO_UART_FUNC_UART0_RX); /* M13 TWI.SCL */
  /* GPIO 27 default UART RX, FPGA U15 TX */
  /* GPIO 28 default UART TX, FPGA V15 RX */
  /* GPIO 29 default TWI.SDA, FPGA L13 DDC DAT */
  /* GPIO 30 default TWI.SCL, FPGA M13 DDC CLK */
#elif TANG_MEGA138KPRO
  bflb_gpio_uart_init(gpio, GPIO_PIN_28, GPIO_UART_FUNC_UART0_TX); /* K25 PLL1_TWI SCL */
  bflb_gpio_uart_init(gpio, GPIO_PIN_27, GPIO_UART_FUNC_UART0_RX); /* K26 PLL1_TWI SDA */
  /* GPIO 10 default UART TX, FPGA N16, RX */
  /* GPIO 11 default UART RX, FPGA P15, TX */
  /* GPIO 27 default PLL1_TWI SDA, FPGA K26, SDA */
  /* GPIO 28 default PLL1_TWI SCL, FPGA K25, SCL */
#elif TANG_MEGA60K
  /* RX no FPGA connection available, adhoc wiring needed */
  bflb_gpio_uart_init(gpio, GPIO_PIN_28, GPIO_UART_FUNC_UART0_TX);
  bflb_gpio_uart_init(gpio, GPIO_PIN_30, GPIO_UART_FUNC_UART0_RX);
/* GPIO 17 BL616_IO17_ModeSel, no FPGA connection, 31004 assembly,  */
/* GPIO 27 default UART RX, FPGA U15 TX */
/* GPIO 28 default UART TX, FPGA V15 RX */
/* GPIO 30 default TWI.SCL, FPGA M13 DDC CLK, only 31005 assembly */
#elif TANG_PRIMER25K
  bflb_gpio_uart_init(gpio, GPIO_PIN_11, GPIO_UART_FUNC_UART0_TX);
  bflb_gpio_uart_init(gpio, GPIO_PIN_12, GPIO_UART_FUNC_UART0_RX);
  /* GPIO 12 access at S3 button, remove C22 capacitor */
#endif

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
}

static void wifi_init(void);

// local board_init used as a replacemement for global board_init
static void mn_board_init(void) {
    int ret = -1;
    uintptr_t flag;
    size_t heap_len;

    /* lock */
    flag = bflb_irq_save();
    ret = bflb_flash_init();

    /* system clock */
    system_clock_init();

    peripheral_clock_init();

    /* irq init */
    bflb_irq_initialize();

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

#ifdef TANG_CONSOLE60K
  /* USB-C OTG Power enable */
    bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
    bflb_gpio_set(gpio, GPIO_PIN_20);
#elif TANG_MEGA60K
  /* IPS5310 power enable */
    //bflb_gpio_init(gpio, GPIO_PIN_16, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
    //bflb_gpio_set(gpio, GPIO_PIN_16);
#endif

    /* console init (uart or wo) */
    console_init();

    heap_len = ((size_t)&__HeapLimit - (size_t)&__HeapBase);
    kmem_init((void *)&__HeapBase, heap_len);

    /* boot info dump */
    bl_show_log();
    /* version info dump */
    bl_show_component_version();

#if defined(CONFIG_ANTI_ROLLBACK) && !defined(CONFIG_BOOT2)
    bflb_check_anti_rollback();
#endif

  /* flash info dump */
  bl_show_flashinfo();
  if (ret != 0) {
        printf("flash init fail !!!\r\n");
      }

    printf("uart  sig1:%08x, sig2:%08x\r\n", getreg32(GLB_BASE + GLB_UART_CFG1_OFFSET), getreg32(GLB_BASE + GLB_UART_CFG2_OFFSET));
    printf("clock gen1:%08x, gen2:%08x\r\n", getreg32(GLB_BASE + GLB_CGEN_CFG1_OFFSET), getreg32(GLB_BASE + GLB_CGEN_CFG2_OFFSET));

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

    printf("board init done\r\n");
    printf("===========================\r\n");
}

void mcu_hw_init(void) {
  mn_board_init();

  printf("\r\n\r\n" LOGO "           FPGA Companion for BL616\r\n\r\n");

  gpio = bflb_device_get_by_name("gpio");

#ifdef M0S_DOCK
  // init on-board LEDs
  bflb_gpio_init(gpio, GPIO_PIN_27, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
  bflb_gpio_init(gpio, GPIO_PIN_28, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
  
  // both leds off
  bflb_gpio_set(gpio, GPIO_PIN_27);
  bflb_gpio_set(gpio, GPIO_PIN_28);
#elif TANG_MEGA138KPRO
  /* LED6 enable */
  bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_reset(gpio, GPIO_PIN_20);
#elif TANG_PRIMER25K
  /* LED5 enable */
  bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_reset(gpio, GPIO_PIN_20);
#endif
  mcu_hw_spi_init();

  uart0 = bflb_device_get_by_name("uart0");
  shell_init_with_task(uart0);

#ifdef M0S_DOCK
  wifi_init();
#elif TANG_CONSOLE60K
  wifi_init();
#elif TANG_MEGA60K
  wifi_init();
#endif
  usb_host();

}

extern void hid_keyboard_init(uint8_t busid, uintptr_t reg_base);

void mcu_hw_reset(void) {
  debugf("HW reset");

  struct bflb_device_s *wdg;
  struct bflb_wdg_config_s wdg_cfg;
  wdg_cfg.clock_source = WDG_CLKSRC_32K;
  wdg_cfg.clock_div = 31;
  wdg_cfg.comp_val = 1000;
  wdg_cfg.mode = WDG_MODE_RESET;

  wdg = bflb_device_get_by_name("watchdog");
  bflb_wdg_init(wdg, &wdg_cfg);
  bflb_wdg_start(wdg); // to be sure...

  gpio = bflb_device_get_by_name("gpio");
  bflb_irq_disable(gpio->irq_num);
  bflb_gpio_deinit(gpio, GPIO_PIN_0);
  bflb_gpio_deinit(gpio, GPIO_PIN_1);
  bflb_gpio_deinit(gpio, GPIO_PIN_2);
  bflb_gpio_deinit(gpio, GPIO_PIN_3);
  bflb_gpio_init(gpio, GPIO_PIN_2, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_reset(gpio, GPIO_PIN_2);
  debugf("usb host deinit");
  usbh_deinitialize(0);
  bflb_mtimer_delay_ms(250);

#ifdef TANG_CONSOLE60K
  /* USB-C OTG Power enable */
  bflb_gpio_deinit(gpio, GPIO_PIN_20);
  bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
  bflb_gpio_reset(gpio, GPIO_PIN_20);
#endif

  debugf("start usb hid device mode");
  hid_keyboard_init(0, USB_BASE);
  debugf("deinit done and ready for POR reset");
  bflb_mtimer_delay_ms(100);
  GLB_SW_POR_Reset();
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
//static volatile unsigned int petsc2;

static char *wifi_ssid = NULL;
static char *wifi_key = NULL;
static int s_retry_num = 0;
static wifi_conf_t conf = { .country_code = "CN" }; // "CN","US","JP","EU"
static QueueHandle_t wifi_event_queue;

void wifi_event_handler(uint32_t code) {
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

static TaskHandle_t wifi_fw_task;

static void wifi_init(void) {
  wifi_event_queue = xQueueCreate(10, sizeof(char));

  if (0 != rfparam_init(0, NULL, 0)) {
    debugf("PHY RF init failed!");
    return;
  }
  
  debugf("PHY RF init success!");

  // petsc2 = 0;
  tcpip_init(NULL, NULL);
  
  /* enable wifi clock */
  GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_IP_WIFI_PHY | GLB_AHB_CLOCK_IP_WIFI_MAC_PHY | GLB_AHB_CLOCK_IP_WIFI_PLATFORM);
  GLB_AHB_MCU_Software_Reset(GLB_AHB_MCU_SW_WIFI);

  /* Enable wifi irq */
  extern void interrupt0_handler(void);
  bflb_irq_attach(WIFI_IRQn, (irq_callback)interrupt0_handler, NULL);
  bflb_irq_enable(WIFI_IRQn);

  xTaskCreate(wifi_main, (char *)"fw", WIFI_STACK_SIZE, NULL, TASK_PRIORITY_FW, &wifi_fw_task);
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
    debugf("IP  :%s \r\n", str_tmp);
    snprintf(str, sizeof(str), "IP  :%s \r\n", str_tmp);
    at_wifi_puts(str);

    ip4addr_ntoa_r((ip4_addr_t *) &mask.addr, str_tmp, sizeof(str_tmp));
    debugf("MASK:%s \r\n", str_tmp);
    snprintf(str, sizeof(str), "MASK:%s \r\n", str_tmp);
    at_wifi_puts(str);
    
    ip4addr_ntoa_r((ip4_addr_t *) &gw.addr, str_tmp, sizeof(str_tmp));
    debugf("GW  :%s \r\n", str_tmp);
    snprintf(str, sizeof(str), "GW  :%s \r\n", str_tmp);
    at_wifi_puts(str);

    ip4addr_ntoa_r((ip4_addr_t *) &dns.addr, str_tmp, sizeof(str_tmp));
    debugf("DNS  :%s \r\n", str_tmp);
    snprintf(str, sizeof(str), "DNS :%s \r\n", str_tmp);
    at_wifi_puts(str);

}

void mcu_hw_wifi_connect(char *ssid, char *key) {
/*
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
*/
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
    debugf("\r\nWiFI: STA failed!\r\n");
  } else {
    wait4event(4, 4);
    if (wifi_mgmr_sta_state_get() == 1 ) {
      at_wifi_puts("\r\nWiFI: Connected\r\n");
      wifi_info();
    } else {
      debugf("\r\nWiFI: Connection failed!\r\n");
      at_wifi_puts("\r\nWiFI: Connection failed!\r\n");
      }
    }
}

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

  char str[74];
  snprintf(str, sizeof(str), "\r\nconnecting to host: %s port: %d \r\n", host, port);
  at_wifi_puts(str);
/*
  if (petsc2 == 1) {
    int len = strlen(host);
    for (int i = 0; i < len; i++) {
        host[i] = pet2asc(host[i]);
    }
  }
*/
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

SHELL_CMD_EXPORT_ALIAS(lsusb, lsusb, ls usb);

