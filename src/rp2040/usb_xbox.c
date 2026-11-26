/*
  rp2040/usb_xbox.c
*/

#include "board.h"
#ifdef ENABLE_USB_XBOX
 
#include <stdint.h>
#include "usb_xbox.h"
#include "xinput_host.h"
#include "../hid.h"
#include "../usb_controller_maps_tools.h"

static struct {
  uint8_t dev_addr;
  uint8_t instance;
  uint8_t js_index;
  uint8_t state;
  uint8_t state_x;
  uint8_t state_y;
  uint8_t state_btn_extra;

  const UsbGamepadMap *map;
  bool map_found;
  bool map_checked;
} xbox_state[MAX_XBOX_DEVICES];

void usb_xbox_init(void) {
  for(int i=0;i<MAX_XBOX_DEVICES;i++)
    xbox_state[i].dev_addr = 0xff;
}

/* ======================================================================= */
/* ======                   XBOX controllers                     ========= */
/* ======================================================================= */

static uint8_t byteScaleAnalog(int16_t xbox_val) {
  // Scale the xbox value from [-32768, 32767] to [1, 255]
  // Offset by 32768 to get in range [0, 65536], then divide by 256 to get in range [1, 255]
  uint8_t scale_val = (xbox_val + 32768) / 256;
  if (scale_val == 0) return 1;
  return scale_val;
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
	    ((p->wButtons & XINPUT_GAMEPAD_BACK           )?0x04:0x00) |
	    ((p->wButtons & XINPUT_GAMEPAD_START          )?0x08:0x00) |
      ((p->wButtons & XINPUT_GAMEPAD_LEFT_THUMB     )?0x40:0x00) |
      ((p->wButtons & XINPUT_GAMEPAD_RIGHT_THUMB    )?0x80:0x00);
	  
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
	    xbox_state[idx].state_x = ax;
	    xbox_state[idx].state_y = ay;

	    hid_joystick_push(xbox_state[idx].js_index, state, ax, ay, state_btn_extra);
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

    tusb_desc_device_t device;
    if(tuh_descriptor_get_device_local(dev_addr, &device))  {
      // lookup for VID/PID/Version
      const void *map = usb_controller_maps_find(device.idVendor, device.idProduct, device.bcdDevice);
    
      if (map) {
	usb_controller_maps_show(map);
      
	xbox_state[idx].map = map;
	xbox_state[idx].map_found = 1;
	xbox_state[idx].map_checked = 1;
      } else {
	usb_debugf("No map for VID=%04x PID=%04x, VERSION=%04x",
		   device.idVendor, device.idProduct, device.bcdDevice);
	xbox_state[idx].map = NULL;
	xbox_state[idx].map_found = 0;
	xbox_state[idx].map_checked = 1;
      }    
    }
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

  mcu_hw_usb_check_devices();
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
  mcu_hw_usb_check_devices();
}

int usb_xbox_present(void) {
  int retval = 0;
  
  for(int idx=0;idx<MAX_XBOX_DEVICES;idx++)
    if(xbox_state[idx].dev_addr != 0xff)
      retval++;

  return retval;
}

#endif
