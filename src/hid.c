// hid.c

#include "hid.h"
#include "debug.h"
#include "sysctrl.h"
#include "osd.h"
#include "menu.h"

#include "inifile.h"
#include "mcu_hw.h"

#include <string.h>  // for memcpy

// keep a map of joysticks to be able to report
// them individually
static uint8_t joystick_map = 0;

uint8_t hid_allocate_joystick(void) {
  uint8_t idx;
  for(idx=0;joystick_map & (1<<idx);idx++);
  joystick_map |= (1<<idx);
  usb_debugf("Allocating joystick %d (map = %02x)", idx, joystick_map);
  return idx;
}

void hid_release_joystick(uint8_t idx) {
  joystick_map &= ~(1<<idx);
  usb_debugf("Releasing joystick %d (map = %02x)", idx, joystick_map);
}
  
static void kbd_tx(uint8_t byte) {
  mcu_hw_spi_begin();
  mcu_hw_spi_tx_u08(SPI_TARGET_HID);
  mcu_hw_spi_tx_u08(SPI_HID_KEYBOARD);
  mcu_hw_spi_tx_u08(byte);
  mcu_hw_spi_end();
}

void kbd_parse(__attribute__((unused)) const hid_report_t *report, struct hid_kbd_state_S *state,
	       const unsigned char *buffer, int nbytes) {
  // we expect boot mode packets which are exactly 8 bytes long
  if(nbytes != 8) return;
  
  // check if modifier have changed
  if((buffer[0] != state->last_report[0]) && !osd_is_visible()) {
    for(int i=0;i<8;i++) {
      // modifier keys map to key codes 0x68+
      
      // modifier released?
      if((state->last_report[0] & (1<<i)) && !(buffer[0] & (1<<i)))
	kbd_tx(0x80 | (i+0x68));
      // modifier pressed?
      if(!(state->last_report[0] & (1<<i)) && (buffer[0] & (1<<i)))
	kbd_tx(i+0x68);
    }
  } 
  
  // check if regular keys have changed
  for(int i=0;i<6;i++) {
    if(buffer[2+i] != state->last_report[2+i]) {
      // key released?
      if(state->last_report[2+i]) {
	if(!osd_is_visible() ) {
	  // check if the reported key is the OSD activation hotkey
	  // and suppress reporting it to the core
	  if(state->last_report[2+i] != inifile_option_get(INIFILE_OPTION_HOTKEY))
	    kbd_tx(0x80 | state->last_report[2+i]);
	} else
	  menu_notify(MENU_EVENT_KEY_RELEASE);
      }
      
      // key pressed?
      if(buffer[2+i])  {
	static unsigned long msg;
	msg = 0;
	
	// F12 toggles the OSD state. Therefore F12 must never be forwarded
	// to the core and thus must have an empty entry in the keymap. ESC
	// can only close the OSD. This is now configurable via INIFILE_OPTION_HOTKEY

	// Caution: Since the OSD closes on the press event, the following
	// release event will be sent into the core. The core should thus
	// cope with release events that did not have a press event before
	if(buffer[2+i] == inifile_option_get(INIFILE_OPTION_HOTKEY))
	  msg = osd_is_visible()?MENU_EVENT_HIDE:MENU_EVENT_SHOW;
	else if(osd_is_visible() && buffer[2+i] == 0x29 /* ESC key */ )
 	  msg = MENU_EVENT_BACK;
	else {
	  if(!osd_is_visible())
	    kbd_tx(buffer[2+i]);
	  else {
	    // check if cursor up/down or space has been pressed
	    if(buffer[2+i] == 0x51) msg = MENU_EVENT_DOWN;      
	    if(buffer[2+i] == 0x52) msg = MENU_EVENT_UP;
	    if(buffer[2+i] == 0x4e) msg = MENU_EVENT_PGDOWN;      
	    if(buffer[2+i] == 0x4b) msg = MENU_EVENT_PGUP;
	    if((buffer[2+i] == 0x2c) || (buffer[2+i] == 0x28))
	      msg = MENU_EVENT_SELECT;
	  }
	}

	// send message to menu task
	if(msg) menu_notify(msg);
      }   
    }
  }
  memcpy(state->last_report, buffer, 8);
}

// collect bits from byte stream and assemble them into a signed word
static uint16_t collect_bits(const uint8_t *p, uint16_t offset, uint8_t size, bool is_signed) {
  // mask unused bits of first byte
  uint8_t mask = 0xff << (offset&7);
  uint8_t byte = offset/8;
  uint8_t bits = size;
  uint8_t shift = offset&7;
  
  //  iusb_debugf("0 m:%x by:%d bi=%d sh=%d ->", mask, byte, bits, shift);
  uint16_t rval = (p[byte++] & mask) >> shift;
  mask = 0xff;
  shift = 8-shift;
  bits -= shift;
  
  // first byte already contained more bits than we need
  if(shift > size) {
    // mask unused bits
    rval &= (1<<size)-1;
  } else {
    // further bytes if required
    while(bits) {
      mask = (bits<8)?(0xff>>(8-bits)):0xff;
      rval += (p[byte++] & mask) << shift;
      shift += 8;
      bits -= (bits>8)?8:bits;
    }
  }
  
  if(is_signed) {
    // do sign expansion
    uint16_t sign_bit = 1<<(size-1);
    if(rval & sign_bit) {
      while(sign_bit) {
	rval |= sign_bit;
	sign_bit <<= 1;
      }
    }
  }
  
  return rval;
}

void mouse_parse(const hid_report_t *report, __attribute__((unused)) struct hid_mouse_state_S *state,
		 const unsigned char *buffer, int nbytes) {
  // we expect at least three bytes:
  if(nbytes < 3) return;
  
  // collect info about the two axes
  int a[2];
  for(int i=0;i<2;i++) {  
    bool is_signed = report->joystick_mouse.axis[i].logical.min > 
      report->joystick_mouse.axis[i].logical.max;

    a[i] = collect_bits(buffer, report->joystick_mouse.axis[i].offset, 
			report->joystick_mouse.axis[i].size, is_signed);
  }

  // ... and two buttons
  uint8_t btns = 0;
  for(int i=0;i<2;i++)
    if(buffer[report->joystick_mouse.button[i].byte_offset] & 
       report->joystick_mouse.button[i].bitmask)
      btns |= (1<<i);

  mcu_hw_spi_begin();
  mcu_hw_spi_tx_u08(SPI_TARGET_HID);
  mcu_hw_spi_tx_u08(SPI_HID_MOUSE);
  mcu_hw_spi_tx_u08(btns);
  mcu_hw_spi_tx_u08(a[0]);
  mcu_hw_spi_tx_u08(a[1]);
  mcu_hw_spi_end();
}

void joystick_parse(const hid_report_t *report, struct hid_joystick_state_S *state,
		    const unsigned char *buffer, __attribute__((unused)) int nbytes) {
  //  usb_debugf("joystick: %d %02x %02x %02x %02x", nbytes,
  //  	 buffer[0]&0xff, buffer[1]&0xff, buffer[2]&0xff, buffer[3]&0xff);

  // collect info about the two axes
  int a[2];
  for(int i=0;i<2;i++) {  
    bool is_signed = report->joystick_mouse.axis[i].logical.min > 
      report->joystick_mouse.axis[i].logical.max;
    
    a[i] = collect_bits(buffer, report->joystick_mouse.axis[i].offset, 
			report->joystick_mouse.axis[i].size, is_signed);
  }

  // ... and four buttons
  unsigned char joy = 0;
  for(int i=0;i<4;i++)
    if(buffer[report->joystick_mouse.button[i].byte_offset] & 
       report->joystick_mouse.button[i].bitmask)
      joy |= (0x10<<i);

  // ... and the eight extra buttons
  unsigned char btn_extra = 0;
  for(int i=4;i<12;i++)
    if(buffer[report->joystick_mouse.button[i].byte_offset] & 
      report->joystick_mouse.button[i].bitmask) 
      btn_extra |= (1<<(i-4));

  // map directions to digital
  if(a[0] > 0xc0) joy |= 0x01;
  if(a[0] < 0x40) joy |= 0x02;
  if(a[1] > 0xc0) joy |= 0x04;
  if(a[1] < 0x40) joy |= 0x08;

  int ax = a[0];
  int ay = a[1];

  if((joy != state->last_state) || 
     (ax != state->last_state_x) || 
     (ay != state->last_state_y) || 
     (btn_extra != state->last_state_btn_extra))  {
    state->last_state = joy;
    state->last_state_x = ax;
    state->last_state_y = ay;
    state->last_state_btn_extra = btn_extra;
    usb_debugf("JOY%d: D %02x X %02x Y %02x EB %02x", state->js_index, joy, ax, ay, btn_extra);

    mcu_hw_spi_begin();
    mcu_hw_spi_tx_u08(SPI_TARGET_HID);
    mcu_hw_spi_tx_u08(SPI_HID_JOYSTICK);
    mcu_hw_spi_tx_u08(state->js_index);
    mcu_hw_spi_tx_u08(joy);
    mcu_hw_spi_tx_u08(ax); // e.g. gamepad X
    mcu_hw_spi_tx_u08(ay); // e.g. gamepad Y
    mcu_hw_spi_tx_u08(btn_extra); // e.g. gamepad extra buttons
    mcu_hw_spi_end();
  }
}

void rii_joy_parse(const unsigned char *buffer) {
  unsigned char b = 0;
  if(buffer[0] == 0xcd && buffer[1] == 0x00) b = 0x10;      // cd == play/pause  -> center
  if(buffer[0] == 0xe9 && buffer[1] == 0x00) b = 0x08;      // e9 == V+          -> up
  if(buffer[0] == 0xea && buffer[1] == 0x00) b = 0x04;      // ea == V-          -> down
  if(buffer[0] == 0xb6 && buffer[1] == 0x00) b = 0x02;      // b6 == skip prev   -> left
  if(buffer[0] == 0xb5 && buffer[1] == 0x00) b = 0x01;      // b5 == skip next   -> right

  usb_debugf("RII Joy: %02x %02x", 0, b);
  
  mcu_hw_spi_begin();
  mcu_hw_spi_tx_u08(SPI_TARGET_HID);
  mcu_hw_spi_tx_u08(SPI_HID_JOYSTICK);
  mcu_hw_spi_tx_u08(0);  // Rii joystick always report as joystick 0
  mcu_hw_spi_tx_u08(b);
  mcu_hw_spi_tx_u08(0);  // analog X
  mcu_hw_spi_tx_u08(0);  // analog Y
  mcu_hw_spi_tx_u08(0);  // extra buttons
  mcu_hw_spi_end();
}

void hid_parse(const hid_report_t *report, hid_state_t *state, uint8_t const* data, uint16_t len) {
  //  usb_debugf("hid parse %d, expect %d", len, report->report_size);
  if(!len) return;
  
  // hexdump((void*)data, len);

  // the following is a hack for the Rii keyboard/touch combos to use the
  // left top multimedia pad as a joystick. These special keys are sent
  // via the mouse/touchpad part
  if(report->report_id_present &&
     report->type == REPORT_TYPE_MOUSE &&
     len == 3 &&
     data[0] != report->report_id) {
    rii_joy_parse(data+1);
    return;
  }

  // check and skip report id if present
  if(report->report_id_present && (len-1 == report->report_size)) {
    if(data[0] != report->report_id) {
      usb_debugf("FAIL %d != %d", data[0], report->report_id);
      return;
    }
        
    // skip report id
    data++; len--;
  }
  
  if(len == report->report_size) {
    if(report->type == REPORT_TYPE_KEYBOARD)
      kbd_parse(report, &state->kbd, data, len);
    
    if(report->type == REPORT_TYPE_MOUSE)
      mouse_parse(report, &state->mouse, data, len);
    
    if(report->type == REPORT_TYPE_JOYSTICK)
      joystick_parse(report, &state->joystick, data, len);
  }
}

// hid event triggered by FPGA
void hid_handle_event(void) {
  mcu_hw_spi_begin();
  mcu_hw_spi_tx_u08(SPI_TARGET_HID);
  mcu_hw_spi_tx_u08(SPI_HID_GET_DB9);
  mcu_hw_spi_tx_u08(0x00);
  uint8_t db9 = mcu_hw_spi_tx_u08(0x00);
  mcu_hw_spi_end();

  debugf("DB9: %02x", db9);
}
