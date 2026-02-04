// hid.c

#include "hid.h"
#include "debug.h"
#include "sysctrl.h"
#include "osd.h"
#include "menu.h"

#include "inifile.h"
#include "mcu_hw.h"

#include <string.h>  // for memcpy
#include "usb_controller_maps.h"

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

void get_keyboard_change(unsigned char* a, unsigned char* b, int start ,int length, unsigned char* res) {
  int i1, i2;
  int res_len = 0;

  for (i1 = start; i1 < start + length; i1++) {
    if (a[i1] == 0) continue;
    unsigned char temp = a[i1];
    char found = 0;
    for (i2 = start; i2 < start + length; i2++) {
      if (b[i2] == 0) continue;
      if (b[i2] == temp) {found = 1; break;}
    }
    if (found == 0) {
      res[res_len] = temp;
      res_len++;
    }
  };
  res[6] = res_len;
}

// new keyboard parser 
void newkbd_parse(__attribute__((unused)) const hid_report_t *report, struct hid_kbd_state_S *state,
               const unsigned char *buffer, int nbytes)
{
  static const unsigned char keyboard_change_defaults[2][7] = {{0,0,0,0,0,0,0},{0,0,0,0,0,0,0}};
  unsigned char keyboard_change[2][7] = {{0,0,0,0,0,0,0},{0,0,0,0,0,0,0}};

  // we expect boot mode packets which are exactly 8 bytes long
  if (nbytes != 8)
    return;
  usb_debugf("keyboard: %d %02x %02x %02x %02x", nbytes,
             buffer[0] & 0xff, buffer[1] & 0xff, buffer[2] & 0xff, buffer[3] & 0xff);

  // check if modifier have changed
  if ((buffer[0] != state->last_report[0]) && !osd_is_visible())
  {
    for (int i = 0; i < 8; i++)
     // modifier keys map to key codes 0x68+
    {
//   if (core_map_modifier_key(i))
      {
        // modifier released?
        if ((state->last_report[0] & (1 << i)) && !(buffer[0] & (1 << i)))
//      kbd_tx(0x80 | core_map_modifier_key(i));
        kbd_tx(0x80 | (i+0x68));
        // modifier pressed?
        if (!(state->last_report[0] & (1 << i)) && (buffer[0] & (1 << i)))
//      kbd_tx(core_map_modifier_key(i));
        kbd_tx(i+0x68);
      }
    }
  }

  // check if regular keys have changed
  // get sets of keys released and pressed
  memcpy(keyboard_change, keyboard_change_defaults, sizeof(keyboard_change_defaults));
  get_keyboard_change(state->last_report, buffer, 2, 6, keyboard_change[0]); //released keys
  get_keyboard_change(buffer, state->last_report, 2, 6, keyboard_change[1]); //pressed keys

  // released keys
  if (keyboard_change[0][6] > 0) {
      usb_debugf("Keys released (%d): %d, %d, %d, %d", keyboard_change[0][6], keyboard_change[0][0], keyboard_change[0][1], keyboard_change[0][2], keyboard_change[0][3]);
      for(int i = 0; i < keyboard_change[0][6]; i++) {
        // key released
        if (!osd_is_visible())
        {
          // check if the reported key is the OSD activation hotkey
          // and suppress reporting it to the core
          if (keyboard_change[0][i] != inifile_option_get(INIFILE_OPTION_HOTKEY))
//        kbd_tx(0x80 | core_map_key(keyboard_change[0][i]));
          kbd_tx(0x80 | (0x3f & keyboard_change[0][i]));
        }
        else
          menu_notify(MENU_EVENT_KEY_RELEASE);
      }
  }

  // pressed keys
  if (keyboard_change[1][6] > 0) {
    usb_debugf("Keys pressed (%d): %d, %d, %d, %d", keyboard_change[1][6], keyboard_change[1][0], keyboard_change[1][1], keyboard_change[1][2], keyboard_change[1][3]);
    for(int i = 0; i<keyboard_change[1][6]; i++) {
      
        static unsigned long msg;
        msg = 0;

        // F12 toggles the OSD state. Therefore F12 must never be forwarded
        // to the core and thus must have an empty entry in the keymap. ESC
        // can only close the OSD. This is now configurable via INIFILE_OPTION_HOTKEY

        // Caution: Since the OSD closes on the press event, the following
        // release event will be sent into the core. The core should thus
        // cope with release events that did not have a press event before
        if (keyboard_change[1][i] == inifile_option_get(INIFILE_OPTION_HOTKEY))
          msg = osd_is_visible() ? MENU_EVENT_HIDE : MENU_EVENT_SHOW;
        else if (osd_is_visible() && keyboard_change[1][i] == 0x29 /* ESC key */)
          msg = MENU_EVENT_BACK;
        else
        {
          if (!osd_is_visible())
//        kbd_tx(core_map_key(keyboard_change[1][i]));
          kbd_tx(0x3f & keyboard_change[1][i]);
          else
          {
            // check if cursor up/down or space has been pressed
            if (keyboard_change[1][i] == 0x51)
              msg = MENU_EVENT_DOWN;
            if (keyboard_change[1][i] == 0x52)
              msg = MENU_EVENT_UP;
            if (keyboard_change[1][i] == 0x4e)
              msg = MENU_EVENT_PGDOWN;
            if (keyboard_change[1][i] == 0x4b)
              msg = MENU_EVENT_PGUP;
            if ((keyboard_change[1][i] == 0x2c) || (keyboard_change[1][i] == 0x28))
              msg = MENU_EVENT_SELECT;
          }
        }

        // send message to menu task
        if (msg)
          menu_notify(msg);
    }   
  }

  memcpy(state->last_report, buffer, 8);
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
	if(buffer[2+i] == inifile_option_get(INIFILE_OPTION_HOTKEY)) {
	  // for now, shift-F12 activates the system menu
	  if(buffer[0] & 0x22) msg = MENU_EVENT_SYSTEM;
	  else	  msg = osd_is_visible()?MENU_EVENT_HIDE:MENU_EVENT_SHOW;
	} else if(osd_is_visible() && buffer[2+i] == 0x29 /* ESC key */ )
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

// directions in 'joy' 
#define DIR_RIGHT 0x01
#define DIR_LEFT 0x02
#define DIR_DOWN 0x04
#define DIR_UP 0x08

  // --------------------------------------------------------------------
  // HAT: read and map
  // --------------------------------------------------------------------
  unsigned char hat_dir = 0;

  // HAT present ???
  if (report->joystick_mouse.hat.size > 0)
  {
    // HAT is unsigned
    int hat_raw = collect_bits(buffer,
                               report->joystick_mouse.hat.offset,
                               report->joystick_mouse.hat.size,
                               /*is_signed=*/false);

    int lmin = report->joystick_mouse.hat.logical.min;
    int lmax = report->joystick_mouse.hat.logical.max;

    int steps = (lmax >= lmin) ? (lmax - lmin + 1) : 0;
    int v = hat_raw - lmin;

    bool neutral = false;
    if (steps >= 8)
    {
      // 8-directional HAT
      if (v < 0 || v > 7)
        neutral = true;
    }
    else if (steps >= 4)
    {
      // 4-directional HAT
      if (v < 0 || v > 3)
        neutral = true;
    }
    else
    {
      neutral = true;
    }

    if (!neutral)
    {
      if (steps >= 8)
      {
        // 8-way: 0:N, 1:NE, 2:E, 3:SE, 4:S, 5:SW, 6:W, 7:NW
        switch (v)
        {
        case 0:
          hat_dir |= DIR_UP;
          break;
        case 1:
          hat_dir |= (DIR_UP | DIR_RIGHT);
          break;
        case 2:
          hat_dir |= DIR_RIGHT;
          break;
        case 3:
          hat_dir |= (DIR_DOWN | DIR_RIGHT);
          break;
        case 4:
          hat_dir |= DIR_DOWN;
          break;
        case 5:
          hat_dir |= (DIR_DOWN | DIR_LEFT);
          break;
        case 6:
          hat_dir |= DIR_LEFT;
          break;
        case 7:
          hat_dir |= (DIR_UP | DIR_LEFT);
          break;
        }
      }
      else
      {
        // 4-way: 0:N, 1:E, 2:S, 3:W
        switch (v)
        {
        case 0:
          hat_dir |= DIR_UP;
          break;
        case 1:
          hat_dir |= DIR_RIGHT;
          break;
        case 2:
          hat_dir |= DIR_DOWN;
          break;
        case 3:
          hat_dir |= DIR_LEFT;
          break;
        }
      }

      // FIX: give HAT a priority
      joy &= ~(DIR_RIGHT | DIR_LEFT | DIR_DOWN | DIR_UP);
      joy |= hat_dir;
    }

    usb_debugf("HAT raw=%d norm=%d steps=%d dir=0x%02x%s",
               hat_raw, v, steps, hat_dir, neutral ? " (neutral)" : "");
  }

  if((joy != state->last_state) ||
     (ax != state->last_state_x) || 
     (ay != state->last_state_y) || 
     (btn_extra != state->last_state_btn_extra))  {
    state->last_state = joy;
    state->last_state_x = ax;
    state->last_state_y = ay;
    state->last_state_btn_extra = btn_extra;
    usb_debugf("JOY%d: D %02x X %02x Y %02x EB %02x", state->js_index, joy, ax, ay, btn_extra);

    if(osd_is_visible()) {	       
      // if OSD is visible, then process events locally
      menu_joystick_state(joy);	      
    } else {      
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

#define DIR_RIGHT 0x01
#define DIR_LEFT 0x02
#define DIR_DOWN 0x04
#define DIR_UP 0x08

#define AX_HIGH 0xC0
#define AX_LOW 0x40

void parse_with_sdl_mapping(const hid_report_t *report,
                            struct hid_joystick_state_S *state,
                            const unsigned char *buffer, int nbytes,
                            const UsbGamepadMap *map)
{
  (void)nbytes;

#define READ_BUTTON_IDX(idx_) \
  ((idx_) < 0 ? 0 : ((buffer[report->joystick_mouse.button[(idx_)].byte_offset] & report->joystick_mouse.button[(idx_)].bitmask) ? 1 : 0))

#define READ_AXIS_U8(idx_, out_u8_)                                        \
  do                                                                       \
  {                                                                        \
    int tmp_;                                                              \
    if ((idx_) < 0)                                                        \
    {                                                                      \
      (out_u8_) = 0x80; /* neutral fallback */                             \
    }                                                                      \
    else                                                                   \
    {                                                                      \
      bool _is_signed = report->joystick_mouse.axis[(idx_)].logical.min >  \
                        report->joystick_mouse.axis[(idx_)].logical.max;   \
      tmp_ = (int)collect_bits(buffer,                                     \
                               report->joystick_mouse.axis[(idx_)].offset, \
                               report->joystick_mouse.axis[(idx_)].size,   \
                               _is_signed);                                \
      if (tmp_ < 0)                                                        \
        tmp_ = 0;                                                          \
      if (tmp_ > 255)                                                      \
        tmp_ = 255;                                                        \
      (out_u8_) = (uint8_t)tmp_;                                           \
    }                                                                      \
  } while (0)

#define READ_HAT_DIR(out_bits_)                                          \
  do                                                                     \
  {                                                                      \
    (out_bits_) = 0;                                                     \
    if (report->joystick_mouse.hat.size > 0)                             \
    {                                                                    \
      int hat_raw = (int)collect_bits(buffer,                            \
                                      report->joystick_mouse.hat.offset, \
                                      report->joystick_mouse.hat.size,   \
                                      /*is_signed=*/false);              \
      int lmin = report->joystick_mouse.hat.logical.min;                 \
      int lmax = report->joystick_mouse.hat.logical.max;                 \
      int steps = (lmax >= lmin) ? (lmax - lmin + 1) : 0;                \
      int v = hat_raw - lmin;                                            \
      int neutral = 0;                                                   \
      if (steps >= 8)                                                    \
      {                                                                  \
        neutral = (v < 0 || v > 7);                                      \
      }                                                                  \
      else if (steps >= 4)                                               \
      {                                                                  \
        neutral = (v < 0 || v > 3);                                      \
      }                                                                  \
      else                                                               \
      {                                                                  \
        neutral = 1;                                                     \
      }                                                                  \
      if (!neutral)                                                      \
      {                                                                  \
        if (steps >= 8)                                                  \
        {                                                                \
          switch (v)                                                     \
          {                                                              \
          case 0:                                                        \
            (out_bits_) |= DIR_UP;                                       \
            break;                                                       \
          case 1:                                                        \
            (out_bits_) |= (DIR_UP | DIR_RIGHT);                         \
            break;                                                       \
          case 2:                                                        \
            (out_bits_) |= DIR_RIGHT;                                    \
            break;                                                       \
          case 3:                                                        \
            (out_bits_) |= (DIR_RIGHT | DIR_DOWN);                       \
            break;                                                       \
          case 4:                                                        \
            (out_bits_) |= DIR_DOWN;                                     \
            break;                                                       \
          case 5:                                                        \
            (out_bits_) |= (DIR_DOWN | DIR_LEFT);                        \
            break;                                                       \
          case 6:                                                        \
            (out_bits_) |= DIR_LEFT;                                     \
            break;                                                       \
          case 7:                                                        \
            (out_bits_) |= (DIR_LEFT | DIR_UP);                          \
            break;                                                       \
          }                                                              \
        }                                                                \
        else                                                             \
        {                                                                \
          switch (v & 3)                                                 \
          {                                                              \
          case 0:                                                        \
            (out_bits_) |= DIR_UP;                                       \
            break;                                                       \
          case 1:                                                        \
            (out_bits_) |= DIR_RIGHT;                                    \
            break;                                                       \
          case 2:                                                        \
            (out_bits_) |= DIR_DOWN;                                     \
            break;                                                       \
          case 3:                                                        \
            (out_bits_) |= DIR_LEFT;                                     \
            break;                                                       \
          }                                                              \
        }                                                                \
      }                                                                  \
    }                                                                    \
  } while (0)

  unsigned char joy = 0;
  if (READ_BUTTON_IDX(map->btn_a))
    joy |= 0x10;
  if (READ_BUTTON_IDX(map->btn_b))
    joy |= 0x20;
  if (READ_BUTTON_IDX(map->btn_x))
    joy |= 0x40;
  if (READ_BUTTON_IDX(map->btn_y))
    joy |= 0x80;

  unsigned char dpad = 0;

  if (map->btn_dpad_up >= 0 || map->btn_dpad_right >= 0 ||
      map->btn_dpad_down >= 0 || map->btn_dpad_left >= 0)
  {
    if (READ_BUTTON_IDX(map->btn_dpad_up))
      dpad |= DIR_UP;
    if (READ_BUTTON_IDX(map->btn_dpad_right))
      dpad |= DIR_RIGHT;
    if (READ_BUTTON_IDX(map->btn_dpad_down))
      dpad |= DIR_DOWN;
    if (READ_BUTTON_IDX(map->btn_dpad_left))
      dpad |= DIR_LEFT;
  }

  if (dpad == 0 && map->dpad_hat >= 0 && report->joystick_mouse.hat.size > 0)
  {
    READ_HAT_DIR(dpad);
  }

  if (dpad == 0)
  {
    uint8_t lx_raw = 0x80, ly_raw = 0x80;
    READ_AXIS_U8(map->axis_lx, lx_raw);
    READ_AXIS_U8(map->axis_ly, ly_raw);

    if (lx_raw > AX_HIGH)
      dpad |= DIR_RIGHT;
    if (lx_raw < AX_LOW)
      dpad |= DIR_LEFT;
    if (ly_raw > AX_HIGH)
      dpad |= DIR_DOWN;
    if (ly_raw < AX_LOW)
      dpad |= DIR_UP;
  }

  joy &= ~(DIR_RIGHT | DIR_LEFT | DIR_DOWN | DIR_UP);
  joy |= dpad;

  uint8_t ax = 0x80, ay = 0x80;
  READ_AXIS_U8(map->axis_lx, ax);
  READ_AXIS_U8(map->axis_ly, ay);
  ax = buffer[0];
  ay = buffer[1];
  if (map->axis_lx_invert)
    ax = 255 - ax;
  if (map->axis_ly_invert)
    ay = 255 - ay;

  unsigned char btn_extra = 0;
  if (READ_BUTTON_IDX(map->btn_back))
    btn_extra |= 0x01;
  if (READ_BUTTON_IDX(map->btn_start))
    btn_extra |= 0x02;
  if (READ_BUTTON_IDX(map->btn_leftshoulder))
    btn_extra |= 0x04;
  if (READ_BUTTON_IDX(map->btn_rightshoulder))
    btn_extra |= 0x08;

  if (map->axis_lt >= 0)
  {
    uint8_t lt = 0x00;
    READ_AXIS_U8(map->axis_lt, lt);
    if (map->axis_lt_invert)
      lt = 255 - lt;
    if (lt > 0x80)
      btn_extra |= 0x10;
  }
  if (map->axis_rt >= 0)
  {
    uint8_t rt = 0x00;
    READ_AXIS_U8(map->axis_rt, rt);
    if (map->axis_rt_invert)
      rt = 255 - rt;
    if (rt > 0x80)
      btn_extra |= 0x20;
  }

  if (READ_BUTTON_IDX(map->btn_leftstick))
    btn_extra |= 0x40;
  if (READ_BUTTON_IDX(map->btn_rightstick))
    btn_extra |= 0x80;

  if ((joy != state->last_state) ||
      (ax != state->last_state_x) ||
      (ay != state->last_state_y) ||
      (btn_extra != state->last_state_btn_extra))
  {
    state->last_state = joy;
    state->last_state_x = ax;
    state->last_state_y = ay;
    state->last_state_btn_extra = btn_extra;

    usb_debugf("MAP%d: D %02x X %02x Y %02x EB %02x",
               state->js_index, joy, ax, ay, btn_extra);

    usb_debugf("MAP->AX %02x", map->axis_lx);
    usb_debugf("MAP->AY %02x", map->axis_ly);

    mcu_hw_spi_begin();
    mcu_hw_spi_tx_u08(SPI_TARGET_HID);
    mcu_hw_spi_tx_u08(SPI_HID_JOYSTICK);
    mcu_hw_spi_tx_u08(state->js_index);
    mcu_hw_spi_tx_u08(joy);
    mcu_hw_spi_tx_u08(ax);
    mcu_hw_spi_tx_u08(ay);
    mcu_hw_spi_tx_u08(btn_extra);
    mcu_hw_spi_end();
  }

#undef READ_BUTTON_IDX
#undef READ_AXIS_U8
#undef READ_HAT_DIR
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
      if (report->map_found && report->map) {
        // use SDL
        parse_with_sdl_mapping(report, &state->joystick, data, len, report->map);
      }
      else {
        // fallback
      joystick_parse(report, &state->joystick, data, len);
      }
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

  // forward the DB9 event to the menu system
  // if the OSD is visible. It's up to the core
  // to suppress internal joystick handling while
  // the OSD is open.
  if(osd_is_visible())
    menu_joystick_state(db9);
}
