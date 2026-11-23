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

#include "hid2latin1.h"
#include "hidkeys.h"

// ========= keyboard mappings ============

#if 0
#define kbd_remap_debugf(a, ...)  debugf(a, ##__VA_ARGS__)
#else
#define kbd_remap_debugf(a, ...)
#endif

// list of key mappings
static struct key_map_S {
  struct {
    uint8_t key;
    uint8_t mod;
  } from;

  struct {
    uint8_t key;
    uint8_t mod;
  } to;
  struct key_map_S *next;
} *key_map = NULL;

void hid_set_key_mapping(uint8_t key_from, uint8_t mod_from, uint8_t key_to, uint8_t mod_to) {
  const char *from_str = hidkeys_get_name(key_from);
  const char *to_str = hidkeys_get_name(key_to);
  if(from_str && to_str) {
    int len_from = hidkeys_get_modifier_name(NULL, mod_from);
    int len_to = hidkeys_get_modifier_name(NULL, mod_to);
    {
      char from_mod_str[len_from+1];
      char to_mod_str[len_to+1];
      hidkeys_get_modifier_name(from_mod_str, mod_from);
      hidkeys_get_modifier_name(to_mod_str, mod_to);

      usb_debugf("key '%s%s'='%s%s'", from_mod_str, from_str, to_mod_str, to_str);
    }
  }

  // search end of key map list
  struct key_map_S **map = &key_map;
  while(*map) map = &((*map)->next);

  // and append a new entry
  *map = pvPortMalloc(sizeof(struct key_map_S));
  (*map)->from.key = key_from;
  (*map)->from.mod = mod_from;
  (*map)->to.key = key_to;
  (*map)->to.mod = mod_to;
  (*map)->next = NULL;
}

// get a key remapping from the liste of remapped keys. This
// returns the key to be sent in the lower 8 bits and the
// modifiers that were required for this mapping in the
// upper 8 bits
static uint16_t hid_get_key_mapping(uint8_t key, uint8_t mod, uint8_t *from_mod, uint8_t *to_mod) {
  // kbd_remap_debugf("get mapping for %02x %d\n", key, mod);
  
  struct key_map_S *map = key_map;

  // for the modifier it's only important that all
  // those requested are in fact active. It's totally
  // fine if additional modifier are active.  
  while(map) {
    if(map->from.key == key && map->from.mod == (mod & map->from.mod)) {
      *from_mod = map->from.mod;
      *to_mod = map->to.mod;
      return map->to.key;
    }
    
    map = map->next;
  }
  
  return 0;   // no mapped key
}

bool hid_get_key_mapping_index(int index, uint8_t *from, uint8_t *from_mod,
			       uint8_t *to, uint8_t *to_mod) {
  struct key_map_S *map = key_map;

  while(map && index--)
    map = map->next;

  if(map) {
    *from = map->from.key;
    *from_mod = map->from.mod;
    *to = map->to.key;
    *to_mod = map->to.mod;
    return true;
  }
  
  return false;   // no mapped key
}

// ========= gamepad/joystick button mappings ============
// These allow to map a buttom combo to a hid key code allowing
// to use parts of the joystick or gamepad as a keyboard replacement

// list of button mappings
static struct btn_map_S {
  uint16_t btns;
  uint8_t key;
  uint8_t mod;
  struct btn_map_S *next;
} *button_map = NULL;

void hid_set_button_mapping(uint16_t btns, uint8_t key, uint8_t mod) {
  // print the result, allocate enough stack space for full string
  int len = hidkeys_get_btnmap_str(NULL, btns);
  if(len) {
    char str[len+1];
    hidkeys_get_btnmap_str(str, btns);
    const char *keyname = hidkeys_get_name(key);
    int len_mod = hidkeys_get_modifier_name(NULL, mod);
    {
      char str_mod[len_mod+1];          
      hidkeys_get_modifier_name(str_mod, mod);
      if(keyname) usb_debugf("button %s=%s%s", str, str_mod, keyname);
      else        usb_debugf("button %s=%s%d", str, str_mod, key);
    }
  }

  // search end of button map list
  struct btn_map_S **map = &button_map;
  while(*map) map = &((*map)->next);

  // and append a new entry
  *map = pvPortMalloc(sizeof(struct btn_map_S));
  (*map)->btns = btns;
  (*map)->key = key;
  (*map)->mod = mod;
  (*map)->next = NULL;
}	    

static uint8_t hid_get_button_mapping(uint8_t joy, uint8_t btn_extra, uint8_t *mod) {
  struct btn_map_S *map = button_map;

  while(map) {
    if(map->btns == (btn_extra << 8) + joy) {
      *mod = map->mod;
      return map->key;
    }
    
    map = map->next;
  }
  
  return 0;   // no mapped key
}

bool hid_get_button_mapping_index(int index, uint16_t *btns, uint8_t *key, uint8_t *mod) {
  struct btn_map_S *map = button_map;

  while(map && index--)
    map = map->next;

  if(map) {
    *btns = map->btns;
    *key = map->key;
    *mod = map->mod;
    return true;
  }
  
  return false;   // no mapped key
}

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

static void kbd_tx_modmap(bool make, uint8_t mod) {
  for(int i=0;i<8;i++) {
    if(mod & (1<<i)) {
      kbd_remap_debugf("MOD %s '%s'", make?"MAKE":"BREAK", hidkeys_get_name(0x68+i));
      kbd_tx((make?0:0x80)|(0x68+i));
    }
  }
}	    

static inline bool is_make(uint8_t a) { return !(a&0x80); }
static inline uint8_t code(uint8_t a) { return a&0x7f; }

static void kbd_tx_remap(uint8_t byte) {
  // Modifier handling is a bit tricky.

  // in order to apply key mappings, we need to keep track
  // of the currently active modifier keys
  static uint8_t current_modifier_map = 0;

  static struct {
    struct {
      // currently active remapped actual keycode
      uint8_t key;
      // modifiers that were required for this mapping
      uint8_t mod;
    } from;
    struct {
      // and the byte that actually has been sent instead
      uint8_t key;
      // and the modifiers sent with it
      uint8_t mod;
    } to;      
  } mapped = { {0,0}, {0,0}};
    
  // TODO: The modifier map may change while the key is being pressed
  // We need to handle that case as well
  
  // check if this key is to be remapped
  // modfier need to be all that the
  uint8_t from_mod, to_mod, remap = hid_get_key_mapping(code(byte), current_modifier_map, &from_mod, &to_mod);  

  if(is_make(byte) && !mapped.from.key && remap) {
    // pressed key is being remapped while no remapping is in progress
    mapped.from.key = byte;
    mapped.from.mod = from_mod;
    mapped.to.key = remap;
    mapped.to.mod = to_mod;
    kbd_remap_debugf("KEY MAKE REMAP '%s' to '%s', mod %08b\n", hidkeys_get_name(byte), hidkeys_get_name(mapped.to.key), mapped.from.mod);

    // if the map was triggered by a key requiring modifiers, then release
    // all these modfiers before sending the remapped key code
    kbd_tx_modmap(false, mapped.from.mod);

    // press optional modfiers of the target key
    kbd_tx_modmap(true, mapped.to.mod);

    // The core may need some time between the modfiers changing and the actual key code
    // This should be done cleaner without blocking
    vTaskDelay(pdMS_TO_TICKS(10));

    // finally transmit the modified key itself
    kbd_tx(mapped.to.key);

  } else if(!is_make(byte) && mapped.from.key == code(byte)) {
    // released key is the one that is currently being remapped    
    kbd_remap_debugf("KEY BREAK REMAP '%s' to '%s'\n", hidkeys_get_name(code(byte)), hidkeys_get_name(mapped.to.key));
    kbd_tx(0x80 | mapped.to.key);

    // release optional modfiers of the target key
    kbd_tx_modmap(false, mapped.to.mod);
    
    // if the map was triggeed by a key requiring modifiers, then (re-)press
    // those afterwards, unless they have already been released by the user
    kbd_tx_modmap(true, mapped.from.mod);
    
    // forget about mapping
    mapped.from.key = mapped.to.key = mapped.from.mod = mapped.to.mod = 0;
  } else {
    // key is neither to be remapped nor the currently remapped one    
    kbd_remap_debugf("KEY %s '%s'\n", is_make(byte)?"MAKE":"BREAK", hidkeys_get_name(code(byte)));
    
    // keep track of modifiers
    if(code(byte) >= 0x68) {
      if(is_make(byte)) current_modifier_map |=  (1<<(code(byte)-0x68));
      else              current_modifier_map &= ~(1<<(code(byte)-0x68));
      
      kbd_remap_debugf("MOD MAP %08b\n", current_modifier_map);
      
      // if we are currently remapping and the modifier being release has already
      // been fake released for the mapping, then don't send this release code
      if(mapped.from.key && (mapped.from.mod & (1<<(code(byte)-0x68)))) {
	kbd_remap_debugf("MOD has already been released!\n");
	
	// from now on that modifier is not under mapping control anymore
	// and the user may use it. Thus remove it from the list of modifers
	// required for the current mapping
	mapped.from.mod &= ~(1<<(code(byte)-0x68));
      } else
	kbd_tx(byte);  
      
    } else
      kbd_tx(byte);  
  }
}

void kbd_parse(__attribute__((unused)) const hid_report_t *report, struct hid_kbd_state_S *state,
	       const unsigned char *buffer, int nbytes) {

  // check if the given code is in the report
  bool is_in_report(unsigned char code, const unsigned char *report) {
    for(int j=0;j<6;j++)
      if(report[j] == code)
	return true;

    return false;
  }
  
  // we expect boot mode packets which are exactly 8 bytes long
  if(nbytes != 8) return;
  // determine menu hotkey
  int hotkey = inifile_config_get_int("menu", "hotkey", 0x45); // default is f12
  if(!hotkey) hotkey = 0x45;
  // check if modifier have changed
  if((buffer[0] != state->last_report[0]) && !osd_is_visible()) {
    for(int i=0;i<8;i++) {
      // modifier keys map to key codes 0x68+
      
      // modifier released?
      if((state->last_report[0] & (1<<i)) && !(buffer[0] & (1<<i)))
        kbd_tx_remap(0x80 | (i+0x68));
      // modifier pressed?
      if(!(state->last_report[0] & (1<<i)) && (buffer[0] & (1<<i)))
        kbd_tx_remap(i+0x68);
    }
  }
  
  // check if regular keys have changed
  // key released?
  for(int i=0;i<6;i++) {
    // process all slots that were used in the last report
    if(!(state->last_report[2+i])) continue;
    // check if key reported in last report is still in current report 
    if (!is_in_report(state->last_report[2+i], buffer+2)) {
      if(!osd_is_visible() ) {
        // check if the reported key is the OSD activation hotkey
        // and suppress reporting it to the core
        if(state->last_report[2+i] != hotkey)
          kbd_tx_remap(0x80 | state->last_report[2+i]);
      } else
	menu_notify(MENU_EVENT_KEY_RELEASE);
    }
  }
  // key pressed?
  for(int i=0;i<6;i++) {
    // process all slots that are used in current report
    if(!(buffer[2+i])) continue;
    // check if key currently reported was not present in last report
    if (!is_in_report(buffer[2+i], state->last_report+2)) {
      static unsigned long msg;
      msg = 0;
	
      // F12 toggles the OSD state. Therefore F12 must never be forwarded
      // to the core and thus must have an empty entry in the keymap. ESC
      // can only close the OSD. This is now configurable via the menu hotkey
      // Caution: Since the OSD closes on the press event, the following
      // release event will be sent into the core. The core should thus
      // cope with release events that did not have a press event before
      if(buffer[2+i] == hotkey) {
        // for now, shift-F12 activates the system menu
        if(buffer[0] & 0x22) msg = MENU_EVENT_SYSTEM;
        else	  msg = MENU_EVENT_TOGGLE;
      } else if(osd_is_visible() && buffer[2+i] == 0x29 /* ESC key */ )
        msg = MENU_EVENT_BACK;
      else {
        if(!osd_is_visible())
           kbd_tx_remap(buffer[2+i]);
        else {
          // check if cursor up/down or space has been pressed
          if(buffer[2+i] == 0x51) msg = MENU_EVENT_DOWN;
          if(buffer[2+i] == 0x52) msg = MENU_EVENT_UP;
          if(buffer[2+i] == 0x4e) msg = MENU_EVENT_PGDOWN;
          if(buffer[2+i] == 0x4b) msg = MENU_EVENT_PGUP;
          if((buffer[2+i] == 0x2c) || (buffer[2+i] == 0x28))
            msg = MENU_EVENT_SELECT;

	  hid_process_latin1(buffer[0], buffer[2+i]);
        }
      }

      // send message to menu task
      if(msg) menu_notify(msg);
    }
  }

  memcpy(state->last_report, buffer, 8);
}

// release all keys and modifiers still held, e.g. when the keyboard is unplugged
void kbd_release_all(struct hid_kbd_state_S *state) {
  int hotkey = inifile_config_get_int("menu", "hotkey", 0x45);
  if(!hotkey) hotkey = 0x45;
  for(int i=0;i<8;i++)
    if(state->last_report[0] & (1<<i))
      kbd_tx_remap(0x80 | (i+0x68));

  // the menu hotkey was never forwarded to the core, so it is not released
  for(int i=2;i<8;i++)
    if(state->last_report[i] && state->last_report[i] != hotkey)
      kbd_tx_remap(0x80 | state->last_report[i]);

  memset(state->last_report, 0, sizeof(state->last_report));
}

// collect bits from byte stream and assemble them into a signed word
static uint16_t collect_bits(const uint8_t *p, uint16_t offset, uint8_t size, bool is_signed) {
  // mask unused bits of first byte
  uint8_t mask = 0xff << (offset&7);
  uint8_t byte = offset/8;
  uint8_t bits = size;
  uint8_t shift = offset&7;

  // usb_debugf("0 m:%x by:%d bi=%d sh=%d ->", mask, byte, bits, shift);
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

  // collect info about at least two axes, third is optional scroll wheel
  int a[3] = { 0,0,0 };
  for(int i=0;i<3;i++) {
    if(report->joystick_mouse.axis[i].size) {
      bool is_signed = report->joystick_mouse.axis[i].logical.min >
	report->joystick_mouse.axis[i].logical.max;

      a[i] = collect_bits(buffer, report->joystick_mouse.axis[i].offset,
			  report->joystick_mouse.axis[i].size, is_signed);

      // Each axis goes to the FPGA as one signed byte. A mouse with wider
      // axes (12 or 16 bits are common) would otherwise have a fast movement
      // cut to its low eight bits and seem to jump backwards.
      // collect_bits() returns the sign-extended value in 16 bits.
      if(is_signed) {
	int16_t wide = (int16_t)a[i];
	if(wide > INT8_MAX) wide = INT8_MAX;
	if(wide < INT8_MIN) wide = INT8_MIN;
	a[i] = wide;
      }
    }
  }

  // ... and at least two buttons
  uint8_t btns = 0;
  for(int i=0;i<3;i++)
    if(buffer[report->joystick_mouse.button[i].byte_offset] &
       report->joystick_mouse.button[i].bitmask)
      btns |= (1<<i);

  mcu_hw_spi_begin();
  mcu_hw_spi_tx_u08(SPI_TARGET_HID);
  mcu_hw_spi_tx_u08(SPI_HID_MOUSE);
  mcu_hw_spi_tx_u08(btns);
  mcu_hw_spi_tx_u08(a[0]);
  mcu_hw_spi_tx_u08(a[1]);
  if(report->joystick_mouse.axis[2].size)
    mcu_hw_spi_tx_u08(a[2]);
  mcu_hw_spi_end();
}

void hid_joystick_push(uint8_t index, uint8_t joy, uint8_t ax, uint8_t ay, uint8_t btn_extra) {
  usb_debugf("JOY%d: D %02x X %02x Y %02x EB %02x", index, joy, ax, ay, btn_extra);

  // check if there's a gamepad menu button enabled in the config
  if(inifile_config_has("menu", "gamepad_trigger")) {
    int btn = inifile_config_get_int("menu", "gamepad_trigger", 0);

    // buttons 0..3 are encoded in the upper bits of 'joy', further
    // 8 buttons are in btn_extra

    static bool last_detected = false;
    bool detected = false;
    if(btn < 4) {
      if(joy & (1<<(btn+4)))
	detected = true;
    } else if(btn < 12) {
      if(btn_extra & (1<<(btn-4)))
	detected = true;
    }

    if(detected && !last_detected) {
      usb_debugf("menu trigger pressed");
      menu_notify(MENU_EVENT_TOGGLE);
    }

    last_detected = detected;
  }

  if(osd_is_visible()) {
    // if OSD is visible, then process events locally
    menu_joystick_state(joy);
  } else {
    static int8_t mapped_key = 0;
    static int8_t mapped_mod = 0;
    
    // check if the current button combo is mapped
    // to a key code
    uint8_t mod, key = hid_get_button_mapping(joy, btn_extra, &mod);
    if(key) {
      if(!mapped_key && mapped_key != key) {
	// make/press all modifiers involved
	kbd_tx_modmap(true, mod);

	// The core may need some time between the modfiers changing and the actual key code
	// This should be done cleaner without blocking
	vTaskDelay(pdMS_TO_TICKS(10));

	usb_debugf("key make '%s'", hidkeys_get_name(key));
	kbd_tx(key);
	mapped_key = key;
	mapped_mod = mod;
      }
    } else {
      if(mapped_key) {
	usb_debugf("key break '%s'", hidkeys_get_name(mapped_key));
	kbd_tx(0x80|mapped_key);
	mapped_key = 0;

	// break/release all modifiers involved
	kbd_tx_modmap(false, mapped_mod);
	mapped_mod = 0;	    
      }
      
      mcu_hw_spi_begin();
      mcu_hw_spi_tx_u08(SPI_TARGET_HID);
      mcu_hw_spi_tx_u08(SPI_HID_JOYSTICK);
      mcu_hw_spi_tx_u08(index);
      mcu_hw_spi_tx_u08(joy);
      mcu_hw_spi_tx_u08(ax);
      mcu_hw_spi_tx_u08(ay);
      mcu_hw_spi_tx_u08(btn_extra);
      mcu_hw_spi_end();
    }
  }
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

    // scale axis down to 8 bits if required
    if(report->joystick_mouse.axis[i].size > 8)
      a[i] >>= (report->joystick_mouse.axis[i].size - 8);
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

  // HAT present?
  if (report->joystick_mouse.hat.size > 0)
    {
    // give HAT a priority and clear and ignore previous axix results
    unsigned char hat_dir = 0;

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

      // This only erases and overwrites joy from HAT info if the
      // hat is not on neutral position. This allows to use regular
      // exis reports by default and only use hat events, if the
      // hat is actually being controlled by the user. Only problem:
      // if a gamepad reports a position != center on either the x or
      // y axis by default then the hat will not overwrite this while
      // the hat is in neutral position.
      joy &= ~(DIR_RIGHT | DIR_LEFT | DIR_DOWN | DIR_UP);
      joy |= hat_dir;
    }

    //    usb_debugf("HAT raw=%d norm=%d steps=%d dir=0x%02x%s",
    //               hat_raw, v, steps, hat_dir, neutral ? " (neutral)" : "");
  }

  if((joy != state->last_state) ||
     (ax != state->last_state_x) ||
     (ay != state->last_state_y) ||
     (btn_extra != state->last_state_btn_extra))  {
    state->last_state = joy;
    state->last_state_x = ax;
    state->last_state_y = ay;
    state->last_state_btn_extra = btn_extra;

    hid_joystick_push(state->js_index, joy, ax, ay, btn_extra);
  }
}

void consumer_parse(const hid_report_t *report, struct hid_consumer_state_S *state,
		    const unsigned char *buffer, __attribute__((unused)) int nbytes) {
  usb_debugf("consumer: %d %02x", nbytes, buffer[0]);

  // search for button state changes
  for(int i=0;i<32;i++) {
    if(buffer[report->joystick_mouse.button[i].byte_offset] &
       report->joystick_mouse.button[i].bitmask) {
      if(!(state->last_state & (1<<i))) {
	usb_debugf("press event %d", i);
	state->last_state |= (1<<i);
	// tell menu system about this event
	menu_notify(i);
      }
    } else {
      if((state->last_state & (1<<i))) {
	usb_debugf("release event %d", i);
	state->last_state &= ~(1<<i);
	// tell menu system that key is now released
	menu_notify(MENU_EVENT_KEY_RELEASE);
      }
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

static int sanitize_trigger_axis(const UsbGamepadMap *map, int axis_idx,
                                 int other_axis_idx)
{
  if (axis_idx < 0)
    return -1;

  if (axis_idx == other_axis_idx)
    return -1;

  if (axis_idx == map->axis_lx || axis_idx == map->axis_ly ||
      axis_idx == map->axis_rx || axis_idx == map->axis_ry)
    return -1;

  return axis_idx;
}

static inline bool read_button_idx(const uint8_t *buffer, const hid_report_t *report, int idx) {
  if(idx < 0) return 0;
  
  return ((buffer[report->joystick_mouse.button[idx].byte_offset] & report->joystick_mouse.button[idx].bitmask) ? 1 : 0);
}

static inline uint8_t read_axis_u8(const uint8_t *buffer, const hid_report_t *report, int idx, bool physmap) {
  if(idx < 0) return 0x80; /* neutral fallback */

  /* re-assign index as the parser may have re-ordered the axes */
  /* TODO: This prevents the data from twin gamepad from working */
  /* which is thus now blacklisted in rp2040/mcu_hw.c */
  /* but it makes the retro link SNES gamepad work */
  if(physmap) {
    idx = report->joystick_mouse.axis[idx].index;
    if(idx < 0) return 0x80; /* neutral fallback */
  }
    
  bool is_signed = report->joystick_mouse.axis[idx].logical.min >
    report->joystick_mouse.axis[idx].logical.max;
  int tmp = (int)collect_bits(buffer,
			      report->joystick_mouse.axis[idx].offset,
			      report->joystick_mouse.axis[idx].size,
			      is_signed);
  if(tmp < 0)   tmp = 0;
  if(tmp > 255) tmp = 255;
  return (uint8_t)tmp;
}

static inline uint8_t read_hat_dir(const uint8_t *buffer, const hid_report_t *report) {
  uint8_t retval = 0;

  if (report->joystick_mouse.hat.size <= 0)
    return retval;

  int hat_raw = (int)collect_bits(buffer,
				  report->joystick_mouse.hat.offset,
				  report->joystick_mouse.hat.size,
				  /*is_signed=*/false);
  
  int lmin = report->joystick_mouse.hat.logical.min;
  int lmax = report->joystick_mouse.hat.logical.max;
  int steps = (lmax >= lmin) ? (lmax - lmin + 1) : 0;
  int v = hat_raw - lmin;

  int neutral = 0;
  if (steps >= 8)
    neutral = (v < 0 || v > 7);
  else if (steps >= 4)
    neutral = (v < 0 || v > 3);
  else
    neutral = 1;
  
  if (!neutral) {
    if (steps >= 8) {
      switch (v) {
      case 0:
	retval |= DIR_UP;
	break;
      case 1:
	retval |= (DIR_UP | DIR_RIGHT);
	break;
      case 2:
	retval |= DIR_RIGHT;
	break;
      case 3:
	retval |= (DIR_RIGHT | DIR_DOWN);
	break;
      case 4:
	retval |= DIR_DOWN;
	break;
      case 5:
	retval |= (DIR_DOWN | DIR_LEFT);
	break;
      case 6:
	retval |= DIR_LEFT;
	break;
      case 7:
	retval |= (DIR_LEFT | DIR_UP);
	break;
      }
    } else {
      switch (v & 3) {
      case 0:
	retval |= DIR_UP;
	break;
      case 1:
	retval |= DIR_RIGHT;
	break;
      case 2:
	retval |= DIR_DOWN;
	break;
      case 3:
	retval |= DIR_LEFT;
	break;
      }
    }
  }
  return retval;
}

void parse_with_sdl_mapping(const hid_report_t *report,
                            struct hid_joystick_state_S *state,
                            const unsigned char *buffer, int nbytes,
                            const UsbGamepadMap *map)
{
  (void)nbytes;

  // hexdump(buffer, nbytes);

  unsigned char joy = 0;
  if (read_button_idx(buffer, report, map->btn_a))
    joy |= 0x10;
  if (read_button_idx(buffer, report, map->btn_b))
    joy |= 0x20;
  if (read_button_idx(buffer, report, map->btn_x))
    joy |= 0x40;
  if (read_button_idx(buffer, report, map->btn_y))
    joy |= 0x80;

  unsigned char dpad = 0;

  // parse directions from a button based configuration
  if (map->btn_dpad_up >= 0 || map->btn_dpad_right >= 0 ||
      map->btn_dpad_down >= 0 || map->btn_dpad_left >= 0)
  {
    if (read_button_idx(buffer, report, map->btn_dpad_up))
      dpad |= DIR_UP;
    if (read_button_idx(buffer, report, map->btn_dpad_right))
      dpad |= DIR_RIGHT;
    if (read_button_idx(buffer, report, map->btn_dpad_down))
      dpad |= DIR_DOWN;
    if (read_button_idx(buffer, report, map->btn_dpad_left))
      dpad |= DIR_LEFT;
  }

  // otherwise parse directions from a hat
  if (dpad == 0 && map->dpad_hat >= 0 && report->joystick_mouse.hat.size > 0)
    dpad = read_hat_dir(buffer, report);

  // otherwise parse dpad axes
  if (dpad == 0 && map->dpad_axis_up>=0 && map->dpad_axis_left>=0)
  {
    /* TODO: It seems only this needs the mapping to the physical index */
    uint8_t dpup_raw = read_axis_u8(buffer, report, map->dpad_axis_up, true);
    uint8_t dpleft_raw = read_axis_u8(buffer, report, map->dpad_axis_left, true);
    if (dpleft_raw > AX_HIGH)
      dpad |= DIR_RIGHT;
    if (dpleft_raw < AX_LOW)
      dpad |= DIR_LEFT;
    if (dpup_raw > AX_HIGH)
      dpad |= DIR_DOWN;
    if (dpup_raw < AX_LOW)
      dpad |= DIR_UP;
  }

  // otherwise parse directions from analogue axes
  if (dpad == 0)
    {
      uint8_t lx_raw = read_axis_u8(buffer, report, map->axis_lx, false);
      uint8_t ly_raw = read_axis_u8(buffer, report, map->axis_ly, false);
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

  uint8_t ax = read_axis_u8(buffer, report, map->axis_lx, false);
  uint8_t ay = read_axis_u8(buffer, report, map->axis_ly, false);
  if (map->axis_lx_invert)
    ax = 255 - ax;
  if (map->axis_ly_invert)
    ay = 255 - ay;

  unsigned char btn_extra = 0;
  int axis_lt = sanitize_trigger_axis(map, map->axis_lt, map->axis_rt);
  int axis_rt = sanitize_trigger_axis(map, map->axis_rt, map->axis_lt);

  if (read_button_idx(buffer, report, map->btn_leftshoulder))
    btn_extra |= 0x01;
  if (read_button_idx(buffer, report, map->btn_rightshoulder))
    btn_extra |= 0x02;
  if (read_button_idx(buffer, report, map->btn_back))
    btn_extra |= 0x04;
  if (read_button_idx(buffer, report, map->btn_start))
    btn_extra |= 0x08;

  if (axis_lt >= 0) {
    uint8_t lt = read_axis_u8(buffer, report, axis_lt, false);
    if (map->axis_lt_invert)
      lt = 255 - lt;
    if (lt > 0x80)
      btn_extra |= 0x10;
  }
  if (axis_rt >= 0) {
    uint8_t rt = read_axis_u8(buffer, report, axis_rt, false);
    if (map->axis_rt_invert)
      rt = 255 - rt;
    if (rt > 0x80)
      btn_extra |= 0x20;
  }

  if (read_button_idx(buffer, report, map->btn_leftstick))
    btn_extra |= 0x40;
  if (read_button_idx(buffer, report, map->btn_rightstick))
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

    hid_joystick_push(state->js_index, joy, ax, ay, btn_extra);
  }
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
      // usb_debugf("report id mismatch %d != %d, ignoring report", data[0], report->report_id);
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

    if(report->type == REPORT_TYPE_JOYSTICK) {
      if (report->map_found && report->map) {
        // use SDL
        parse_with_sdl_mapping(report, &state->joystick, data, len, report->map);
      }
      else {
        // fallback
	joystick_parse(report, &state->joystick, data, len);
      }
    }

    if(report->type == REPORT_TYPE_CONSUMER)
      consumer_parse(report, &state->consumer, data, len);
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

  // Forward the DB9 event to the menu system
  // if the OSD is visible. It's up to the core
  // to suppress internal joystick handling while
  // the OSD is open. Mask out the second fire button
  // as the wirless cx40+ sends it wrongly while
  // no other db9 joystick seems to send it at all.
  if(osd_is_visible())
    menu_joystick_state(db9 & 0x1f);
}
