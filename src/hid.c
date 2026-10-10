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
#include "ps2helper.h"

#include "hid2latin1.h"

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

void kbd_parse_usage_state(struct hid_kbd_state_S *state, uint8_t modifiers,
                           const uint8_t keys[32])
{
  if (modifiers != state->modifiers && !osd_is_visible()) {
    for (int bit = 0; bit < 8; bit++) {
      if ((state->modifiers & (1U << bit)) && !(modifiers & (1U << bit)))
        kbd_tx_hid_ps2_break(bit, 1);
      if (!(state->modifiers & (1U << bit)) && (modifiers & (1U << bit)))
        kbd_tx_hid_ps2_make(bit, 1);
    }
  }

  for (uint16_t usage = 4; usage <= UINT8_MAX; usage++) {
    if (usage >= 0xe0 && usage <= 0xe7)
      continue;

    bool was_pressed = (state->keys[usage / 8] & (1U << (usage % 8))) != 0;
    bool is_pressed = (keys[usage / 8] & (1U << (usage % 8))) != 0;
    if (was_pressed == is_pressed)
      continue;

    if (!is_pressed) {
      if (!osd_is_visible()) {
        if (usage != inifile_option_get(INIFILE_OPTION_HOTKEY))
          kbd_tx_hid_ps2_break((uint8_t)usage, 0);
      } else {
        menu_notify(MENU_EVENT_KEY_RELEASE);
      }
      continue;
    }

    unsigned long msg = 0;
    if (usage == inifile_option_get(INIFILE_OPTION_HOTKEY)) {
      msg = (modifiers & 0x22) ? MENU_EVENT_SYSTEM : MENU_EVENT_TOGGLE;
    } else if (osd_is_visible() && usage == 0x29) {
      msg = MENU_EVENT_BACK;
    } else {
      if (!osd_is_visible()) {
        kbd_tx_hid_ps2_make((uint8_t)usage, 0);
      } else {
        if (usage == 0x51) msg = MENU_EVENT_DOWN;
        if (usage == 0x52) msg = MENU_EVENT_UP;
        if (usage == 0x4e) msg = MENU_EVENT_PGDOWN;
        if (usage == 0x4b) msg = MENU_EVENT_PGUP;
        if (usage == 0x2c || usage == 0x28) msg = MENU_EVENT_SELECT;
        hid_process_latin1(modifiers, (uint8_t)usage);
      }
    }

    if (msg)
      menu_notify(msg);
  }

  state->modifiers = modifiers;
  memcpy(state->keys, keys, sizeof(state->keys));
  memset(state->last_report, 0, sizeof(state->last_report));
  state->last_report[0] = modifiers;
  int slot = 2;
  for (uint16_t usage = 4; usage <= UINT8_MAX && slot < 8; usage++) {
    if (usage >= 0xe0 && usage <= 0xe7)
      continue;
    if (keys[usage / 8] & (1U << (usage % 8)))
      state->last_report[slot++] = (uint8_t)usage;
  }
}

void kbd_parse(__attribute__((unused)) const hid_report_t *report,
               struct hid_kbd_state_S *state,
               const unsigned char *buffer, int nbytes)
{
  uint8_t keys[32] = { 0 };
  uint8_t modifiers;

  if (nbytes != 8)
    return;

  modifiers = buffer[0];
  for (int i = 2; i < 8; i++) {
    uint8_t usage = buffer[i];
    if (usage >= 1 && usage <= 3)
      return;
    if (usage >= 0xe0 && usage <= 0xe7)
      modifiers |= (uint8_t)(1U << (usage - 0xe0));
    else if (usage > 3)
      keys[usage / 8] |= (uint8_t)(1U << (usage % 8));
  }

  kbd_parse_usage_state(state, modifiers, keys);
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

static void joystick_push(uint8_t index, uint8_t joy, uint8_t ax, uint8_t ay, uint8_t btn_extra) {    
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

    joystick_push(state->js_index, joy, ax, ay, btn_extra);
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

void parse_with_sdl_mapping(const hid_report_t *report,
                            struct hid_joystick_state_S *state,
                            const unsigned char *buffer, int nbytes,
                            const UsbGamepadMap *map)
{
  (void)nbytes;

  // hexdump(buffer, nbytes);
  
#define READ_BUTTON_IDX(idx_) \
  ((idx_) < 0 ? 0 : ((buffer[report->joystick_mouse.button[(idx_)].byte_offset] & report->joystick_mouse.button[(idx_)].bitmask) ? 1 : 0))

#define READ_AXIS_U8(idx_, out_u8_)                                           \
  do                                                                          \
  {                                                                           \
    int tmp_;								      \
    if ((idx_) < 0)                                                           \
    {                                                                         \
      (out_u8_) = 0x80; /* neutral fallback */                                \
    }                                                                         \
    else                                                                      \
    {                                                                         \
      /* re-assign index as the parser may have re-ordered the axes */	      \
      int pidx_ = report->joystick_mouse.axis[(idx_)].index;                  \
      if ((pidx_) < 0)                                                        \
      {                                                                       \
        (out_u8_) = 0x80; /* neutral fallback */                              \
      }                                                                       \
      else                                                                    \
      {                                                                       \
        bool _is_signed = report->joystick_mouse.axis[(pidx_)].logical.min >  \
                          report->joystick_mouse.axis[(pidx_)].logical.max;   \
        tmp_ = (int)collect_bits(buffer,                                      \
                                 report->joystick_mouse.axis[(pidx_)].offset, \
                                 report->joystick_mouse.axis[(pidx_)].size,   \
                                 _is_signed);                                 \
        if (tmp_ < 0)                                                         \
          tmp_ = 0;                                                           \
        if (tmp_ > 255)                                                       \
          tmp_ = 255;                                                         \
        (out_u8_) = (uint8_t)tmp_;                                            \
      }                                                                       \
    }                                                                         \
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

  // parse directions from a button based configuration
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

  // otherwise parse directions from a hat
  if (dpad == 0 && map->dpad_hat >= 0 && report->joystick_mouse.hat.size > 0)
  {
    READ_HAT_DIR(dpad);
  }

  // otherwise parse dpad axes
  if (dpad == 0 && map->dpad_axis_up>=0 && map->dpad_axis_left>=0)
  {
    uint8_t dpup_raw, dpleft_raw;
    READ_AXIS_U8(map->dpad_axis_up, dpup_raw);
    READ_AXIS_U8(map->dpad_axis_left, dpleft_raw);    
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
    uint8_t lx_raw, ly_raw;
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
  if (map->axis_lx_invert)
    ax = 255 - ax;
  if (map->axis_ly_invert)
    ay = 255 - ay;

  unsigned char btn_extra = 0;
  int axis_lt = sanitize_trigger_axis(map, map->axis_lt, map->axis_rt);
  int axis_rt = sanitize_trigger_axis(map, map->axis_rt, map->axis_lt);

  if (READ_BUTTON_IDX(map->btn_leftshoulder))
    btn_extra |= 0x01;
  if (READ_BUTTON_IDX(map->btn_rightshoulder))
    btn_extra |= 0x02;
  if (READ_BUTTON_IDX(map->btn_back))
    btn_extra |= 0x04;
  if (READ_BUTTON_IDX(map->btn_start))
    btn_extra |= 0x08;

  if (axis_lt >= 0) {
    uint8_t lt = 0x00;
    READ_AXIS_U8(axis_lt, lt);
    if (map->axis_lt_invert)
      lt = 255 - lt;
    if (lt > 0x80)
      btn_extra |= 0x10;
  }
  if (axis_rt >= 0) {
    uint8_t rt = 0x00;
    READ_AXIS_U8(axis_rt, rt);
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

    joystick_push(state->js_index, joy, ax, ay, btn_extra);
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
