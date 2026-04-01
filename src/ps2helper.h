/* ps2_helper.h */

#ifndef PS2HELPER_H
#define PS2HELPER_H

#include <stdint.h>
#include <string.h>
#include "mcu_hw.h"

#define PS2_E0 0x0100
#define PS2_NONE 0x0000
#define PS2_SPECIAL_PRINT 0xFFFE
#define PS2_SPECIAL_PAUSE 0xFFFF

/* HID modifier bits (buffer[0]) -> PS/2 Set 2 scancodes */
static const uint16_t hid_modbit_to_ps2[8] = {
  0x14,            /* bit0: LCtrl */
  0x12,            /* bit1: LShift */
  0x11,            /* bit2: LAlt */
  PS2_E0 | 0x1F,   /* bit3: LGUI (Win) is E0 1F */
  PS2_E0 | 0x14,   /* bit4: RCtrl */
  0x59,            /* bit5: RShift */
  PS2_E0 | 0x11,   /* bit6: RAlt (AltGr) */
  PS2_E0 | 0x27    /* bit7: RGUI */
};

void ps2_make_sc(uint8_t byte, uint8_t mod);
void ps2_break_sc(uint8_t byte, uint8_t mod);
uint16_t hid_to_ps2_set2(uint8_t hid);

#endif // PS2HELPER_H