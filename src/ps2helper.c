#include "ps2helper.h"
#include <stdint.h>
#include <string.h>
#include "spi.h"
#include "mcu_hw.h"

void ps2_make_sc(uint8_t byte, uint8_t mod)
{
    uint16_t sc;

    mcu_hw_spi_begin();
    mcu_hw_spi_tx_u08(SPI_TARGET_HID);
    mcu_hw_spi_tx_u08(SPI_HID_KEYBOARD);
    if (mod != 0) { 
        mcu_hw_spi_tx_u08(byte+0x68);
        sc = hid_to_ps2_set2(hid_modbit_to_ps2[byte]);
      }
    else {
        mcu_hw_spi_tx_u08(byte);
        sc = hid_to_ps2_set2(byte);
      }

    if(sc == PS2_NONE) { 
        mcu_hw_spi_end(); 
        return; 
    }

    if(sc == PS2_SPECIAL_PRINT) {
        /* PrintScreen make: E0 12 E0 7C */
        mcu_hw_spi_tx_u08(0xE0);
        mcu_hw_spi_tx_u08(0x12);
        mcu_hw_spi_tx_u08(0xE0);
        mcu_hw_spi_tx_u08(0x7C);
        mcu_hw_spi_end();
        return;
    }

    if(sc == PS2_SPECIAL_PAUSE) {
        /* Pause make */
        mcu_hw_spi_tx_u08(0xE1); 
        mcu_hw_spi_tx_u08(0x14); 
        mcu_hw_spi_tx_u08(0x77);
        mcu_hw_spi_tx_u08(0xE1); 
        mcu_hw_spi_tx_u08(0xF0); 
        mcu_hw_spi_tx_u08(0x14);
        mcu_hw_spi_tx_u08(0xF0); 
        mcu_hw_spi_tx_u08(0x77);
        mcu_hw_spi_end();
        return;
    }

    if(sc & PS2_E0) mcu_hw_spi_tx_u08(0xE0);

    mcu_hw_spi_tx_u08((uint8_t)(sc & 0xFF));
    mcu_hw_spi_end();
}

void ps2_break_sc(uint8_t byte, uint8_t mod)
{
    mcu_hw_spi_begin();
    mcu_hw_spi_tx_u08(SPI_TARGET_HID);
    mcu_hw_spi_tx_u08(SPI_HID_KEYBOARD);
    if (mod != 0) mcu_hw_spi_tx_u08(0x80 | byte+0x68);
        else mcu_hw_spi_tx_u08(0x80 | byte);

    uint16_t sc = hid_to_ps2_set2(byte);

    if(sc == PS2_NONE) { 
        mcu_hw_spi_end(); 
        return; 
    }

    if(sc == PS2_SPECIAL_PRINT) {
        /* PrintScreen break */
        mcu_hw_spi_tx_u08(0xE0); 
        mcu_hw_spi_tx_u08(0xF0); 
        mcu_hw_spi_tx_u08(0x7C);
        mcu_hw_spi_tx_u08(0xE0); 
        mcu_hw_spi_tx_u08(0xF0); 
        mcu_hw_spi_tx_u08(0x12);
        mcu_hw_spi_end();
        return;
    }

    if(sc == PS2_SPECIAL_PAUSE) {
        /* Pause has no break */
        mcu_hw_spi_end();
        return;
    }

    if(sc & PS2_E0) mcu_hw_spi_tx_u08(0xE0);

    mcu_hw_spi_tx_u08(0xF0);
    mcu_hw_spi_tx_u08((uint8_t)(sc & 0xFF));
    mcu_hw_spi_end();
}


/* HID(Usage Page 0x07) -> PS/2 Set 2 MAKE scancode.
   Returns:
     0      : unmapped/unsupported
     PS2_E0 | xx : extended
     PS2_SPECIAL_* for PrintScreen/Pause */
uint16_t hid_to_ps2_set2(uint8_t hid)
{
  switch(hid) {
    /* Letters */
    case 0x04: return 0x1C; case 0x05: return 0x32; case 0x06: return 0x21; case 0x07: return 0x23;
    case 0x08: return 0x24; case 0x09: return 0x2B; case 0x0A: return 0x34; case 0x0B: return 0x33;
    case 0x0C: return 0x43; case 0x0D: return 0x3B; case 0x0E: return 0x42; case 0x0F: return 0x4B;
    case 0x10: return 0x3A; case 0x11: return 0x31; case 0x12: return 0x44; case 0x13: return 0x4D;
    case 0x14: return 0x15; case 0x15: return 0x2D; case 0x16: return 0x1B; case 0x17: return 0x2C;
    case 0x18: return 0x3C; case 0x19: return 0x2A; case 0x1A: return 0x1D; case 0x1B: return 0x22;
    case 0x1C: return 0x35; case 0x1D: return 0x1A;

    /* Number row */
    case 0x1E: return 0x16; case 0x1F: return 0x1E; case 0x20: return 0x26; case 0x21: return 0x25;
    case 0x22: return 0x2E; case 0x23: return 0x36; case 0x24: return 0x3D; case 0x25: return 0x3E;
    case 0x26: return 0x46; case 0x27: return 0x45;

    /* Controls */
    case 0x28: return 0x5A; /* Enter */
    case 0x29: return 0x76; /* ESC */
    case 0x2A: return 0x66; /* Backspace */
    case 0x2B: return 0x0D; /* Tab */
    case 0x2C: return 0x29; /* Space */
    case 0x39: return 0x58; /* Caps Lock */

    /* Punctuation */
    case 0x2D: return 0x4E; /* - */
    case 0x2E: return 0x55; /* = */
    case 0x2F: return 0x54; /* [ */
    case 0x30: return 0x5B; /* ] */
    case 0x31: return 0x5D; /* \ (US) */
    case 0x33: return 0x4C; /* ; */
    case 0x34: return 0x52; /* ' */
    case 0x35: return 0x0E; /* ` */
    case 0x36: return 0x41; /* , */
    case 0x37: return 0x49; /* . */
    case 0x38: return 0x4A; /* / */

    /* Non-US 102nd key (often "< > |" on ISO keyboards) */
    case 0x64: return 0x61;

    /* Function keys */
    case 0x3A: return 0x05; case 0x3B: return 0x06; case 0x3C: return 0x04; case 0x3D: return 0x0C;
    case 0x3E: return 0x03; case 0x3F: return 0x0B; case 0x40: return 0x83; case 0x41: return 0x0A;
    case 0x42: return 0x01; case 0x43: return 0x09; case 0x44: return 0x78; case 0x45: return 0x07;

    /* System cluster */
    case 0x46: return PS2_SPECIAL_PRINT; /* Print Screen */
    case 0x47: return 0x7E;             /* Scroll Lock */
    case 0x48: return PS2_SPECIAL_PAUSE; /* Pause */

    /* Insert block (E0) */
    case 0x49: return PS2_E0 | 0x70; /* Insert */
    case 0x4A: return PS2_E0 | 0x6C; /* Home */
    case 0x4B: return PS2_E0 | 0x7D; /* Page Up */
    case 0x4C: return PS2_E0 | 0x71; /* Delete */
    case 0x4D: return PS2_E0 | 0x69; /* End */
    case 0x4E: return PS2_E0 | 0x7A; /* Page Down */

    /* Arrows (E0) */
    case 0x4F: return PS2_E0 | 0x74; /* Right */
    case 0x50: return PS2_E0 | 0x6B; /* Left */
    case 0x51: return PS2_E0 | 0x72; /* Down */
    case 0x52: return PS2_E0 | 0x75; /* Up */

    /* Keypad */
    case 0x53: return 0x77;           /* Num Lock */
    case 0x54: return PS2_E0 | 0x4A;  /* KP / */
    case 0x55: return 0x7C;           /* KP * */
    case 0x56: return 0x7B;           /* KP - */
    case 0x57: return 0x79;           /* KP + */
    case 0x58: return PS2_E0 | 0x5A;  /* KP Enter */
    case 0x59: return 0x69;           /* KP 1 */
    case 0x5A: return 0x72;           /* KP 2 */
    case 0x5B: return 0x7A;           /* KP 3 */
    case 0x5C: return 0x6B;           /* KP 4 */
    case 0x5D: return 0x73;           /* KP 5 */
    case 0x5E: return 0x74;           /* KP 6 */
    case 0x5F: return 0x6C;           /* KP 7 */
    case 0x60: return 0x75;           /* KP 8 */
    case 0x61: return 0x7D;           /* KP 9 */
    case 0x62: return 0x70;           /* KP 0 */
    case 0x63: return 0x71;           /* KP . */

    /* Application/Menu key (E0) */
    case 0x65: return PS2_E0 | 0x2F;

    default:   return PS2_NONE;
  }
}
