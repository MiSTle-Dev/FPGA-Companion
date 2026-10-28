/*
  hidkeys.c

  named map of hid key codes
*/

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <strings.h>
#include "hidkeys.h"

// characters
static const char *hid_keys[] = {
  NULL,  // 00
  NULL,  // 01
  NULL,  // 02
  NULL,  // 03

  // characters
  "A",   // 04
  "B",   // 05
  "C",   // 06
  "D",   // 07
  "E",   // 08
  "F",   // 09
  "G",   // 0a
  "H",   // 0b
  "I",   // 0c
  "J",   // 0d
  "K",   // 0e
  "L",   // 0f
  "M",   // 10
  "N",   // 11
  "O",   // 12
  "P",   // 13
  "Q",   // 14
  "R",   // 15
  "S",   // 16
  "T",   // 17
  "U",   // 18
  "V",   // 19
  "W",   // 1a
  "X",   // 1b
  "Y",   // 1c
  "Z",   // 1d
  
  // numbers
  "1",   // 1e
  "2",   // 1f
  "3",   // 20
  "4",   // 21
  "5",   // 22
  "6",   // 23
  "7",   // 24
  "8",   // 25
  "9",   // 26
  "0",   // 27
  
  // other keys
  "Enter",      // 28
  "Esc",        // 29
  "Back Space", // 2a
  "Tab",        // 2b
  "Space",      // 2c
  "-",          // 2d
  "=",          // 2e
  "[",          // 2f
  "]",          // 30
  "\\",         // 31
  "Eur 1",      // 32 backslash on some eur keyboards (near enter)
  ";",          // 33
  "' ",         // 34
  "`",          // 35
  ",",          // 36
  ".",          // 37
  "/",          // 38
  "Caps Lock",  // 39
  
  // function keys
  "F1",   // 3a
  "F2",   // 3b
  "F3",   // 3c
  "F4",   // 3d
  "F5",   // 3e
  "F6",   // 3f
  "F7",   // 40
  "F8",   // 41
  "F9",   // 42
  "F10",  // 43
  "F11",  // 44
  "F12",  // 45

  "Print Scr",  // 46
  "Scroll Lock",// 47
  "Pause",      // 48
  "Insert",     // 49
  "Home",       // 4a
  "Page Up",    // 4b
  "Delete",     // 4c
  "End",        // 4d
  "Page Down",  // 4e

  // cursor keys
  "Right",      // 4f
  "Left",       // 50
  "Down",       // 51
  "Up",         // 52

  "Num Lock",   // 53

  // keypad
  "KP /",       // 54
  "KP *",       // 55
  "KP -",       // 56
  "KP +",       // 57
  "KP enter",   // 58
  "KP 1",       // 59
  "KP 2",       // 5a
  "KP 3",       // 5b
  "KP 4",       // 5c
  "KP 5",       // 5d
  "KP 6",       // 5e
  "KP 7",       // 5f
  "KP 8",       // 60
  "KP 9",       // 61
  "KP 0",       // 62
  "KP .",       // 63
  "Eur 2",      // 64

  NULL,         // 65
  NULL,         // 66
  NULL,         // 67

  // remapped modifier keys
  "Left Ctrl",   // 68
  "Left Shift",  // 69
  "Left Alt",    // 6a
  "Left Meta",   // 6b
  "Right Ctrl",  // 6c
  "Right Shift", // 6d
  "Right Alt",   // 6e
  "Right Meta"   // 6f
};

int8_t hidkeys_get_code(const char *str) {
  for(int i=0;i<0x70;i++) {
    if(hid_keys[i])
      if(strcasecmp(str, hid_keys[i]) == 0)
      return i;
  }
  return -1;
}

const char *hidkeys_get_name(int8_t code) {
  if(code < 0x70)
    return hid_keys[code];
  
  return NULL;
}
