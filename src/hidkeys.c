/*
  hidkeys.c

  named map of hid key codes
*/

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>    // for strtol
#include <ctype.h>     // for tolower

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
  "KP Enter",   // 58
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

  "Menu",       // 65
  "Power",      // 66
  "KP =",       // 67

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

// parse a string as a decimal numeric or as a hex
static uint16_t parse_numeric_str(const char *str) {
  int may_be_dec = 1;
  int may_be_hex = 1;

  const char *p = str;

  // first char is a dollar sign: cannot be decimal, rest
  // may be hex
  if(*p == '$') {
    may_be_dec = 0;
    p++;
    str++;
  }
  
  while(*p) {
    if(!isdigit((int)*p)) {
      // not a digit: cannot be decimal
      may_be_dec = 0;

      // also not a hex character: cannot be hex
      if(!((*p >= 'a' && *p <= 'f') || (*p >= 'A' && *p <= 'F')))
	may_be_hex = 0;
    }
    p++;
  }

  if(may_be_dec)       return strtol(str, NULL, 10);
  else if(may_be_hex)  return strtol(str, NULL, 16);
  return 0;
}

static int strcasecmp_nospace(const char *a, const char *b, char extra_term) {
  for (;;) {
    while (isspace((unsigned char)*a)) a++;
    while (isspace((unsigned char)*b)) b++;
    if (!*a || *a == extra_term ||   // reached end of string a
	!*b || *b == extra_term ||   // reached end of string b
	tolower((unsigned char)*a) != tolower((unsigned char)*b)) // strings differ
      break;
    a++;
    b++;
  }

  // check if both strings are on terminator
  if((!*a || *a == extra_term) && (!*b || *b == extra_term))
    return 0;
  
  return tolower((unsigned char)*a) - tolower((unsigned char)*b);
}

uint8_t hidkeys_get_code(const char *str) {

  // key strings may start with colon (:) seperated
  // modifier keys
  
  // parse key names before trying to parse as numeric, since
  // e.g. the key 'F8' is a valid hex num and the number
  // keys are, anyways ...
  
  for(int i=0;i<0x70;i++) 
    if(hid_keys[i])
      if(strcasecmp_nospace(str, hid_keys[i], '\0') == 0)
	return i;

  // try to parse as numeric
  int num = parse_numeric_str(str);
  if(num) return num;
  
  return 0;  // 0 is not a valid hid key code
}

static const char *modifier_keys[] = {
  // regular single keys
  "Left Ctrl",  "Left Shift",  "Left Alt",  "Left Meta",
  "Right Ctrl", "Right Shift", "Right Alt", "Right Meta"
};

int hidkeys_get_modifier_name(char *str, uint8_t mod) {
  int len = 0;

  // first of all, terminate the string. This will also make
  // sure that a valid empty string is returned if no modifier
  // is set
  if(str) *str = '\0';
  
  for(int i=0;i<8;i++) {
    if(mod & (1<<i)) {
      if(str) {
	strcat(str, modifier_keys[i]);
	strcat(str, ":");
      }
      len += strlen(modifier_keys[i]) + 1;
    }
  }

  return len;
}

uint8_t hidkeys_get_code_with_modifier(const char *str, uint8_t *modifier) {
  *modifier = 0;
  
  // key strings may start with colon (:) seperated list of
  // modifier keys
  while(strchr(str, ':')) {
    int i;
    for(i=0;i<8;i++) {
      if(strcasecmp_nospace(str, modifier_keys[i], ':') == 0) {
	*modifier |= (1<<i);

	// skip behind ':'
	while(*str && *str != ':') str++;
	str++;
	  
	// skip whitespaces
	while(*str && isspace((int)*str)) str++;

	break;
      }
    }

    // if i has reached 8, then the modfier could not
    // be parsed. So just skip to the next one
    if(i == 8) {
      // skip behind ':'
      while(*str && *str != ':') str++;
      str++;
      
      // skip whitespaces
      while(*str && isspace((int)*str)) str++;
    }
    
  }

  // the main key may still be a modifier and so can the target. You
  // can thus map e.g. shift + left control to right control
  return hidkeys_get_code(str);
}

const char *hidkeys_get_name(uint8_t code) {
  if(code < 0x70)
    return hid_keys[code];
  
  return NULL;
}

static const char *hid_buttons[] = {
  "Right", "Left", "Down", "Up",       // direction bits in the "joy" byte
  "A", "B", "X", "Y",                  // button bits in the "joy" byte
  "Left Shoulder", "Right Shoulder",   // bits in "btn_extra" byte
  "Select", "Start",                   // -"-
  "<btn12>", "<bnt13>", "<btn14>", "<btn15>",  // so far not used
  NULL
};  

int hidkeys_get_btnmap(const char *str) {
  int map = 0;

  // the button map may be a list of button names seperated by ':'
  const char *p = str;
  while(*p) {
    int i;
    
    for(i=0;hid_buttons[i];i++) {
      if(strcasecmp_nospace(p, hid_buttons[i], ':') == 0) {
	map |= 1<<i;

	// forward to next ':' or end of string
	while(*p && *p != ':') p++;

	// and skip the ':' if there was one
	if(*p == ':') p++;
	
	// skip any whitespace
	while(*p && isspace((int)*p)) p++;
	
	break;
      }
    }

    // if one entry could not be found, then the whole comparison failed
    if(!hid_buttons[i]) {  
      // check is str actually is a decimal or hex string
      return parse_numeric_str(str);
    }
  }

  return map;
}

// convert joystick button map into a human-readable string. This writes
// to buffer str (if not NULL) and returns the length of the resulting string
int hidkeys_get_btnmap_str(char *str, uint16_t map) {
  int retval = 0;
  
  for(int i=0;i<15;i++) {
    if(map & (1<<i)) {
      if(!retval) {
	retval += strlen(hid_buttons[i]);
	if(str) strcpy(str, hid_buttons[i]);
      } else {
        retval += strlen(hid_buttons[i]) + 1;
	if(str) {
	  strcat(str, ":");
	  strcat(str, hid_buttons[i]);
	}
      }
    }
  }

  return retval;
}
