/*
  core_atarist.c

  Atari ST specific menus, keytables etc
*/

#include "core_atarist.h"
#include "sdc.h"
#include "debug.h"

#ifdef ENABLE_LEGACY_ATARIST
const char *core_atarist_default_images[] = {
  CARD_MOUNTPOINT "/disk_a.st",
  CARD_MOUNTPOINT "/disk_b.st",
  CARD_MOUNTPOINT "/acsi_0.hd",
  CARD_MOUNTPOINT "/acsi_1.hd",
  NULL
};

static const char main_form[] =
  "<MiSTeryNano>,;"                     // main form has no parent
  // --------
  "F,Disk A:,0|st;"                     // fileselector for Disk A:
  "S,System,1;"                         // System submenu is form 1
  "S,Drives,2;"                         // Storage submenu
  "S,Settings,3;"                       // Settings submenu is form 3
  "B,Reset,R;";                         // system reset

static const char system_form[] =
  "System,0|2;"                         // return to form 0, entry 2
  // --------
  "L,Chipset:,ST|Mega ST|STE,C;"        // selection stored in variable "C"
  "L,Memory:,4MB|8MB,M;"                // ...
  "L,Video:,Color|Mono,V;"
  "L,Cartridge:,None|Cubase 2&3,Q;"     // Cubase dongle support
  "L,Mouse:,USB|Atari ST|Amiga,J;"      // Mouse (DB9) mapping
  "L,USB Joy:,Port1/Port0,Port0/Port1,U;" //USB Joys order
  "L,TOS Slot:,Primary|Secondary,T;"    // Select TOS
  "B,Cold Boot,B;";                     // system reset with memory reset

static const char storage_form[] =
  "Drives,0|3;"                         // return to form 0, entry 3
  // --------
  "F,Disk A:,0|st;"                     // fileselector for Disk A:
  "F,Disk B:,1|st;"                     // fileselector for Disk B:
  "F,ACSI #0:,2|hd+img;"                // fileselector for ACSI 0
  "F,ACSI #1:,3|hd+img;"                // fileselector for ACSI 1
  "L,Disk prot.:,None|A:|B:|Both,P;";   // Enable/Disable Floppy write protection

static const char settings_form[] =
  "Settings,0|4;"                       // return to form 0, entry 4
  // --------
  "L,Screen:,Normal|Wide,W;"
  "L,Scanlines:,None|25%|50%|75%,S;"
  "L,Volume:,Mute|33%|66%|100%,A;"
  "B,Save settings,S;";

const char *core_atarist_forms[] = {
  main_form,
  system_form,
  storage_form,
  settings_form
};

// variable ids must match the ones in the menu string
menu_legacy_variable_t core_atarist_variables[] = {
  { 'C', { 0 }},    // default chipset = ST
  { 'M', { 0 }},    // default memory = 4MB
  { 'V', { 0 }},    // default video = color
  { 'S', { 0 }},    // default scanlines = none
  { 'A', { 1 }},    // default volume = 33%
  { 'W', { 0 }},    // default normal (4:3) screen
  { 'P', { 0 }},    // default no floppy write protected
  { 'Q', { 0 }},    // default cubase dongle not enabled
  { 'J', { 0 }},    // default mouse USB, DB9 connector joystick
  { 'U', { 0 }},    // default Port1/Port0 assingned for USB Joys
  { 'T', { 0 }},    // default primary TOS slot
  { '\0',{ 0 }}
};

#define MISS          (0)
#define MATRIX(a,b)   (b*16+a)

const unsigned char core_atarist_keymap[] = {
  MISS,         // 00: NoEvent
  MISS,         // 01: Overrun Error
  MISS,         // 02: POST fail
  MISS,         // 03: ErrorUndefined
 
  // characters
  MATRIX( 4,5), // 04: a
  MATRIX( 7,6), // 05: b
  MATRIX( 6,6), // 06: c
  MATRIX( 5,6), // 07: d
  MATRIX( 5,4), // 08: e
  MATRIX( 6,5), // 09: f
  MATRIX( 7,4), // 0a: g
  MATRIX( 7,5), // 0b: h
  MATRIX( 8,4), // 0c: i
  MATRIX( 8,5), // 0d: j
  MATRIX( 8,6), // 0e: k
  MATRIX( 9,5), // 0f: l
  MATRIX( 8,7), // 10: m
  MATRIX( 7,7), // 11: n
  MATRIX( 9,3), // 12: o
  MATRIX( 9,4), // 13: p
  MATRIX( 4,4), // 14: q
  MATRIX( 6,3), // 15: r
  MATRIX( 5,5), // 16: s
  MATRIX( 6,4), // 17: t
  MATRIX( 8,3), // 18: u
  MATRIX( 6,7), // 19: v
  MATRIX( 5,3), // 1a: w
  MATRIX( 5,7), // 1b: x
  MATRIX( 7,3), // 1c: y
  MATRIX( 4,7), // 1d: z

  // top number key row
  MATRIX( 4,2), // 1e: 1
  MATRIX( 5,1), // 1f: 2
  MATRIX( 5,2), // 20: 3
  MATRIX( 6,1), // 21: 4
  MATRIX( 6,2), // 22: 5
  MATRIX( 7,1), // 23: 6
  MATRIX( 7,2), // 24: 7
  MATRIX( 8,1), // 25: 8
  MATRIX( 8,2), // 26: 9
  MATRIX( 9,1), // 27: 0
  
  // other keys
  MATRIX(11,5), // 28: return
  MATRIX( 4,1), // 29: esc
  MATRIX(11,1), // 2a: backspace
  MATRIX( 4,3), // 2b: tab		  
  MATRIX( 9,7), // 2c: space

  MATRIX( 9,2), // 2d: -
  MATRIX(10,1), // 2e: =
  MATRIX(10,3), // 2f: [			  
  MATRIX(10,4), // 30: ]
  MATRIX(11,4), // 31: backslash 
  MATRIX(11,4), // 32: backslash on some eur keyboards(near enter)
  MATRIX(10,5), // 33: ;
  MATRIX(11,6), // 34: ' 
  MATRIX(10,2), // 35: `
  MATRIX( 9,6), // 36: ,
  MATRIX(10,6), // 37: .
  MATRIX(11,7), // 38: /
  MATRIX(10,7), // 39: caps lock
  
  // function keys
  MATRIX( 1,0), // 3a: F1
  MATRIX( 2,0), // 3b: F2
  MATRIX( 3,0), // 3c: F3
  MATRIX( 4,0), // 3d: F4
  MATRIX( 5,0), // 3e: F5
  MATRIX( 6,0), // 3f: F6
  MATRIX( 7,0), // 40: F7
  MATRIX( 8,0), // 41: F8
  MATRIX( 9,0), // 42: F9
  MATRIX(10,0), // 43: F10
  MATRIX(10,0), // 44: F11
  MATRIX(10,0), // 45: F12

  MATRIX(13,0), // 46: PrtScr -> KP-(
  MISS,         // 47: Scroll Lock
  MISS,         // 48: Pause
  MATRIX(11,3), // 49: Insert
  MATRIX(12,2), // 4a: Home
  MATRIX(11,0), // 4b: PageUp -> HELP
  MATRIX(11,2), // 4c: Delete
  MATRIX(13,1), // 4d: End -> KP-)
  MATRIX(12,0), // 4e: PageDown -> UNDO
  
  // cursor keys
  MATRIX(12,5), // 4f: right
  MATRIX(12,3), // 50: left
  MATRIX(12,4), // 51: down
  MATRIX(12,1), // 52: up
  
  MISS,         // 53: Num Lock

  // keypad
  MATRIX(14,0), // 54: KP /
  MATRIX(14,1), // 55: KP *
  MATRIX(14,3), // 56: KP -
  MATRIX(14,5), // 57: KP +
  MATRIX(14,7), // 58: KP Enter
  MATRIX(12,6), // 59: KP 1
  MATRIX(13,6), // 5a: KP 2
  MATRIX(14,6), // 5b: KP 3
  MATRIX(13,4), // 5c: KP 4
  MATRIX(13,5), // 5d: KP 5
  MATRIX(14,4), // 5e: KP 6
  MATRIX(13,2), // 5f: KP 7
  MATRIX(13,3), // 60: KP 8
  MATRIX(14,2), // 61: KP 9
  MATRIX(12,7), // 62: KP 0
  MATRIX(13,7), // 63: KP .
  MATRIX( 4,6), // 64: EUR-2
};  

const unsigned char core_atarist_modifier[] = {
  /* usb modifer bits:
     0     1      2    3    4     5      6    7
     LCTRL LSHIFT LALT LGUI RCTRL RSHIFT RALT RGUI
  */

  MATRIX( 0,4), // ctrl
  MATRIX( 1,5), // lshift
  MATRIX( 2,6), // alt
  MISS,
  MATRIX( 0,4), // ctrl (right)
  MATRIX( 3,7), // rshift
  MISS,
  MISS
};
#endif
