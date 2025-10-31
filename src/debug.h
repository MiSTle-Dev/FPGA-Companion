#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>

#define debugf(x, ...)  printf(x "\r\n", ##__VA_ARGS__)

#define ini_debugf(a, ...)  debugf("\033[0;31mINI: " a "\033[0m", ##__VA_ARGS__)  // red
#define sys_debugf(a, ...)  debugf("\033[0;32mSYS: " a "\033[0m", ##__VA_ARGS__)  // green
#define sdc_debugf(a, ...)  debugf("\033[0;33mSDC: " a "\033[0m", ##__VA_ARGS__)  // yellow
// #define usb_debugf(a, ...)  debugf("\033[0;34mUSB: " a "\033[0m", ##__VA_ARGS__)  // blue -> too dark to read
#define usb_debugf(a, ...)  debugf("\033[0;36mUSB: " a "\033[0m", ##__VA_ARGS__)  // cyan
#define hidp_debugf(a, ...) debugf("\033[0;35mHDP: " a "\033[0m", ##__VA_ARGS__)  // magenta
#define osd_debugf(a, ...)  debugf("\033[0;36mOSD: " a "\033[0m", ##__VA_ARGS__)  // cyan
#define menu_debugf(a, ...) debugf("\033[1;33mMNU: " a "\033[0m", ##__VA_ARGS__)  // bold yellow
#define jtag_debugf(a, ...)  debugf("\033[0;36mJTAG: " a "\033[0m", ##__VA_ARGS__)  // cyan

#include <ctype.h>
static inline void hexdump(const void *data, int size) {
  int i, b2c;
  int n=0;
  char *ptr = (char*)data;
  
  if(!size) return;
  
  while(size>0) {
    printf("%04x: ", n);

    b2c = (size>16)?16:size;
    for(i=0;i<b2c;i++)      printf("%02x ", 0xff&ptr[i]);
    printf("  ");
    for(i=0;i<(16-b2c);i++) printf("   ");
    for(i=0;i<b2c;i++)      printf("%c", isprint((int)ptr[i])?ptr[i]:'.');
    printf("\r\n");
    ptr  += b2c;
    size -= b2c;
    n    += b2c;
  }
}

#endif // DEBUG_H
