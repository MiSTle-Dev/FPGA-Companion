/*
  at_wifi.c

  background process sending and receiving data from FPGA "port" which
  usually us serial/rs232 data. This is parsed to provice an hayes at
  command like interface to e.g. wifi
*/

#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#else
#include <FreeRTOS.h>
#include <timers.h>
#include <task.h>
#include <queue.h>
#endif

#include "at_wifi.h"

#include <string.h>
#include <stdlib.h>

#include "sysctrl.h"

#include "debug.h"
#include "mcu_hw.h"

#define AT_WIFI_STATE_OFFLINE      0
#define AT_WIFI_STATE_ONLINE       1
#define AT_WIFI_STATE_USER_OFFLINE 2

static int at_wifi_state = AT_WIFI_STATE_OFFLINE;
static unsigned int petsc2 = 0;  // default ASC-2 mode

static int at_wifi_escape_state = 0;
static TickType_t at_wifi_escape_tick = 0;

static QueueHandle_t rx_queue = NULL;

static void port_putc(unsigned char byte) {
  at_wifi_escape_tick = xTaskGetTickCount();  // reset idle counter
  sys_port_write(0, &byte, 1);
}

void at_wifi_puts(const char *str) {
  at_wifi_escape_tick = xTaskGetTickCount();
  sys_port_write(0, (const unsigned char*)str, strlen(str));
}

void at_wifi_puts_n(const char *str, int len) {
  at_wifi_escape_tick = xTaskGetTickCount();
  // this is actually being used by the hardware layer to send data into the core
  sys_port_write(0, (const unsigned char*)str, len);
}

static void port_line(char *command) {
  if(strcasecmp(command, "athelp") == 0) {
    at_wifi_puts("Supported commands:\r\n");
    at_wifi_puts("  atscan                     - scan for WiFi networks\r\n");
    at_wifi_puts("  atssid <ssid>,<passphrase> - connect to WiFi network\r\n");
    at_wifi_puts("  atd <server>:<port>        - connect to <server> via TCP <port>\r\n");
    at_wifi_puts("  ato                        - re-enter online mode\r\n");
    at_wifi_puts("  ath                        - hang-up / disconnect from server\r\n");
    at_wifi_puts("  <1sec>+++<1sec>            - escape to offline mode\r\n");
    at_wifi_puts("  atpetscii                  - petscii input\r\n");
    at_wifi_puts("  atascii                    - ascii input\r\n");
  } else if(strcasecmp(command, "atscan") == 0) {
    mcu_hw_wifi_scan();
  } else if(strncasecmp(command, "atssid", 6) == 0) {
    // skip to ssid
    char *s = command+6;
    while(*s == ' ') s++;
    // find comma in command
    char *k = s;
    while(*k && *k != ',') k++;
    if(*k == ',') *k++ = '\0';

    // do petsci translation if requested
    if (petsc2 == 1) {
      int len = strlen(s);
      for (int i = 0; i < len; i++) {
        s[i] = pet2asc(s[i]);
      }
      len = strlen(k);
      for (int i = 0; i < len; i++) {
        k[i] = pet2asc(k[i]);
      }
    }
    mcu_hw_wifi_connect(s, k);
    
  } else if(strncasecmp(command, "atd", 3) == 0) {
    char *s = command+3;
    while(*s == ' ') s++;
    // find ':' in command
    char *k = s;
    while(*k && *k != ':') k++;    // skip to port
    if(*k == ':') *k++ = '\0';

    // do petsci ascii translation if requested
    if (petsc2 == 1) {
      int len = strlen(s);
      for (int i = 0; i < len; i++)
        s[i] = pet2asc(s[i]);
    }
    
    mcu_hw_tcp_connect(s, atoi(k));
  } else if(strncasecmp(command, "ato", 3) == 0) {
    if(at_wifi_state == AT_WIFI_STATE_USER_OFFLINE) {
      at_wifi_puts("OK\r\n");
      at_wifi_state = AT_WIFI_STATE_ONLINE;
    }
  } else if(strncasecmp(command, "atpetscii", 9) == 0) {
      petsc2 = 1;
      at_wifi_puts("petsc-2\r\n");
  } else if(strncasecmp(command, "atascii", 7) == 0) {
      petsc2 = 0;
      at_wifi_puts("asc-2\r\n");
  } else if(strncasecmp(command, "ath", 3) == 0) {
    at_wifi_puts("OK\r\n");
    mcu_hw_tcp_disconnect();
  } else
    at_wifi_puts("Unknown command, try 'athelp'\r\n");
}

// Port implements an "AT" like interface to e.g. be used
// to control a WiFi modem
void at_wifi_port_byte(unsigned char byte) {
  xQueueSendToBack(rx_queue, &byte, ( TickType_t ) 10);
}

#define INPUT_BUFFER_SIZE 64

static void at_wifi_tx(unsigned char byte) {
  static char cmd[INPUT_BUFFER_SIZE] = "";  // may include a full server name ...

  // actually try to send bytes unless the user explicitely entered manual offline mode
  if(at_wifi_state != AT_WIFI_STATE_USER_OFFLINE) {
  
    // try to send via tcp. Handle it locally if it isn't accepted
    if(mcu_hw_tcp_data(byte)) {
      // if we get here we are (still) connected
      
      if(at_wifi_state != AT_WIFI_STATE_ONLINE) {
	debugf("AT WIFI: online");
	at_wifi_state = AT_WIFI_STATE_ONLINE;
	at_wifi_escape_state = 0;
      }
      
      if(at_wifi_state == AT_WIFI_STATE_ONLINE) {
	// user may want to go into offline mode
	// this is triggered by transmitting <1 sec pause>+++<1 sec pause>
	switch(at_wifi_escape_state) {
	case 0:
	  // at least a pause of 1 second? Good, expect +++
	  if(at_wifi_escape_tick && (xTaskGetTickCount() - at_wifi_escape_tick) > 1000) {
	    at_wifi_escape_tick = 0;
	    at_wifi_escape_state = 1;
	  }
	  break;
	
	case 1:
	case 2:
	  // more than 1 second before second or third '+'? 
	  if(at_wifi_escape_tick && ((xTaskGetTickCount() - at_wifi_escape_tick) > 1000)) {
	    at_wifi_escape_tick = 0;
	    at_wifi_escape_state = 1;
	  } else if(byte == '+') {
	    at_wifi_escape_tick = xTaskGetTickCount();
	    at_wifi_escape_state++;
	  }
	  break;
	  
	case 3:
	  if((xTaskGetTickCount() - at_wifi_escape_tick) > 1000) {
	    at_wifi_escape_tick = 0;
	    at_wifi_escape_state = 0;
	    at_wifi_state = AT_WIFI_STATE_USER_OFFLINE;
	    strcpy(cmd, "");
	  }
	}
      }
      
      // if we are still in regular online mode return now and parse nothing
      if(at_wifi_state == AT_WIFI_STATE_ONLINE)    
	return;
    
    } else {
      // if we get here we have just been disconnected
      if(at_wifi_state != AT_WIFI_STATE_OFFLINE) {
	debugf("AT WIFI: offline");
	at_wifi_state = AT_WIFI_STATE_OFFLINE;
      }    
    }
  }
    
  // echo byte back. Echo everything but newlines. In case
  // of cr append a newline  
  if(byte != '\n') {  
    port_putc(byte);
    if(byte == '\r') port_putc('\n');
  }

  // cr will always return into state 0
  if(byte == '\r') {
    port_line(cmd);
    strcpy(cmd, "");
  } else {
    // ignore any newline
    if((byte != '\n') && strlen(cmd) < sizeof(cmd)-1) {
      cmd[strlen(cmd)+1] = '\0';
      cmd[strlen(cmd)] = byte;
    }
  }
}

static void at_wifi_task(__attribute__((unused)) void *parms) {
  menu_debugf("at/wifi task running");

  // wait for user events
  while(1) {
    unsigned char byte;
    if(xQueueReceive(rx_queue, &byte, pdMS_TO_TICKS(1000))) 
      at_wifi_tx(byte);
  }
}

void at_wifi_init(void) {
  debugf("AT WIFI init");

  // start a thread to handle at/wifi io
  rx_queue = xQueueCreate(8, sizeof( unsigned char ) );
  xTaskCreate(at_wifi_task, (char *)"at_wifi_task", 2048, NULL, configMAX_PRIORITIES-10, NULL);
}

uint8_t pet2asc(uint8_t c) {
    if ('A' <=  c && c <= 'Z')
      c += 'a' - 'A';
    else if ('a' <=  c && c <= 'z')
      c -= 'a' - 'A';
    else if (192 <= c && c <= 223) 
      c -= 96;
    else if (224 <= c && c <= 254)
      c -= 64;
  
    return c;
  }

uint8_t asc2pet(uint8_t c) {
  if (c > 64 && c < 91)
    c += 128;
  else if (c > 96 && c < 123)
    c -= 32;
  else if (c > 192 && c < 219)
    c -= 128;

  return c;
}
