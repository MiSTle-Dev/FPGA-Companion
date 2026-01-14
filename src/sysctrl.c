/*
  sysctrl.c

  MiSTeryNano system control interface
*/

#include <stdio.h>
#include <stdlib.h>

#include "spi.h"
#include "sysctrl.h"
#include "sdc.h"
#include "osd.h"
#include "menu.h"
#include "inifile.h"

#include "debug.h"
#include "config.h"
#include "mcu_hw.h"
#include "at_wifi.h"

// we are using "puff" to decompress a gzip'd FPGA config as this is a slow, yet
// very small and memory efficient implementation of the deflate de-compression
#include "puff.h"

const char *sys_get_config_name(void) {
  static const char *config_xml =
    CARD_MOUNTPOINT "/config.xml";
  return config_xml;
}

static void sys_begin(unsigned char cmd) {
  mcu_hw_spi_begin();  
  mcu_hw_spi_tx_u08(SPI_TARGET_SYS);
  mcu_hw_spi_tx_u08(cmd);
}  

int sys_status_is_valid(void) {
  sys_begin(SPI_SYS_STATUS);
  mcu_hw_spi_tx_u08(0);
  unsigned char b0 = mcu_hw_spi_tx_u08(0);
  unsigned char b1 = mcu_hw_spi_tx_u08(0);
  unsigned char core_id = mcu_hw_spi_tx_u08(0);
  unsigned char coldboot = mcu_hw_spi_tx_u08(0);
  mcu_hw_spi_end();  

  if((b0 == 0x5c) && (b1 == 0x42)) {
    sys_debugf("Core ID: %02x", core_id);
    if(core_id)
      sys_debugf("Warning: Core ID should be 0. Is this an old legacy core?");

    // coldboot status equals core_id on cores not supporting cold
    // boot status
    if(coldboot != core_id) {
      // check colboot status
      sys_debugf("Coldboot status is %02x", coldboot);
    }
  }
  
  return((b0 == 0x5c) && (b1 == 0x42));
}

void sys_set_leds(char leds) {
  sys_begin(SPI_SYS_LEDS);
  mcu_hw_spi_tx_u08(leds);
  mcu_hw_spi_end();  
}

void sys_set_rgb(unsigned long rgb) {
  sys_begin(SPI_SYS_RGB);
  mcu_hw_spi_tx_u08((rgb >> 16) & 0xff); // R
  mcu_hw_spi_tx_u08((rgb >> 8) & 0xff);  // G
  mcu_hw_spi_tx_u08(rgb & 0xff);         // B
  mcu_hw_spi_end();    
}

unsigned char sys_get_buttons(void) {
  unsigned char btns = 0;
  
  sys_begin(SPI_SYS_BUTTONS);
  mcu_hw_spi_tx_u08(0x00);
  btns = mcu_hw_spi_tx_u08(0);
  mcu_hw_spi_end();

  return btns;
}

void sys_set_val(char id, uint8_t value) {
  sys_debugf("SYS set value %c = %d", id, value);
  
  sys_begin(SPI_SYS_SETVAL);   // send value command
  mcu_hw_spi_tx_u08(id);              // value id
  mcu_hw_spi_tx_u08(value);           // value itself
  mcu_hw_spi_end();  
}

unsigned char sys_irq_ctrl(unsigned char ack) {
  sys_begin(SPI_SYS_IRQ_CTRL);
  mcu_hw_spi_tx_u08(ack);
  unsigned char ret = mcu_hw_spi_tx_u08(0);
  mcu_hw_spi_end();  
  return ret;
}

static void sys_port_begin(unsigned char cmd) {
  sys_begin(SPI_SYS_PORT);
  mcu_hw_spi_tx_u08(cmd); 
}

static uint8_t sys_port_tx_free(unsigned char port) {
  sys_port_begin(SPI_SYS_PORT_STATUS);
  // first byte always returns the number of ports implemented by core
  uint8_t ports = mcu_hw_spi_tx_u08(port);

  // the follwing bytes are specific to the requested port
  uint8_t type = mcu_hw_spi_tx_u08(0);
  mcu_hw_spi_tx_u08(0);  // skip rx_available
  uint8_t tx_available = mcu_hw_spi_tx_u08(0);  
  mcu_hw_spi_end();

  // return false if there's no such port
  if(!ports || type != 0)
    return 0;

  // if(tx_available) printf("TX avail: %d\n", tx_available);
  return tx_available;
}
  
void sys_port_write(unsigned char port, const unsigned char *ptr, int len) {
  while(len) {
    // wait for tx buffer space to become available. This may actually
    // wait forever when no program running in the core actually reads
    // port data
    uint8_t bytes2send = sys_port_tx_free(port);
    while(!bytes2send) {
      vTaskDelay(pdMS_TO_TICKS(1));
      bytes2send = sys_port_tx_free(port);
    }

    // don't send more bytes than still left to be sent
    if(bytes2send > len) bytes2send = len;
    
    // send as many bytes as space in buffer
    sys_port_begin(SPI_SYS_PORT_PUT); // port command: send byte(s)
    mcu_hw_spi_tx_u08(port);
    for(int i=0;i<bytes2send;i++)
      mcu_hw_spi_tx_u08(*ptr++);
    mcu_hw_spi_end();

    len -= bytes2send;
  }
}

bool sys_port_get_status(unsigned char port) {
  static const char *parity_str[] = { "N", "O", "E", "?" }; 
  static const char *stop_str[] = { "1", "1.5", "2", "?" }; 
  struct port_serial_status serial_status;

  sys_port_begin(SPI_SYS_PORT_STATUS);
  // first byte always returns the number of ports implemented by core
  uint8_t ports = mcu_hw_spi_tx_u08(port);

  // the follwing bytes are specific to the requested port
  uint8_t type = mcu_hw_spi_tx_u08(0);
  uint8_t rx_available = mcu_hw_spi_tx_u08(0);
  uint8_t tx_available = mcu_hw_spi_tx_u08(0);
  
  // read further info in case it's a serial port (type == 0)
  if(type == 0) {
    // read additional 32 bit
    uint8_t *ptr = (uint8_t*)&serial_status;
    for(int i=0;i<4;i++) *ptr++ = mcu_hw_spi_tx_u08(0);    
  }

  mcu_hw_spi_end();

  debugf("Number of ports: %d", ports);

  // return -1 if there's no such port
  if(!ports || type != 0)
    return false;

  debugf("Port 0: Type = %d, rx_available = %d, tx_available = %d", type, rx_available, tx_available);
  
  if(type == 0) {
    debugf("  Serial status:");
    debugf("  Bitrate:  %d", serial_status.bitrate);
    debugf("  Databits: %d", serial_status.databits);
    debugf("  Partity:  %s", parity_str[serial_status.parity]);
    debugf("  Stopbits: %s", stop_str[serial_status.stopbits]);
  }
  
  return true;
}
  
static uint8_t sys_port_rx_available(unsigned char port) {
  sys_port_begin(SPI_SYS_PORT_STATUS);
  // first byte always returns the number of ports implemented by core
  uint8_t ports = mcu_hw_spi_tx_u08(port);

  // the follwing bytes are specific to the requested port
  uint8_t type = mcu_hw_spi_tx_u08(0);
  uint8_t rx_available = mcu_hw_spi_tx_u08(0);
  
  mcu_hw_spi_end();

  // return false if there's no such port
  if(!ports || type != 0)
    return 0;

  return rx_available;
}
  
// read status and byte from port out
static int16_t sys_port_get(unsigned char port) {
  uint8_t rx_avail = sys_port_rx_available(0);
  if(!rx_avail) return -1;           // no such port or no data available

  // TODO: we can actually read more than one byte at a time with this
  sys_port_begin(SPI_SYS_PORT_GET);  // port command: get byte
  mcu_hw_spi_tx_u08(port);
  mcu_hw_spi_tx_u08(1);              // read one byte from fifo
  uint8_t d = mcu_hw_spi_tx_u08(0);  // read last byte without increasing fifo pointer
  mcu_hw_spi_end();
  
  return d;
}

static void sys_handle_event(bool ignore_coldboot) {
  // the FPGAs cold boot flag was set indicating that the
  // FPGA has be reloaded while the MCU was running. Reset
  // the MCU to re-initialize everything and get into a
  // sane state

  // request interrupt source
  sys_begin(SPI_SYS_IRQ_SRC);
  mcu_hw_spi_tx_u08(0);
  unsigned char irq_src = mcu_hw_spi_tx_u08(0);
  mcu_hw_spi_end();

  if(irq_src & 2) {
    // read port 0 data for wifi emulation
    int16_t byte = sys_port_get(0);
    while(byte >= 0) {
      at_wifi_port_byte(byte);
      byte = sys_port_get(0);
    }
  }
  
  if(irq_src & 4) {
    // read the button state. This will also re-enable
    // button interrupts. buttons[0] usually is
    // the reset button, buttons[1] is unused by
    // the core
    unsigned char buttons = sys_get_buttons();
    sys_debugf("Buttons: %02x", buttons);

    static TickType_t reset_timer = 0;
    if(buttons & 1) {
      // button has just been pressed
      if(!reset_timer) reset_timer = xTaskGetTickCount();
    } else {
      // do a full mcu reset if reset button has been pressed for 2 seconds
      if(reset_timer && ((xTaskGetTickCount() - reset_timer) > pdMS_TO_TICKS(2000)))
	mcu_hw_reset();
      
      reset_timer = 0;
    }

    // the second button controls the OSD, so it can be used in conjunction
    // with 
    if(buttons & 2)
      menu_notify(osd_is_visible()?MENU_EVENT_HIDE:MENU_EVENT_SHOW);    
  }

  /* check if the FPGA has the coldboot flag set */
  if(irq_src & 1) {
    if(ignore_coldboot)
      sys_debugf("FPGA cold boot detected, ignoring for now");
    else {
      sys_debugf("FPGA cold boot detected, reseting MCU ...");

#if defined(TANG_CONSOLE60K) || defined(TANG_NANO20K) || (MISTLE_BOARD == 4)
      // Check for USB-JTAG activity and don't reset (and thus break
      // the JTAG activity)
      if(mcu_hw_jtag_is_active())
	jtag_highlight_debugf("Suppressing MCU reset due to JTAG activity");
      else	
#endif
	mcu_hw_reset();
    }
  }
}

void sys_handle_interrupts(unsigned char pending, bool ignore_coldboot) {
  // debugf("IRQ = %02x", pending);

  if(pending & 0x01) // irq 0 = SYSCTRL
    sys_handle_event(ignore_coldboot);

  if(pending & 0x02) // irq 1 = HID
    hid_handle_event();
  
  if(pending & 0x08) // irq 3 = SDC
    sdc_handle_event();
  
  //  if(pending & 0x10) // irq 4 = AUDIO
  //    audio_handle_event();
}

#ifndef FPGA_BOOT_TIMEOUT
#define FPGA_BOOT_TIMEOUT   10000   // prolonged for GW5AST138K
#endif

bool sys_wait4fpga(void) {
  sys_debugf("Waiting for FPGA to become ready");
  
  // try to establish connection to FPGA for five seconds. Assume the FPGA is
  // not properly configured after that
  int fpga_ok, timeout = FPGA_BOOT_TIMEOUT/10;  // timeout is in 10ms units
  do {
    fpga_ok = sys_status_is_valid();
    if(!fpga_ok) {
      vTaskDelay(pdMS_TO_TICKS(10));
      timeout--;
    }
  } while(timeout && !fpga_ok);

  if(timeout) {
    sys_debugf("FPGA ready after %dms!", FPGA_BOOT_TIMEOUT-10*timeout);

    sys_set_rgb(0x000040);  // blue
    return true;
  }
  
  sys_debugf("FPGA not ready after %d seconds!", FPGA_BOOT_TIMEOUT/1000);

  // Set LED to red. This is basically useless and will only work if the
  // FPGA receives requests but cannot answer them
  sys_set_rgb(0x400000);  // red
  return false;
}

void sys_run_action(config_action_t *action) {
  if(!action) return;
  
  sys_debugf("Running action '%s'", action->name);

  // execute all commands
  for(int i=0;action->commands[i].code != CONFIG_ACTION_COMMAND_IDLE;i++) {
    switch(action->commands[i].code) {
    case CONFIG_ACTION_COMMAND_SET:
      sys_debugf("SET('%c',%d)", action->commands[i].set.id,
		 action->commands[i].set.value);
      sys_set_val(action->commands[i].set.id, action->commands[i].set.value);
      break;
      
    case CONFIG_ACTION_COMMAND_DELAY:
      sys_debugf("DELAY(%d)", action->commands[i].delay.ms);
      vTaskDelay(pdMS_TO_TICKS(action->commands[i].delay.ms));
      break;
      
    case CONFIG_ACTION_COMMAND_LOAD:
      sys_debugf("LOAD %s", action->commands[i].filename);

      // try to read ini file
      inifile_read(action->commands[i].filename);
      break;
      
    case CONFIG_ACTION_COMMAND_SAVE:
      sys_debugf("SAVE %s", action->commands[i].filename);
      inifile_write(action->commands[i].filename);
      break;
      
    case CONFIG_ACTION_COMMAND_HIDE:
      sys_debugf("HIDE OSD");
      osd_enable(OSD_INVISIBLE);  // hide OSD
      break;
      
    case CONFIG_ACTION_COMMAND_LINK:
      sys_debugf("LINK");
      sys_run_action(action->commands[i].action);
      break;
    }
  }
}

void sys_run_action_by_name(char *name) {
  // check for init action
  config_action_t *action = config_get_action(name);
  if(action) sys_run_action(action);
}

// this modfied version of puff takes an input function top read data
static unsigned char puff_get_byte(void) {
  return mcu_hw_spi_tx_u08(0);
}

char *sys_get_config(void) {
  char *ret = NULL;
  // (try to) read xml directly from core

  // read the xml two times, one time to determine
  // the size and a second time to actually read it
  
  sys_begin(SPI_SYS_READ_CFG);
  mcu_hw_spi_tx_u08(0);

  char c = mcu_hw_spi_tx_u08(0);
  debugf("config[0] = %02x", c&0xff);
  
  // any valid XML starts with the character '<'. Older
  // cores not supporting built-in configs won't return that
  unsigned int len = 0;  
  if(c == '<') {
    for(len=0;len < 8191 && c;len++)
      c = mcu_hw_spi_tx_u08(0);

    mcu_hw_spi_end();  

    sys_debugf("core xml config size: %d", len);
    if(len < 100) return NULL;
    
    // allocate enough space
    ret = malloc(len+1);
    
    // and read the data into the buffer
    sys_begin(SPI_SYS_READ_CFG);
    mcu_hw_spi_tx_u08(0);
    
    unsigned int i;
    for(i=0;i<len;i++) ret[i] = mcu_hw_spi_tx_u08(0);
    ret[i] = '\0';
    
    mcu_hw_spi_end();

    return ret;
  }
  
  else if(c == 0x1f) {
    // skip over gzip header
    int id1 = mcu_hw_spi_tx_u08(0);    // second id byte
    int method = mcu_hw_spi_tx_u08(0); // compression method
    int flags  = mcu_hw_spi_tx_u08(0); // file flags

    // check a few more header fields. We only accept flag 8 which
    // is "filename"
    if((id1 != 0x8b)||(method != 8)||(flags & ~8)) {
      sys_debugf("Unexpected GZIP header %02x/%02x/%02x", id1, method, flags);
      mcu_hw_spi_end();
      return NULL;
    }

    // skip remaining six gzip header fields
    for(int i=0;i<6;i++) mcu_hw_spi_tx_u08(0);

    // skip filename if present
    if(flags & 8) while(mcu_hw_spi_tx_u08(0));
    
    // first run to determine uncompressed size
    unsigned long dstlen = 0, srclen = 65536;
    int ret = puff(NULL, &dstlen, puff_get_byte, &srclen);
    mcu_hw_spi_end();

    if(ret) {
      sys_debugf("Config gzip puff failed");
      return NULL;
    }

    // second run to actually read and uncompress
    sys_debugf("Malloc %ld bytes config memory", dstlen+1);
    unsigned char *dst = malloc(dstlen+1);  // plus one byte for string termination

    sys_begin(SPI_SYS_READ_CFG);
    mcu_hw_spi_tx_u08(0);

    // again, skip header (this time we already know the flags)
    for(int i=0;i<10;i++) mcu_hw_spi_tx_u08(0);

    // and again, skip filename if present
    if(flags & 8) while(mcu_hw_spi_tx_u08(0));
    
    // first run to determine uncompressed size
    srclen = 65536;
    ret = puff(dst, &dstlen, puff_get_byte, &srclen);
    mcu_hw_spi_end();

    // terminate the config string
    dst[dstlen-1] = '\0';
    
    return (char*)dst;
  }
  
  return NULL;
}
