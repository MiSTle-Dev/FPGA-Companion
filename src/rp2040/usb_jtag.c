/*
  usb_jtag.c
*/

#include "tusb.h"

#include "../mcu_hw.h"
#include "../debug.h"

#if MISTLE_BOARD != 4
#error "USB JTAG is only available for the Dev20k"
#else
#warning "Enabling USB JTAG bridge"
#endif

#if JTAG_CHANNELS == 1
#warning "Implementing single channel USB JTAG."
#warning "Most Gowin related tools expect dual channel by default!"
#elif JTAG_CHANNELS == 2
#else
#error "Only one or two JTAG channels are allowed"
#endif

#include "../jtag.h"

// TODO: make this a table for fast access
static inline uint8_t reverse_byte(uint8_t byte) {
  byte = ((byte & 0x55) << 1) | ((byte & 0xaa) >> 1);
  byte = ((byte & 0x33) << 2) | ((byte & 0xcc) >> 2);
  return ((byte & 0x0f) << 4) | ((byte & 0xf0) >> 4);
}

// wrapper around mcu_hw_jtag_data() allowing for reverse bit order
static void jtag_data(uint8_t lsb, uint8_t *txd, uint8_t *rxd, int len) {
  // The last byte may only be partially used if the number of bits to
  // be transferred is not a multiple of 8. Thus not all bits
  // of the reply buffer are updated. This may cause one-bits to
  // remain in the unused part which in turn seems to confuse software
  // like pyftdi when re-assembling data returned. Thus we explicitely
  // clear the last reply byte.
  if(rxd) rxd[len/8] = 0;

  // reverse incoming data if needed
  if(!lsb && txd)
    for(int i=0;i<(len+7)/8;i++)
      txd[i] = reverse_byte(txd[i]);

  mcu_hw_jtag_data(txd, rxd, len);
  
  // reverse outgoing data if needed
  if(!lsb && rxd)
    for(int i=0;i<(len+7)/8;i++)
      rxd[i] = reverse_byte(rxd[i]);
}

// wrapper around mcu_hw_jtag_tms() allowing for reverse bit order
static uint8_t jtag_tms(uint8_t lsb, uint8_t tdi, uint8_t data, int len) {
  // reverse incoming data if needed
  if(!lsb) data = reverse_byte(data);

  uint8_t rxd = mcu_hw_jtag_tms(tdi, data, len);
  
  // reverse outgoing data if needed
  if(!lsb) rxd = reverse_byte(rxd);

  return rxd;
}

/* ======================================================================== */
/* ===============                USB                        ============== */
/* ======================================================================== */

// this is the status reply sent in all replies
#define REPLY_STATUS   "\x32\x60"

// the reply buffer should be able to hold at least two times the max
// endpoint transfer size of 64 bytes. Since the pico will stop
// accepting incoming requests via USB while the output buffer is full
// the buffer should actually be at least 256 bytes
#define REPLY_BUFFER_SIZE (1024)

struct jtag {
  uint8_t usb_itf;
  bool loopback;
  bool tx_pending, rx_disabled;
  uint16_t reply_len;
  uint8_t reply_buffer[REPLY_BUFFER_SIZE];
  uint16_t pending_writes;
  uint8_t pending_write_cmd;
  uint8_t mode;  // 2=JTAG
  
  // command buffer to assemble incoming commands and
  // which may be split over multiple usb transfers
  struct {
    union {
      uint8_t bytes[4];
      struct {
	uint8_t code;
	union {
	  uint8_t b[2];
	  uint16_t w;
	};
	uint8_t dummy;  // used by CPU command only
      } __attribute__((packed)) cmd;
    } data;
    uint8_t len;
  } cmd_buf;
};

// maintain both channels if enabled
static struct jtag jtag_engine[JTAG_CHANNELS];

void tud_mount_cb(void) {
  usb_debugf("tud_mount_cb()");

  for(int i=0;i<JTAG_CHANNELS;i++) {
    jtag_engine[i].usb_itf = i;
    jtag_engine[i].loopback = false;
    jtag_engine[i].tx_pending = false;
    jtag_engine[i].rx_disabled = false;
    jtag_engine[i].reply_len = 0;
    jtag_engine[i].cmd_buf.len = 0;
  }
}

void tud_umount_cb(void) {
  usb_debugf("tud_umount_cb()");
}

void tud_suspend_cb(bool remote_wakeup_en) {
  usb_debugf("tud_suspend_cb()");
}

void tud_resume_cb(void) {
  usb_debugf("tud_resume_cb()");
}

// parse a non-shifting command
static void cmd_parse(struct jtag *jtag) {
  switch(jtag->cmd_buf.data.cmd.code) {
  case 0x80:
  case 0x82:
    uint8_t value = jtag->cmd_buf.data.cmd.b[0];
    uint8_t dir = jtag->cmd_buf.data.cmd.b[1];

    char bits[9];     
    for(int i=0;i<8;i++) bits[i] = (dir&(0x80>>i))?'O':'I';
    bits[8] = '\0';
    
    usb_debugf("Set data bits %s value 0x%02x, dir 0x%02x=%s", (jtag->cmd_buf.data.cmd.code&2)?"high":" low", value, dir, bits);
    
    /* we currently only support the lower bits */
    if(!(jtag->cmd_buf.data.cmd.code&2)) {
      // second payload byte is direction.
      // Lowest bits 0xb is JTAG (and SPI) mapping
      mcu_hw_jtag_set_pins(dir, value);
    }
    break;
                
  case 0x81:
  case 0x83:
    uint8_t reply = 0;

    // only low byte supported
    //    if(!(jtag->cmd_buf.data.cmd.code&2)) reply = port_gpio_get(jtag);
      
    usb_debugf("Get data bits %s: 0x%02x", (jtag->cmd_buf.data.cmd.code&2)?"high":"low", reply);

    jtag->reply_buffer[jtag->reply_len+2] = reply;
    jtag->reply_len += 1;
    break;
    
  case 0x84:
    usb_debugf("Connect loopback %d", jtag->usb_itf);
    jtag->loopback = true;
    break;

  case 0x85:
    usb_debugf("Disconnect loopback %d", jtag->usb_itf);
    jtag->loopback = false;
    break;

  case 0x86: {
    // send input state in a reply byte
    int divisor = jtag->cmd_buf.data.cmd.w;
    int rate = 12000000 / ((1+divisor) * 2);
    usb_debugf("Set TCK/SK Divisor to %d = %d Mhz", divisor, rate/1000000);
    // pio_jtag_set_clk_freq(&jtag->pio, rate/1000);
  } break;

  case 0x87:
    usb_debugf("Flush");
    break;

  case 0x8a:
    usb_debugf("Disable div by 5 (60MHz master clock)");
    break;
        
  case 0x8b:
    usb_debugf("Enable div by 5 (12MHz master clock)");
    break;

  default:
    usb_debugf("Unexpected command %02x", jtag->cmd_buf.data.cmd.code);
    for(;;);
  }
}

// get command size incl. length bytes or the like. But without the
// payload of the byte stream command
static uint8_t cmd_size(uint8_t cmd) {
  if(cmd & 0x80) {
    // generic command
    
    // command and one more byte
    if((cmd == 0x90) ||   // CPUMode read short address
       (cmd == 0x8e))     // clock for n bits with no data transfer
      return 2;	
    
    // command and two more bytes
    if((cmd == 0x80) || (cmd == 0x82) || // set data bits
       (cmd == 0x86) || // set divisor
       (cmd == 0x91) || // CPUMode read extended address
       (cmd == 0x92) || // CPUMode write short address
       
       (cmd == 0x8f) || // clock for n*8 bits with no data transfer
       (cmd == 0x9c) || // clock for n*8 bits with no data transfer until gpio high
       (cmd == 0x9d) || // clock for n*8 bits with no data transfer until gpio low
       (cmd == 0x9e))   // set IO to only drive on a '0' and tristate on '1'
      return 3;
    
    // command and three more bytes
    if(cmd == 0x93)     // CPUMode write extended address
      return 4;
    
    // otherwise no additional bytes needed, just the command itself
    return 1;
  }
  
  // shift commands
  if(cmd & 0x02) {
    // bit commands have an additional  byte length and a byte payload ...
    if(cmd & 0x50)
      return 3;

    // ... unless they are read-only, then they have only the byte length
    return 2;
  }
  
  // byte shift commands always need an additional two byte length
  return 3;
}

static uint16_t shift_bits(struct jtag *jtag) {
  uint8_t cmd = jtag->cmd_buf.data.cmd.code;

  // calculate data shift length
  uint16_t shift_len = jtag->cmd_buf.data.cmd.b[0]+1; // shift length was given in bits-1

  // request to write something? TDI or TMS?
  uint8_t data;
  if(cmd & 0x50) data = jtag->cmd_buf.data.cmd.b[1];
  
#if 0
  usb_debugf("[%02x] shift %d bits", shift_len, cmd);
  hexdump(&data, (shift_len+7)/8);
#endif
  
  if(cmd & 0x40) {
    usb_debugf("JTAG TMS BIT WRITE %d %02x", shift_len, data);

    uint8_t rx = jtag_tms((cmd&8)?1:0, (data&80)?1:0, data, shift_len);
    if(cmd & 0x20) {
      jtag->reply_buffer[jtag->reply_len + 2] = rx;
      jtag->reply_len += (shift_len+7)/8;
    }
    
    //    pio_jtag_write_tms(&jtag->pio, (cmd&8)?1:0, (data&0x80)?1:0, &data,
    //       (cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL, shift_len);
  } else {
    usb_debugf("JTAG TDI BIT WRITE %d %02x", shift_len, data);

    jtag_data((cmd&8)?1:0, (cmd & 0x10)?&data:NULL,
	      (cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL,
	      shift_len);
      
    //pio_jtag_write_tdi_read_tdo(&jtag->pio, (cmd&8)?1:0, (cmd & 0x10)?&data:NULL,
    //			(cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL, shift_len);
    if(cmd & 0x20) jtag->reply_len += (shift_len+7)/8;
  }

  // usb_debugf("reply len now %d", jtag->reply_len);
  return 0;  // no additional bytes used
} 

static uint16_t shift_bytes(struct jtag *jtag, uint8_t *buf, uint16_t len) {
  uint8_t cmd = jtag->cmd_buf.data.cmd.code;

  // length is given in bytes and command length is variable
  uint16_t shift_len = jtag->cmd_buf.data.cmd.w + 1;

#if 1
  usb_debugf("[%02x] shift %d bytes (%d avail)", cmd, shift_len, len);
  if(shift_len > len) hexdump(buf, len);
  else                hexdump(buf, shift_len);
#endif

  // just return data if in loopback mode. This is not fully implemented yet and
  // will e.g. not work with large data chunks
  if(jtag->loopback) {
    // limit buffer size
    int bytes2copy = (shift_len > len)?len:shift_len;
    if(jtag->reply_len + 2 + bytes2copy > REPLY_BUFFER_SIZE)
      bytes2copy = REPLY_BUFFER_SIZE - jtag->reply_len - 2;
    
    usb_debugf("Copying %d bytes loopback data", bytes2copy);    
    memcpy(jtag->reply_buffer+jtag->reply_len+2, buf, bytes2copy);
    jtag->reply_len += bytes2copy;
    
    return bytes2copy;
  }  
  
  // it may happen that we are supposed to shift out more bits than we have payload
  if((cmd & 0x10) && (shift_len > len)) {
    usb_debugf("Trunc write %d to %d", shift_len, len);

    jtag_data((cmd&8)?1:0, buf,
	      (cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL,
	      len*8);
    
    if(cmd & 0x20) jtag->reply_len += len;
    jtag->pending_writes = shift_len-len;
    jtag->pending_write_cmd = cmd;

    for(;;);
    
    return len;  // all data consumed that was there
  }

  // there is either no data to be sent or the data present is sufficient for
  // the full transfer
  jtag_data((cmd&8)?1:0, (cmd & 0x10)?buf:NULL,
	    (cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL,
	    shift_len*8);

    //  pio_jtag_write_tdi_read_tdo(&jtag->pio, (cmd&8)?1:0, (cmd & 0x10)?buf:NULL,
  //			      (cmd & 0x20)?(jtag->reply_buffer + jtag->reply_len + 2):NULL,
  //			      shift_len*8);
  if(cmd & 0x20) jtag->reply_len += shift_len;
  
  // W-TDI set? There was payload used
  return (cmd & 0x10)?shift_len:0;
}

static uint16_t cmd_shift_parse(struct jtag *jtag, uint8_t *buf, uint16_t len) {
  // get command byte
  uint8_t cmd = jtag->cmd_buf.data.cmd.code;
  
  /* command bits  
     0: W-VE  1: BIT  2: R-VE  3: LSB  4: W-TDI  5: R-TDO  6: W-TMS  7: 0 */

  // check if it's an allowed command as not all possible command bit patterns are
  // actually valid. 
  if((cmd != 0x10) && (cmd != 0x11) && (cmd != 0x12) && (cmd != 0x13) &&
     (cmd != 0x18) && (cmd != 0x19) && (cmd != 0x1a) && (cmd != 0x1b) &&
     (cmd != 0x20) && (cmd != 0x22) && (cmd != 0x24) && (cmd != 0x26) &&
     (cmd != 0x28) && (cmd != 0x2a) && (cmd != 0x2c) && (cmd != 0x2e) &&
     (cmd != 0x31) && (cmd != 0x33) && (cmd != 0x34) && (cmd != 0x36) &&
     (cmd != 0x39) && (cmd != 0x3b) && (cmd != 0x3c) && (cmd != 0x3e) &&
     // TMS commands
     (cmd != 0x4a) && (cmd != 0x4b) && (cmd != 0x6a) && (cmd != 0x6b) &&
     (cmd != 0x6e) && (cmd != 0x6f)) {

    // reply with 0xfa / bad command
    jtag->reply_buffer[2+jtag->reply_len++] = 0xfa;    
    return 0;
  }
  
  if(cmd & 2) return shift_bits(jtag);
  else        return shift_bytes(jtag, buf, len);
}

static bool check_reply_buffer(struct jtag *jtag) {
  // the reply buffer has a limited size. Once it runs
  // too full we need to stop accepting incoming requests
  // as the buffer may otherwise overflow. Since we cannot
  // know how much data the next request will ask to be
  // returned we stop the receiver once we have less than
  // 64 bytes in the reply buffer left.

  // reply_len does not include the two header bytes
  // usb_debugf("Reply buffer usage is %d of %d", jtag->reply_len+2, REPLY_BUFFER_SIZE);
  
  if(jtag->reply_len+2 > REPLY_BUFFER_SIZE-64) {
    // the buffer should actually never overflow
    if(jtag->reply_len+2 > REPLY_BUFFER_SIZE) 
      printf(">>>>>>>>>>>>>> REPLY BUFFER DID OVERFLOW!!! <<<<<<<<<<<<<<<<<\n");
    else
      usb_debugf("FLOW: reply buffer may overflow");
      
    // don't allow any more replies
    usb_debugf("FLOW: stopping receiver");
    return false;
  }
  return true;
}

static void check_for_outgoing_data(struct jtag *jtag) {
  if(jtag->tx_pending) return;
  
  // check if there's now data in the reply buffer and request to return it
  if(jtag->reply_len) {
#if 0
    usb_debugf("REPLY: %d", jtag->reply_len);
    hexdump(jtag->reply_buffer+2, jtag->reply_len);
#endif
    
    // data is always stored from byte 2 on in the reply buffer, so that the
    // status can be placed in front
    memcpy(jtag->reply_buffer, REPLY_STATUS, 2);

    // as a full speed device we can return max 62 bytes per USB transfer
    if(jtag->reply_len >= 62) {
      usb_debugf("TX partial 64");
      tud_vendor_n_write(jtag->usb_itf, jtag->reply_buffer, 64);
      tud_vendor_n_write_flush(jtag->usb_itf);
      // shift data down
      memmove(jtag->reply_buffer+2, jtag->reply_buffer+64, REPLY_BUFFER_SIZE-64);
      jtag->reply_len -= 62;
    } else {    
      usb_debugf("TX 2+%d", jtag->reply_len);
      if(jtag->reply_len < 8)
	hexdump(jtag->reply_buffer, jtag->reply_len+2);
      
      // Send all data back to host
      tud_vendor_n_write(jtag->usb_itf, jtag->reply_buffer, jtag->reply_len+2);
      tud_vendor_n_write_flush(jtag->usb_itf);
      jtag->reply_len = 0;
    }
    jtag->tx_pending = true;

    if(jtag->rx_disabled && check_reply_buffer(jtag)) {
      usb_debugf("FLOW: re-enable receiver <------------------------------------");
      jtag->rx_disabled = false;
      
      // TODO: check if this is the correct solution:
      tud_vendor_n_read_flush(jtag->usb_itf);
    }
  } else {
    usb_debugf("TX 2+0");
    tud_vendor_n_write(jtag->usb_itf, REPLY_STATUS, 2);
    tud_vendor_n_write_flush(jtag->usb_itf);
    jtag->tx_pending = true;
  }
}

// https://github.com/MiSTle-Dev/PICO-MPSSE/blob/main/pico_mpsse/pico_mpsse.c
void tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint16_t bufsize) {
#if 1
  usb_debugf("tud_vendor_rx_cb(%d, %p, %d)", itf, buffer, bufsize); 
  hexdump(buffer, bufsize);
#endif
  
  struct jtag *jtag = &jtag_engine[itf];

  // parse incoming command  
  while(bufsize) {
    // move next byte into command buffer
    // printf("BUF %d -> %02x to %d\n", bufsize, *buffer, jtag->cmd_buf.len);
    jtag->cmd_buf.data.bytes[jtag->cmd_buf.len++] = *buffer++;
    bufsize--;

    if(jtag->cmd_buf.len &&
       cmd_size(jtag->cmd_buf.data.cmd.code) <= jtag->cmd_buf.len) {
      if(jtag->cmd_buf.data.cmd.code & 0x80) {
	cmd_parse(jtag);
      } else  {
	int skip = cmd_shift_parse(jtag, buffer, bufsize);
	buffer += skip;
	bufsize -= skip;
      }
	
      // flush the command buffer
      jtag->cmd_buf.len = 0;
    }
  }  

  // re-enable receiver, now that all data has been processed. But make sure
  // we can actually accept more data
  if(!check_reply_buffer(jtag)) {
    jtag->rx_disabled = true;

    printf("disable rx\n");
    for(;;);
  } else
    tud_vendor_n_read_flush(itf);

  // TODO: check why the following is needed
  check_for_outgoing_data(jtag);
}

void tud_vendor_tx_cb(uint8_t itf, uint32_t bufsize) {
  struct jtag *jtag = &jtag_engine[itf];

  usb_debugf("tud_vendor_tx_cb(%d)", itf);
  jtag->tx_pending = false;
  check_for_outgoing_data(jtag);
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
  // usb_debugf("tud_vendor_control_xfer_cb()");

  bool dir_in = (request->bmRequestType_bit.direction == TUSB_DIR_IN) ? true : false; 
  //  usb_debugf("if=0x%02x stage=%d req=0x%02x type=0x%02x dir=%s wValue=0x%04x wIndex=0x%04x wLength=%d",
  //  	     request->wIndex, stage, request->bRequest, request->bmRequestType_bit.type,
  //	     dir_in ? "IN" : "OUT", request->wValue, request->wIndex, request->wLength);

  if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR) {
    usb_debugf("Ignoring unexpected type 0x%02x", request->bmRequestType);
    return false;
  }

#if JTAG_CHANNELS == 2
  struct jtag *jtag = NULL;
  if(request->wIndex >= 1 && request->wIndex <= 2)
    jtag = &jtag_engine[request->wIndex-1];
#else
  struct jtag *jtag = jtag_engine;
#endif

  if(stage != CONTROL_STAGE_SETUP)
    return true;

  if(!dir_in) {
    switch(request->bRequest) {
      
    case 0x00:
      usb_debugf("RESET, #%d=%d", request->wIndex, request->wValue);
      if(request->wValue == 0) check_for_outgoing_data(jtag);  // TODO: <<<<---- remove this
      break;

    case 0x01:
      usb_debugf("SET MODEM CONTROL, #%d=%d", request->wIndex, request->wValue);
      break;
	  
    case 0x02:
      usb_debugf("SET FLOW CONTROL, #%d=%d", request->wIndex, request->wValue);
      break;
      
    case 0x03:
      usb_debugf("SET BAUD RATE, #%d=%d", request->wIndex, request->wValue);
      break;
      
    case 0x04:
      usb_debugf("SET DATA, #%d=%d", request->wIndex, request->wValue);
      break;
      
    case 0x05:
      usb_debugf("POLL MODEM STATUS, #%d=0x%02x", request->wIndex, request->wValue);
      break;
      
    case 0x06:
      usb_debugf("SET EVENT CHARACTER, #%d=0x%02x", request->wIndex, request->wValue);
      break;
      
    case 0x07:
      usb_debugf("SET ERROR CHARACTER, #%d=0x%02x", request->wIndex, request->wValue);
      break;
      
    case 0x09:
      usb_debugf("SET LATENCY TIMER, #%d=%d", request->wIndex, request->wValue);
      break;
      
    case 0x0b:
      usb_debugf("SET BITMODE, #%d=0x%02x", request->wIndex, request->wValue);
      if(request->wIndex >= 1 && request->wIndex <= 2) {
	struct jtag *jtag = &jtag_engine[request->wIndex-1];
	jtag->mode = request->wValue>>8;
	//  port_gpio_set_dir(jtag, pkt->wValue & 0xff);
      }
      break;
	
    default:
      usb_debugf("Unsupported request %02x", request->bRequest);
      return false;
      break;
    }

    tud_control_status(rhport, request);
    return true;
  } else {
    usb_debugf("UNEXPECTED IN");
    
    /* ... in transfer */

#if 0
    ctrl_rsp[0] = 0x00;
    rsp_len = 1;
 
    // Call tud_control_xfer to send the response, returning its return
    // code
    return tud_control_xfer(rhport, request, ctrl_rsp, rsp_len);
#endif

  }
    
  return false;
}
