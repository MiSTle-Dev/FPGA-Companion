/* ========================================================================= */
/* ======                              JTAG                           ====== */
/* ========================================================================= */

#include "board.h"
#ifdef ENABLE_JTAG

static bool jtag_is_active = false;

// Enabling any of the following debug mechanisms will disable the use
// of PIO driven HW JTAG on the rp2350 and use software driven JTAG instead.
// #define DEBUG_JTAG    // debug raw JTAG IO
// #define DEBUG_TAP     // set to follow the JTAG state machine

// Enable PIO JTAG whenever running on a RP2350
// TODO: This should be disabled when TAP debugging is wanted
#ifdef PICO_RP2350
#if defined(DEBUG_JTAG) || defined(DEBUG_TAP)
#warning "Disabling JTAG PIO since JTAG debugging is enabled"
#else
#define USE_PIO_JTAG
#endif
#endif

#ifdef USE_PIO_JTAG
static pio_jtag_inst_t pio_jtag;
static uint32_t pio_jtag_clock_value = 6000000;
enum { PIO_JTAG_MAX_BITS = sizeof(pio_jtag.write_buffer) * 4 };
#else
// Using the gpio functions directly without any further delay results
// in a max clock rate of about 6Mhz.
#define MCU_HW_JTAG_CLK_HI()   gpio_put(PIN_JTAG_TCK, 1);
#define MCU_HW_JTAG_CLK_LOW()  gpio_put(PIN_JTAG_TCK, 0);
#endif

#ifdef DEBUG_TAP
#define JTAG_STATE_TEST_LOGIC_RESET  0
#define JTAG_STATE_RUN_TEST_IDLE     1
#define JTAG_STATE_SELECT_DR_SCAN    2
#define JTAG_STATE_CAPTURE_DR        3
#define JTAG_STATE_SHIFT_DR          4
#define JTAG_STATE_EXIT1_DR          5
#define JTAG_STATE_PAUSE_DR          6
#define JTAG_STATE_EXIT2_DR          7
#define JTAG_STATE_UPDATE_DR         8
#define JTAG_STATE_SELECT_IR_SCAN    9
#define JTAG_STATE_CAPTURE_IR       10
#define JTAG_STATE_SHIFT_IR         11
#define JTAG_STATE_EXIT1_IR         12
#define JTAG_STATE_PAUSE_IR         13
#define JTAG_STATE_EXIT2_IR         14
#define JTAG_STATE_UPDATE_IR        15

// state flow table, telling which state follows onto which state depending on TMS
const struct state_flow_S {
  uint8_t tms[2];
  const char *name;  
} state_flow[] = {
  //  next state when TMS == 0    next state when TMS == 1      state name
  { {JTAG_STATE_RUN_TEST_IDLE, JTAG_STATE_TEST_LOGIC_RESET }, "Test-Logic-Reset" }, // 0
  { {JTAG_STATE_RUN_TEST_IDLE, JTAG_STATE_SELECT_DR_SCAN   }, "Run-Test/Idle"    }, // 1
  
  { {JTAG_STATE_CAPTURE_DR,    JTAG_STATE_SELECT_IR_SCAN   }, "Select-DR-Scan"   }, // 2
  { {JTAG_STATE_SHIFT_DR,      JTAG_STATE_EXIT1_DR         }, "Capture-DR"       }, // 3
  { {JTAG_STATE_SHIFT_DR,      JTAG_STATE_EXIT1_DR         }, "Shift-DR"         }, // 4
  { {JTAG_STATE_PAUSE_DR,      JTAG_STATE_UPDATE_DR        }, "Exit1-DR"         }, // 5
  { {JTAG_STATE_PAUSE_DR,      JTAG_STATE_EXIT2_DR         }, "Pause-DR"         }, // 6
  { {JTAG_STATE_SHIFT_DR,      JTAG_STATE_UPDATE_DR        }, "Exit2-DR"         }, // 7
  { {JTAG_STATE_RUN_TEST_IDLE, JTAG_STATE_SELECT_DR_SCAN   }, "Update-DR"        }, // 8
  
  { {JTAG_STATE_CAPTURE_IR,    JTAG_STATE_TEST_LOGIC_RESET }, "Select-IR-Scan"   }, // 9
  { {JTAG_STATE_SHIFT_IR,      JTAG_STATE_EXIT1_IR         }, "Capture-IR"       }, // 10
  { {JTAG_STATE_SHIFT_IR,      JTAG_STATE_EXIT1_IR         }, "Shift-IR"         }, // 11
  { {JTAG_STATE_PAUSE_IR,      JTAG_STATE_UPDATE_IR        }, "Exit1-IR"         }, // 12
  { {JTAG_STATE_PAUSE_IR,      JTAG_STATE_EXIT2_IR         }, "Pause-IR"         }, // 13
  { {JTAG_STATE_SHIFT_IR,      JTAG_STATE_UPDATE_IR        }, "Exit2-IR"         }, // 14
  { {JTAG_STATE_RUN_TEST_IDLE, JTAG_STATE_SELECT_DR_SCAN   }, "Update-IR"        }  // 15
};

static uint8_t tap_state = JTAG_STATE_TEST_LOGIC_RESET;
static uint8_t tap_ir_bits;
static uint32_t tap_ir;
static uint32_t tap_dr_bits;
static uint8_t tap_dr[4], tap_dr_byte;  // we capture only the first 32 bits
static uint32_t tap_dr_sum;

const struct gowin_ir_S {
  int ir;
  const char *name;  
} gowin_ir[] = {
  { 0x00, "Bypass" },
  { 0x02, "Noop" },
  { 0x03, "Read SRAM" },
  { 0x05, "Erase SRAM" },
  { 0x09, "XFER Done" },
  { 0x11, "IDCode" },
  { 0x12, "Address Init" },
  { 0x13, "UserCode" },
  { 0x15, "ConfigEnable" },
  { 0x16, "Transfer SPI" },
  { 0x17, "Transfer Bitstream" },
  { 0x21, "Program Key" },  
  { 0x23, "Security" },  
  { 0x24, "Program EFuse" },  
  { 0x25, "Read Key" },
  { 0x29, "Program Key" },  
  { 0x3a, "ConfigDisable" },
  { 0x3c, "Reconfig" },
  { 0x3d, "BSCAN 2 SPI" },
  { 0x41, "Status" },
  { 0x42, "GAO#1" },
  { 0x43, "GAO#2" },
  { 0x71, "EFlash Program" },  
  { 0x75, "EFlash Erase" },  
  { 0x7a, "Switch to MCU JTAG" },  
  { 0xff, "Bypass" },
  {   -1, "<unknown command>" }  
};

static void jtag_tap_advance_state(uint8_t tms, uint8_t tdi) {
  // capture instruction register write
  if(tap_state == JTAG_STATE_SHIFT_IR) {
    if(tdi) tap_ir |= (1<<tap_ir_bits);
    tap_ir_bits++;
  }
  
  // capture data register write
  if(tap_state == JTAG_STATE_SHIFT_DR) {
    if(tdi && ((tap_dr_bits/8)<sizeof(tap_dr)))
      tap_dr[tap_dr_bits/8] |= (1<<(tap_dr_bits&7));

    // update sum, whenever the last bit of a byte
    // has been written
    if(tdi) tap_dr_byte |= (1<<(tap_dr_bits&7));
    if((tap_dr_bits&7) == 7) {
      tap_dr_sum += tap_dr_byte;
      tap_dr_byte = 0;
    }
    
    tap_dr_bits++;
  }

  // check if we'd do into TEST_LOGIC_RESET state
  if(tap_state != JTAG_STATE_TEST_LOGIC_RESET &&
     state_flow[tap_state].tms[tms] == JTAG_STATE_TEST_LOGIC_RESET) {
    jtag_highlight_debugf("TEST LOGIC RESET");
  }
    
  tap_state = state_flow[tap_state].tms[tms];

  // clear IR if we just entered the capture IR state
  if(tap_state == JTAG_STATE_CAPTURE_IR) {
    tap_ir_bits = 0;  
    tap_ir = 0;
  }

  if(tap_state == JTAG_STATE_CAPTURE_DR) {
    tap_dr_bits = 0;
    for(unsigned int i=0;i<sizeof(tap_dr);i++) tap_dr[i] = 0;
    tap_dr_sum = 0;
    tap_dr_byte = 0;
  }
    
  // display IR if we just entered the update IR state
  if(tap_state == JTAG_STATE_UPDATE_IR) {
    // since we know which FPGA we are dealing with, we can disect
    // this even further
    int i;
    for(i=0;gowin_ir[i].ir != -1 && gowin_ir[i].ir != (int)tap_ir;i++);
    jtag_highlight_debugf("IR %02lx/%d: GOWIN %s", tap_ir, tap_ir_bits, gowin_ir[i].name);
  }

  if(tap_state == JTAG_STATE_UPDATE_DR) {
    if(!(tap_dr_bits&7))  jtag_highlight_debugf("DR %lu bytes, sum %ld", tap_dr_bits/8, tap_dr_sum);
    else jtag_highlight_debugf("DR %lu bytes + %lu bits, sum %ld", tap_dr_bits/8, tap_dr_bits&7, tap_dr_sum);
    hexdump(tap_dr, sizeof(tap_dr));
  }
}

#ifdef DEBUG_JTAG
static const char *jtag_tap_state_name(void) {
  return state_flow[tap_state].name;
}
#endif
#endif

bool mcu_hw_jtag_is_active(void) {
  return jtag_is_active;
}

void mcu_hw_jtag_set_pins(uint8_t dir, uint8_t data) {
  // bit order is TMS/TDO/TDI/TCK, for JTAG this will be IOII  
  uint8_t pins[] = { PIN_JTAG_TCK, PIN_JTAG_TDI, PIN_JTAG_TDO, PIN_JTAG_TMS };

#ifdef DEBUG_JTAG
  jtag_debugf("PIN DIR 0x%02x, DATA 0x%02x", dir, data);
#endif

  // only the lowest four bits are actually implemented
  // TODO: consider another bit for RECONF
  for(int i=0;i<4;i++) {
    if(dir & (1<<i))  gpio_put(pins[i], (data & (1<<i))?1:0);
    gpio_set_dir(pins[i], (dir & (1<<i))?GPIO_OUT:GPIO_IN);
  }  

  // check if the pin direction pattern matches JTAG
  if((dir & 0x0f) == 0x0b) {
    jtag_is_active = true;

#ifdef DEBUG_JTAG
    jtag_debugf("DIR pattern matches JTAG");
#endif
    // JTAG may actually be disabled on the FPGA. Try to detect the
    // FPGA and if none is detected, try to reconfigure it
#ifdef USE_PIO_JTAG
    if(!pio_jtag.pio) {
      jtag_debugf("Enabling PIO JTAG on PIO2");
    
      // setup PIO JTAG for PIO2
      pio_jtag.pio = pio2;
      pio_jtag.pin_tck = PIN_JTAG_TCK;
      pio_jtag.pin_tdi = PIN_JTAG_TDI;
      pio_jtag.pin_tdo = PIN_JTAG_TDO;
      pio_jtag.pin_tms = PIN_JTAG_TMS;
      pio_jtag.write_pending = false;
      
      // Use 6MHz JTAG clock by default
      pio_jtag_init(&pio_jtag, pio_jtag_clock_value/1000);
    }
#endif

    // send a bunch of 1's to return into Test-Logic-Reset state.
    mcu_hw_jtag_tms(1, 0b11111, 5);
  
    // send TMS 0/1/0/0 to get into SHIFT-DR state
    mcu_hw_jtag_tms(1, 0b0010, 4);

    // shift data into DR
    uint32_t idcode;
    mcu_hw_jtag_data(NULL, (uint8_t*)&idcode, 32);
    
    // finally return into Test-Logic-Reset state.
    mcu_hw_jtag_tms(1, 0b11111, 5);
  
    jtag_debugf("IDCODE = %08lx", idcode);

    // anything but all 1's or all 0's indicates that JTAG seems to
    // be working
    if((idcode == 0xffffffff) || (idcode == 0x00000000)) {
      jtag_highlight_debugf("JTAG doesn't seem to work. Forcing non-flash reconfig");
      mcu_hw_fpga_reconfig(false);
    }
  } else
    jtag_is_active = false;
}

// send up to 8 TMS bits with a given fixed TDI state
uint8_t mcu_hw_jtag_tms(uint8_t tdi, uint8_t data, int len) {
#ifdef USE_PIO_JTAG
  uint8_t rx[2] = {0};
  pio_jtag_write_tms(&pio_jtag, true, tdi, &data, rx, len);
  return rx[0];
#else
  int dlen = len & 7;
  uint8_t mask = 1;
  uint8_t rx = 0;

  gpio_put(PIN_JTAG_TDI, tdi);  

  while(len--) {
    gpio_put(PIN_JTAG_TMS, (data & mask)?1:0);  
    MCU_HW_JTAG_CLK_HI();

#ifdef DEBUG_TAP
    jtag_tap_advance_state((data & mask)?1:0, tdi);
#ifdef DEBUG_JTAG
    jtag_debugf("TMS %d TDI %d TDO %d -> %s", (data & mask)?1:0, tdi, gpio_get(PIN_JTAG_TDO),
		jtag_tap_state_name());
#endif
#else
#ifdef DEBUG_JTAG
    jtag_debugf("TMS %d TDI %d TDO %d", (data & mask)?1:0, tdi, gpio_get(PIN_JTAG_TDO));
#endif
#endif
    
    if(gpio_get(PIN_JTAG_TDO)) rx |= mask;
    
    MCU_HW_JTAG_CLK_LOW();

    mask <<= 1;
  }

  // adjust for the fact that we aren't really shifting
  if(dlen) rx <<= 8-dlen;
  return rx;
#endif
}

void mcu_hw_jtag_data(uint8_t *txd, uint8_t *rxd, int len) {
#ifdef USE_PIO_JTAG
  // jtag_debugf("mcu_hw_jtag_data(%p,%p,%d)", txd, rxd, len);  
  // Each JTAG bit expands to two bits in the PIO driver's write buffer.
  while(len > 0) {
    int bits = len > PIO_JTAG_MAX_BITS ? PIO_JTAG_MAX_BITS : len;
    // The PIO driver also receives a trailing byte for byte-aligned transfers.
    uint8_t rx[PIO_JTAG_MAX_BITS / 8 + 1];
    pio_jtag_write_tdi_read_tdo(&pio_jtag, true, txd, rxd ? rx : NULL, bits);
    if(rxd) {
      memcpy(rxd, rx, (bits + 7) / 8);
      rxd += bits / 8;
    }
    if(txd) txd += bits / 8;
    len -= bits;
  }
#else
  uint8_t mask = 1;
  int dlen = len & 7;

#ifdef DEBUG_TAP
  // data transmissions are only expected in states SHIFT_DR and SHIFT_IR
  if((tap_state != JTAG_STATE_SHIFT_IR) && (tap_state != JTAG_STATE_SHIFT_DR))
    jtag_debugf("Warning: data i/o in non-shifting state %d!", tap_state);
#endif

  // data transmission always keeps TMS at zero
  gpio_put(PIN_JTAG_TMS, 0);

  // special version for txd-only with a multiple of 8 bits
  // as that's the most common case
  if(txd && !rxd && !(len&7)) {
#ifdef DEBUG_TAP
    for(int i=0;i<len;i++)
      jtag_tap_advance_state(0, txd[i>>3] & (1<<(i&7)));
#endif

    len >>= 3;
    while(len--) {
      // set data bit and clock tck at once
      gpio_put(PIN_JTAG_TDI, *txd & 0x01); MCU_HW_JTAG_CLK_HI(); MCU_HW_JTAG_CLK_LOW();
      gpio_put(PIN_JTAG_TDI, *txd & 0x02); MCU_HW_JTAG_CLK_HI(); MCU_HW_JTAG_CLK_LOW();
      gpio_put(PIN_JTAG_TDI, *txd & 0x04); MCU_HW_JTAG_CLK_HI(); MCU_HW_JTAG_CLK_LOW();
      gpio_put(PIN_JTAG_TDI, *txd & 0x08); MCU_HW_JTAG_CLK_HI(); MCU_HW_JTAG_CLK_LOW();
      gpio_put(PIN_JTAG_TDI, *txd & 0x10); MCU_HW_JTAG_CLK_HI(); MCU_HW_JTAG_CLK_LOW();
      gpio_put(PIN_JTAG_TDI, *txd & 0x20); MCU_HW_JTAG_CLK_HI(); MCU_HW_JTAG_CLK_LOW();
      gpio_put(PIN_JTAG_TDI, *txd & 0x40); MCU_HW_JTAG_CLK_HI(); MCU_HW_JTAG_CLK_LOW();
      gpio_put(PIN_JTAG_TDI, *txd & 0x80); MCU_HW_JTAG_CLK_HI(); MCU_HW_JTAG_CLK_LOW();
      txd++;
    }
  } else {  
    while(len) {
      // send 1 of nothing was given
      int tx_bit = txd?((*txd & mask)?1:0):1;
      
      // set data bit and clock tck at once
      gpio_put(PIN_JTAG_TDI, tx_bit);  
      MCU_HW_JTAG_CLK_HI();
      
#ifdef DEBUG_TAP
      jtag_tap_advance_state(0, tx_bit);
#endif
      
#ifdef DEBUG_JTAG
      jtag_debugf("TMS 0 TDI %d TDO %d", tx_bit, gpio_get(PIN_JTAG_TDO));
#endif
      
      if(rxd) {
	// shift in from lsb
	if(gpio_get(PIN_JTAG_TDO)) *rxd |=  mask;
	else                       *rxd &= ~mask;
      }
      
      MCU_HW_JTAG_CLK_LOW();
      
      // advance bit mask
      mask <<= 1;
      if(!mask) {
	mask = 0x01;
	if(rxd) rxd++;      
	if(txd) txd++;
      }
      len--;
    }    
  }

  // We aren't really shifting, but instead setting bits
  // via mask. This makes a difference for the last byte
  // when not reading all 8 bits
  if(dlen && rxd) {
    // jtag_highlight_debugf("last byte %02x, rshift = %d", *rxd, dlen);
    *rxd <<= 8-dlen;
  }
#endif
}

void mcu_hw_jtag_toggleClk(uint32_t clk_len) {
#ifdef USE_PIO_JTAG
  // this is being used if the MCU is locally generating JTAG signales e.g. when
  // uploading a core from sd card
  while(clk_len > PIO_JTAG_MAX_BITS) {
    pio_jtag_write_tdi(&pio_jtag, true, NULL, PIO_JTAG_MAX_BITS);
    clk_len -= PIO_JTAG_MAX_BITS;
  }
  if(clk_len) pio_jtag_write_tdi(&pio_jtag, true, NULL, clk_len);
#else
  while(clk_len--) {
    MCU_HW_JTAG_CLK_HI();
    MCU_HW_JTAG_CLK_LOW();
  }
#endif
}

void mcu_hw_jtag_set_clock(uint32_t clk) {
  // Only PIO mode actually allows to adjust the clock. GPIO mode
  // will always run at ~6MHz
#ifdef USE_PIO_JTAG
  jtag_debugf("mcu_hw_jtag_set_clock(%lu)", clk);
  pio_jtag_clock_value = clk;
  
  pio_jtag_set_clk_freq(&pio_jtag, clk/1000);
#endif  
}

void mcu_hw_jtag_init(void) {
  // -------- init FPGA control pins ---------

#ifdef PIN_nCFG  
  // FPGA mode pins. Init as inputs, so the buttons work
  gpio_init(PIN_MODE0); // gpio_put(PIN_MODE0, 0);
  gpio_set_dir(PIN_MODE0, GPIO_IN);
  gpio_init(PIN_MODE1); // gpio_put(PIN_MODE1, 0);
  gpio_set_dir(PIN_MODE1, GPIO_IN);
  // FPGA reconfig pin, active low
  gpio_init(PIN_nCFG); gpio_put(PIN_nCFG, 1);
  gpio_set_dir(PIN_nCFG, GPIO_OUT);
#endif
  
  // -------- init FPGA JTAG pins ---------
  gpio_init(PIN_JTAG_TCK); gpio_init(PIN_JTAG_TDI);
  gpio_init(PIN_JTAG_TMS); gpio_init(PIN_JTAG_TDO);
  // init to all input, the JTAG engine will reconfigure
  // them if required
  mcu_hw_jtag_set_pins(0x00, 0x00);
}
#endif
