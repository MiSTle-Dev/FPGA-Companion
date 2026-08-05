
#ifndef _PIO_JTAG_H
#define _PIO_JTAG_H

#include "hardware/pio.h"
#include "hardware/dma.h"

#ifdef PICO_RP2350
#define PIO_INDEX(a)  (((a)->pio==pio2)?2:((a)->pio==pio1)?1:0)
#else
#define PIO_INDEX(a)  (((a)->pio==pio1)?1:0)
#endif

typedef struct pio_jtag_inst {
  PIO pio;
  uint sm;
  uint pin_tck, pin_tdi, pin_tdo, pin_tms;
  uint pins_upper[4];

  bool pio_enabled;
  uint8_t gpio_dir;
  
  int tx_dma_chan;
  int rx_dma_chan;
  dma_channel_config tx_c;
  dma_channel_config rx_c;
  
  // buffer allowing double buffered background writing
  // this is twice the size of the 64 byte USB endpoint to have enough
  // room for the interleaved bits
  bool write_pending;
  uint8_t write_buffer[128];
} pio_jtag_inst_t;

// bits to request single bit states via pio_set_outputs()
#define PIO_JTAG_BIT_TDI  1  // D1
#define PIO_JTAG_BIT_TMS  2  // D3
#define PIO_JTAG_BIT_TCK  4  // D0

//interface to the PIO engine itself
void pio_jtag_init(pio_jtag_inst_t* jtag, uint freq);
void pio_jtag_set_clk_freq(pio_jtag_inst_t *jtag, uint freq_khz);
void pio_set_outputs(pio_jtag_inst_t *jtag, uint8_t bits);

// convenience and support functions
void pio_jtag_enable(pio_jtag_inst_t* jtag, bool enable); 
void pio_jtag_write_tms(pio_jtag_inst_t* jtag, bool lsb, uint tdi, const uint8_t *src, uint8_t *dst, size_t len);		    
void pio_jtag_write_tdi_read_tdo(pio_jtag_inst_t* jtag, bool lsb, const uint8_t *src, uint8_t *dst, size_t len);
void pio_jtag_write_tdi(pio_jtag_inst_t* jtag, bool lsb, const uint8_t *src, size_t len);
void pio_jtag_read_tdo(pio_jtag_inst_t* jtag, bool lsb, uint8_t *dst, size_t len);

void pio_jtag_clock(pio_jtag_inst_t* jtag, uint32_t pulses);
#endif
