/*
  jtag.h

  mainly gowin related JTAG
*/

#ifndef JTAG_H
#define JTAG_H

#include <stdbool.h>
#include <stdint.h>

#include "mcu_hw.h"

// flags for jtag_shiftDR_part()
#define JTAG_FLAG_BEGIN 1   // get from RUN-TEST/IDLE into SHIFT-DR before transfer
#define JTAG_FLAG_END   2   // return into RUN-TEST/IDLE after transfer

void jtag_command_u08(uint8_t cmd);
uint32_t jtag_command_u08_read32(uint8_t cmd);

void jtag_shiftDR(uint8_t *tx, uint8_t *rx, uint16_t len);
void jtag_shiftDR_part(uint8_t *tx, uint8_t *rx, uint16_t len, uint8_t flags);

// toggle the jtag clock for the given time
static inline void jtag_clk_us(uint32_t us) { mcu_hw_jtag_toggleClk(15 * us); }

uint32_t jtag_open(void);
void jtag_close(void);

#endif // JTAG_H
