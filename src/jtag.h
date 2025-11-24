/*
  jtag.h

  mainly gowin related JTAG
*/

#ifndef JTAG_H
#define JTAG_H

#include <stdbool.h>
#include <stdint.h>

/* known GOWIN JTAG commands */
#define JTAG_COMMAND_GOWIN_BYPASS0             0x00
#define JTAG_COMMAND_GOWIN_SAMPLE              0x01
#define JTAG_COMMAND_GOWIN_NOOP                0x02
#define JTAG_COMMAND_GOWIN_SRAM_READ           0x03
#define JTAG_COMMAND_GOWIN_EXTEST              0x04
#define JTAG_COMMAND_GOWIN_ERASE_SRAM          0x05
#define JTAG_COMMAND_GOWIN_UNKNOWN1            0x08  // seems to be "DONE WRITING USERCODE"
#define JTAG_COMMAND_GOWIN_XFER_DONE           0x09
#define JTAG_COMMAND_GOWIN_UNKNOWN2            0x0a  // seems to be "WRITE USERCODE"
#define JTAG_COMMAND_GOWIN_IDCODE              0x11
#define JTAG_COMMAND_GOWIN_INIT_ADDR           0x12
#define JTAG_COMMAND_GOWIN_USERCODE            0x13
#define JTAG_COMMAND_GOWIN_CONFIG_ENABLE       0x15
#define JTAG_COMMAND_GOWIN_TRANSFER_SPI        0x16
#define JTAG_COMMAND_GOWIN_XFER_WRITE          0x17
#define JTAG_COMMAND_GOWIN_PROGRAM_KEY_1       0x21
#define JTAG_COMMAND_GOWIN_SECURITY            0x23
#define JTAG_COMMAND_GOWIN_PROGRAM_EFUSE       0x24
#define JTAG_COMMAND_GOWIN_PROGRAM_KEY_2       0x29  // gowin doesn't explain why there are two "PROGRAM KEY"
#define JTAG_COMMAND_GOWIN_READ_KEY            0x25
#define JTAG_COMMAND_GOWIN_CONFIG_DISABLE      0x3a
#define JTAG_COMMAND_GOWIN_RECONFIG            0x3c
#define JTAG_COMMAND_GOWIN_STATUS              0x41
#define JTAG_COMMAND_GOWIN_GAO_1               0x42
#define JTAG_COMMAND_GOWIN_GAO_2               0x43
#define JTAG_COMMAND_GOWIN_EFLASH_PROGRAM      0x71
#define JTAG_COMMAND_GOWIN_EFLASH_ERASE        0x75
#define JTAG_COMMAND_GOWIN_SWITCH_MCU_JTAG     0x7a
#define JTAG_COMMAND_GOWIN_BYPASS              0xff

#define JTAG_GOWIN_STATUS_CRC_ERROR		(1 << 0)
#define JTAG_GOWIN_STATUS_BAD_COMMAND		(1 << 1)
#define JTAG_GOWIN_STATUS_ID_VERIFY_FAILED	(1 << 2)
#define JTAG_GOWIN_STATUS_TIMEOUT		(1 << 3)
#define JTAG_GOWIN_STATUS_MEMORY_ERASE		(1 << 5)
#define JTAG_GOWIN_STATUS_PREAMBLE		(1 << 6)
#define JTAG_GOWIN_STATUS_SYSTEM_EDIT_MODE	(1 << 7)
#define JTAG_GOWIN_STATUS_PRG_SPIFLASH_DIRECT 	(1 << 8)
#define JTAG_GOWIN_STATUS_NON_JTAG_CNF_ACTIVE 	(1 << 10)
#define JTAG_GOWIN_STATUS_BYPASS		(1 << 11)
#define JTAG_GOWIN_STATUS_GOWIN_VLD		(1 << 12)
#define JTAG_GOWIN_STATUS_DONE_FINAL		(1 << 13)
#define JTAG_GOWIN_STATUS_SECURITY_FINAL	(1 << 14)
#define JTAG_GOWIN_STATUS_READY			(1 << 15)
#define JTAG_GOWIN_STATUS_POR			(1 << 16)
#define JTAG_GOWIN_STATUS_FLASH_LOCK		(1 << 17)

bool jtag_open(void);
void jtag_close(void);

// these are actually for 8 bit commands and thus somewhat gowin specific
void jtag_command(uint8_t cmd);
uint32_t jtag_command_read32(uint8_t cmd);

// these are obviously gowin specific
bool jtag_gowin_eraseSRAM(void);
bool jtag_gowin_writeSRAM_prepare(void);
bool jtag_gowin_writeSRAM_transfer(uint8_t *data, uint16_t len, bool first, bool last);
bool jtag_gowin_writeSRAM_postproc(uint32_t checksum);

#endif // JTAG_H
