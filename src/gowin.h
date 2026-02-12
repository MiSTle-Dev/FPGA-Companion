#ifndef GOWIN_H
#define GOWIN_H

#include <stdbool.h>
#include <stdint.h>

// https://cdn.gowinsemi.com.cn/TN711E.pdf
/* known GOWIN JTAG commands */
#define GOWIN_COMMAND_BYPASS0             0x00
#define GOWIN_COMMAND_SAMPLE              0x01
#define GOWIN_COMMAND_NOOP                0x02
#define GOWIN_COMMAND_SRAM_READ           0x03
#define GOWIN_COMMAND_EXTEST              0x04
#define GOWIN_COMMAND_ERASE_SRAM          0x05
#define GOWIN_COMMAND_UNKNOWN1            0x08  // seems to be "DONE WRITING USERCODE"
#define GOWIN_COMMAND_XFER_DONE           0x09
#define GOWIN_COMMAND_UNKNOWN2            0x0a  // seems to be "WRITE USERCODE"
#define GOWIN_COMMAND_IDCODE              0x11
#define GOWIN_COMMAND_INIT_ADDR           0x12
#define GOWIN_COMMAND_USERCODE            0x13
#define GOWIN_COMMAND_CONFIG_ENABLE       0x15
#define GOWIN_COMMAND_TRANSFER_SPI        0x16
#define GOWIN_COMMAND_XFER_WRITE          0x17
#define GOWIN_COMMAND_XFER_SVF            0x1b
#define GOWIN_COMMAND_PROGRAM_KEY_1       0x21
#define GOWIN_COMMAND_SECURITY            0x23
#define GOWIN_COMMAND_PROGRAM_EFUSE       0x24
#define GOWIN_COMMAND_READ_KEY            0x25
#define GOWIN_COMMAND_PROGRAM_KEY_2       0x29  // gowin doesn't explain why there are two "PROGRAM KEY"
#define GOWIN_COMMAND_PRGM_USER_DATA      0x2a
#define GOWIN_COMMAND_RD_USER_DATA        0x2b
#define GOWIN_COMMAND_RD_EFUSE_ALL_DATA   0x2d
#define GOWIN_COMMAND_CONFIG_DISABLE      0x3a
#define GOWIN_COMMAND_WRITE_DATA          0x3b
#define GOWIN_COMMAND_RECONFIG            0x3c
#define GOWIN_COMMAND_BSCAN_2_SPI         0x3d
#define GOWIN_COMMAND_REINIT              0x3f
#define GOWIN_COMMAND_STATUS              0x41
#define GOWIN_COMMAND_GAO_1               0x42
#define GOWIN_COMMAND_GAO_2               0x43
#define GOWIN_COMMAND_WRITE_DATA_QSSPI    0x6b
#define GOWIN_COMMAND_EFLASH_PROGRAM      0x71
#define GOWIN_COMMAND_EFLASH_READ         0x73
#define GOWIN_COMMAND_EFLASH_ERASE        0x75
#define GOWIN_COMMAND_SWITCH_MCU_JTAG     0x7a
#define GOWIN_COMMAND_BYPASS              0xff

#define GOWIN_STATUS_CRC_ERROR            (1 <<  0)
#define GOWIN_STATUS_BAD_COMMAND          (1 <<  1)
#define GOWIN_STATUS_ID_VERIFY_FAILED	  (1 <<  2)
#define GOWIN_STATUS_TIMEOUT              (1 <<  3)
#define GOWIN_STATUS_AUTO_BOOT_2ND_FAIL   (1 <<  4) // GW2AR 0
#define GOWIN_STATUS_MEMORY_ERASE         (1 <<  5)
#define GOWIN_STATUS_PREAMBLE             (1 <<  6)
#define GOWIN_STATUS_SYSTEM_EDIT_MODE	  (1 <<  7)
#define GOWIN_STATUS_PRG_SPIFLASH_DIRECT  (1 <<  8)
#define GOWIN_STATUS_AUTO_BOOT_1ST_FAILED (1 <<  9) // GW2AR 0
#define GOWIN_STATUS_NON_JTAG_CNF_ACTIVE  (1 << 10)
#define GOWIN_STATUS_BYPASS               (1 << 11)
#define GOWIN_STATUS_I2C_SRAM_F           (1 << 12) // AST138 AT60K, GW2AR 0
#define GOWIN_STATUS_DONE_FINAL           (1 << 13)
#define GOWIN_STATUS_SECURITY_FINAL       (1 << 14)
#define GOWIN_STATUS_ENCRYPTED_FORMAT     (1 << 15) // AST138 AT60K
#define GOWIN_STATUS_KEY_IS_RIGHT         (1 << 16) // AST138 AT60K
#define GOWIN_STATUS_SSPI_MODE            (1 << 17) // AST138 AT60K, GW2AR 0
#define GOWIN_STATUS_SER_CRC_DONE         (1 << 18) // GW2AR 0
#define GOWIN_STATUS_SER_CRC_ERR          (1 << 19)
#define GOWIN_STATUS_SER_ECC_CORR         (1 << 20)
#define GOWIN_STATUS_SER_ECC_UNCORR       (1 << 21)
#define GOWIN_STATUS_SER_RUNNING          (1 << 22)
#define GOWIN_STATUS_CPU_BUS_WIDTH_0      (1 << 23)
#define GOWIN_STATUS_CPU_BUS_WIDTH_1      (1 << 24)
#define GOWIN_STATUS_SYNC_DET_TERY_0      (1 << 25) // AT60K A25
#define GOWIN_STATUS_SYNC_DET_TERY_1      (1 << 26) // AT60K A25
#define GOWIN_STATUS_DECOMP_FAIL          (1 << 27) // AT60K A25
#define GOWIN_STATUS_MFG_DONE             (1 << 28) // AT60K A25
#define GOWIN_STATUS_INIT                 (1 << 29) // AT60K A25
#define GOWIN_STATUS_WAKEUP               (1 << 30) // AT60K A25
#define GOWIN_STATUS_AUTO_ERASE           (1 << 31) // AT60K A25

#define IDCODE_GW2AR18      0x81b
#define IDCODE_GW5AT60    0x1481b
#define IDCODE_GW5AST138  0x1081b
#define IDCODE_GW5A25     0x1281b

bool gowin_upload_core(const char *name);
void gowin_fpgaReset(void);

#endif // GOWIN_H
