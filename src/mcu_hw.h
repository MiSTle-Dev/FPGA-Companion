#ifndef MCU_HW_H
#define MCU_HW_H

#include <stdbool.h>
#include <stdint.h>

#define LOGO "\r\n\r\n\033[1;33m"\
"       __  __ _ ____ _____ _\r\n"\
"      |  \\/  (_) ___|_   _| | ___\r\n"\
"      | |\\/| | \\___ \\ | | | |/ o_)\r\n"\
"      |_|  |_|_|____/ |_| |_|\\___|\033[0m\r\n"\
"      \033[1;36mhttp://github.com/MiSTle-Dev\033[0m\r\n"

void mcu_hw_init(void);
void mcu_hw_main_loop(void);

void mcu_hw_irq_ack(void);
void mcu_hw_reset(void);

// HW SPI interface
void mcu_hw_spi_begin(void);
unsigned char mcu_hw_spi_tx_u08(unsigned char b);
void mcu_hw_spi_end(void);

// received a byte via the io port (e.g. rs232 from core)
void mcu_hw_port_byte(unsigned char);

void mcu_hw_wifi_scan(void);
void mcu_hw_wifi_connect(char *ssid, char *key);
void mcu_hw_tcp_connect(char *ip, int port);
void mcu_hw_tcp_disconnect(void);
bool mcu_hw_tcp_data(unsigned char byte);

// some boards provide a connection to the FPGAs JTAG interface
#ifdef TANG_CONSOLE60K
// 0 = FPGA , 1 = BL616
#define PIN_TF_SDIO_SEL GPIO_PIN_16
#elif TANG_NANO20K
// dummy Nano20k unused pin
#define PIN_TF_SDIO_SEL GPIO_PIN_17
#endif
#if defined(TANG_CONSOLE60K) || defined(TANG_NANO20K)
void mcu_hw_jtag_set_pins(uint8_t dir, uint8_t data);
uint8_t mcu_hw_jtag_tms(uint8_t tdi, uint8_t data, int len);
void mcu_hw_jtag_data(uint8_t *txd, uint8_t *rxd, int len);
void mcu_hw_fpga_reconfig(bool state);
bool mcu_hw_jtag_is_active(void);
void jtag_toggleClk(uint32_t);

// this board also has the ability to boot the FPGA from SD card
#include "sdc_direct.h"
#define FPGA_BOOT_TIMEOUT 5000    // give FPGA 5 seconds to boot
#define BOOT_FROM_SDC sdc_boot    // afterwards this will be called

#elif MISTLE_BOARD == 4
// currently only the Dev20k
void mcu_hw_jtag_set_pins(uint8_t dir, uint8_t data);
uint8_t mcu_hw_jtag_tms(uint8_t tdi, uint8_t data, int len);
void mcu_hw_jtag_data(uint8_t *txd, uint8_t *rxd, int len);
void mcu_hw_fpga_reconfig(bool state);
bool mcu_hw_jtag_is_active(void);

// this board also has the ability to boot the FPGA from SD card
#include "sdc_direct.h"
#define FPGA_BOOT_TIMEOUT 5000    // give FPGA 5 seconds to boot
#define BOOT_FROM_SDC sdc_boot    // afterwards this will be called

// give file system driver in sdc.c access to the local sd card
#define SDC_DIRECT_READ   sdc_direct_read
#define SDC_DIRECT_WRITE  sdc_direct_write

#endif

#endif // MCU_HW_H
