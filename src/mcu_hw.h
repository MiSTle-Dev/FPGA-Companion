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

bool mcu_hw_hid_present(void);
void mcu_hw_usb_sector_read(void *buffer, int sector, int count);
void mcu_hw_upload_core(char *name);
bool mcu_hw_usb_msc_present(void);

// received a byte via the io port (e.g. rs232 from core)
void mcu_hw_port_byte(unsigned char);

void mcu_hw_wifi_scan(void);
void mcu_hw_wifi_connect(char *ssid, char *key);
void mcu_hw_tcp_connect(char *ip, int port);
void mcu_hw_tcp_disconnect(void);
bool mcu_hw_tcp_data(unsigned char byte);

// some boards provide a connection to the FPGAs JTAG interface
#ifdef TANG_CONSOLE60K
#define PIN_TF_SDIO_SEL GPIO_PIN_16
#define DIRECT_SDC_SUPPORTED
#endif
#if defined(TANG_CONSOLE60K)||defined(TANG_NANO20K)||defined(TANG_MEGA138KPRO)||defined(TANG_MEGA60K)||defined(TANG_PRIMER25K)
void mcu_hw_jtag_set_pins(uint8_t dir, uint8_t data);
uint8_t mcu_hw_jtag_tms(uint8_t tdi, uint8_t data, int len);
void mcu_hw_jtag_data(uint8_t *txd, uint8_t *rxd, int len);
void mcu_hw_fpga_reconfig(bool state);
bool mcu_hw_jtag_is_active(void);
void mcu_hw_jtag_toggleClk(uint32_t);
void jtag_writeTDI_msb_first_gpio_out_mode(uint8_t *tx, unsigned int bytes, bool end);
void jtag_enter_gpio_out_mode(void); 
void jtag_exit_gpio_out_mode(void);

// the BL616 based platforms implement custom JTAG routines for fast bulk transfers
#define MCU_HW_JTAG_GPIO_OUT_MODE
void mcu_hw_jtag_enter_gpio_out_mode(void);
void mcu_hw_jtag_exit_gpio_out_mode(void);
void mcu_hw_jtag_writeTDI_msb_first_gpio_out_mode(uint8_t *tx, unsigned int bytes, bool end);

// this board also has the ability to boot the FPGA from SD card
#define ENABLE_JTAG
#define FPGA_BOOT_TIMEOUT 5000    // give FPGA 5 seconds to boot

#elif defined (MISTLE_BOARD)

#if MISTLE_BOARD == 4
#include "sdio.h"
// currently only the Dev20k
void mcu_hw_jtag_set_pins(uint8_t dir, uint8_t data);
uint8_t mcu_hw_jtag_tms(uint8_t tdi, uint8_t data, int len);
void mcu_hw_jtag_data(uint8_t *txd, uint8_t *rxd, int len);
void mcu_hw_fpga_reconfig(bool state);
bool mcu_hw_jtag_is_active(void);
void mcu_hw_jtag_toggleClk(uint32_t);

// this board also has the ability to boot the FPGA from SD card
#include "sdio.h"
#define DIRECT_SDC_SUPPORTED
#define ENABLE_JTAG
#define FPGA_BOOT_TIMEOUT 5000    // give FPGA 5 seconds to boot

// give file system driver in sdio.c access to the local sdio sd card
#define SDIO_DIRECT_READ   sdio_sector_read
#define SDIO_DIRECT_WRITE  sdio_sector_write
#endif

#endif

#endif // MCU_HW_H
