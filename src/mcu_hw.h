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
void mcu_hw_jtag_tms(uint8_t tdi, uint32_t data, int len);
void mcu_hw_jtag_data(uint8_t *txd, uint8_t *rxd, int len);

#endif // MCU_HW_H
