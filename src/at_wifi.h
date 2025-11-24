/*
  at_wifi.h
*/

#ifndef AT_WIFI_H
#define AT_WIFI_H

void at_wifi_init(void);
void at_wifi_port_byte(unsigned char);
void at_wifi_puts(const char *);
void at_wifi_puts_n(const char *, int);
uint8_t pet2asc(uint8_t c);
uint8_t asc2pet(uint8_t c);

#endif // AT_WIFI_H
