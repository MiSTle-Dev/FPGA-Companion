/*
  at_wifi.h
*/

#ifndef AT_WIFI_H
#define AT_WIFI_H

void at_wifi_init(void);
void at_wifi_port_byte(unsigned char);
void at_wifi_puts(char *);
void at_wifi_puts_n(char *, int);

#endif // AT_WIFI_H
