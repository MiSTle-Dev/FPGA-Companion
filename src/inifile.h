/*
  inifile.h
 */

#ifndef INIFILE_H
#define INIFILE_H

#include <stdbool.h>
#include <stdint.h>

#define INIFILE_OPTION_HOTKEY   0   // HID key code
#define INIFILE_OPTION_LED      1   // 0 = blink, 1 = on, 0 = off

// handle the core specific ini file
int inifile_read(char *);
void inifile_write(char *);
int inifile_option_get(int id);

// handle the global ini file (config.ini)
void inifile_config_read(void);
bool inifile_config_has(char *section, char *name);
uint32_t inifile_config_get_ip_n(char *section, char *name, uint32_t def, int n);
uint32_t inifile_config_get_ip(char *section, char *name, uint32_t def);
int inifile_config_get_int(char *section, char *name, int def);
char *inifile_config_get_str(char *section, char *name);
int inifile_config_num_values(char *section, char *name);

#endif // INIFILE_H
