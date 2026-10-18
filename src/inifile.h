/*
  inifile.h
 */

#ifndef INIFILE_H
#define INIFILE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define INIFILE_OPTION_HOTKEY   0   // HID key code
#define INIFILE_OPTION_LED      1   // 0 = blink, 1 = on, 0 = off

// handle the core specific ini file
int inifile_read(char *);
int inifile_write(char *);   // 0 = written, -1 = failed
int inifile_option_get(int id);

// the ini selected through a <cfgselector> is stored in the default ini
int inifile_selection_get(const char *def, char *buf, size_t size);  // 0 = def read, buf empty if none stored
int inifile_selection_set(const char *def, const char *name);        // 0 = stored

// handle the global ini file (config.ini)
void inifile_config_read(void);
bool inifile_config_is_read(void);  // true once config.ini has been processed
void inifile_config_wait(void);     // blocks until then, one waiting task at a time
bool inifile_config_has(char *section, char *name);
uint32_t inifile_config_get_ip_n(char *section, char *name, uint32_t def, int n);
uint32_t inifile_config_get_ip(char *section, char *name, uint32_t def);
int inifile_config_get_int(char *section, char *name, int def);
char *inifile_config_get_str(char *section, char *name);
int inifile_config_num_values(char *section, char *name);

#endif // INIFILE_H
