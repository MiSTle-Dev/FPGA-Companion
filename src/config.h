/*
  config.h - MiSTeryNano FPGA Companion

 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdarg.h>

#ifndef ESP_PLATFORM
#define CONFIG_MAX_PRIORITY          (32)
#else
#define CONFIG_MAX_PRIORITY          (25)
#endif

#define MAX_DRIVES                   (8)
#define MAX_HID_DEVICES              (6)
#define MAX_XBOX_DEVICES             (2)


/* ================== configuration as requested by the FPGA ================ */

#define CONFIG_ACTION_COMMAND_IDLE  0
#define CONFIG_ACTION_COMMAND_SET   1
#define CONFIG_ACTION_COMMAND_DELAY 2
#define CONFIG_ACTION_COMMAND_SAVE  3
#define CONFIG_ACTION_COMMAND_LOAD  4
#define CONFIG_ACTION_COMMAND_HIDE  5
#define CONFIG_ACTION_COMMAND_LINK  6

typedef struct {
  unsigned char code;
  union {
    struct {
      unsigned char id;
      unsigned char value;
    } set;
    struct {
      unsigned short ms;
    } delay;    
    char *filename;
    struct config_action_S *action;
  };
} config_action_command_t;

typedef struct config_action_S {
  char *name;
  config_action_command_t *commands;
} config_action_t;

typedef struct {
  unsigned char index;
  char *label;
  char *def;
  char **ext;
  config_action_t *action;
} config_fsel_t;

typedef struct {
  char *label;
  unsigned char value;
} config_listentry_t;

typedef struct {
  unsigned char id;
  char *label;
  unsigned char def;
  config_listentry_t **listentries;
  config_action_t *action;
} config_list_t;

typedef struct {
  char *label;
  config_action_t *action;
} config_button_t;

#define CONFIG_MENU_ENTRY_UNKNOWN       0
#define CONFIG_MENU_ENTRY_MENU          1
#define CONFIG_MENU_ENTRY_FILESELECTOR  2
#define CONFIG_MENU_ENTRY_LIST          3
#define CONFIG_MENU_ENTRY_BUTTON        4

typedef struct {
  unsigned char type;
  union {
    struct config_menu_S *menu;
    config_fsel_t *fsel;    
    config_list_t *list;    
    config_button_t *button;
  };  
} config_menu_entry_t;

typedef struct config_menu_S {
  char *label;
  config_menu_entry_t *entries;
} config_menu_t;

typedef struct {
  char *name;
  int version;
  config_action_t **actions;
  config_menu_t *menu;
} config_t;

// the global config
extern config_t *cfg;

void config_init(void);
config_action_t *config_get_action(const char *);
void config_dump(void);
const char *config_menuentry_get_type_str(config_menu_entry_t *);

#endif // CONFIG_H
