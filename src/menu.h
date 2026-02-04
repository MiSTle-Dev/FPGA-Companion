#ifndef MENU_H
#define MENU_H

#include "osd.h"
#include "sdc.h"

#define MENU_EVENT_NONE          0  // just redraws the menu

#define MENU_EVENT_UP            1  // key events
#define MENU_EVENT_DOWN          2
#define MENU_EVENT_LEFT          3
#define MENU_EVENT_RIGHT         4
#define MENU_EVENT_SELECT        5
#define MENU_EVENT_PGUP          6
#define MENU_EVENT_PGDOWN        7
#define MENU_EVENT_BACK          8
#define MENU_EVENT_KEY_RELEASE   9

#define MENU_EVENT_SHOW         10
#define MENU_EVENT_HIDE         11

// The system menu may want to deal with
// the USB mass storage once it's mounted
#define MENU_EVENT_USB_MOUNTED  12
#define MENU_EVENT_USB_UMOUNTED 13

#define MENU_EVENT_SYSTEM       14

typedef struct menu_variable {
  char id;
  int value;
  struct menu_variable *next;
} menu_variable_t;

extern TaskHandle_t menu_handle;

void menu_init(void);
menu_variable_t *menu_get_variables(void);
void menu_set_value(unsigned char id, unsigned char value);
void menu_do(int);
void menu_notify(unsigned long msg);
void menu_joystick_state(unsigned char state);
void menu_button_state(unsigned char state);

// functions dealing with the internal system
void menu_system_enable(char on);

#endif // MENU_H
