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

#define MENU_EVENT_TOGGLE       10

// The system menu may want to deal with
// the USB mass storage once it's mounted
#define MENU_EVENT_USB_MOUNTED  11
#define MENU_EVENT_USB_UMOUNTED 12

#define MENU_EVENT_SYSTEM       13

#define MENU_EVENT_BLUETOOTH_CONNECTED  14
#define MENU_EVENT_BLUETOOTH_DISCONNECTED  15
#define MENU_EVENT_BLUETOOTH_SCAN  16
#define MENU_EVENT_BLUETOOTH_PIN_CODE_REQUEST 17

#define MENU_EVENT_KEY_LATIN1    256  // 256..511

typedef struct menu_variable {
  char id;
  int value;
  struct menu_variable *next;
} menu_variable_t;

extern TaskHandle_t menu_handle;

void menu_init(void);
menu_variable_t *menu_get_variables(void);
void menu_set_value(unsigned char id, char value);
void menu_do(int);
void menu_notify(unsigned long msg);
void menu_joystick_state(unsigned char state);
void menu_button_state(unsigned char state);
void menu_run_current_image_action(void);

#endif // MENU_H
