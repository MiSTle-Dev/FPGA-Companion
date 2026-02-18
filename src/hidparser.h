#include "usb_controller_maps.h"

#ifndef HIDPARSER_H
#define HIDPARSER_H

#include <stdint.h>

#define REPORT_TYPE_NONE     0
#define REPORT_TYPE_MOUSE    1
#define REPORT_TYPE_KEYBOARD 2
#define REPORT_TYPE_JOYSTICK 3

#define MAX_AXES 8

// currently only joysticks are supported
typedef struct {
  uint8_t type: 2;               // REPORT_TYPE_...
  uint8_t report_id_present: 1;  // REPORT_TYPE_...
  uint8_t report_id;
  uint8_t report_size;

  union {
    struct {
      struct {
	uint16_t offset;
	uint8_t size;
	struct {
	  uint16_t min;
	  uint16_t max;
	} logical;
      } axis[MAX_AXES];               // x and y axis + wheel or right hat
      
      struct {
	uint8_t byte_offset;
	uint8_t bitmask;
       } button[32]; // 12 buttons max
      
      struct {
	uint16_t offset;
	uint8_t size;
	struct {
	  uint16_t min;
	  uint16_t max;
	} logical;
	struct {
	  uint16_t min;
	  uint16_t max;
	} physical;
      } hat;                   // 1 hat (joystick only)
    } joystick_mouse;
  };

	/* --- SDL map base --- */
	const UsbGamepadMap *map; // NULL if no match; points to .rodata (flash)
	uint8_t map_found : 1;	  // 1=ok, 0=no
	uint8_t map_checked : 1;  // 1=already checked
	uint8_t _reserved_flags : 6;

} hid_report_t;

bool parse_report_descriptor(const uint8_t *rep, uint16_t rep_size, hid_report_t *conf, uint16_t *rbytes);

#endif // HIDPARSER_H
