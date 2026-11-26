#ifndef USB_CONTROLLER_MAPS_IMPL_H
#define USB_CONTROLLER_MAPS_IMPL_H

#include <stdio.h>
#include <stdint.h>
#include "../usb_controller_maps.h"

extern const UsbGamepadMap *usb_controller_maps_get_data_frog_pc07(const UsbGamepadMap *);
extern const UsbGamepadMap *usb_controller_maps_find(uint16_t vid, uint16_t pid, int version_optional);
extern void usb_controller_maps_show(const UsbGamepadMap *map);

#endif // USB_CONTROLLER_MAPS_IMPL_H
