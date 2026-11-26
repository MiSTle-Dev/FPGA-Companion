#ifndef USB_HID_H
#define USB_HID_H

#define USB_HID_KEYBOARD 1
#define USB_HID_JOYSTICK 2
#define USB_HID_MOUSE    4
#define USB_HID_ALL      (USB_HID_KEYBOARD | USB_HID_JOYSTICK | USB_HID_MOUSE)

void usb_hid_init(void);
int usb_hid_present(int type);

#endif // USB_HID_H
