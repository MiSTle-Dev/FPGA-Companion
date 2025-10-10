#ifndef SPI_H
#define SPI_H

#ifndef ESP_PLATFORM
#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <task.h>
#endif

#define SPI_TARGET_SYS    0   // system control target
#define SPI_SYS_STATUS    0
#define SPI_SYS_LEDS      1
#define SPI_SYS_RGB       2
#define SPI_SYS_BUTTONS   3
#define SPI_SYS_SETVAL    4
#define SPI_SYS_IRQ_CTRL  5
#define SPI_SYS_IRQ_SRC   6
#define SPI_SYS_PORT      7
#define SPI_SYS_READ_CFG  8

// port subcommands
#define SPI_SYS_PORT_STATUS 0
#define SPI_SYS_PORT_GET  1
#define SPI_SYS_PORT_PUT  2

#define SPI_TARGET_HID    1   // human interface devices
#define SPI_HID_STATUS    0
#define SPI_HID_KEYBOARD  1
#define SPI_HID_MOUSE     2
#define SPI_HID_JOYSTICK  3
#define SPI_HID_GET_DB9   4

#define SPI_TARGET_OSD    2   // on-screen-display
#define SPI_OSD_ENABLE    1
#define SPI_OSD_WRITE     2

#define SPI_TARGET_SDC    3   // sd card
#define SPI_SDC_STATUS    1   // get sd card status
#define SPI_SDC_CORE_RW   2   // trigger core read/write
#define SPI_SDC_MCU_READ  3   // read sector into MCU (e.g. for dir listing)
#define SPI_SDC_INSERTED  4   // inform core that some disk image has been insered
#define SPI_SDC_MCU_WRITE 5   // write sector from MCU
#define SPI_SDC_DIRECT    6   // inform core that disk image may direclty be accessed
#define SPI_SDC_INS_LARGE 7   // inform core that some large disk image > 4GB has been insered
#define SPI_SDC_IMAGE     8   // read rom image command

// image subcommands
#define SPI_SDC_IMAGE_STATUS  0
#define SPI_SDC_IMAGE_SELECT  1
#define SPI_SDC_IMAGE_WRITE   2

#define SPI_TARGET_AUDIO  4   // audio (e.g. to play fake floppy sounds)
#define SPI_AUDIO_ENABLE  1
#define SPI_AUDIO_BUFFER  2   // return audio buffer usage
#define SPI_AUDIO_WRITE   3
  
// this is still on usb_host.c but should eventially go
// into a separate hid.c
extern void hid_handle_event(void);

#endif // SPI_H
