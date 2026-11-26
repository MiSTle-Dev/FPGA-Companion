/*
  rp2040/board.h


*/

#ifndef BOARD_H
#define BOARD_H

// ENABLE_BLUETOOTH and ENABLE_WIFI are set in the CMakeList for
// boards 0, 1, 6 and 7. As they are also used in non-rp2040 files
// which don't include board.h

#if MISTLE_BOARD == 0
//#warning "Building for Raspberry Pi Pico or Pico-W"
#define BOARD_NAME "Pico, Pico-W"
#elif MISTLE_BOARD == 1
//#warning "Building for Raspberry Pi Pico2 or Pico2-W"
#define BOARD_NAME "Pico2, Pico2-W"
#elif MISTLE_BOARD == 2
//#warning "Building for Waveshare RP2040-Zero"
#define BOARD_NAME "WS RP2040-Zero"
#define ENABLE_JTAG
#elif MISTLE_BOARD == 3
//#warning "Building for MiSTeryShield20k-Lite"
#define BOARD_NAME "Shield20k-Lite"
#elif MISTLE_BOARD == 4
//#warning "Building for MiSTeryDev20k"
#define BOARD_NAME "MiSTeryDev20k"
#define ENABLE_JTAG
#define ENABLE_USB_DEVICE
#define ENABLE_SDIO
#elif MISTLE_BOARD == 5
//#warning "Building for MiSTeryShieldPico_TN20k"
#define BOARD_NAME "ShieldPico TN20k"
#define ENABLE_JTAG
#define ENABLE_USB_DEVICE
#elif MISTLE_BOARD == 6
//#warning "Building for GW3A/GW5A DEV 25K"
#define BOARD_NAME "MiSTle Dev25k"
#define ENABLE_JTAG
#define ENABLE_USB_DEVICE
#define ENABLE_SDIO
#elif MISTLE_BOARD == 7
//#warning "Building for MiSTeryShieldPico2_TN20k"
#define BOARD_NAME "ShieldPico2 TN20k"
#define ENABLE_JTAG
#define ENABLE_USB_DEVICE
#else
#error "Not a supported MiSTle board!"
#endif

// ------------------------------------------------------------------
// ------------------------------- JTAG -----------------------------
// ------------------------------------------------------------------

#ifdef ENABLE_JTAG
#include "../jtag.h"

#ifdef PICO_RP2350
// only the rp2350 has the additional PIO needed
#define ENABLE_PIO_JTAG
#include "pio_jtag.h"
#endif

#if MISTLE_BOARD == 2
// Waveshare Zero
#define PIN_JTAG_TDI   9
#define PIN_JTAG_TMS  10
#define PIN_JTAG_TDO  11
#define PIN_JTAG_TCK  12

#elif MISTLE_BOARD == 4 || MISTLE_BOARD == 6
// dev20k and dev25k
#define PIN_JTAG_TDI  12   // pin 15, gpio 12
#define PIN_JTAG_TMS  13   // pin 16, gpio 13
#define PIN_JTAG_TDO  14   // pin 17, gpio 14
#define PIN_JTAG_TCK  15   // pin 18, gpio 15

// special FPGA configuration pins
#define PIN_nCFG      21   // pin 32, PIO21
#define PIN_MODE0     23   // pin 35, PIO23
#define PIN_MODE1     24   // pin 36, PIO24

#elif MISTLE_BOARD == 5 || MISTLE_BOARD == 7
// Shield Pico and Pico2 TN20k
#define PIN_JTAG_TDI  12   // pin 15, gpio 12
#define PIN_JTAG_TMS  13   // pin 16, gpio 13
#define PIN_JTAG_TDO  14   // pin 17, gpio 14
#define PIN_JTAG_TCK  15   // pin 18, gpio 15

#else
#error "JTAG enabled but no JTAG pins specified for this board!"
#endif

#endif

// all baords support debug via serial
#include "../debug.h"

// ------------------------------------------------------------------
// ------------------------------- SPI ------------------------------
// ------------------------------------------------------------------

#if MISTLE_BOARD == 2
// the waveshare mini does not expose the default spi0 pins, so we need
// to specify them
#define SPI_RX_PIN     4
#define SPI_SCK_PIN    6
#define SPI_TX_PIN     7
#define SPI_CSN_PIN    5
#define SPI_IRQ_PIN    8
#define SPI_BUS     spi0
#else
// the regular pi pico uses spi0 by default
#define SPI_RX_PIN   PICO_DEFAULT_SPI_RX_PIN
#define SPI_SCK_PIN  PICO_DEFAULT_SPI_SCK_PIN
#define SPI_TX_PIN   PICO_DEFAULT_SPI_TX_PIN
#define SPI_CSN_PIN  PICO_DEFAULT_SPI_CSN_PIN
#define SPI_IRQ_PIN  22
#define SPI_BUS      spi_default
#endif

// ------------------------------------------------------------------
// ------------------------------- LEDS -----------------------------
// ------------------------------------------------------------------

#define DEFAULT_LED_MODE  1   // 0=blink, 1=on, 2=off

#if MISTLE_BOARD == 2
// the ws2040 zero is the only board that uses a ws2812 led
#define WS2812_PIN    16
#include "ws2812.pio.h"
#else
// all other boards use use GPIO4, 5 and 6 for status
// indicator leds. These are e.g. present on the
// PiPico shield
#define LED_MOUSE_PIN    4
#define LED_KEYBOARD_PIN 5
#define LED_JOYSTICK_PIN 6
#endif

// ------------------------------------------------------------------
// ------------------------------- USB ------------------------------
// ------------------------------------------------------------------

#include "tusb.h"
#include "tusb_option.h"

extern void mcu_hw_usb_check_devices(void);

// most usb devices are supported by all boards
#define ENABLE_USB_XBOX
#include "usb_xbox.h"
#define ENABLE_USB_HID
#include "usb_hid.h"

#ifndef TUSB_VERSION_NUMBER
#error "Cannot determine TinyUSB version!"
#endif

#if TUSB_VERSION_NUMBER < 2000
#error "Please update your TinyUSB installation!"
#endif

#include "tusb_config.h"
#if defined(WS2812_PIN) && CFG_TUH_RPI_PIO_USB == 1
#error "WS2812B and PIO USB cannot be used simultaneously!"
#endif

#endif // BOARD_H
