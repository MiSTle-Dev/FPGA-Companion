# MiSTeryNano FPGA Companion

The MiSTeryNano FPGA Companion implements support functions for FPGA
based retro computing projects like [MiSTeryNano
project](https://github.com/MiSTle-Dev/MiSTeryNano). While the FPGA
typically implements the hardware of the retro machine itself the
Companion uses a microcontroller to add support for modern peripherals
like USB keyboard, mice and SD cards. It also implements an
on-screen-display menu to allow the user to configure the retro
machine.

The FPGA Companion replaces the MiSTeryNano firmware that was
formerly part of the [MiSTeryNano
project](https://github.com/MiSTle-Dev/MiSTeryNano). It is also
used by the [NanoMig](https://github.com/MiSTle-Dev/nanomig),
the [NanoMac](https://github.com/MiSTle-Dev/nanomac), the
[C64Nano](https://github.com/vossstef/tang_nano_20k_c64), the
[VIC20Nano](https://github.com/vossstef/VIC20Nano), the
[A2600Nano](https://github.com/vossstef/A2600Nano) and the 
[NanoApple2](https://github.com/vossstef/NanoApple2).

## Supported MCUs

While the MiSTeryNano was initially designed with a BL616 MCU as the
support MCU the FPGA Companion introduces more flexibility and allows
to choose from different MCUs to act as the support MCU.  From the
FPGAs perspective these behave identical although not all MCUs may
support all functions to the same extent and e.g. the ESP32 is rather
limited when it comes to USB support.

Currently the FPGA Companion can be used with the following MCUs:

 - [M0S/BL616](https://wiki.sipeed.com/hardware/en/maixzero/m0s/m0s.html) or TANG onboard Bl616, see the [build instuctions](src/bl616), and
 - [Raspberry Pi Pico/RP2040](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html), see the [build instructions](src/rp2040)
 - [ESP32 S2/S3](https://www.espressif.com/en/products/socs/esp32-s2), see the [build instructions](src/esp32)

## Features and disadvantages of the different MCUs

The inital version of MiSTeryNano relied on the BL616 as a support
MCU.  Some shortcomings of that platform caused the code to be ported
to other MCUs which then may have their own advantages and
disadvantages.

### BL616

  - Pros
    - Very powerful Risc-V CPU
    - USB 2.0 highspeed host support
    - WiFi 6 support
    - Bluetooth 5.2 support
  - Cons
    - Limited SDK support
      - USB needs manual update of the CherryUSB stack
      - No classic Bluetooth support
      - Limited WiFi support

### RP2040

  - Pros
    - Powerful and well-supported SDK
    - Widely available and cheap
    - Fullspeed USB host support
  - Cons
    - No built-in bluetooth and WiFi support
      - Only available via seperate modules (e.g. on Pico(W))

### ESP32-S2/S3

  - Pros
    - Powerful and well-supported SDK
    - Widely available and cheap
    - Built-in Bluetooth and WiFi
  - Cons
    - Very limited USB host support
      - Only one device (no hub)
      - USB stack complex to use

## Related projects

You might also want to check out the following related projects:

  - [MiSTeryNano](https://github.com/MiSTle-Dev/MiSTeryNano) HDL implementation of the Atari ST home computer
  - [NanoMIG](https://github.com/MiSTle-Dev/NanoMIG) HDL implementation of the Commodore Amiga home computer
  - [NanoMAC](https://github.com/MiSTle-Dev/NanoMac) HDL implementation of the Apple Macintosh Plus computer
  - [C64 Nano](https://github.com/vossstef/tang_nano_20k_c64) HDL implementationm of the Commodore C64 home computer
  - [VIC20 Nano](https://github.com/vossstef/VIC20Nano) HDL implementation of the Commodore VIC20 home computer
  - [A2600 Nano](https://github.com/vossstef/A2600Nano) HDL implementation of the Atari 2600 game console
  - [NanoApple2](https://github.com/vossstef/NanoApple2) HDL implementation of the Apple IIe home computer
