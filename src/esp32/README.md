# MiSTeryNano FPGA companion ESP32 variant

This is the variant of the
[MiSTeryNano](https://github.com/MiSTle-Dev/MiSTeryNano) FPGA companion
firmware for the ESP32-S2 and ESP32-S3. 

## USB Support

You need an ESP32 variant with native USB support, preferrably through
a second USB connector. Such a board has two USB-C connectors, one
usually being labeled COM and servicing a USB UART (COM port) as
typical for ESP32 boards and the second one giving access to the
ESP32's native USB port which can be operated in USB host mode to
connect a device like a keyboard or a mouse.

Currently the ESP32-S2 and ESP32-S3 are supported. Still, USB support
on these devices is rather limited and only one directly connected USB
device is supported. Hubs are not supported and thus the use of a
(wireless) keyboard/mouse combo device like the [Rii
R8](http://www.riitek.eu/DE/Produkte/RT-MWK08RF_DE.html) makes the
most sense for a retro home computer setup. This does not leave a port
for a USB joystick which will instead have to be e.g. a retro joystick
connected directly to the FPGA.

### Hardware modifications

Most ESP32 boards that come with two USB ports are designed to
be powered from one of three power sources: The COM USB, the
native USB or an external 5V power source, but no to provide power
_to_ any of these.

For this project it's preferred if the whole setup can be pwoered from
the COM USB port providing power to other USB devices connected to the
native USB. Furthermore the FPGA may also be powered from this source.
Some boards have solder jumpers e.g. labeled ```USB OTG``` and
```IN-OUT``` which can provide this. On other boards it may be
necessary to remove/bridge diodes.

## Features and limitations

  - Requires an ESP32 variante with USB host support (ESP32-S2 or S3)
  - Only one USB device possible (no hubs)
  - Can provide wireless network connectivity (not yet implemented)
  - Can provide support for Bluetooth peripherials (not yet implemented)

## Building

Get the esp-idf as explained [here](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html) and install the toolchain:

```
git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s2,esp32s3
. ./export.sh
````

You'll then need to make a few adjustments:

First specify which CPU you are using. Either

```
idf.py set-target esp32s2 
```

or

```
idf.py set-target esp32s3 
```

Then in ```esp-idf/components/fatfs/src/ffconf.h``` you need to
manually set
```
#define FF_FS_EXFAT    1
```

Then enter the menu by

```
idf.py menuconfig
```

and change a few settings:

- Navigate into ```Component Config``` and then into ```FAT Filesystem Support```.
  - Activate the ```Enable fast seek algorithm ...``` option
  - Set the ```Sector Size``` to 512
  - Set ```Long filename support``` to ```Long filename buffer in heap```
  - Set ```Enable String functions``` to ```1: Enable without LR-CRLF conversion```
  - Enable ```Use FATFS volume label```
- Navigate to ```Component Config``` and then to ```Wear Leveling```
  - Set ```Wear Levelling library sector size``` to 512
- Navigate to ```Component Config``` and then to ```FreeRTOS``` and ```Kernel```
  - Set ```configTICK_RATE_HZ``` to 1000

Finally start the build with 

```
idf.py build
```

If successful the resulting binary can be flashed:

```
idf.py -p /dev/ttyUSB0 flash
```

Additional debug output is sent via UART at 115200 bit/s

# Pin usage

| Pin | Signal | Description |
|---|---|---|
| TX | UART_TX  | Serial debug output |
| GPIO20  | USB D+ | USB host D+ |
| GPIO19  | USB D- | USB host D-   |
| GPIO14 | IRQn | SPI interrupt from FPGA |
| GPIO13 | MISO | SPI data from FPGA |
| GPIO12 | SCK | SPI clock to FPGA |
| GPIO11 | MOSI | SPI data to FPGA |
| GPIO10 | CSn | SPI chip select to FPGA |

# Example wiring

![Tang Nano 20k with ESP32s2](esp32s2_tn20k.png)
