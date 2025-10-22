# MiSTeryNano FPGA companion Pi Pico / RP2040 variant

The is the variant of the MiSTeryNano FPGA companion firmware
for the Raspberry Pi Pico (W).

## Building

### Install the Pi Pico Toolchain (Ubuntu)

```bash
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

### Install the Pi Pico Toolchain (Fedora)

```bash
sudo dnf group install "development-tools"
sudo dnf install clang cmake gcc-arm-linux-gnu arm-none-eabi-gcc-cs-c++ arm-none-eabi-gcc-cs arm-none-eabi-binutils arm-none-eabi-newlib
```

### Install the Pi Pico Toolchain (Windows11)

see > ```Install the Pi Pico Toolchain for VisualStudioCode``` as below

### Download and install the Pi Pico SDK

Download the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
and set the ```PICO_SDK_PATH``` environment variable to point to the
SDKs root directory.

```bash
git clone https://github.com/raspberrypi/pico-sdk --recursive
export PICO_SDK_PATH=<full-path-to-clones-sdk>
```

### PIO-USB component for TinyUSB

> [!IMPORTANT]
> Make sure that the PIO-USB component is installed

```bash
cd pico-sdk/lib/tinyusb
python3 tools/get_deps.py rp2040
```

### Optional: Update TinyUSB and PIO-USB to the latest version

The TinyUSB stack and the PIO-USB mostly work for the FPGA Companion.
But they do have some problems with plugging and unplugging devices.
Later versions have some fixes and updates for this. Therefore, you
may want to update to the latest version.

```bash
cd pico-sdk/lib/tinyusb
git checkout master
git pull
cd hw/mcu/raspberry_pi/Pico-PIO-USB
git fetch origin
git checkout main 
git pull
```

Additionally patch [```src/class/hid/hid_host.c```](https://github.com/hathach/tinyusb/blob/79445c2386adefb207a76a70d87578b53c3e7922/src/class/hid/hid_host.c#L487) to:

```
    tuh_hid_report_received_cb(daddr, idx, (XFER_RESULT_SUCCESS == result)?epbuf->epin:NULL, (uint16_t) xferred_bytes);
```

For more details on this see [TinyUSB issue #3296](https://github.com/hathach/tinyusb/issues/3296) and [issue #3307](https://github.com/hathach/tinyusb/issues/3307).

### Clone this respository

This repository has to be cloned recursively to make sure the submodules
are included.

```bash
git clone https://github.com/MiSTle-Dev/FPGA-Companion.git  --recursive
cd FPGA-Companion
```

Alternally do a

```bash
git submodule update --init
```

after a non-recursive clone to update the submodules.

### Run cmake and make

To start the build process create a build directory and start the
compilation:

```bash
cd src/rp2040
mkdir build
cd build
cmake ..
make
```

The resulting file named ```fpga_companion.uf2``` is loaded onto the
Pico as usual. Once successfully booted the Pico's LED will blink.

Additional debug output is sent via UART at 921600 bit/s on GP0 on
a regular Pi Pico or Pico(W) and at 460800 bit/s on Waveshare RP2040-Zero.

### Install the Pi Pico Toolchain for VisualStudioCode (Windows11 / Linux)

Install [Git for Windows](https://gitforwindows.org)

Install [VSCode](https://code.visualstudio.com) [Raspberry Pi Pico](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico) plugin.

> [!IMPORTANT]
> Make sure that the PIO-USB component is installed

**Windows:**

Open Start Search, type “cmd” or Win + R and type “cmd”

```shell
cd %HOMEPATH%/.pico-sdk\sdk\2.1.1\lib\tinyusb
python tools/get_deps.py rp2040
```

**Linux:**

```bash
cd ~/.pico-sdk/sdk/2.1.1/lib/tinyusb
python tools/get_deps.py rp2040
```

```Import Project``` by selecting folder location ```rp2040```, choose 'default' settings and Press ```Compile Project```

The resulting file named ```fpga_companion.uf2``` is loaded onto the
Pico as usual. Once successfully booted the Pico's LED will blink.

For a *Waveshare RP2040-Zero* build select ```Switch Board``` and choose waveshare_rp2040_zero

## Pin usage

On the regular Pico, the FPGA Companion uses the following pins:

| Pin | Signal | Description |
|---|---|---|
| GP0  | UART_TX  | Serial debug output |
| GP2  | USB D+ | USB PIO host D+ |
| GP3  | USB D- | USB PIO host D- |
| GP4  | LED 1 | indicator mouse |
| GP5  | LED 2 | indicator keyboard |
| GP6  | LED 3 | indicator joystick |
| GP16 | MISO | SPI data from FPGA |
| GP17 | CSn | SPI chip select to FPGA |
| GP18 | SCK | SPI clock to FPGA |
| GP19 | MOSI | SPI data to FPGA |
| GP22 | IRQn | SPI interrupt from FPGA |

## Using the internal micro USB connector

The RP2040 can use the pins GP2 and GP3 for a USB host port as
depicted below. This is the default configuration for the
FPGA Companion, although it's possible to use the standard
on-board micro USB connector instead.

To do so in the file ```tusb_config.h``` the PIO USB
needs to be disabled by setting the following value
to 0:

```c
// change to 0 if using on-board native micro USB
// change to 1 if using pico-pio-usb as host controller for raspberry rp2040
#define CFG_TUH_RPI_PIO_USB   1
```

To use this you need a micro-USB to USB-A-OTG adapter.

## Example wiring

![Tang Nano 20k with Raspberry Pi Pico](pico_tn20k.png)

The FPGA Companion for the Raspberry Pi Pico can also be used
on the [MiSTeryShield20k RPiPico](https://github.com/vossstef/tang_nano_20k_c64/tree/main/board/misteryshield20k_rpipico).

## Using the MiSTeryShield20k Lite

The [MiSTeryShield20k Lite](https://github.com/MiSTle-Dev/MiSTeryNano/tree/main/board/misteryshield20k_lite) does not
include WiFi and does not allow to detect its absence automatically. Some versions also need a custom oscillator
setting. Thus a FPGA Companion version specifially for the MiSTeryShield20k Lite can be enabled as follows.

```bash
cmake -DSH20KLITE=ON ..
```

## Using the Raspberry Pi Pico2 or Pico2(W)

The FPGA Companion also runs on the Pico2/RP2350. This has
not extensively been tested, yet.

The RP2350 needs a special version of the FreeRTOS. Clone the official
Pico port of FreeRTOS into the rp2040 directory:

```bash
cd src/rp2040
git clone https://github.com/raspberrypi/FreeRTOS-Kernel
```

Then go into the ```src/rp2040/build``` directory and select to build for the
PICO2 and Pico2(W):

```bash
cmake -DPICO2=ON ..
```

## Using the Waveshare RP2040-Zero

The Raspberry Pi Pico is rather big and only comes with a Micro USB
port. The [Waveshare RP2040-Zero](https://www.waveshare.com/rp2040-zero.htm) is a little
smaller and comes with a USB-C connector which makes it easier to use
it as a USB host using a regular USB-C to USB-A host adapter.

To build the firmware for the RP2040-Zero use the following
command:

```bash
cmake -DWS2040_ZERO=ON ..
```

The build process will then end with the following message:

```text
Firmware has been built for Waveshare RP2040-Zero.
```

The SPI pins used on the RP2040-Zero differ from the ones used on the
regular pico:

| Pin | Signal | Description |
|---|---|---|
| GP0  | UART_TX  | Serial debug output |
| GP4 | MISO | SPI data from FPGA |
| GP5 | CSn | SPI chip select to FPGA |
| GP6 | SCK | SPI clock to FPGA |
| GP7 | MOSI | SPI data to FPGA |
| GP8 | IRQn | SPI interrupt from FPGA |

Also the RP2040-Zero comes with a WS2812 RGB led instead of a regular
LED. Driving the RGB LED requies a PIO unit and thus cannot be used at
the same time as the PIO-USB. The native USB of the RP2040-Zero must
therefore always be used, anyway.

## Convenient development

The RP2040 is by default somewhat inconvenient to develop for
as it has to be mounted manually as mass-storage to copy the
uf2 file onto it.

It's thus recommended to use a second Pi-Pico as a SWD programming
adapter. The details are explained in appendix A of
[Getting started with Raspberry Pi Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) under section ```Debug with a second Pico```.

The ```CMakeLists.txt``` file already contains a matching target and
with openocd installed a simple ```make flash``` should upload the
new firmware via the second Pico.
