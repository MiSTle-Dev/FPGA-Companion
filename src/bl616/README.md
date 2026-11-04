# MiSTle FPGA companion BL616 variant

This is the variant of the FPGA companion firmware for the BL616 MCU.

## Example wiring M0S Dock (BL616 MPU)

M0S Dock consists of a M0S PCBA subassembly that is mounted on a carrier PCBA as a M0S Dock.
![Tang Nano 20k with M0S Dock](m0s_dock_tn20k.png)

|M0S pin<BR>/ Dock|Signal      |BL616 GPIO|Note        |
|------|------------- |--------|--------------|
|20    |JTAG TMS[^1]  |GPIO  0 |              |
|21    |JTAG TCK[^1]  |GPIO  1 |              |
|22    |              |GPIO  2 |Bootstrap |
|23    |JTAG TDI[^1]  |GPIO  3 |              |
|IO 12 |CSN           |GPIO 12 |              |
|IO 13 |SCK           |GPIO 13 |              |
|IO 10 |MISO          |GPIO 10 |              |
|IO 11 |MOSI          |GPIO 11 |              |
|IO 14 |IRQN          |GPIO 14 |              |
|14    |JTAG TDO[^1]  |GPIO 20 |              |
|RX    |BL616 UART RX |GPIO 21 |debug console |
|TX    |BL616 UART TX |GPIO 22 |debug console |
|GND   |GND           |        |              |
|+5V   |+5V   R       |        |              |

[^1]: JTAG FPGA loader

Location JTAG connection TN20k PCBA top side.  

![Tang Nano 20k JTAG](../rp2040/jtag_tn20k.png)

Location GPIO 0,1,3,20 for JTAG purposes on M0S sub-module PCBA. M0S Dock top side. 

![M0S](M0S.png)

## Tang integrated onboard BL616 MPU

JTAG signals, UART RX and TX are re-purposed as SPI and control interface. MPU is Master and FPGA is slave. UART RX GPIO is board specific and used as an SPI interrupt input. The UART TX GPIO is a board-specific signal used to control the FPGA’s dedicated JTAG hw pins and determine whether the interface operates in native JTAG or SPI mode. Nano 20k always stays in JTAG active enabled mode as both JTAG and SPI are as dedicated hw pins available.  

**Tang Nano 20k**

| BL616 GPIO | Nano 20k 3921 | Nano 20k 3923 | re-use   | Note          |
|------------|---------------|---------------|----------|---------------|
| GPIO 0     | SPI _SS       | SPI _SS       | -        |               |
| GPIO 1     | SPI SCK       | SPI SCK       | -        |               |
| GPIO 2     | SPI MISO      | unused        | -        | Bootstrap     |
| GPIO 3     | SPI MOSI      | unused        | -        |               |
| GPIO 10    | JTAG TCK      | JTAG TCK      | -        |               |
| GPIO 11    | BL616 UART TX | BL616 UART TX | -        | debug console |
| GPIO 12    | JTAG TDI      | JTAG TDI      | -        |               |
| GPIO 13    | BL616 UART RX | BL616 UART RX | SPI _IRQ |               |
| GPIO 14    | JTAG TDO      | JTAG TDO      | -        |               |
| GPIO 16    | JTAG TMS      | JTAG TMS      | -        |               |
| GPIO 20    | unused        | unused        | UART RX  | debug console |
| GPIO 27    | unused        | SPI MOSI      | -        |               |
| GPIO 30    | unused        | SPI MISO      | -        |               |

Nano20k uses default BL616 UART TX for the debug console.  

**Primer / Console / Mega**

| BL616 GPIO | Tang Board wiring BL616 | re-use     | Note          |
|------------|-------------------------|------------|---------------|
| GPIO 0     | JTAG TMS                | SPI _SS    |               |
| GPIO 1     | JTAG TCK                | SPI SCK    |               |
| GPIO 2     | JTAG TDO                | SPI MISO   | Bootstrap     |
| GPIO 3     | JTAG TDI                | SPI MOSI   |               |
| GPIO x     | BL616 UART RX           | SPI _IRQ   |               |
| GPIO x     | BL616 UART TX           | V_JTAGSELN | 1=JTAG, 0=SPI |
| GPIO x     | BL616 TWI SCL[^1]       | UART TX    | debug console |

[^1]: The extra TWI SCL connection is only available for Console60k/138k and Mega138k Pro.  

Primer25K BL616 debug console can be used after a required hardware modification: remove capacitor C22 to make GPIO12 available at button S3 for debug TX console access. Mega60k can likely re-use GPIO16 or GPIO17 for debug output. 

[All in One Build](#all-in-one-build)

## Compiling and uploading code for the BL616 (Linux)

Download the Bouffalo toolchain:

https://github.com/bouffalolab/toolchain/releases/tag/default

Download `xuantie-gcc-riscv-v2.6.1-linux-x86_64.tar.xz` into your home directory.

Extract it into your home directory. The archive contains the
`xuantie-gcc-riscv-v2.6.1-linux-x86_64` folder:
```bash
tar -xf "$HOME/xuantie-gcc-riscv-v2.6.1-linux-x86_64.tar.xz" -C "$HOME"
```
Add the compiler to your search path:
```bash
export PATH="$HOME/xuantie-gcc-riscv-v2.6.1-linux-x86_64/bin:$PATH"
```
and the Bouffalo SDK:
```bash
git clone --recurse-submodules https://github.com/bouffalolab/bouffalo_sdk.git "$HOME/bouffalo_sdk"
```
```bash
export BL_SDK_BASE="$HOME/bouffalo_sdk"
```
Clone the FPGA-Companion repository:
```bash
cd ~/Documents
git clone --recurse-submodules https://github.com/MiSTle-Dev/FPGA-Companion.git
cd FPGA-Companion/src/bl616
```

To build all boards listed in `buildall.sh`, run:
```bash
./buildall.sh
```

To build only **M0S Dock**, use the existing clone:

```bash
cd "$HOME/Documents/FPGA-Companion/src/bl616"
make clean
CROSS_COMPILE="$HOME/xuantie-gcc-riscv-v2.6.1-linux-x86_64/bin/riscv64-unknown-elf-" BL_SDK_BASE="$HOME/bouffalo_sdk" make TANG_BOARD=m0sdock
```

for a specific board select in between: m0sdock nano20k nano20k_v3923 console60k mega60k mega138kpro primer25k console138k

```shell
cd "$HOME/Documents/FPGA-Companion/src/bl616"
make clean
BL_SDK_BASE="$HOME/bouffalo_sdk" make TANG_BOARD=nano20k_v3923
```

For future terminals, add the following exports to `~/.bashrc`:

```bash
export BL_SDK_BASE="$HOME/bouffalo_sdk"
export PATH="$HOME/xuantie-gcc-riscv-v2.6.1-linux-x86_64/bin:$PATH"
```

Reload the settings with `source ~/.bashrc`.

A simple make or make CHIP=bl616 COMX=/dev/ttyACMxyz flash in your bl616 folder will do then.

for onboard BL616 see: [All in One Build](#all-in-one-build)

### Flashing the firmware

The resulting binary can be flashed onto the M0S. If you don't have
further debugger or prorgammer hardware connected to the M0S Dock, then
you need to perform the following manual steps:

First, you need to unplug the M0S from USB, press the BOOT button and plug the M0S Dock back into the PCs USB with the
BOOT button still pressed. Once connected release the BOOT button. The device
should now be in bootloader mode and show up with its bootloader on the PC:

```bash
$ lsusb
...
Bus 002 Device 009: ID 349b:6160 Bouffalo Bouffalo CDC DEMO
...
```

Also an ACM port should have been created for this device as e.g.
reported in the kernel logs visible with ```dmesg```:

```text
usb 2-1.7.3.3: new high-speed USB device number 9 using ehci-pci
usb 2-1.7.3.3: config 1 interface 0 altsetting 0 endpoint 0x83 has an invalid bInterval 0, changing to 7
usb 2-1.7.3.3: New USB device found, idVendor=349b, idProduct=6160, bcdDevice= 2.00
usb 2-1.7.3.3: New USB device strings: Mfr=1, Product=2, SerialNumber=0
usb 2-1.7.3.3: Product: Bouffalo CDC DEMO
usb 2-1.7.3.3: Manufacturer: Bouffalo
cdc_acm 2-1.7.3.3:1.0: ttyACM3: USB ACM device
```

Once it shows up that way it can be flashed.  
If you have built the firmware yourself and have the SDK installed you can simply enter the following command:

```bash
cd "$HOME/Documents/FPGA-Companion/src/bl616"
BL_SDK_BASE="$HOME/bouffalo_sdk" make CHIP=bl616 COMX=/dev/ttyACM3 flash
```

If you have downloaded the firmware from the [release page](https://github.com/MiSTle-Dev/FPGA-Companion/releases) you can use the graphical [BLFlashCube](https://github.com/CherryUSB/bouffalo_sdk/tree/master/tools/bflb_tools/bouffalo_flash_cube) tool using the ```fpga_companion_bl616_cfg.ini``` file.

or alternatively for the .ini file including the companion fw and the fpga-companion:

```bash
cd "$HOME/Documents/FPGA-Companion/src/bl616"
"$HOME/bouffalo_sdk/tools/bflb_tools/bouffalo_flash_cube/BLFlashCommand-ubuntu" --interface=uart --baudrate=2000000 --port=/dev/ttyACM0 --chipname=bl616 --cpu_id= --config buildall/flash_nano20k.ini
```

After successful download you need to unplug the device again and reinsert it *without* the BOOT button pressed to boot into the newly installed firmware.

## Compiling and uploading code for the BL616 (Windows 11)

Download the Bouffalo toolchain:

https://github.com/bouffalolab/toolchain/releases/tag/default

Download xuantie-gcc-riscv-v2.6.1-windows-x86_64.7z into your home directory.

Install [Git for Windows](https://gitforwindows.org)

Use **Command Prompt (`cmd.exe`)**, not PowerShell, for the Windows commands below.
Open it using Win + R, then enter `cmd`.


clone the Bouffalo SDK:  

```bat
cd /d "%USERPROFILE%\Documents"
git clone --recurse-submodules https://github.com/bouffalolab/bouffalo_sdk.git
```

**Looks that recent SDK build still does not build/link even when enabling CONFIG_GCC_COMPILE_LTO, use Linux instead**

Set Windows SDK Environment Variable:  

```text
Open Start Search, type “env”, and select “Edit the system environment variables”.  
  
BL_SDK_BASE=%USERPROFILE%\bouffalo_sdk
```

Set Windows search PATH for Toolchain:  

```text
%USERPROFILE%\xuantie-gcc-riscv-v2.6.1-windows-x86_64\bin
%USERPROFILE%\bouffalo_sdk\tools\make
%USERPROFILE%\bouffalo_sdk\tools\cmake
%USERPROFILE%\bouffalo_sdk\tools\ninja
%USERPROFILE%\bouffalo_sdk\tools\bflb_tools\bouffalo_flash_cube
```

Close shell

```shell
exit
```

Open Start Search, type “cmd” or Win + R and type “cmd”  
check individually proper start of each single tool

```shell
make -v
cmake --version
ninja --help
riscv64-unknown-elf-gcc -v
```

Clone the FPGA-Companion repository into `%USERPROFILE%\Documents`:

```bat
cd /d "%USERPROFILE%\Documents"
git clone --recurse-submodules https://github.com/MiSTle-Dev/FPGA-Companion.git
cd FPGA-Companion\src\bl616
```

Compile the firmware for **M0S Dock**  

```bat
cd /d "%USERPROFILE%\Documents\FPGA-Companion\src\bl616"
make clean
make TANG_BOARD=m0sdock
```

For a specific board, select one of: m0sdock nano20k nano20k_v3923 console60k console138k mega60k mega138kpro primer25k.

```bat
cd /d "%USERPROFILE%\Documents\FPGA-Companion\src\bl616"
make clean
make TANG_BOARD=nano20k_v3923
```

## All in One Build

A build script creates for the several setups specific binaries and .ini files including the needed ``bl616_fpga_partner_`` firmware. The ``buildall`` folder will contain all needed files for a release. So far Nano20k, Console60k, Console138k, Primer25k, Mega138k Pro, Mega NEO Dock 60k apart from M0S Dock are supported.  
Primer20k and TN9k are excluded as their BL702 doesn't support required USB host mode.

On Windows (Command Prompt):
```bat
cd /d "%USERPROFILE%\Documents\FPGA-Companion\src\bl616"
buildall.bat
```

On Linux:
```bash
cd "$HOME/Documents/FPGA-Companion/src/bl616"
./buildall.sh
```

### Flashing the firmware M0S Dock

First, you need to unplug the M0S_DOCK from USB, press the BOOT button and plug the M0S Dock back into the PCs USB with the
BOOT button still pressed. Once connected release the BOOT button. The device
should now be in bootloader mode and show up with its bootloader on the PC.

Figure out MPU bootloader COM port and use shell command to program:  
Press Windows + R keyboard shortcut to launch the Windows Run box, type “devmgmt.msc” , and click the OK button.  

Device Manager will open.  
Locate Ports (COM & LPT) in the list.  
Check for the COM ports by expanding the same.  

Replace `COM3` with the port shown in Device Manager:
```bat
cd /d "%USERPROFILE%\Documents\FPGA-Companion\src\bl616"
make CHIP=bl616 COMX=COM3 flash
```

If you have downloaded the firmware from the [release page](https://github.com/MiSTle-Dev/FPGA-Companion/releases) you can use the graphical [BLFlashCube](https://github.com/CherryUSB/bouffalo_sdk/tree/master/tools/bflb_tools/bouffalo_flash_cube) tool using the ```fpga_companion_bl616_cfg.ini``` file.

or alternatively a shell based tool:

For example, to flash the Nano20k image generated by the build script:
```bat
cd /d "%USERPROFILE%\Documents\FPGA-Companion\src\bl616"
"%USERPROFILE%\bouffalo_sdk\tools\bflb_tools\bouffalo_flash_cube\BLFlashCommand.exe" --interface=uart --baudrate=2000000 --port=COM3 --chipname=bl616 --cpu_id= --config buildall\flash_nano20k.ini
```

After successful download you need to unplug the device again and reinsert it *without* the BOOT button pressed to boot into the newly installed firmware.
