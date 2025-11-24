# MiSTeryNano FPGA companion BL616 variant

The is the variant of the MiSTeryNano FPGA companion firmware
for the BL616 MCU (M0S Dock).

> [!IMPORTANT]
> * Updated SDK have to be installed!  

## Example wiring M0S Dock

![Tang Nano 20k with M0S Dock](m0s_dock_tn20k.png)  

## Tang integrated onboard BL616 MPU

JTAG signals and UART RX are re-purposed as SPI interface. MPU is Master and FPGA is slave.  
BL616 GPIO pin mapping is fix in SDK and no option to change. UART RX GPIO is board specific.

|Tang Board wiring BL616|BL616 GPIO|SPI re-use|Note|
|-------------|-----|--------|-----|
|JTAG TMS     |GPIO0|SPI _SS |  |
|JTAG TCK     |GPIO1|SPI SCK |  |
|JTAG TDO     |GPIO2|SPI MISO| EN_CHIP BL616 |
|JTAG TDI     |GPIO3|SPI MOSI|  |
|BL616 UART RX|GPIO x|_IRQ    |  |

[Windows 11 Build AiO](#tang-onboard-bl616)

## Compiling and uploading code for the BL616 (Linux)

Download the Bouffalo toolchain:

```bash
git clone https://github.com/bouffalolab/toolchain_gcc_t-head_linux.git
```

and the patched Bouffalo SDK with latest version common CherryUSB Stack:  
```bash
git clone --recurse-submodules https://github.com/MiSTle-Dev/bouffalo_sdk.git
```

Compile the firmware for **M0S Dock**

```bash
git clone --recurse-submodules https://github.com/MiSTle-Dev/FPGA-Companion.git
cd FPGA-Companion
git submodule init
git submodule update
CROSS_COMPILE=<where you downloaded the toolchain>/toolchain_gcc_t-head_linux/bin/riscv64-unknown-elf- BL_SDK_BASE=<where you downloaded the sdk>/bouffalo_sdk/ make
```

You can simplify the ```make``` a bit by setting in your bashrc BL_SDK_BASE and include the toolchain_gcc_t-head_linux in the search path.

```bash
nano ./bashrc
export BL_SDK_BASE=xyz 
PATH=$PATH:/abc/toolchain_gcc_t-head_linux/bin
```

A simple make or make CHIP=bl616 COMX=/dev/ttyACMxyz flash in your bl616 folder will do then.

for onboard BL616 see: [Windows 11 Build AiO](#tang-onboard-bl616)

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
BL_SDK_BASE=<where you downloaded the sdk>/bouffalo_sdk/ make CHIP=bl616 COMX=/dev/ttyACM3 flash
```

If you have downloaded the firmware from the [release page](https://github.com/MiSTle-Dev/FPGA-Companion/releases) you can use the graphical [BLFlashCube](https://github.com/CherryUSB/bouffalo_sdk/tree/master/tools/bflb_tools/bouffalo_flash_cube) tool using the ```fpga_companion_bl616_cfg.ini``` file.

After successful download you need to unplug the device again and reinsert it *without* the BOOT button pressed to boot into the newly installed firmware.

## Compiling and uploading code for the BL616 (Windows 11)

Install [Git for Windows](https://gitforwindows.org)

Install [cmake for Windows](https://cmake.org/download)

Install Bouffalo RISC-V MCU toolchain

```shell
Open Start Search, type “cmd” or Win + R and type “cmd” 

cd %HOMEPATH%
git clone https://github.com/bouffalolab/toolchain_gcc_t-head_windows.git
```

and the patched Bouffalo SDK with latest version common CherryUSB Stack:  

```shell
cd %HOMEPATH%
git clone --recurse-submodules https://github.com/MiSTle-Dev/bouffalo_sdk.git
```

Set Windows SDK Environment Variable:  

```text
Open Start Search, type “env”, and select “Edit the system environment variables”.  
  
BL_SDK_BASE=C:\Users\xyzuser\bouffalo_sdk
```

Set Windows search PATH for Toolchain:  

```shell
C:\Users\xyzuser\toolchain_gcc_t-head_windows\bin
C:\Users\xyzuser\bouffalo_sdk\tools\make
C:\Users\xyzuser\bouffalo_sdk\tools\ninja
```

Close shell

```shell
exit
```

Open Start Search, type “cmd” or Win + R and type “cmd”  
check individually proper start of each single tool

```shell
make -v
cmake -version
ninja --help
riscv64-unknown-elf-gcc -v
```

Download FPGA companion repository

```shell
cd %HOMEPATH%/Documents
git clone --recurse-submodules https://github.com/MiSTle-Dev/FPGA-Companion.git
cd FPGA-Companion
git submodule init
git submodule update
```

Compile the firmware for **M0S Dock**  

```shell
cd %HOMEPATH%/Documents\FPGA-Companion\src\bl616
make clean
make

or for a specific board
make TANG_BOARD=console60k
( m0sdock nano20k console60k mega60k mega138kpro primer25k )
```

## tang onboard bl616

A build script creates for the several setups specific binaries and .ini files including the needed ``bl616_fpga_partner_`` firmware. The ``buildall`` folder will contain all needed files for a release. So far Nano20k, Console60k, Console138k, Primer25k, Mega138k Pro, Mega NEO Dock 60k apart from M0S Dock are supported. Primer20k and TN9k are excluded as their BL702 doesn't support required USB host mode.

```shell
buildall.bat
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

```shell
make CHIP=bl616 COMX=COMabc  flash
```

If you have downloaded the firmware from the [release page](https://github.com/MiSTle-Dev/FPGA-Companion/releases) you can use the graphical [BLFlashCube](https://github.com/CherryUSB/bouffalo_sdk/tree/master/tools/bflb_tools/bouffalo_flash_cube) tool using the ```fpga_companion_bl616_cfg.ini``` file.

or alternatively the shell based tool:

```shell
cd %HOMEPATH%/Documents\FPGA-Companion\src\bl616
BLFlashCommand.exe --port COM_tbd --config %HOMEPATH%/Documents\FPGA-Companion\src\bl616\buildall\flash_nano20k.ini
```

After successful download you need to unplug the device again and reinsert it *without* the BOOT button pressed to boot into the newly installed firmware.
