# BL616 FPGA Partner firmware

The BL616 FPGA Partner firmware is a replacement firmware for the
factory installed firmware of the BL616 MCU on the Tang
FPGA boards. The FPGA Partner adds the ability to later install and
run a secondary firmware on the BL616 in parallel. This feature can be
used to install the FPGA Companion on the on-board BL616 MCU.

> [!IMPORTANT]
> You don't need this if you have connected a separate MCU like the
> [Raspberry Pi Pico](../../rp2040) or a [M0S Dock](..) to run the FPGA Companion. The
> FPGA Partner is only needed if you want to use the on-board
> BL616 of a Tang device to run the FPGA Companion.

The FPGA Partner exists for many of the Tang devices including
the Tang Nano 20k, the Tang Primer 25k Dock, the Tang Neo Dock
or the Tang Mega 138k Dock.

## How to flash

You can flash these with the command line tool. To get the
BL616 into firmware update mode, power the Tang up with the
```UPDATE``` button pressed (see [here](https://github.com/MiSTle-Dev/.github/wiki/Firmware-Installation-BL616-%C2%B5C) for more details). The BL616 will then
indentify itself as ```Bouffalo CDC DEMO``` and a virtual COM port
will be created.

Use the following commands to flash using the command line tool to e.g. flash
the Tang Nano 20k:

```
$ BLFlashCommand --interface=uart --baudrate=2000000 --port=/dev/ttyACM0 --chipname=bl616 --cpu_id= --config=bl616_fpga_partner_20kNano_cfg.ini
```

Afterward power cycle the device, and it should show up as FT2232 device named ```SIPEED USB Debugger```.
The device should still work normal and be able to update and flash the FPGA.

> [!IMPORTANT]
> Some early versions of the Tang Nano 20k don't have encryption enabled.
> This is needed for the FPGA Partner to work. If you flash the FPGA Partner
> to one of these early Tang Nano 20k's it will not work. Instead, it will
> stay in ```Bouffalo CDC DEMO``` update mode. In this case you cannot
> use the FPGA Partner. You need to follow [these instructions](../friend_20k)
> to restore your device to factory state.

## Installing the FPGA Companion as secondary firmware

Once the FPGA Partner is installed and the device still works as expected, then
the FPGA Companion can be installed additionally. The necessary files are usuaally part
of the [FPGA Companion Releases](https://github.com/MiSTle-Dev/FPGA-Companion/releases).

## More information

- [Information about the Tang Nano 20k variants](https://github.com/MiSTle-Dev/.github/wiki/Versions_TangNano20k)
- [More details on how to flash the BL616](https://github.com/MiSTle-Dev/.github/wiki/Firmware-Installation-BL616-%C2%B5C). 
- [Original 20k FRIEND firmware](../friend_20k)
