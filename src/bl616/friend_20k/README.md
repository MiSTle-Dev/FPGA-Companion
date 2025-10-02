# Friend 20K firmware

This is the original firmware that came pre-installed with the BL616
on the Tang Nano 20K. These files should allow you to return the board
to factory state in case the firmware has been overwritten.

There are two variants, the regular one and an encrypted one. Both
provide a similar functionality. Sipeed decided to encrypt the
firmware on boards sold about the beginning of 2024. These newer
boards will not accept or run any non-encrypted firmware and replacing
its default firmware will essentially brick the Tang Nano 20K. This
encrypted firmware will allow restoring the newer boards to factory
state as well and thus unbrick the device.

It's currently unknown if the firmware versions can be identified
beforehand. So you might have to try both of them and to check which
one's the one working for your board.

## How to flash

> [!IMPORTANT]
> If your Tang Nano 20k shows up on USB as the FTDI FT2232 compatible
> ```20k friend``` there is no need to flash it again. This is only
> required if you replaced the firmware and now want to go back to
> the factory state.

You can flash either of these with the command line tool. To get the
BL616 into firmware update mode, power the Tang Nano 20k up with the
```UPDATE``` (next to the HDMI connector) pressed. The BL616 will then
identify itself as ```Bouffalo CDC DEMO``` and a virtual COM port
will be created. Use the following commands to flash using the
command line tool:

```
$ BLFlashCommand --interface=uart --baudrate=2000000 --port=/dev/ttyACM0 --chipname=bl616 --cpu_id= --config=friend_20k_cfg.ini
```

or

```
$ BLFlashCommand --interface=uart --baudrate=2000000 --port=/dev/ttyACM0 --chipname=bl616 --cpu_id= --config=friend_20k_encrypted_cfg.ini
```

Afterward power cycle the device, and it should show up as the ```20K's FRIEND``` friend
again and work as normal. If it still shows up as the ```Bouffalo CDC DEMO``` then you
most probably flashed the wrong version (encrypted onto an unencrypted device or
vice versa). In that case you can simply flash the correct version.

## More information

- [Information about the Tang Nano 20k variants](https://github.com/MiSTle-Dev/.github/wiki/Versions_TangNano20k)
- [More details on how to flash the BL616](https://github.com/MiSTle-Dev/.github/wiki/Firmware-Installation-BL616-%C2%B5C). 
- [BL616 FPGA PARTNER firmware](../bl616_fpga_partner)
