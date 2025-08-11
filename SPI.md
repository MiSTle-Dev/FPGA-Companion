# FPGA Companion SPI protocol

In a FPGA Companion based setup the workload is distributed between
the FPGA and a seperate MCU. The FPGA implements the hardware of the
original device (e.g. the Atari ST) and the MCU mainly deals with
modern peripherals like USB keyboards, mice and SD cards with their
file systems (FAT, exFAT).

This setup requires that the FPGA and the MCU communicate permanently
so the MCU can instruct the FPGA to display the on-screen-display
(OSD), to send keyboard and mouse events into the core and to receive
floppy disk read requests from the FPGA. And since the SD card is
connected to the FPGA as well, the MCU needs to request SD card data
from the FPGA. This is all done via a SPI bus.

This part of MiSTeryNano is a work and progress and the information
below may be outdated. The basic principles should still be valid.

## Internal vs. external MCU

Theoretically it's possible to use the Tang Nano 20k's internal BL616
MCU or an externally connected MCU like a Raspberry Pi Pico or a M0S
Dock. Using the internal MCU would have the advantage that no
additional component is required. The major disadvantage would be that
this MCU is usually being used to flash the FPGA from a
PC. Repurposing the internal MCU makes flashing the FPGA more
difficult and thus hinders FPGA development. Using an external MCU
like e.g. a M0S Dock instead leaves the internal MCU and it's ability
to flash the FPGA untouched. Development therefore usually takes place
with an external MCU.

## SPI bus

The SPI bus between the MCU and the FPGA consists of four connections
e.g. for a M0S Dock/BL616. For other MCUs see the READMEs in their
respective subdirectories.

|      | M0S Dock <-> FPGA | int. MCU <-> FPGA |                                      |
|------|-------------------|-------------------|--------------------------------------|
| CSN  | GPIO12 -> 56      | GPIO0 -> 86       | SPI select, active low               |
| SCK  | GPIO13 -> 54      | GPIO1 -> 13       | SPI clock, idle low                  |
| MOSI | GPIO11 -> 41      | GPIO3 -> 76       | SPI data from MCU to FPGA            |
| MISO | GPIO10 <- 42      | GPIO10 <- 6       | SPI data from FPGA to MCU    |
| IRQ  | GPIO14 <- 51      | GPIO12 <- 7       | Interrupt from FPGA to MCU, active low |

Usually the SPI bus is run at 20Mhz.

The internal MCU of the Tang Nano 20K is supposed to use GPIO2 to FPGA
pin 6 for MISO data. But due to a design error on some Tang Nano 20k,
GPIO2 is unusable for fast signals. Thus, GPIO10 is being used
instead. GPIO10 is one of the JTAG pins on FPGA side. Using this pin
as a regular IO requires to disable JTAG in the FPGA. As a
consequence, programming the FPGA needs [special care](MODES.md). This
only affects the Tang Nano 20k's internal MCU. An external M0S based
solution is not affected by this.

The SPI master is the MCU and the SPI target is the FPGA. Thus, only
the MCU initiates SPI communication. The SPI is operated in MODE1 with
clock idle state being low and data being sampled on the falling edge
of clock.

Since release 1.2.2 an interrupt signal (IRQ) is being used to allow
the FPGA to notify the MCU of events. This avoids the need to
constantly poll the FPGA from the MCU and the FPGA can simply raise
the interrupt signal whenever it requires the cooperation of the MCU.
Currently the following interrupt sources are being used:

| bit | name | usage |
|-----|------|-------|
| 0   | SYS  | FPGA has been (re-)initialized (cold boot) or port data has been received |
| 1   | HID  | DB9 joystick event detected by FPGA |
| 3   | SDC  | FPGA requests SD card sector translation |

## Protocol

SPI communication is initiated by the MCU by driving CSN low.

The basic SPI communication runs between [```spi.c```](src/spi.c) on the
MCU side and [```mcu_spi.v```](https://github.com/MiSTle-Dev/MiSTeryNano/blob/main/src/misc/mcu_spi.v) on FPGA side.

The first byte of each message identifies the target inside the FPGA
the MCU wants to address. Currently implemented are:

| value | name | description | MCU implementation | FPGA implementation |
|-------|------|-------------|--------------------|---------------------|
| 0     | SYS  | Generic system control | [```sysctrl.c```](src/sysctrl.c) | [```sysctrl.v```](https://github.com/MiSTle-Dev/MiSTeryNano/blob/main/src/misc/sysctrl.v) |
| 1     | HID  | Human Interface Devices, e.g. keyboard & mice | [```usb_host.c```](src/hid.c) | [```hid.v```](https://github.com/MiSTle-Dev/MiSTeryNano/blob/main/src/misc/hid.v) |
| 2     | OSD  | On-Screen-Display | [```osd_u8g2.c```](src/osd_u8g2.c) | [```osd_u8g2.v```](https://github.com/MiSTle-Dev/MiSTeryNano/blob/main/src/misc/osd_u8g2.v) |
| 3     | SDC  | SD Card   | [```sdc.c```](src/sdc.c) | [```sd_card.v```](https://github.com/MiSTle-Dev/MiSTeryNano/blob/main/src/misc/sd_card.v) |
| 4     | AUDIO | Audio output | TBD | TBD |

Any data after the first target byte is being sent to the target
inside the FPGA for further parsing. The communication inside the FPGA
between ```mcu_spi.v``` and the various targets is byte-wide using
four signals:

| signal  | description |
|---------|-------------|
| ```strobe```  | Indicate the arrival of a data byte for the target |
| ```start```   | Valid for the first byte of a message |
| ```dout[8]``` | Databyte from MCU to target |
| ```din[8]```  | Databyte from target to MCU |

The first byte transferred with ```start``` active (the second byte
within each SPI message) usually is a command byte for the target.

### SYS target

The SYS (system control) target currently supports these commands:

| value | name | description |
|---------|-------------|-------------|
| 0 | ```SPI_SYS_STATUS```   | Request status from FPGA |
| 1 | ```SPI_SYS_LEDS``` | Send a LED status into the FPGA |
| 2 | ```SPI_SYS_RGB``` | Send RGB color info into FPGA |
| 3 | ```SPI_SYS_BTN``` | Request button state from FPGA |
| 4 | ```SPI_SYS_SET``` | Set a variable in the FPGA |
| 5 | ```SPI_SYS_IRQ_CTRL``` | Interrupt control |
| 6 | ```SPI_SYS_IRQ_SRC``` | Request interrupt source |
| 7 | ```SPI_SYS_PORT``` | Handle port IO (e.g rs232) |
| 8 | ```SPI_SYS_READ_CFG``` | Read configuration |

The ```SPI_SYS_STATUS``` command currently just returns $5c and $42 in
the Atari ST core. This can be used to check if the FPGA has started
up and has the right core installed. Other cores may use different
bytes to allow the MCU to identify the type of the running core.

The ```SPI_SYS_LEDS``` command allows to control LEDs 4 and 5 on the Tang
Nano from the MCU. The other four LEDs are controlled by the core itself.
This may not always be wired up and some releases may use all LEDs by the
FPGA directly to signal certain operation states and events.

The ```SPI_SYS_RGB``` command can be used to send a 24 bit (three bytes)
RGB (red/green/blue) value into the core which it can e.g. use to drive
the on-board ws2812 led.

The ```SPI_SYS_BTN``` command will return the state of the two S0 and
S1 buttons on the Tang Nano 20k. This can e.g. be used to control the
MCU via those buttons. Bit 0 typically reports the reset button and
bit 1 reports a user button if preset which may e.g. used to open the
on-sreen-display.

The ```SPI_SYS_SET``` command has two data bytes. The first byte is
the ASCII ID's of a variable to be set and the second byte is a 8 bit
value. E.g.  the Atari ST core supports the configuration of the
chipset between three values (ST, Mega ST and STE) via the OSD. This
setting is sent using the ID 'C' (for Chipset) with a value of 0 to
2. The use of these values is core specific.

With ```SPI_SYS_IRQ_CTRL``` the first data byte allows to acknowledge
the eight possible interrupt sources inside the FPGA. The second data
byte will return the pending interrupts. The eight interrupts sources
are currently mapped to the SPI targets, e.g. the SD card target 3 is
mapped to interrupt bit 3.

With ```SPI_SYS_IRQ_SRC``` command allows to request the system IRQ
source. The source is returned as a bit field with the second data
byte. Currently supported are bit 0 which indicates a FPGA cold boot
detection and bit 1 which indicates the port data is available. This
is currently being used whenever data has been received via the "AT
Wifi" emulation which allows to simulate a RS232 port over TCP/Telnet
on MCUs that support WiFi. But 2 is set if a FPGA button change has
been detected which could then be read (and acknowledged) using the
```SPI_SYS_BTN``` command.

The ```SPI_SYS_PORT``` command handles port redirection for RS232
implementation or the like. The first payload bytes specifies a
subcommand. Currently three subcommands are implemented. These are
```SPI_SYS_PORT_STATUS```, ```SPI_SYS_PORT_PUT``` and
```SPI_SYS_PORT_GET``` allowing the MCU to read the ports status
(e.g. free FIFO space or bitrates set by the core), to send bytes into
the core or to read bytes back from the core. The ```SPI_SYS_PORT```
is currently being used to implement a TCP/Telnet emulation of a
serial port over Wifi using AT commands. This allows to e.g. dial into
BBS from the core.

The ```SPI_SYS_READ_CONFIG``` allows the FPGA Companion to read the
core specific configuration in XML format. This data mainly contains
the menu structure used by this partticular core and makes the core
development independent from the FPGA Companion. The config is
returned beginning with the second payload byte read via this
command. The core may return the configuation either as plain XML text
or in GZIP compressed format. A plain text XML must begin with the
```<``` character and be terminated by a zero byte character. The
GZIP'd XML must begin with a $1f byte and will be parsed until the
decompression us successful or aborted by a decompression error. The
gzip'd hex file can be regenereted under Linux using like so:

```
$ gzip -n atarist.xml
$ xxd -c1 -p atarist.xml.gz > atarist_xml.hex
```

The advantage of the compressed version is that it only occupies about
25% of the uncompressed XML in the FPGAs internal ROM resources.

### HID target

The HID target currently supports three commands:

| value | name | description |
|---------|-------------|-------------|
| 0 | ```SPI_HID_STATUS```   | Request status from FPGA |
| 1 | ```SPI_HID_KEYBOARD``` | Send a keyboard data byte into the FPGA |
| 2 | ```SPI_HID_MOUSE``` | Send a mouse data byte into the FPGA |
| 3 | ```SPI_HID_JOYSTICK``` | Send a joystick data byte into the FPGA |
| 4 | ```SPI_HID_GET_DB9``` | Read the DB9 joystick status from the FPGA |

The ```SPI_HID_STATUS``` message is currently unused. It will be used to report the
HID requirements. This may e.g. include the keycode mapping required
by the core. This currently returns 0x5c and 0x42 in bytes 4 and 5.

The ```SPI_HID_KEYBOARD``` messages are core specific. Currently each message
consists of one byte only containing the press/release status and the
row and column of the key inside the keyboard matrix. Typically
the F12 key is not forwarded into the core. Instead the F12 key is
used to open and close the OSD. While the OSD is visible no keyboard
data is sent into the core. Instead the keyboard is being used to
control the OSD.

The ```SPI_HID_MOUSE``` messages contain three data bytes. The first
byte carries the state of the mouse buttons in bits 0 and 1. The
second and third bytes contain relative x and y movements.

The ```SPI_HID_JOYSTICK``` messages contains two data bytes, the
first addressing the joystick and the second containing classic 8 bit
digital joystick data. The address byte is needed since unlike
keyboards and mice, the core needs to distinguish between multiple
joysticks. The upper four bits of the second byte contain up to four
fire buttons. These can simply be or'd together for standard DB9
joystick emulation.

The ```SPI_HID_GET_DB9``` command allows the MCU to request the
state of the DB9 joystick port from the FPGA. This can be used
to e.g. control the OSD via a joystick connected to that port.

### OSD target

The OSD (on-screen-display) is a monochrome 128x64 frame buffer inside
the FPGA that can be displayed centered on the main screen on request
by the MCU. The 1024 bytes of video memory (128x64/8) are organized
"vertically" as typically used with small OLED displays. This is to
easy the use of the [u8g2](https://github.com/olikraus/u8g2) library
which is a graphics library for small monochrome displays and which
is being used in MiSTeryNano.

The OSD target supports the following commands:

| value | name | description |
|---------|-------------|-------------|
| 1 | ```SPI_OSD_ENABLE```  | Show or hide the OSD |
| 2 | ```SPI_OSD_WRITE``` | Send graphics data to the OSD |

The ```SPI_OSD_ENABLE``` command has one data byte. The lowest bit of this
indicates whether the OSD is to be shown (1) or hidden (0).

The ```SPI_OSD_WRITE``` command is followed by a byte containing an offset value
and 1 or more data bytes. The offset indicates which tile the graphics data
starts at. E.g. an offset of 100 indicates that writing should start at tile column
100 which is the 800th pixel column. Since the display is 128 pixels wide the
800th column is the 32th pixel column in tile row 6 (6*128+32=800). This is exactly
how the u8g2 library expects to address a display.

### SDC target

The SDC (SD card) target allows the MCU to use the SD card connected
to the FPGA. It can either read whole sectors for its own purposed
(e.g. letting the user browse the file system) or it can assist the
FPGA core in accessing data inside files on the SD card. This is
necessary since most cores expect to deal with single media (e.g. a
single floppy disk). The MiSTeryNano instead stores these in media
files (e.g. Atari ST .ST disk image files) which in turn reside inside
the file system of the SD card. So these are basically core specific
file systems stored inside a SD card file system. While the FPGA core
deals with the core specific file system inside the image files it's
up to the MCU to handle the SD cards own file system.

The SDC target supports the following commands:

| value | name | description |
|---------|-------------|-------------|
| 1 | ```SPI_SDC_STATUS```  | Read SD card status |
| 2 | ```SPI_SDC_CORE_RW``` | Request the core to read or write a sector for it's own purpose |
| 3 | ```SPI_SDC_MCU_READ``` | Request to read data for MCU usage |
| 4 | ```SPI_SDC_INSERTED``` | Inform core about the selection of disk images |
| 5 | ```SPI_SDC_MCU_WRITE``` | Request to write data on behalf of the MCU |
| 6 | ```SPI_SDC_DIRECT``` | Inform core that image may be accessed directly |
| 7 | ```SPI_SDC_INS_LARGE``` | Inform core about the selection of large disk images |

The ```SPI_SDC_STATUS``` command is used to poll the SD card status. The first
byte returned is a generic status byte indicating whether the card
could be initialized and the card type (SDv1, SDv2, SDHC). Bit 1 of
the status byte indicates whether the SD card is currenly busy reading
or writing data. The second byte contains core requests. Currently the
lower two bits are used by the core to request sector data. In the
Atari ST core bit 0 indicates a request for floppy A: and bit 1 for
floppy B:. The following four bytes contain the sector number the core
wants to read.

The ```SPI_SDC_CORE_RW``` command requests the core to read or write a sector
from or to SD card and use it for it's own purposes. The command is
followed by four bytes containing the sector number to be read or
written. This command is usually a sent in reply to bit 0 or 1 being
set in the reply to```SPI_SDC_STATUS```. The core has requested to
read or write a sector (e.g. sector 0 if it wants to read or write the
first sector on the floppy) and the MCU now instructs it to use a
certain sector on SD card. This will be the first sector of the disk
images on SD card which will be the first data sector in the image
file. No actual data is being returned to the MCU with this command.

In contrast ```SPI_SDC_MCU_READ``` requests the FPGA to return the contents
of a sector from the SD card is to be returned to the MCU. The following four
data bytes indicate again which sector is to be read. The FPGA will return
busy bytes (!=0) as long as the SD card is being read. Once data is ready, the
command returns a single 0 byte followed by 512 bytes of sector data. This
data is not visible to the core. Instead it used by the MCU to display the
contents of the SD card and to determine where data is stored inside the images
files stored on the SD card.

The ```SPI_SDC_INSERTED``` command and the following five data bytes
are used to inform the core about drive and the size of the selected
disk image. This is needed to translate between the track/sector/side
values typically used when reading from floppy disk into sector
offsets into the image file. A size of 0 indicates that no image is
currently selected. The core should then behave as if e.g. no floppy
disk is inserted.

Using ```SPI_SDC_MCU_WRITE``` the MCU can request a sector to be written to
SD card. The first four data bytes indicate which sector on the SD card
is to be written. The following 512 bytes are the data to be written.
Afterwards command will return bytes != 0 as long as the card is busy
writing.

The ```SPI_SDC_DIRECT``` command tells the core that the image
specified by the first data byte is continous on SD card (not
fragmented) and may be accessed by the core directly without sector
translation by the MCU. The following four data bytes contain the LBA
of the first sector of the selected image on SD card and may
optionally be used by the core to directly access the SD card.

The ```SPI_SDC_INS_LARGE``` command and the following nine data bytes
inform the core that a large image (>4GB) has been inserted. This
commands works otherwise like ```SPI_SDC_INSERTED```. Cores not
able to deal with images that large should ignore this command and
thus prevent themselves from accidentially damaging the image during
a write.

### AUDIO target

The audio target has not been implemented, yet. It's purpose is to
give the MCU the ability to output audio via the core. This may
e.g. be used for accoustic feedback during menu usage or for the
simulation of floppy disk sounds or the like.