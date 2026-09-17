# Networking

The FPGA Companion supports various ways of networking. The BL616 and ESP32
MCUs come with WiFi built-in, the Pi Pico family of devices also comes in
variants that have integrated WiFi support in the -W variants.

Furthermore the BL616 and the RP2xxx/Pico support wired ethernet through
certain USB Ethernet adapters. Adapters like the D-Link DUB-E100 based on
the AX88772 family of chips are supported by both platforms. Additionally
the BL616 supports adapters based on the RTL8152 chip family.

## Use cases

Currently the following use cases are implemented:

  - Serial AT modem emulation giving the core basic networking
    support to a simple AT command interface similar to classic
    telephone modems
  - NTP time synchronization allows the Companion to provide
    real time clock information to the core for RTC implementation
  - FTP allows to access the Companion from a PC via FTP to up- and
    download files from and to the SD card inside the device
  - Telnet allows to read log and debug info from the companion

## Setup

In some cases no configuration is needed at all. Once a supported
ethernet USB adapter is connected, the Companion will configure
networking automatically through DHCP and display the IP address
briefly through the cores OSD. In the likely case that a valid
NTP server address is available through DHCP, the Companion will
synchronize the time and potentially forward this to the core.

The use of WiFi requires valid WiFi credentials. These need to
be configured though a file named ```config.ini``` stored in the
root of the SD-card. An examples file looks like this:

```
; network settings
[NETWORK]
MODE=manual          ; manual or dhcp
IP=192.168.0.12
MASK=255.255.255.0
GW=192.168.0.1

[NTP]
IP=192.53.103.108,192.53.103.104,192.53.103.103
TIMEZONE=+2          ; UTC+2

[WIFI]
SSID=MYSSID
PASS=MYPASSPHRASE
```

## AT WiFi Interface

Some cores support the AT Wifi Interface. These are currently the
Atari ST, the Macintosh and the C64 cores. The AT Command interface
provides a simple set of AT commands. Information about these
are also available through the AT command interface itself:

```
Supported commands:
  atscan                     - scan for WiFi networks
  atssid <ssid>,<passphrase> - connect to WiFi network
  atd <server>:<port>        - connect to <server> via TCP <port>
  ato                        - re-enter online mode
  ath                        - hang-up / disconnect from server
  <1sec>+++<1sec>            - escape to offline mode
  atpetscii                  - petscii input
  atascii                    - ascii input
```

This is meant to be used to dial into classic BBS accessible through internet
like the [Darkforce BBS](ttps://www.telnetbbsguide.com/bbs/dark-force-bbs/). This
would be accessed using the following command:

```
atd darkforce-bbs.dyndns.org:1040
```
