/*
  plip_port.h

  PLIP: the parallel port of the core (NanoMig, Amiga parallel port) as network
  interface. The Amiga runs Roadshow with plipbox.device, the core implements the
  plipbox peer in hardware (SPI target 5) and hands the Ethernet frames over to the
  Companion. Here they become an lwIP interface whose peer (the Amiga) is forwarded
  to the lan, like the PPP peer of ppp_port.c.

  config.ini:  [PLIP]  ENABLE=1    switch the virtual port on (default 0)
                       IP=...      address of the Companion on this link, default 192.168.254.1
                       PEER=...    address of the Amiga, default 192.168.254.2
                       LOOPBACK=1  core echoes $FFFD test frames itself (no network)
*/

#ifndef PLIP_PORT_H
#define PLIP_PORT_H

// read [PLIP] from config.ini and start the task if enabled (call after config.ini has been read)
void plip_port_init(void);

// interrupt of the core (frame arrived, frame fetched). May be called from any task
void plip_handle_event(void);

#endif // PLIP_PORT_H
