/*
  ppp_port.h

  PPP server on port 0 of the core (e.g. the Amiga serial port). The byte
  stream of the port is fed into lwIP's PPPoS, so a classic PPP client
  (Roadshow + ppp-serial.device) can reach the network through the Companion.
*/

#ifndef PPP_PORT_H
#define PPP_PORT_H

#include <stdbool.h>

// peers are hosts behind an interface of the Companion (ppp or plip) which appear as hosts of the
// lan: proxy ARP and forwarding in both directions, see ppp_port.c
#define PORT_PEERS  2
#define PEER_PPP    0
#define PEER_PLIP   1

struct netif;
struct ip4_addr;

// interface and address of a peer, then up/down when it can be reached. Core lock held, lwIP thread or task
void ppp_port_peer_set(int id, struct netif *n, const struct ip4_addr *ip, bool slow);
void ppp_port_peer_up(int id);
void ppp_port_peer_down(int id);
void ppp_port_peer_poll(int id);   // up, but the lan may have got its address later

// true once lwIP has been initialized (implemented in mcu_hw.c)
bool mcu_hw_tcpip_ready(void);

// start waiting for an incoming PPP session. False if lwIP is not up yet
bool ppp_port_listen(void);

// ask the session to terminate (returns immediately, see ppp_port_ended)
void ppp_port_close(void);

// true once the session has ended since the last ppp_port_listen()
bool ppp_port_ended(void);

// wait (at most ms) until all queued PPP output has been sent to the port
void ppp_port_flush(int ms);

// bytes received from the port. May be called from any task
void ppp_port_input(const unsigned char *data, int len);

#endif // PPP_PORT_H
