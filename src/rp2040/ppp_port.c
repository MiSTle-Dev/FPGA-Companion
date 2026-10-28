/*
  ppp_port.c

  PPP server on port 0 of the core (e.g. the Amiga serial port).

  rx: port bytes are handed to lwIP via pppos_input_tcpip(), which takes the
      lwIP core lock. This is the only legal way to feed PPPoS from a task
      other than the tcpip thread.
  tx: lwIP calls ppp_out() from the tcpip thread with the core lock held.
      That must never wait for the core, since sys_port_write() blocks as long
      as the core does not read. So ppp_out() only queues and a separate task
      writes to the port.
*/

#include "ppp_port.h"

#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include "lwip/tcpip.h"
#include "lwip/tcp.h"
#include "lwip/priv/tcp_priv.h"
#include "lwip/dns.h"
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "lwip/prot/ethernet.h"
#include "lwip/prot/etharp.h"
#include "lwip/prot/iana.h"
#include "lwip/ip4.h"
#include "lwip/ip4_frag.h"
#include "lwip/prot/tcp.h"
#include "lwip/inet_chksum.h"
#include "netif/ppp/ppp.h"
#include "netif/ppp/pppos.h"

#include "../sysctrl.h"
#include "../inifile.h"
#include "../debug.h"

#if !LWIP_TCPIP_CORE_LOCKING
#error ppp_port needs LWIP_TCPIP_CORE_LOCKING
#endif

#ifndef TX_SIZE
#if defined(PICO_RP2350) && PICO_RP2350
#define TX_SIZE   8192    // power of two, 0.7 s of data at 115200 Bd
#else
#define TX_SIZE   4096    // RP2040 (Pico W) has little ram left
#endif
#endif
#define TX_CHUNK  128     // bytes per sys_port_write(), so tx_rd advances steadily

static struct netif ppp_netif;
static ppp_pcb *ppp;
static TaskHandle_t tx_task;
static volatile bool ended;

// free running counters, tx_wr is only written by ppp_out(), tx_rd only by tx_thread()
static uint8_t tx_buf[TX_SIZE];
static uint32_t tx_rd, tx_wr;
static volatile uint32_t tx_drops;     // frames refused because the ring was full
static volatile bool tx_blocked;       // ... and tcp has to be told when there is room again

// called by lwIP, tcpip thread, never block here. The stdio uart is as slow as the link, so
// not even a printf() belongs here: it would hold up the whole stack for ms per line.
static u32_t ppp_out(ppp_pcb *pcb, const void *data, u32_t len, void *ctx) {
  (void)pcb; (void)ctx;

  uint32_t wr = tx_wr;
  uint32_t rd = __atomic_load_n(&tx_rd, __ATOMIC_ACQUIRE);
  if(len > TX_SIZE - (wr - rd)) {
    tx_drops++;
    tx_blocked = true;
    return 0;   // lwIP reports ERR_IF: tcp keeps the segment queued, other packets are lost
  }

  for(u32_t i = 0; i < len; i++)
    tx_buf[(wr + i) & (TX_SIZE - 1)] = ((const uint8_t *)data)[i];

  __atomic_store_n(&tx_wr, wr + len, __ATOMIC_RELEASE);
  xTaskNotifyGive(tx_task);
  return len;
}

// tcpip thread: segments that ppp_out() refused are sent now
static void tx_resume(void *arg) {
  (void)arg;
  tcp_txnow();
}

static void tx_thread(void *arg) {
  (void)arg;
  uint32_t reported = 0;
  TickType_t last_report = 0;

  for(;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint32_t rd = tx_rd, wr;
    while(rd != (wr = __atomic_load_n(&tx_wr, __ATOMIC_ACQUIRE))) {
      uint32_t n = wr - rd;
      uint32_t until_wrap = TX_SIZE - (rd & (TX_SIZE - 1));
      if(n > until_wrap) n = until_wrap;
      if(n > TX_CHUNK)   n = TX_CHUNK;

      sys_port_write(0, &tx_buf[rd & (TX_SIZE - 1)], n);   // may wait for the core
      rd += n;
      __atomic_store_n(&tx_rd, rd, __ATOMIC_RELEASE);

      // half empty again: let tcp send what was refused (try again at the next chunk if the mailbox is full)
      if(tx_blocked && wr - rd <= TX_SIZE / 2 && tcpip_try_callback(tx_resume, NULL) == ERR_OK)
        tx_blocked = false;
    }

    // at most one report every 2 s, from here and not from the lwIP thread
    TickType_t now = xTaskGetTickCount();
    if(tx_drops != reported && now - last_report >= pdMS_TO_TICKS(2000)) {
      debugf("PPP: tx queue full, %u frame(s) refused", (unsigned)(tx_drops - reported));
      reported = tx_drops;
      last_report = now;
    }
  }
}

/* ---- LAN side: the peer is meant to look like a host of the local network ----
   If [ppp] peer is an unused address of the network the Companion is connected to
   (wifi or usb ethernet), the Companion forwards between the two and answers ARP
   requests for the peer's address (proxy ARP). The router then needs no route to
   the peer and does the NAT to the internet like for any other host. All of this
   runs with the lwIP core lock held. */

#define MAX_LAN 2
static struct { struct netif *netif; netif_input_fn input; } lan[MAX_LAN];
static ip4_addr_t peer_ip;
static bool peer_up;

static bool peer_on_link(struct netif *n) {
  return !ip4_addr_isany_val(*netif_ip4_addr(n)) &&
    ip4_addr_netcmp(&peer_ip, netif_ip4_addr(n), netif_ip4_netmask(n));
}

// ip4_route() hook (lwipopts.h): the peer is always reached via ppp, even if it is in a lan subnet
struct netif *ppp_port_route(const ip4_addr_t *dest) {
  return (peer_up && ip4_addr_cmp(dest, &peer_ip)) ? &ppp_netif : NULL;
}

// TCP segments passing through are adapted to the slow ppp link (the options must lie completely
// in the first pbuf, every change updates the TCP checksum incrementally, RFC 1624):
// - SYN: lower the announced MSS to what fits into the ppp mtu, remove the window scale option
//   (in both directions of a connection, so that both ends agree that nothing is scaled)
// - everything from the peer: limit the window it advertises. Otherwise a download server fills
//   the window of the peer (typically far more than the tx ring holds), the ring overflows, the
//   server sees losses, backs off and recovers slowly, in the end the line is used worse than
//   with a small window.
#ifndef PPP_WND_SEGS
#define PPP_WND_SEGS 4      // segments the peer may have in flight
#endif

static void tcp_patch(u8_t *tcp, u16_t pos, u8_t val) {   // pos: offset in the tcp header
  u32_t old = tcp[pos], c;
  u16_t sum = (tcp[16] << 8) | tcp[17];
  if(old == val) return;
  c = (u16_t)~sum + (u16_t)~((pos & 1) ? old : old << 8) + ((pos & 1) ? val : val << 8);
  c = (c & 0xffff) + (c >> 16);
  c = (c & 0xffff) + (c >> 16);
  sum = (u16_t)~c;
  tcp[16] = sum >> 8; tcp[17] = sum & 0xff;
  tcp[pos] = val;
}

static void tcp_fixup(struct pbuf *p, u16_t hl, u16_t len, bool from_peer) {
  u8_t *ip = (u8_t *)p->payload, *tcp = ip + hl;
  u16_t thl, i, mss, max, wnd, wmax;

  if(ppp_netif.mtu <= IP_HLEN + TCP_HLEN) return;
  max = ppp_netif.mtu - IP_HLEN - TCP_HLEN;
  if(IPH_PROTO((struct ip_hdr *)ip) != IP_PROTO_TCP || (lwip_ntohs(IPH_OFFSET((struct ip_hdr *)ip)) & IP_OFFMASK)) return;
  if(len < hl + TCP_HLEN || p->len < hl + TCP_HLEN) return;

  u8_t flags = TCPH_FLAGS((struct tcp_hdr *)tcp);
  if(from_peer && !(flags & TCP_RST)) {
    wnd = (tcp[14] << 8) | tcp[15];
    wmax = PPP_WND_SEGS * max;
    if(wnd > wmax) { tcp_patch(tcp, 14, wmax >> 8); tcp_patch(tcp, 15, wmax & 0xff); }
  }

  if(!(flags & TCP_SYN)) return;
  thl = TCPH_HDRLEN_BYTES((struct tcp_hdr *)tcp);
  if(thl <= TCP_HLEN || hl + thl > len || hl + thl > p->len) return;

  for(i = TCP_HLEN; i < thl; ) {
    u8_t kind = tcp[i];
    if(kind == 0) break;                             // end of options
    if(kind == 1) { i++; continue; }                 // nop
    if(i + 1 >= thl || tcp[i + 1] < 2 || i + tcp[i + 1] > thl) break;
    if(kind == 2 && tcp[i + 1] == 4) {               // MSS
      mss = (tcp[i + 2] << 8) | tcp[i + 3];
      if(mss > max) { tcp_patch(tcp, i + 2, max >> 8); tcp_patch(tcp, i + 3, max & 0xff); }
    } else if(kind == 3 && tcp[i + 1] == 3) {        // window scale -> 3 x nop
      tcp_patch(tcp, i, 1); tcp_patch(tcp, i + 1, 1); tcp_patch(tcp, i + 2, 1);
    }
    i += tcp[i + 1];
  }
}

// ip4_input() hook (lwipopts.h) replacing ip4_forward(): lwIP 2.2.1 clears the IP and TCP
// checksums of forwarded packets and never recomputes them. Here only TTL and the IP header
// checksum change, the payload (TCP, UDP, ICMP) is passed on untouched, there is no NAT.
// Returns 1 if the packet was consumed, 0 to let lwIP handle it as usual.
int ppp_port_ip4_input(struct pbuf *p, struct netif *inp) {
  struct ip_hdr *ip = (struct ip_hdr *)p->payload;
  struct netif *out, *n;
  const ip4_addr_t *hop;
  ip4_addr_t dst;
  u16_t hl, len;

  if(!peer_up || p->len < IP_HLEN || IPH_V(ip) != 4) return 0;
  hl = IPH_HL_BYTES(ip);
  len = lwip_ntohs(IPH_LEN(ip));
  if(hl < IP_HLEN || p->len < hl || p->tot_len < len || len < hl || IPH_TTL(ip) <= 1) return 0;

  ip4_addr_copy(dst, ip->dest);
  if(ip4_addr_isany_val(dst) || ip4_addr_ismulticast(&dst) || ip4_addr_isbroadcast(&dst, inp)) return 0;
  NETIF_FOREACH(n)                                   // addressed to the companion itself
    if(ip4_addr_cmp(&dst, netif_ip4_addr(n))) return 0;

  if(inp == &ppp_netif) {                            // peer -> lan or internet
    if(!ip4_addr_cmp(&ip->src, &peer_ip)) return 0;  // only its own address
    out = ip4_route(&dst);
    if(!out || out == &ppp_netif) return 0;
    hop = ip4_addr_netcmp(&dst, netif_ip4_addr(out), netif_ip4_netmask(out)) ? &dst : netif_ip4_gw(out);
    if(ip4_addr_isany(hop)) hop = &dst;
  } else if(ip4_addr_cmp(&dst, &peer_ip)) {          // lan -> peer
    out = &ppp_netif;
    hop = &peer_ip;
  } else return 0;

  // only lan -> peer can be too big (the ppp mtu is small), dropped silently if it must not be fragmented
  if(len > out->mtu && (lwip_ntohs(IPH_OFFSET(ip)) & IP_DF)) return 0;
  if(p->tot_len > len) pbuf_realloc(p, len);         // strip ethernet padding

  tcp_fixup(p, hl, len, inp == &ppp_netif);
  IPH_TTL_SET(ip, IPH_TTL(ip) - 1);
  IPH_CHKSUM_SET(ip, 0);
  IPH_CHKSUM_SET(ip, inet_chksum(ip, hl));

  if(len > out->mtu) ip4_frag(p, out, hop);
  else if(out == &ppp_netif) out->output(out, p, hop);
  else {                                             // ppp buffers have no room for the ethernet header
    struct pbuf *q = pbuf_alloc(PBUF_LINK, p->tot_len, PBUF_RAM);
    if(q) {
      if(pbuf_copy(q, p) == ERR_OK) out->output(out, q, hop);
      pbuf_free(q);
    }
  }
  pbuf_free(p);
  return 1;
}

// ARP reply (or gratuitous request) in the name of the peer
static void arp_send(struct netif *n, u16_t op, const struct eth_addr *eth_dst,
		     const struct eth_addr *hw_dst, const ip4_addr_t *ip_dst) {
  struct pbuf *p = pbuf_alloc(PBUF_LINK, SIZEOF_ETH_HDR + SIZEOF_ETHARP_HDR, PBUF_RAM);
  if(!p) return;

  struct eth_hdr *eth = (struct eth_hdr *)p->payload;
  struct etharp_hdr *arp = (struct etharp_hdr *)((u8_t *)p->payload + SIZEOF_ETH_HDR);

  memcpy(&eth->dest, eth_dst, ETH_HWADDR_LEN);
  memcpy(&eth->src, n->hwaddr, ETH_HWADDR_LEN);
  eth->type = PP_HTONS(ETHTYPE_ARP);

  arp->hwtype   = PP_HTONS(LWIP_IANA_HWTYPE_ETHERNET);
  arp->proto    = PP_HTONS(ETHTYPE_IP);
  arp->hwlen    = ETH_HWADDR_LEN;
  arp->protolen = sizeof(ip4_addr_t);
  arp->opcode   = PP_HTONS(op);
  memcpy(&arp->shwaddr, n->hwaddr, ETH_HWADDR_LEN);
  memcpy(&arp->sipaddr, &peer_ip, sizeof(ip4_addr_t));
  memcpy(&arp->dhwaddr, hw_dst, ETH_HWADDR_LEN);
  memcpy(&arp->dipaddr, ip_dst, sizeof(ip4_addr_t));

  n->linkoutput(n, p);
  pbuf_free(p);
}

// netif->input of the lan interfaces: answer ARP requests for the peer, pass on everything
static err_t lan_input(struct pbuf *p, struct netif *inp) {
  if(p->len >= SIZEOF_ETH_HDR + SIZEOF_ETHARP_HDR &&
     ((struct eth_hdr *)p->payload)->type == PP_HTONS(ETHTYPE_ARP)) {
    struct etharp_hdr *arp = (struct etharp_hdr *)((u8_t *)p->payload + SIZEOF_ETH_HDR);
    ip4_addr_t dip, sip;
    memcpy(&dip, &arp->dipaddr, sizeof(dip));
    memcpy(&sip, &arp->sipaddr, sizeof(sip));

    if(arp->opcode == PP_HTONS(ARP_REQUEST) && peer_up &&
       ip4_addr_cmp(&dip, &peer_ip) && peer_on_link(inp))
      arp_send(inp, ARP_REPLY, &arp->shwaddr, &arp->shwaddr, &sip);
  }

  for(int i = 0; i < MAX_LAN; i++)
    if(lan[i].netif == inp) return lan[i].input(p, inp);

  pbuf_free(p);                   // not reached, only hooked interfaces get here
  return ERR_OK;
}

static void lan_attach(void) {
  struct netif *n;
  peer_up = true;

  NETIF_FOREACH(n) {
    if(n == &ppp_netif || !(n->flags & NETIF_FLAG_ETHARP) || !peer_on_link(n)) continue;

    int slot = -1;
    for(int i = 0; i < MAX_LAN && slot < 0; i++) if(lan[i].netif == n) slot = i;
    for(int i = 0; i < MAX_LAN && slot < 0; i++) if(!lan[i].netif)    slot = i;
    if(slot < 0) continue;

    if(n->input != lan_input) {   // interface may have been re-added since
      lan[slot].netif = n;
      lan[slot].input = n->input;
      n->input = lan_input;
    }

    // tell the network that the peer's address is now behind us
    static const struct eth_addr bcast = {{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}};
    static const struct eth_addr zero  = {{0, 0, 0, 0, 0, 0}};
    arp_send(n, ARP_REQUEST, &bcast, &zero, &peer_ip);
    debugf("PPP: %s is reachable via %c%c", ip4addr_ntoa(&peer_ip), n->name[0], n->name[1]);
  }
}

static void lan_detach(void) {
  peer_up = false;
  for(int i = 0; i < MAX_LAN; i++) {
    if(lan[i].netif && lan[i].netif->input == lan_input) lan[i].netif->input = lan[i].input;
    lan[i].netif = NULL;
  }
}

// called by lwIP, tcpip thread
static void ppp_status(ppp_pcb *pcb, int err, void *ctx) {
  (void)ctx;

  if(err == PPPERR_NONE) {
    debugf("PPP: up, our address %s", ip4addr_ntoa(netif_ip4_addr(pcb->netif)));
    lan_attach();
  } else {
    debugf("PPP: down, error %d", err);
    lan_detach();
    ended = true;
  }
}

// ppp->phase, only valid under the core lock
static int ppp_phase(void) {
  LOCK_TCPIP_CORE();
  int phase = ppp ? ppp->phase : PPP_PHASE_DEAD;
  UNLOCK_TCPIP_CORE();
  return phase;
}

bool ppp_port_listen(void) {
  if(!mcu_hw_tcpip_ready()) return false;

  // lwIP reports the end of a session late: after a Terminate-Request of the peer it
  // waits FSM_DEFTIMEOUT seconds for the line to drop. ppp_listen() needs the dead phase.
  for(int i = 0; i < 100 && ppp_phase() != PPP_PHASE_DEAD; i++)
    vTaskDelay(pdMS_TO_TICKS(50));

  if(!tx_task && xTaskCreate(tx_thread, "ppp_tx", 1024, NULL,
			     configMAX_PRIORITIES-10, &tx_task) != pdPASS)
    return false;

  // addresses of both ends of the link, [PPP] in config.ini can override them
  ip4_addr_t ours, his;
  ip4_addr_set_u32(&ours, lwip_htonl(inifile_config_get_ip("ppp", "ip",   0xc0a8fd01)));  // 192.168.253.1
  ip4_addr_set_u32(&his,  lwip_htonl(inifile_config_get_ip("ppp", "peer", 0xc0a8fd02)));  // 192.168.253.2

  LOCK_TCPIP_CORE();

  if(!ppp) ppp = pppos_create(&ppp_netif, ppp_out, ppp_status, NULL);

  err_t err = ERR_MEM;
  if(ppp) {
    ppp_set_ipcp_ouraddr(ppp, &ours);
    ppp_set_ipcp_hisaddr(ppp, &his);
    peer_ip = his;

    // pass the name server on, if we have one
    const ip_addr_t *dns = dns_getserver(0);
    if(!ip_addr_isany(dns)) ppp_set_ipcp_dnsaddr(ppp, 0, ip_2_ip4(dns));

    ended = false;
    err = ppp_listen(ppp);
  }

  UNLOCK_TCPIP_CORE();

  if(err != ERR_OK) debugf("PPP: listen failed, %d", err);
  return err == ERR_OK;
}

void ppp_port_close(void) {
  if(!ppp) return;

  LOCK_TCPIP_CORE();
  ppp_close(ppp, 0);
  UNLOCK_TCPIP_CORE();
}

bool ppp_port_ended(void) {
  return ended;
}

void ppp_port_flush(int ms) {
  while(ms > 0 && __atomic_load_n(&tx_rd, __ATOMIC_ACQUIRE) != __atomic_load_n(&tx_wr, __ATOMIC_ACQUIRE)) {
    vTaskDelay(pdMS_TO_TICKS(5));
    ms -= 5;
  }
}

void ppp_port_input(const unsigned char *data, int len) {
  if(ppp && len > 0) pppos_input_tcpip(ppp, data, len);
}
