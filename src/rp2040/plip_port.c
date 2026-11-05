/*
  plip_port.c

  PLIP over SPI target 5 of the core, see plip_host.v:

    5, cmd     target and command. The reply to a byte is the answer to the byte before it,
               so 'cmd' returns stale data, the next byte returns the status ...
    0 STATUS   status, rx_len hi, rx_len lo
    1 READ     status, 0x00, frame bytes
    2 RELEASE  one byte: the receive buffer is free again
    3 WRITE    frame bytes into the transmit buffer (needs tx_valid = 0)
    4 COMMIT   len hi, len lo
    5 CONFIG   bit 0 = enable, bit 1 = loopback
    6 STATS    status, 0x00, rx (32), tx (32), roundtrip clocks (32), msb first

  A frame is the Ethernet frame without checksum, 14 .. 1514 bytes. The SPI bytes are
  sent one call at a time, a burst of bytes without a gap would be too fast for the core.

  The interface is an IP interface (like ppp): the ethernet header is added and removed
  here, ARP requests of the Amiga are answered for every address except its own (it
  sends everything to us, we forward). lwIP's ARP is not used, the MAC address of the
  Amiga is taken from its frames.

  rx: plip_task reads the frame, releases the buffer at once and hands the packet to lwIP
      with netif->input (tcpip_input, takes no lock).
  tx: plip_output() is called from the tcpip thread with the core lock held and only
      queues the frame, plip_task sends it as soon as the core can take it.
*/

#include "plip_port.h"
#include "ppp_port.h"

#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include "lwip/tcpip.h"
#include "lwip/tcp.h"
#include "lwip/priv/tcp_priv.h"
#include "lwip/netif.h"
#include "lwip/ip4.h"
#include "lwip/prot/ethernet.h"
#include "lwip/prot/etharp.h"
#include "lwip/prot/iana.h"

#include "../spi.h"
#include "../mcu_hw.h"
#include "../inifile.h"
#include "../debug.h"


#define ST_RX_VALID  0x01
#define ST_TX_VALID  0x02

#define FRAME_MAX   1514
#define PLIP_MTU    1500
#define TX_SLOTS    3
#define POLL_MS     50       // the interrupt is the normal trigger, this catches a missed one
#define STATS_MS    30000

static struct netif plip_netif;
static TaskHandle_t task;

static const uint8_t our_mac[6] = { 0x02, 'P', 'L', 'I', 'P', 0x01 };   // locally administered
static uint8_t peer_mac[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };    // until the Amiga has sent something

static ip4_addr_t our_ip, peer_ip;

// transmit queue, free running counters. tx_wr is only written by plip_output(), tx_rd by the task
static struct { uint8_t data[FRAME_MAX] __attribute__((aligned(4))); uint16_t len; } txq[TX_SLOTS];
static uint32_t tx_wr, tx_rd;
static volatile bool tx_blocked;        // a frame was refused, tcp has to be told when there is room
static volatile uint32_t tx_drops;

static uint8_t rxbuf[FRAME_MAX] __attribute__((aligned(4)));
static uint32_t n_rx, n_tx, n_rx_bad;

/* ---- SPI ---- */

static void net_begin(uint8_t cmd) {
  mcu_hw_spi_begin();
  mcu_hw_spi_tx_u08(SPI_TARGET_NET);
  mcu_hw_spi_tx_u08(cmd);
}

static uint8_t net_status(uint16_t *len) {
  net_begin(SPI_NET_STATUS);
  uint8_t st = mcu_hw_spi_tx_u08(0);
  uint8_t hi = mcu_hw_spi_tx_u08(0);
  uint8_t lo = mcu_hw_spi_tx_u08(0);
  mcu_hw_spi_end();
  *len = ((hi & 7) << 8) | lo;
  return st;
}

static void net_config(bool enable, bool loopback) {
  net_begin(SPI_NET_CONFIG);
  mcu_hw_spi_tx_u08((enable ? 1 : 0) | (loopback ? 2 : 0));
  mcu_hw_spi_end();
}

// frame of the Amiga into rxbuf, then free the core's buffer
static void net_read(uint16_t len) {
  net_begin(SPI_NET_READ);
  mcu_hw_spi_tx_u08(0);                    // status
  mcu_hw_spi_tx_u08(0);                    // 0 = ready
  for(uint16_t i = 0; i < len; i++) rxbuf[i] = mcu_hw_spi_tx_u08(0);
  mcu_hw_spi_end();

  net_begin(SPI_NET_RELEASE);
  mcu_hw_spi_tx_u08(0);
  mcu_hw_spi_end();
}

static void net_write(const uint8_t *data, uint16_t len) {
  net_begin(SPI_NET_WRITE);
  for(uint16_t i = 0; i < len; i++) mcu_hw_spi_tx_u08(data[i]);
  mcu_hw_spi_end();

  net_begin(SPI_NET_COMMIT);
  mcu_hw_spi_tx_u08(len >> 8);
  mcu_hw_spi_tx_u08(len & 0xff);
  mcu_hw_spi_end();
}

static void net_log_stats(void) {
  uint8_t b[14];

  net_begin(SPI_NET_STATS);
  for(int i = 0; i < 14; i++) b[i] = mcu_hw_spi_tx_u08(0);
  mcu_hw_spi_end();

  uint32_t core_rx = (b[2]  << 24) | (b[3]  << 16) | (b[4]  << 8) | b[5];
  uint32_t core_tx = (b[6]  << 24) | (b[7]  << 16) | (b[8]  << 8) | b[9];
  uint32_t clks    = (b[10] << 24) | (b[11] << 16) | (b[12] << 8) | b[13];
  debugf("PLIP: core rx %u tx %u rt_clks %u, here rx %u (bad %u) tx %u, tx queue full %u",
	 (unsigned)core_rx, (unsigned)core_tx, (unsigned)clks,
	 (unsigned)n_rx, (unsigned)n_rx_bad, (unsigned)n_tx, (unsigned)tx_drops);
}

/* ---- lwIP side ---- */

// netif->output, called by lwIP (tcpip thread, core lock held): never wait here
static err_t plip_output(struct netif *netif, struct pbuf *p, const ip4_addr_t *ipaddr) {
  (void)netif; (void)ipaddr;

  if(p->tot_len > PLIP_MTU) return ERR_MEM;

  uint32_t wr = tx_wr;
  if(wr - __atomic_load_n(&tx_rd, __ATOMIC_ACQUIRE) >= TX_SLOTS) {
    tx_drops++;
    tx_blocked = true;
    return ERR_MEM;           // tcp keeps the segment queued, other packets are lost
  }

  uint8_t *f = txq[wr % TX_SLOTS].data;
  memcpy(f, peer_mac, 6);
  memcpy(f + 6, our_mac, 6);
  f[12] = ETHTYPE_IP >> 8;
  f[13] = ETHTYPE_IP & 0xff;
  pbuf_copy_partial(p, f + 14, p->tot_len, 0);
  txq[wr % TX_SLOTS].len = 14 + p->tot_len;

  __atomic_store_n(&tx_wr, wr + 1, __ATOMIC_RELEASE);
  if(task) xTaskNotifyGive(task);
  return ERR_OK;
}

static err_t plip_netif_init(struct netif *netif) {
  netif->name[0] = 'p';
  netif->name[1] = 'l';
  netif->output = plip_output;
  netif->mtu = PLIP_MTU;
  netif->hwaddr_len = 6;
  memcpy(netif->hwaddr, our_mac, 6);
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_LINK_UP | NETIF_FLAG_UP;
  return ERR_OK;
}

// tcpip thread: segments that plip_output() refused are sent now
static void tx_resume(void *arg) {
  (void)arg;
  tcp_txnow();
}

// ARP request of the Amiga: we are the answer for every address except its own
static void arp_input(const uint8_t *f, uint16_t len) {
  if(len < 14 + SIZEOF_ETHARP_HDR) return;
  const struct etharp_hdr *arp = (const struct etharp_hdr *)(f + 14);
  if(arp->opcode != PP_HTONS(ARP_REQUEST) || arp->hwlen != 6 || arp->protolen != 4) return;

  ip4_addr_t sip, dip;
  memcpy(&sip, &arp->sipaddr, 4);
  memcpy(&dip, &arp->dipaddr, 4);
  if(ip4_addr_isany_val(sip) || ip4_addr_cmp(&dip, &peer_ip)) return;   // probe or its own address

  uint8_t *r = txq[tx_wr % TX_SLOTS].data;
  if(tx_wr - __atomic_load_n(&tx_rd, __ATOMIC_ACQUIRE) >= TX_SLOTS) return;

  memcpy(r, arp->shwaddr.addr, 6);
  memcpy(r + 6, our_mac, 6);
  r[12] = ETHTYPE_ARP >> 8;
  r[13] = ETHTYPE_ARP & 0xff;
  struct etharp_hdr *a = (struct etharp_hdr *)(r + 14);
  a->hwtype   = PP_HTONS(LWIP_IANA_HWTYPE_ETHERNET);
  a->proto    = PP_HTONS(ETHTYPE_IP);
  a->hwlen    = 6;
  a->protolen = 4;
  a->opcode   = PP_HTONS(ARP_REPLY);
  memcpy(a->shwaddr.addr, our_mac, 6);
  memcpy(&a->sipaddr, &dip, 4);
  memcpy(a->dhwaddr.addr, arp->shwaddr.addr, 6);
  memcpy(&a->dipaddr, &sip, 4);
  txq[tx_wr % TX_SLOTS].len = 14 + SIZEOF_ETHARP_HDR;
  __atomic_store_n(&tx_wr, tx_wr + 1, __ATOMIC_RELEASE);
}

// frame of the Amiga in rxbuf. plip_output() writes tx_wr under the core lock, so an ARP
// reply that is queued from here needs the core lock as well
static void frame_input(uint16_t len) {
  if(len < 14 || len > FRAME_MAX) { n_rx_bad++; return; }
  n_rx++;

  uint16_t type = (rxbuf[12] << 8) | rxbuf[13];
  if(type == ETHTYPE_ARP) {
    memcpy(peer_mac, rxbuf + 6, 6);
    LOCK_TCPIP_CORE();
    arp_input(rxbuf, len);
    UNLOCK_TCPIP_CORE();
  } else if(type == ETHTYPE_IP) {
    memcpy(peer_mac, rxbuf + 6, 6);
    struct pbuf *p = pbuf_alloc(PBUF_RAW, len - 14, PBUF_POOL);
    if(!p) { n_rx_bad++; return; }
    pbuf_take(p, rxbuf + 14, len - 14);
    if(plip_netif.input(p, &plip_netif) != ERR_OK) pbuf_free(p);
  }
  // everything else, e.g. the magic frames of the driver, is only counted
}

/* ---- task ---- */

static void service(void) {
  for(int round = 0; round < 8; round++) {
    uint16_t len;
    bool did = false;
    uint8_t st = net_status(&len);

    if(st & ST_RX_VALID) {
      if(len >= 14 && len <= FRAME_MAX) {
	net_read(len);
	frame_input(len);
      } else {                        // cannot happen, but never keep the Amiga waiting
	net_begin(SPI_NET_RELEASE);
	mcu_hw_spi_tx_u08(0);
	mcu_hw_spi_end();
	n_rx_bad++;
      }
      did = true;
    }

    if(!(st & ST_TX_VALID) && tx_rd != __atomic_load_n(&tx_wr, __ATOMIC_ACQUIRE)) {
      uint32_t rd = tx_rd;
      net_write(txq[rd % TX_SLOTS].data, txq[rd % TX_SLOTS].len);
      __atomic_store_n(&tx_rd, rd + 1, __ATOMIC_RELEASE);
      n_tx++;
      did = true;

      // room again: let tcp send what was refused (try again at the next frame if the mailbox is full)
      if(tx_blocked && tcpip_try_callback(tx_resume, NULL) == ERR_OK) tx_blocked = false;
    }

    if(!did) break;
  }
}

static void plip_task(void *arg) {
  (void)arg;

  while(!mcu_hw_tcpip_ready()) vTaskDelay(pdMS_TO_TICKS(100));

  LOCK_TCPIP_CORE();
  ip4_addr_t mask, gw;
  IP4_ADDR(&mask, 255, 255, 255, 0);
  ip4_addr_set_zero(&gw);
  netif_add(&plip_netif, &our_ip, &mask, &gw, NULL, plip_netif_init, tcpip_input);
  netif_set_link_up(&plip_netif);
  netif_set_up(&plip_netif);
  ppp_port_peer_set(PEER_PLIP, &plip_netif, &peer_ip, false);
  ppp_port_peer_up(PEER_PLIP);
  UNLOCK_TCPIP_CORE();

  net_config(true, inifile_config_get_int("plip", "loopback", 0) != 0);
  char ours[IP4ADDR_STRLEN_MAX], his[IP4ADDR_STRLEN_MAX];
  debugf("PLIP: up, companion %s, amiga %s", ip4addr_ntoa_r(&our_ip, ours, sizeof(ours)),
	 ip4addr_ntoa_r(&peer_ip, his, sizeof(his)));

  TickType_t last_stats = xTaskGetTickCount(), last_lan = last_stats;
  uint32_t last_rx = 0, last_tx = 0;

  for(;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(POLL_MS));
    service();

    TickType_t now = xTaskGetTickCount();
    if(now - last_lan >= pdMS_TO_TICKS(1000)) {    // wifi may get its address long after we started
      last_lan = now;
      LOCK_TCPIP_CORE();
      ppp_port_peer_poll(PEER_PLIP);
      UNLOCK_TCPIP_CORE();
    }
    if(now - last_stats >= pdMS_TO_TICKS(STATS_MS)) {
      last_stats = now;
      if(n_rx != last_rx || n_tx != last_tx) net_log_stats();   // only while something happens
      last_rx = n_rx; last_tx = n_tx;
    }
  }
}

void plip_port_init(void) {
  if(task || !inifile_config_get_int("plip", "enable", 0)) return;

  ip4_addr_set_u32(&our_ip,  lwip_htonl(inifile_config_get_ip("plip", "ip",   0xc0a8fe01)));  // 192.168.254.1
  ip4_addr_set_u32(&peer_ip, lwip_htonl(inifile_config_get_ip("plip", "peer", 0xc0a8fe02)));  // 192.168.254.2

  if(xTaskCreate(plip_task, "plip", 1024, NULL, configMAX_PRIORITIES-10, &task) != pdPASS)
    debugf("PLIP: cannot create task");
}

void plip_handle_event(void) {
  if(task) xTaskNotifyGive(task);
}
