// Host test: plip_port.c + forwarding hook with real lwIP (NO_SYS=1) and a software model of plip_host
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include <assert.h>
#include <stdbool.h>

#include <FreeRTOS.h>
#include <task.h>
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "lwip/ip4.h"
#include "lwip/icmp.h"
#include "lwip/tcp.h"
#include "lwip/inet_chksum.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"

/* ---- RTOS / platform stubs ---- */
static jmp_buf back;
static void (*task_fn)(void *);
static TickType_t ticks;
int xTaskCreate(void (*f)(void*), const char *n, int s, void *a, int p, TaskHandle_t *h) { task_fn = f; if(h) *h = (void*)1; return 1; }
uint32_t ulTaskNotifyTake(int c, TickType_t t) { longjmp(back, 1); }
int xTaskNotifyGive(TaskHandle_t t) { return 1; }
TickType_t xTaskGetTickCount(void) { return ticks; }
void vTaskDelay(TickType_t t) {}
u32_t sys_now(void) { return ticks; }
bool mcu_hw_tcpip_ready(void) { return true; }
int inifile_config_get_int(char *s, char *n, int def) { return !strcmp(n, "enable") ? 1 : def; }
uint32_t inifile_config_get_ip(char *s, char *n, uint32_t def) {
  return !strcmp(n, "peer") ? 0xc0a80132 /* 192.168.1.50 */ : def;
}

/* ---- model of plip_host (SPI target 5) ---- */
static struct {
  int pos; uint8_t cmd, reply;
  bool rx_valid, tx_valid, en, loop;
  uint8_t rx[2048]; int rx_len;
  uint8_t txbuf[2048]; int h_len_hi;
  uint8_t tx_out[8][2048]; int tx_out_len[8]; int tx_out_n;
  int hraddr;
} m;
void mcu_hw_spi_begin(void) { m.pos = -2; m.reply = 0xEE; }
void mcu_hw_spi_end(void) {}
unsigned char mcu_hw_spi_tx_u08(unsigned char b) {
  uint8_t ret = m.reply;
  if(m.pos == -2) { assert(b == 5); m.pos = -1; return ret; }
  if(m.pos == -1) { m.cmd = b; m.reply = (m.loop<<3)|(m.en<<2)|(m.tx_valid<<1)|m.rx_valid; m.pos = 0; return ret; }
  int cnt = m.pos++;
  switch(m.cmd) {
  case 0: m.reply = cnt == 0 ? (m.rx_len >> 8) & 7 : cnt == 1 ? m.rx_len & 0xff : 0; break;
  case 1: if(cnt == 0) { m.hraddr = 0; m.reply = 0; } else m.reply = m.rx[m.hraddr++]; break;
  case 2: if(cnt == 0) m.rx_valid = false; break;
  case 3: if(!m.tx_valid && cnt < 2048) m.txbuf[cnt] = b; break;
  case 4: if(cnt == 0) m.h_len_hi = b & 7;
          if(cnt == 1) {
            int len = (m.h_len_hi << 8) | b;
            if(!m.tx_valid && len) {
              memcpy(m.tx_out[m.tx_out_n], m.txbuf, len); m.tx_out_len[m.tx_out_n++] = len; m.tx_valid = true;
            }
          } break;
  case 5: if(cnt == 0) { m.en = b & 1; m.loop = (b >> 1) & 1; } break;
  case 6: m.reply = 0; break;
  default: m.reply = 0;
  }
  return ret;
}

#define printf_dbg(...)
#include "plip_port.c"
#include "hook_only.c"

/* ---- lanif interface ---- */
static struct netif lanif;
static uint8_t lanif_out[16][1600]; static int lanif_out_len[16], lanif_out_n;
static const uint8_t lan_mac[6] = {2,0,0,0,0,1}, host_mac[6] = {2,0,0,0,0,20}, amiga_mac[6] = {0x02,0xaa,0,0,0,0x50};
static err_t lan_linkoutput(struct netif *n, struct pbuf *p) {
  lanif_out_len[lanif_out_n] = pbuf_copy_partial(p, lanif_out[lanif_out_n], p->tot_len, 0); lanif_out_n++; return ERR_OK;
}
static err_t lan_init(struct netif *n) {
  n->name[0] = 'e'; n->name[1] = 'n'; n->output = etharp_output; n->linkoutput = lan_linkoutput;
  n->mtu = 1500; n->hwaddr_len = 6; memcpy(n->hwaddr, lan_mac, 6);
  n->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_LINK_UP;
  return ERR_OK;
}

/* ---- packet helpers ---- */
static int fails;
#define CHECK(c, msg) do { if(!(c)) { printf("FEHLER: %s (Zeile %d)\n", msg, __LINE__); fails++; } else printf("ok: %s\n", msg); } while(0)

static int mk_ip_icmp(uint8_t *f, const uint8_t *dmac, const uint8_t *smac, uint32_t src, uint32_t dst, int ttl, int plen) {
  memcpy(f, dmac, 6); memcpy(f + 6, smac, 6); f[12] = 8; f[13] = 0;
  uint8_t *ip = f + 14; int len = 20 + 8 + plen;
  memset(ip, 0, len);
  ip[0] = 0x45; ip[2] = len >> 8; ip[3] = len; ip[6] = 0x40; ip[8] = ttl; ip[9] = 1;
  uint32_t s = lwip_htonl(src), d = lwip_htonl(dst); memcpy(ip + 12, &s, 4); memcpy(ip + 16, &d, 4);
  uint8_t *ic = ip + 20; ic[0] = 8; ic[4] = 0x12; ic[5] = 0x34; ic[7] = 1;
  for(int i = 0; i < plen; i++) ic[8 + i] = i;
  uint16_t c = inet_chksum(ic, 8 + plen); ic[2] = c & 0xff; ic[3] = c >> 8;   // inet_chksum is network order
  c = inet_chksum(ip, 20); ip[10] = c & 0xff; ip[11] = c >> 8;
  return 14 + len;
}
static int mk_arp(uint8_t *f, int op, const uint8_t *smac, uint32_t sip, const uint8_t *dmac, uint32_t dip, const uint8_t *eth_dst) {
  memcpy(f, eth_dst, 6); memcpy(f + 6, smac, 6); f[12] = 8; f[13] = 6;
  uint8_t *a = f + 14; a[0] = 0; a[1] = 1; a[2] = 8; a[3] = 0; a[4] = 6; a[5] = 4; a[6] = 0; a[7] = op;
  memcpy(a + 8, smac, 6); uint32_t s = lwip_htonl(sip), d = lwip_htonl(dip); memcpy(a + 14, &s, 4);
  memcpy(a + 18, dmac, 6); memcpy(a + 24, &d, 4);
  return 14 + 28;
}

static uint16_t csum16(const uint8_t *d, int len, uint32_t sum) {
  for(int i = 0; i + 1 < len; i += 2) sum += (d[i] << 8) | d[i + 1];
  if(len & 1) sum += d[len - 1] << 8;
  while(sum >> 16) sum = (sum & 0xffff) + (sum >> 16);
  return sum;
}
// 0 if the tcp checksum (pseudo header) is valid, ip = ip header
static uint16_t tcp_chk(const uint8_t *ip) {
  int tl = ((ip[2] << 8) | ip[3]) - 20;
  uint32_t sum = csum16(ip + 12, 8, 0) + 6 + tl;
  return (uint16_t)~csum16(ip + 20, tl, sum);
}
static int mk_tcp_syn(uint8_t *f, const uint8_t *dmac, const uint8_t *smac, uint32_t src, uint32_t dst, int sport, int dport, uint32_t seq) {
  memcpy(f, dmac, 6); memcpy(f + 6, smac, 6); f[12] = 8; f[13] = 0;
  uint8_t *ip = f + 14; int len = 20 + 24;
  memset(ip, 0, len);
  ip[0] = 0x45; ip[2] = len >> 8; ip[3] = len; ip[6] = 0x40; ip[8] = 64; ip[9] = 6;
  uint32_t s = lwip_htonl(src), d = lwip_htonl(dst); memcpy(ip + 12, &s, 4); memcpy(ip + 16, &d, 4);
  uint8_t *t = ip + 20;
  t[0] = sport >> 8; t[1] = sport; t[2] = dport >> 8; t[3] = dport;
  t[4] = seq >> 24; t[5] = seq >> 16; t[6] = seq >> 8; t[7] = seq;
  t[12] = 6 << 4; t[13] = 0x02; t[14] = 0x20; t[15] = 0x00;       // SYN, window 8192
  t[20] = 2; t[21] = 4; t[22] = 0x05; t[23] = 0xb4;                // MSS 1460
  uint32_t sum = csum16(ip + 12, 8, 0) + 6 + 24;
  uint16_t c = (uint16_t)~csum16(t, 24, sum);
  t[16] = c >> 8; t[17] = c & 0xff;
  c = inet_chksum(ip, 20); ip[10] = c & 0xff; ip[11] = c >> 8;
  return 14 + len;
}
static err_t accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err) { (void)arg; (void)newpcb; (void)err; return ERR_OK; }
static void amiga_send(const uint8_t *f, int len) {   // Amiga -> core -> Companion
  memcpy(m.rx, f, len); m.rx_len = len; m.rx_valid = true; service();
}
static void amiga_fetch(void) { m.tx_valid = false; service(); }
static void lan_inject(const uint8_t *f, int len) {
  struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL); pbuf_take(p, f, len);
  if(lanif.input(p, &lanif) != ERR_OK) pbuf_free(p);
}
static uint32_t ip_of(const uint8_t *ip, int off) { uint32_t v; memcpy(&v, ip + off, 4); return lwip_ntohl(v); }

int main(void) {
  static const uint8_t bcast[6] = {255,255,255,255,255,255}, zero[6] = {0};
  lwip_init();
  // like the Pico: the lan (wifi) has no address yet when the plip task starts
  ip4_addr_t a, mk, gw; IP4_ADDR(&a, 192,168,1,10); IP4_ADDR(&mk, 255,255,255,0); IP4_ADDR(&gw, 192,168,1,1);
  netif_add(&lanif, IP4_ADDR_ANY4, IP4_ADDR_ANY4, IP4_ADDR_ANY4, NULL, lan_init, netif_input);
  netif_set_default(&lanif); netif_set_up(&lanif); netif_set_link_up(&lanif);

  lanif_out_n = 0;
  plip_port_init();
  CHECK(task_fn != NULL, "task created when [plip] enable=1");
  if(!setjmp(back)) task_fn(NULL);
  CHECK(m.en && !m.loop, "core configured: enable, no loopback");
  CHECK(lanif_out_n == 0 && lanif.input != lan_input, "lan has no address yet: nothing hooked, nothing announced");
  netif_set_addr(&lanif, &a, &mk, &gw);                   // wifi got its address
  lanif_out_n = 0;
  ppp_port_peer_poll(PEER_PLIP);                           // what plip_task does every second
  CHECK(lanif_out_n == 1 && lanif_out[0][12] == 8 && lanif_out[0][13] == 6 && lanif.input == lan_input,
	"lan got its address later: hooked and gratuitous ARP for the peer");
  lanif_out_n = 0;
  ppp_port_peer_poll(PEER_PLIP);
  CHECK(lanif_out_n == 0, "next poll: no repeated announcement");

  uint8_t f[1600]; int n;

  // 1. Amiga asks for its gateway: we answer for any address except its own
  n = mk_arp(f, 1, amiga_mac, 0xc0a80132, zero, 0xc0a80101, bcast);
  amiga_send(f, n);
  CHECK(m.tx_out_n == 1 && m.tx_out_len[0] == 42 && !memcmp(m.tx_out[0], amiga_mac, 6) && !memcmp(m.tx_out[0] + 6, our_mac, 6)
	&& m.tx_out[0][21] == 2 && ip_of(m.tx_out[0] + 14, 14) == 0xc0a80101 && !memcmp(m.tx_out[0] + 22, our_mac, 6),
	"ARP reply for 192.168.1.1 with our MAC");
  CHECK(!m.tx_valid == false, "frame waits in the core until the Amiga fetches it");
  amiga_fetch();

  // 2. ARP probe for its own address and from 0.0.0.0 is not answered
  n = mk_arp(f, 1, amiga_mac, 0, zero, 0xc0a80132, bcast); amiga_send(f, n);
  CHECK(m.tx_out_n == 1 && !m.tx_valid, "no answer to an ARP probe for its own address");

  // 3. Amiga pings a lanif host: forwarded, lanif side asks ARP first
  n = mk_ip_icmp(f, our_mac, amiga_mac, 0xc0a80132, 0xc0a80114, 64, 32); amiga_send(f, n);
  CHECK(lanif_out_n == 1 && lanif_out[0][12] == 8 && lanif_out[0][13] == 6 && lanif_out[0][21] == 1, "lanif: ARP request for 192.168.1.20");
  n = mk_arp(f, 2, host_mac, 0xc0a80114, lan_mac, 0xc0a8010a, lan_mac); lan_inject(f, n);
  CHECK(lanif_out_n == 2 && lanif_out[1][13] == 0 && !memcmp(lanif_out[1], host_mac, 6), "lanif: ping forwarded to the host after ARP");
  uint8_t *ip = lanif_out[1] + 14;
  CHECK(ip_of(ip, 12) == 0xc0a80132 && ip_of(ip, 16) == 0xc0a80114 && ip[8] == 63, "src unchanged, ttl 64 -> 63");
  CHECK(inet_chksum(ip, 20) == 0 && inet_chksum(ip + 20, lanif_out_len[1] - 34) == 0, "ip and icmp checksums valid");

  // 4. wrong source address, ttl 1: not forwarded
  lanif_out_n = 0;
  n = mk_ip_icmp(f, our_mac, amiga_mac, 0xc0a80163, 0xc0a80114, 64, 32); amiga_send(f, n);
  n = mk_ip_icmp(f, our_mac, amiga_mac, 0xc0a80132, 0xc0a80114, 1, 32); amiga_send(f, n);
  CHECK(lanif_out_n == 0, "foreign source address and ttl 1 are dropped");

  // 5. host pings the Amiga: proxy ARP first, then forwarded to plip with the learned MAC
  n = mk_arp(f, 1, host_mac, 0xc0a80114, zero, 0xc0a80132, bcast); lan_inject(f, n);
  CHECK(lanif_out_n == 1 && lanif_out[0][21] == 2 && ip_of(lanif_out[0] + 14, 14) == 0xc0a80132 && !memcmp(lanif_out[0] + 22, lan_mac, 6),
	"lanif: proxy ARP reply for the Amiga");
  int before = m.tx_out_n;
  n = mk_ip_icmp(f, lan_mac, host_mac, 0xc0a80114, 0xc0a80132, 64, 100); lan_inject(f, n);
  service();
  CHECK(m.tx_out_n == before + 1, "plip: frame for the Amiga queued");
  uint8_t *o = m.tx_out[m.tx_out_n - 1];
  CHECK(!memcmp(o, amiga_mac, 6) && !memcmp(o + 6, our_mac, 6) && o[12] == 8 && o[13] == 0, "ethernet header: Amiga MAC learned, our MAC");
  CHECK(m.tx_out_len[m.tx_out_n - 1] == 14 + 20 + 8 + 100 && o[22] == 63 && inet_chksum(o + 14, 20) == 0
	&& inet_chksum(o + 34, 108) == 0, "length, ttl 63, checksums");
  amiga_fetch();

  // 6. ping to the companion's address on the link is answered
  before = m.tx_out_n;
  n = mk_ip_icmp(f, our_mac, amiga_mac, 0xc0a80132, 0xc0a8fe01, 64, 16); amiga_send(f, n);
  o = m.tx_out[m.tx_out_n - 1];
  CHECK(m.tx_out_n == before + 1 && o[34] == 0 && ip_of(o + 14, 12) == 0xc0a8fe01 && ip_of(o + 14, 16) == 0xc0a80132,
	"echo reply from the companion");
  amiga_fetch();

  // 6b. ping and tcp connect to the companion's address on the lan (what the user does: ping/telnet/ftp 192.168.1.10)
  before = m.tx_out_n;
  n = mk_ip_icmp(f, our_mac, amiga_mac, 0xc0a80132, 0xc0a8010a, 64, 16); amiga_send(f, n);
  o = m.tx_out_n > before ? m.tx_out[m.tx_out_n - 1] : NULL;
  CHECK(o && o[34] == 0 && ip_of(o + 14, 12) == 0xc0a8010a && ip_of(o + 14, 16) == 0xc0a80132, "echo reply from the companion's lan address");
  amiga_fetch();

  static struct tcp_pcb *lis;
  lis = tcp_new(); tcp_bind(lis, IP_ANY_TYPE, 23); lis = tcp_listen(lis);
  tcp_accept(lis, accept_cb);
  before = m.tx_out_n;
  n = mk_tcp_syn(f, our_mac, amiga_mac, 0xc0a80132, 0xc0a8010a, 1025, 23, 1000);
  amiga_send(f, n);
  o = m.tx_out_n > before ? m.tx_out[m.tx_out_n - 1] : NULL;
  CHECK(o && o[23] == 6 && (o[34 + 13] & 0x12) == 0x12 && ip_of(o + 14, 12) == 0xc0a8010a && ip_of(o + 14, 16) == 0xc0a80132,
	"SYN to the companion's lan address: SYN/ACK comes back through plip");
  CHECK(o && inet_chksum(o + 14, 20) == 0 && tcp_chk(o + 14) == 0, "SYN/ACK checksums valid");
  amiga_fetch();

  // 7. tx queue: core busy -> frames queue (3) and the rest is refused, afterwards all drain
  before = m.tx_out_n;
  n = mk_ip_icmp(f, lan_mac, host_mac, 0xc0a80114, 0xc0a80132, 64, 8);
  for(int i = 0; i < 6; i++) lan_inject(f, n);
  service();
  CHECK(m.tx_out_n == before + 1 && tx_drops == 3 && !tx_blocked, "core busy: one frame handed over, queue of 3 full, 3 refused, tcp told to retry");
  for(int i = 0; i < 6; i++) amiga_fetch();
  CHECK(m.tx_out_n == before + 3, "the 2 frames still queued drain after each fetch");
  amiga_fetch();

  // 8. magic / unknown frames are consumed, short frames counted
  uint8_t mg[64] = {0}; memcpy(mg, bcast, 6); memcpy(mg + 6, amiga_mac, 6); mg[12] = 0xff; mg[13] = 0xfe;
  amiga_send(mg, 60);
  CHECK(!m.rx_valid, "magic frame released");
  amiga_send(mg, 10);
  CHECK(!m.rx_valid && n_rx_bad == 1, "short frame released and counted");

  printf("%s (%d Fehler)\n", fails ? "FAIL" : "PASS", fails);
  return fails != 0;
}
