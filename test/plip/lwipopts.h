#pragma once
#define NO_SYS 1
#define LWIP_TIMERS 1
#define LWIP_SOCKET 0
#define LWIP_NETCONN 0
#define LWIP_IPV4 1
#define LWIP_IPV6 0
#define LWIP_ARP 1
#define LWIP_ETHERNET 1
#define LWIP_ICMP 1
#define LWIP_TCP 1
#define LWIP_UDP 1
#define LWIP_DHCP 0
#define LWIP_DNS 0
#define IP_FORWARD 0
#define MEM_ALIGNMENT 4
#define MEM_SIZE 65536
#define PBUF_POOL_SIZE 32
#define PBUF_POOL_BUFSIZE 256
#define MEMP_NUM_PBUF 32
#define LWIP_STATS 0
#define LWIP_HOOK_IP4_ROUTE_SRC(src, dest) ppp_port_route(dest)
#define LWIP_HOOK_IP4_INPUT(p, inp) ppp_port_ip4_input(p, inp)
#define LWIP_HOOK_FILENAME "hooks.h"
#define LWIP_CHECKSUM_ON_COPY 0
#define LWIP_NETIF_STATUS_CALLBACK 0
