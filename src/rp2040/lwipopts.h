#pragma once

#include <stdint.h>

// See https://www.nongnu.org/lwip/2_1_x/group__lwip__opts.html for details.
// Also, see https://gitlab.com/slimhazard/picow_http/-/wikis/Configuring-lwipopts.h

#define NO_SYS 0

/*
 * The default for MAX_CONCURRENT_CX_HINT is a conservative value, so that
 * the default memory footprint of picow-http is small.
 *
 * This is a common value among most browsers for the maximum number of
 * client connections to a single server IP address. It should usually be
 * sufficient if a single client application connects to the server.
 *
 * Increase this value if more concurrent client connections are required.
 */
#ifndef MAX_CONCURRENT_CX_HINT
#define MAX_CONCURRENT_CX_HINT      6
#endif

#define NUM_SERVER_HINT             1

// allow override in some examples
#ifndef LWIP_SOCKET
#define LWIP_SOCKET                 1
#endif
#if PICO_CYW43_ARCH_POLL
#define MEM_LIBC_MALLOC             1
#else
// MEM_LIBC_MALLOC is incompatible with non polling versions
#define MEM_LIBC_MALLOC             0
#endif
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    (MAX_CONCURRENT_CX_HINT * TCP_MSS)
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              24
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    1
#define TCP_MSS                     1460
#define TCP_WND                     (MAX_CONCURRENT_CX_HINT * TCP_MSS)
#define TCP_SND_BUF                 (MAX_CONCURRENT_CX_HINT * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define TCP_LISTEN_BACKLOG          1
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_TCP_PCB            (MAX_CONCURRENT_CX_HINT)
#define MEMP_NUM_TCP_PCB_LISTEN     (NUM_SERVER_HINT)

#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETCONN                0
#define MEM_STATS                   0
#define SYS_STATS                   0
#define MEMP_STATS                  0
#define LINK_STATS                  0
// #define ETH_PAD_SIZE                2
#define LWIP_CHKSUM_ALGORITHM       3
#define LWIP_DHCP                   1
#define LWIP_IPV4                   1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_DNS                    1
#define LWIP_TCP_KEEPALIVE          1
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define DHCP_DOES_ARP_CHECK         0
#define LWIP_DHCP_DOES_ACD_CHECK    0

#define LWIP_DHCP_GET_NTP_SRV       1
#define LWIP_DHCP_MAX_NTP_SERVERS   2
void sntp_set_system_time(uint32_t sec);
#define SNTP_SET_SYSTEM_TIME        sntp_set_system_time
#define MEMP_NUM_SYS_TIMEOUT        (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 2)

#ifndef NDEBUG
#define LWIP_DEBUG                  1
#define LWIP_STATS                  1
#define LWIP_STATS_DISPLAY          1
#endif

#define ETHARP_DEBUG                LWIP_DBG_OFF
#define NETIF_DEBUG                 LWIP_DBG_OFF
#define PBUF_DEBUG                  LWIP_DBG_OFF
#define API_LIB_DEBUG               LWIP_DBG_OFF
#define API_MSG_DEBUG               LWIP_DBG_OFF
#define SOCKETS_DEBUG               LWIP_DBG_OFF
#define ICMP_DEBUG                  LWIP_DBG_OFF
#define INET_DEBUG                  LWIP_DBG_OFF
#define IP_DEBUG                    LWIP_DBG_OFF
#define IP_REASS_DEBUG              LWIP_DBG_OFF
#define RAW_DEBUG                   LWIP_DBG_OFF
#define MEM_DEBUG                   LWIP_DBG_OFF
#define MEMP_DEBUG                  LWIP_DBG_OFF
#define SYS_DEBUG                   LWIP_DBG_OFF
#define TCP_DEBUG                   LWIP_DBG_OFF
#define TCP_INPUT_DEBUG             LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG            LWIP_DBG_OFF
#define TCP_RTO_DEBUG               LWIP_DBG_OFF
#define TCP_CWND_DEBUG              LWIP_DBG_OFF
#define TCP_WND_DEBUG               LWIP_DBG_OFF
#define TCP_FR_DEBUG                LWIP_DBG_OFF
#define TCP_QLEN_DEBUG              LWIP_DBG_OFF
#define TCP_RST_DEBUG               LWIP_DBG_OFF
#define UDP_DEBUG                   LWIP_DBG_OFF
#define TCPIP_DEBUG                 LWIP_DBG_OFF
#define PPP_DEBUG                   LWIP_DBG_OFF
#define SLIP_DEBUG                  LWIP_DBG_OFF
#define DHCP_DEBUG                  LWIP_DBG_OFF
#define SNTP_DEBUG                  LWIP_DBG_OFF

#define TCPIP_THREAD_STACKSIZE 1024
#define DEFAULT_THREAD_STACKSIZE 1024
#define TCPIP_THREAD_PRIO 4
#define DEFAULT_RAW_RECVMBOX_SIZE 8
#define TCPIP_MBOX_SIZE 8
#define LWIP_TIMEVAL_PRIVATE 0

// not necessary, can be done either way
#define LWIP_TCPIP_CORE_LOCKING_INPUT 1

// ping_thread sets socket receive timeout, so enable this feature
#define LWIP_SO_RCVTIMEO 1

// This section enables HTTPD server with SSI, SGI
// and tells server which converted HTML files to use.
// See: https://www.nongnu.org/lwip/2_0_x/group__httpd__opts.html
#define LWIP_HTTPD 1
#define LWIP_HTTPD_SSI 0
#define LWIP_HTTPD_CGI 0
#define LWIP_HTTPD_SSI_INCLUDE_TAG 0
// #define HTTPD_FSDATA_FILE "htmldata.c"

#define LWIP_HTTPD_DYNAMIC_HEADERS 1
#define LWIP_HTTPD_SUPPORT_EXTSTATUS 1
#define LWIP_HTTPD_SUPPORT_11_KEEPALIVE 1
#define HTTPD_LIMIT_SENDING_TO_2MSS 0
#define LWIP_HTTPD_FILE_EXTENSION 1

#define LWIP_HTTPD_CUSTOM_FILES 1
// Set this to 1 and provide the functions:
// "int fs_open_custom(struct fs_file *file, const char *name)"
// Called first for every opened file to allow opening files that are not included in fsdata(_custom).c
// "void fs_close_custom(struct fs_file *file)"
// Called to free resources allocated by fs_open_custom().

#define LWIP_HTTPD_DYNAMIC_FILE_READ 1
// Set this to 1 to support fs_read() to dynamically read file data.
// Without this (default=off), only one-block files are supported, and the contents must be ready after fs_open().

#define HTTPD_DEBUG LWIP_DBG_OFF
