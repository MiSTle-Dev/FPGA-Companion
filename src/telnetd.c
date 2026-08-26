/*
 * telnetd — remote console/menu mirror on TCP port 23.
 */
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "usb_osal.h"
#include "usb_config.h"
#include "debug.h"
#include "telnetd.h"

#define TELNET_PORT     23
#define TEE_BUF_LEN     2048
#define SHELL_PRINT_LEN 256

extern int shell_set_print(void (*shell_printf)(char *fmt, ...));

/* ---- console tee ring (written by shell output from any thread) ---------- */
static char     s_tee[TEE_BUF_LEN];
static volatile uint32_t s_tee_wr;        /* total lines ever written  */
static uint32_t s_tee_rd;                 /* telnet thread's position  */
static usb_osal_mutex_t s_tee_lock;
static volatile bool s_client_up;

static void telnetd_console_tee(const char *text)
{
    if (!s_client_up || !s_tee_lock)
        return;
    usb_osal_mutex_take(s_tee_lock);
    while (*text)
        s_tee[s_tee_wr++ % TEE_BUF_LEN] = *text++;
    usb_osal_mutex_give(s_tee_lock);
}

static void telnetd_vprintf(const char *fmt, va_list ap)
{
    char buf[SHELL_PRINT_LEN];
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);

    if (len <= 0)
        return;
    if (len >= (int)sizeof(buf))
        len = sizeof(buf) - 1;
    buf[len] = 0;
    telnetd_console_tee(buf);
}

void telnetd_printf(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    telnetd_vprintf(fmt, ap);
    va_end(ap);
}

static void telnetd_shell_printf(char *fmt, ...)
{
    va_list ap;
    va_list copy;

    va_start(ap, fmt);
    va_copy(copy, ap);
    vprintf(fmt, copy);
    va_end(copy);

    telnetd_vprintf(fmt, ap);
    va_end(ap);
}

void telnetd_monitor_shell(void)
{
    shell_set_print(telnetd_shell_printf);
}

/* ---- helpers ------------------------------------------------------------- */
/* Set when a send errors or times out (SO_SNDTIMEO): the peer vanished
 * without closing (crashed client, port scanner) and its window filled.
 * tn_send goes no-op and session() bails, so the next client can connect. */
static bool s_peer_dead;

static int tn_send(int fd, const void *buf, int len)
{
    const char *p = buf;
    while (len > 0 && !s_peer_dead) {
        int n = lwip_send(fd, p, len, 0);
        if (n <= 0) {
            s_peer_dead = true;
            return -1;
        }
        p += n;
        len -= n;
    }
    return s_peer_dead ? -1 : 0;
}

static void tn_puts(int fd, const char *s)
{
    tn_send(fd, s, (int)strlen(s));
}

/* ---- session ------------------------------------------------------------- */
static void session(int fd)
{
    s_peer_dead = false;
    /* char-at-a-time: WILL ECHO, WILL SGA, DO SGA */
    static const uint8_t nego[] = { 255, 251, 1, 255, 251, 3, 255, 253, 3 };
    tn_send(fd, nego, sizeof(nego));
    tn_puts(fd, "\r\nBL616 shell monitor\r\n");

    /* start in console mode: replay the on-screen backlog */
    {
        usb_osal_mutex_take(s_tee_lock);
        s_tee_rd = s_tee_wr;               /* live from here on */
        usb_osal_mutex_give(s_tee_lock);
    }

    for (;;) {
        if (s_peer_dead)
            return;                        /* send timed out/failed */
        /* input (non-blocking-ish: 50 ms poll via SO_RCVTIMEO) */
        uint8_t ch;
        int r = lwip_recv(fd, &ch, 1, 0);
        if (r == 0)
            return;                        /* closed */
        if (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN)
            return;                        /* reset/keepalive-reaped, not the 50 ms poll */
        {
            /* drain the console tee */
            for (;;) {
                char out[128];
                int len = 0;
                usb_osal_mutex_take(s_tee_lock);
                if (s_tee_wr - s_tee_rd > TEE_BUF_LEN)
                    s_tee_rd = s_tee_wr - TEE_BUF_LEN;   /* dropped */
                while (s_tee_rd < s_tee_wr && len < (int)sizeof(out)) {
                    out[len++] = s_tee[s_tee_rd % TEE_BUF_LEN];
                    s_tee_rd++;
                }
                usb_osal_mutex_give(s_tee_lock);
                if (!len)
                    break;
                tn_send(fd, out, len);
            }
        }
    }
}

static void telnetd_thread(void *arg)
{
    (void)arg;
    /* tcpip_init() returns before the tcpip thread has run lwip_init();
     * touching the socket API before that wins a race into uninitialized
     * memp pools and corrupts the stack (manifested as boot hangs around
     * DHCP time). netif_default appearing means core init long finished. */
    while (netif_default == NULL)
        usb_osal_msleep(200);

    int lfd = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (lfd < 0)
        return;
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = PP_HTONS(TELNET_PORT);
    sa.sin_addr.s_addr = PP_HTONL(INADDR_ANY);
    int one = 1;
    lwip_setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    if (lwip_bind(lfd, (struct sockaddr *)&sa, sizeof(sa)) < 0)
        return;
    lwip_listen(lfd, 1);
    debugf("TELNET: LISTENING ON PORT 23");

    for (;;) {
        int fd = lwip_accept(lfd, NULL, NULL);
        if (fd < 0) {
            usb_osal_msleep(500);
            continue;
        }
        struct timeval tv = { .tv_sec = 0, .tv_usec = 50 * 1000 };
        lwip_setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        /* A peer that vanishes without closing (port scan, killed nc) stops
         * ACKing; once its window fills an untimed send blocks this thread
         * forever and the single-session server is wedged until reboot. */
        struct timeval stv = { .tv_sec = 3, .tv_usec = 0 };
        lwip_setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));
        /* Keepalive reaps half-open sessions that go idle (console mode
         * with no log traffic never sends, so SO_SNDTIMEO alone can't). */
        int idle = 10, intvl = 5, cnt = 3;
        lwip_setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
        lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
        s_client_up = true;
        debugf("TELNET: CLIENT CONNECTED");
        session(fd);
        s_client_up = false;
        lwip_close(fd);
        debugf("TELNET: CLIENT DISCONNECTED");
    }
}

void telnetd_init(void)
{
    s_tee_lock = usb_osal_mutex_create();
    /* Same priority band as the other app threads — an over-high priority
     * here (above the tcpip thread) is part of how the init race bites. */
    usb_osal_thread_create("telnetd", 3072, CONFIG_USBHOST_PSC_PRIO + 1,
                           telnetd_thread, NULL);
}
