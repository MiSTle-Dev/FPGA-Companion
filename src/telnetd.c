/*
 * telnetd — remote shell/console mirror on TCP port 23.
 */
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "debug.h"
#include "telnetd.h"

#if defined(PICO_RP2040) || defined(PICO_RP2350)
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#endif

#define TELNET_PORT     23
#define TEE_BUF_LEN     2048
#define SHELL_COMMAND_LEN 256
#define TELNETD_STACK_WORDS 768

#if !defined(PICO_RP2040) && !defined(PICO_RP2350)
extern void shell_exe_cmd(unsigned char *cmd, int len);
extern int shell_set_echo(bool enabled);
#endif

/* ---- console tee ring (written from any thread that prints) ------------- */
static char     s_tee[TEE_BUF_LEN];
static volatile uint32_t s_tee_wr;        /* total bytes ever written  */
static uint32_t s_tee_rd;                 /* telnet thread's position  */
static SemaphoreHandle_t s_tee_lock;
static volatile bool s_client_up;

static void tee_write(const void *data, size_t size)
{
    const char *p = data;

    if (s_client_up && s_tee_lock && data && !xPortIsInsideInterrupt()) {
        static char prev;
        xSemaphoreTake(s_tee_lock, portMAX_DELAY);
        for (size_t i = 0; i < size; i++) {
            if (p[i] == '\n' && prev != '\r')
                s_tee[s_tee_wr++ % TEE_BUF_LEN] = '\r';
            s_tee[s_tee_wr++ % TEE_BUF_LEN] = p[i];
            prev = p[i];
        }
        xSemaphoreGive(s_tee_lock);
    }
}

#if !defined(PICO_RP2040) && !defined(PICO_RP2350)
/* BL616 routes libc output through bflb_console_write(). */
extern ssize_t __real_bflb_console_write(const void *data, size_t size);

ssize_t __wrap_bflb_console_write(const void *data, size_t size)
{
    tee_write(data, size);
    return __real_bflb_console_write(data, size);
}
#else
static void pico_tee_out_chars(const char *data, int size)
{
    tee_write(data, (size_t)size);
}

static stdio_driver_t pico_tee_driver = {
    .out_chars = pico_tee_out_chars,
};
#endif

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

static void telnetd_execute_command(int fd, uint8_t *command, size_t *length)
{
#if defined(PICO_RP2040) || defined(PICO_RP2350)
    (void)command;
    tn_puts(fd, "\r\nRP2040 has no command shell\r\n");
    *length = 0;
#else
    /* an empty line still goes to the shell: that is what redraws the prompt */
    command[(*length)++] = '\r';
    command[(*length)++] = '\n';
    command[*length] = '\0';
    /* the shell would echo the whole line again; we already echoed it live */
    shell_set_echo(false);
    shell_exe_cmd(command, *length);
    shell_set_echo(true);
    *length = 0;
#endif
}

/* ---- session ------------------------------------------------------------- */
static void session(int fd)
{
    uint8_t command[SHELL_COMMAND_LEN];
    size_t command_length = 0;
    bool telnet_command = false;
    bool telnet_option = false;

    s_peer_dead = false;
    /* char-at-a-time: WILL ECHO, WILL SGA, DO SGA */
    static const uint8_t nego[] = { 255, 251, 1, 255, 251, 3, 255, 253, 3 };
    tn_send(fd, nego, sizeof(nego));
#if defined(PICO_RP2040) || defined(PICO_RP2350)
    tn_puts(fd, "\r\nRP2040 console monitor\r\n");
#else
    tn_puts(fd, "\r\nBL616 shell monitor\r\n");
#endif

    xSemaphoreTake(s_tee_lock, portMAX_DELAY);
    s_tee_rd = s_tee_wr;               /* drop the backlog, mirror from here on */
    xSemaphoreGive(s_tee_lock);

    telnetd_execute_command(fd, command, &command_length);

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
        if (r > 0) {
            if (telnet_option) {
                telnet_option = false;
            } else if (telnet_command) {
                telnet_command = false;
                telnet_option = ch >= 251 && ch <= 254;
            } else if (ch == 255) {
                telnet_command = true;
            } else if (ch == '\r' || ch == '\n') {
                telnetd_execute_command(fd, command, &command_length);
            } else if (ch == '\b' || ch == 127) {
                if (command_length > 0) {
                    command_length--;
                    tn_puts(fd, "\b \b");
                }
            } else if (ch >= 32 && ch < 127 && command_length < sizeof(command) - 3) {
                command[command_length++] = ch;
                /* we announced WILL ECHO, so the client shows nothing itself */
                tn_send(fd, &ch, 1);
            }
        }
        {
            /* drain the console tee */
            for (;;) {
                char out[128];
                int len = 0;
                xSemaphoreTake(s_tee_lock, portMAX_DELAY);
                if (s_tee_wr - s_tee_rd > TEE_BUF_LEN)
                    s_tee_rd = s_tee_wr - TEE_BUF_LEN;   /* dropped */
                while (s_tee_rd < s_tee_wr && len < (int)sizeof(out)) {
                    out[len++] = s_tee[s_tee_rd % TEE_BUF_LEN];
                    s_tee_rd++;
                }
                xSemaphoreGive(s_tee_lock);
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
        vTaskDelay(pdMS_TO_TICKS(200));

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
            vTaskDelay(pdMS_TO_TICKS(500));
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
    s_tee_lock = xSemaphoreCreateMutex();
    if (!s_tee_lock) {
        debugf("TELNET: failed to create tee mutex");
        return;
    }

#if defined(PICO_RP2040) || defined(PICO_RP2350)
    stdio_set_driver_enabled(&pico_tee_driver, true);
#endif

    BaseType_t created = xTaskCreate(telnetd_thread, "telnetd", TELNETD_STACK_WORDS,
                NULL, configMAX_PRIORITIES - 3, NULL);
    if (created != pdPASS)
        debugf("TELNET: failed to create server task");
}
