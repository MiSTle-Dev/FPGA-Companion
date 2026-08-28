/*
 * ftpd — minimal FTP server for the SD card volume (port 21).
 *
 * Purpose: copy disk images on/off the SD card over the LAN with a
 * mature client (Cyberduck, FileZilla, lftp) instead of shuttling the
 * card between machines. Plaintext, LAN-threat-model; any USER/PASS is
 * accepted. Passive mode only (every modern client defaults to PASV;
 * PORT gets 502).
 *
 * Implemented: USER PASS SYST FEAT PWD CWD CDUP TYPE PASV LIST NLST RETR
 * STOR DELE MKD RMD RNFR RNTO SIZE REST NOOP QUIT. All paths are relative to
 * CARD_MOUNTPOINT and accessed directly via FatFs, serialized with
 * sdc_lock()/sdc_unlock() the same way sdc.c guards the card. Files
 * currently mounted as a disk/rom image on any drive are protected from
 * STOR/DELE/RNTO (550).
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>

#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "sdc.h"
#if defined(PICO_RP2040) || defined(PICO_RP2350)
#include "tusb_asix/asix_host.h"
#endif

#include "ftpd.h"

#define FTP_PORT             21
#if defined(PICO_RP2040)
#define XFER_CHUNK           2048
#else
#define XFER_CHUNK           4096
#endif
#define CWD_MAX              128
#define FPATH_MAX            (CWD_MAX + 16)   /* CARD_MOUNTPOINT + '/' + relative path */
#define FTPD_STACK_WORDS     1024
#define FTPS_STACK_WORDS     3072

/* Cyberduck (and friends) open a SEPARATE control connection per transfer,
 * so a single-client server times out their uploads. Small session pool;
 * each session uses its own on-stack FIL/DIR, card access is serialized
 * via sdc_lock()/sdc_unlock(). */
#define MAX_SESSIONS 3

typedef struct {
    int     ctl;                      /* control connection        */
    int     pasv;                     /* passive listener          */
    char    cwd[CWD_MAX];             /* "" = volume root          */
    char    rnfr[CWD_MAX];            /* pending RNFR source       */
    uint32_t restart_at;               /* pending REST offset, 0 = none */
    uint8_t xbuf[XFER_CHUNK];
    volatile bool in_use;
} ftps_t;

static ftps_t s_sess[MAX_SESSIONS];

/* build the absolute FatFs path for a volume-root-relative FTP path */
static void full_path(const char *rel, char *out, size_t cap)
{
    if (rel[0])
        snprintf(out, cap, "%s/%s", CARD_MOUNTPOINT, rel);
    else
        snprintf(out, cap, "%s", CARD_MOUNTPOINT);
}

/* true if 'path' (volume-root-relative) is currently mounted as a
 * disk/rom image on any drive, and thus must not be touched */
static bool disk_path_mounted(const char *path)
{
    for (int d = 0; d < MAX_DRIVES + MAX_IMAGES; d++) {
        char *name = sdc_get_image_name(d);
        if (!name)
            continue;
        char *cwd = sdc_get_cwd(d);
        char full[FPATH_MAX];
        snprintf(full, sizeof(full), "%s/%s", cwd ? cwd : CARD_MOUNTPOINT, name);

        const char *rel = full;
        size_t mlen = strlen(CARD_MOUNTPOINT);
        if (!strncmp(rel, CARD_MOUNTPOINT, mlen))
            rel += mlen;
        while (*rel == '/')
            rel++;

        if (!strcasecmp(rel, path))
            return true;
    }
    return false;
}

static void reply(ftps_t *fs, const char *s)
{
    lwip_send(fs->ctl, s, (int)strlen(s), 0);
    lwip_send(fs->ctl, "\r\n", 2, 0);
}

static void replyf(ftps_t *fs, const char *fmt, ...)
{
    char b[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    reply(fs, b);
}

static bool send_all(int fd, const void *buf, size_t len)
{
    const uint8_t *ptr = (const uint8_t *)buf;
    while (len) {
        int sent = lwip_send(fd, ptr, (int)len, 0);
        if (sent <= 0) {
            debugf("FTP: data send failed (fd=%d, sent=%d, remaining=%lu, errno=%d)",
                   fd, sent, (unsigned long)len, errno);
            return false;
        }
        ptr += sent;
        len -= (size_t)sent;
    }
    return true;
}

/* Resolve an FTP path argument against the cwd into out (volume-root
 * relative, no leading slash). Handles absolute paths, "." and "..".
 * Returns false on overflow or attempts to escape the root. */
static bool resolve(ftps_t *fs, const char *arg, char *out, size_t cap)
{
    char tmp[CWD_MAX * 2];
    if (arg[0] == '/')
        snprintf(tmp, sizeof(tmp), "%s", arg + 1);
    else if (fs->cwd[0])
        snprintf(tmp, sizeof(tmp), "%s/%s", fs->cwd, arg);
    else
        snprintf(tmp, sizeof(tmp), "%s", arg);

    /* normalize component by component */
    char comp[CWD_MAX];
    size_t olen = 0;
    out[0] = 0;
    const char *p = tmp;
    while (*p) {
        const char *slash = strchr(p, '/');
        size_t n = slash ? (size_t)(slash - p) : strlen(p);
        if (n >= sizeof(comp))
            return false;
        memcpy(comp, p, n);
        comp[n] = 0;
        p += n + (slash ? 1 : 0);
        if (!n || !strcmp(comp, "."))
            continue;
        if (!strcmp(comp, "..")) {
            char *ls = strrchr(out, '/');
            if (ls)
                *ls = 0;
            else
                out[0] = 0;
            olen = strlen(out);
            continue;
        }
        if (olen + n + 2 > cap)
            return false;
        if (olen)
            out[olen++] = '/';
        memcpy(out + olen, comp, n + 1);
        olen += n;
    }
    return true;
}

/* ---- passive data connections -------------------------------------------- */
static void pasv_close(ftps_t *fs)
{
    if (fs->pasv >= 0) {
        lwip_close(fs->pasv);
        fs->pasv = -1;
    }
}

static void cmd_pasv(ftps_t *fs)
{
    pasv_close(fs);
    fs->pasv = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (fs->pasv < 0) {
        reply(fs, "425 Can't open data connection.");
        return;
    }
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = 0;                  /* ephemeral */
    sa.sin_addr.s_addr = PP_HTONL(INADDR_ANY);
    if (lwip_bind(fs->pasv, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        pasv_close(fs);
        reply(fs, "425 Can't open data connection.");
        return;
    }
    if (lwip_listen(fs->pasv, 1) < 0) {
        pasv_close(fs);
        reply(fs, "425 Can't open data connection.");
        return;
    }
    socklen_t sl = sizeof(sa);
    lwip_getsockname(fs->pasv, (struct sockaddr *)&sa, &sl);
    uint16_t port = lwip_ntohs(sa.sin_port);
    uint32_t ip = netif_default ? netif_ip4_addr(netif_default)->addr : 0;
    const uint8_t *o = (const uint8_t *)&ip;
    replyf(fs, "227 Entering Passive Mode (%u,%u,%u,%u,%u,%u)",
           o[0], o[1], o[2], o[3], port >> 8, port & 0xFF);
}

/* Accept the queued data connection (client connects after the 227). */
static int data_accept(ftps_t *fs)
{
    if (fs->pasv < 0)
        return -1;
    struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
    lwip_setsockopt(fs->pasv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int fd = lwip_accept(fs->pasv, NULL, NULL);
    if (fd >= 0) {
        lwip_setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        lwip_setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    }
    pasv_close(fs);                   /* one transfer per PASV */
    return fd;
}

/* ---- directory listing ---------------------------------------------------- */
static const char *k_mon[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                 "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

typedef enum {
    LIST_OK,
    LIST_FILESYSTEM_ERROR,
    LIST_DATA_ERROR,
} list_result_t;

static list_result_t send_list(int dfd, const char *path, bool names_only,
                               uint8_t *out, size_t out_cap)
{
    char full[FPATH_MAX];
    full_path(path, full, sizeof(full));

#ifdef ESP_PLATFORM
    FF_DIR dir;
#else
    DIR dir;
#endif
    FILINFO fno;

    sdc_lock();
    FRESULT res = f_opendir(&dir, full);
    sdc_unlock();
    if (res != FR_OK) {
        debugf("FTP: cannot open directory %s (FatFs=%d)", full, res);
        return LIST_FILESYSTEM_ERROR;
    }

    size_t out_len = 0;
    for (;;) {
        sdc_lock();
        res = f_readdir(&dir, &fno);
        sdc_unlock();
        if (res != FR_OK) {
            debugf("FTP: cannot read directory %s (FatFs=%d)", full, res);
            break;
        }
        if (fno.fname[0] == 0)
            break;
        if (fno.fattrib & (AM_HID | AM_SYS))
            continue;

        char line[384];
        int n;
        if (names_only) {
            n = snprintf(line, sizeof(line), "%s\r\n", fno.fname);
        } else {
            int yr = 1980 + ((fno.fdate >> 9) & 0x7F);
            int mo = (fno.fdate >> 5) & 0x0F;
            int dy = fno.fdate & 0x1F;
            if (mo < 1 || mo > 12)
                mo = 1;
            n = snprintf(line, sizeof(line),
                         "%crw-r--r--   1 fpga fpga %10lu %s %2d  %4d %s\r\n",
                         (fno.fattrib & AM_DIR) ? 'd' : '-',
                         (unsigned long)fno.fsize, k_mon[mo - 1], dy, yr, fno.fname);
        }
        if (n < 0)
            continue;
        if ((size_t)n >= sizeof(line))
            n = (int)sizeof(line) - 1;

        if (out_len + (size_t)n > out_cap) {
            if (!send_all(dfd, out, out_len)) {
                sdc_lock();
                f_closedir(&dir);
                sdc_unlock();
                return LIST_DATA_ERROR;
            }
            out_len = 0;
        }
        memcpy(out + out_len, line, (size_t)n);
        out_len += (size_t)n;
    }

    if (out_len && !send_all(dfd, out, out_len)) {
        sdc_lock();
        f_closedir(&dir);
        sdc_unlock();
        return LIST_DATA_ERROR;
    }

    sdc_lock();
    f_closedir(&dir);
    sdc_unlock();
    return res == FR_OK ? LIST_OK : LIST_FILESYSTEM_ERROR;
}

/* ---- transfers ------------------------------------------------------------ */
static void do_retr(ftps_t *fs, const char *path)
{
    char full[FPATH_MAX];
    full_path(path, full, sizeof(full));

    uint32_t restart_at = fs->restart_at;
    fs->restart_at = 0;                 /* REST only applies to the next transfer */

    FIL f;
    sdc_lock();
    if (f_open(&f, full, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
        sdc_unlock();
        reply(fs, "550 Not found.");
        return;
    }
    if (restart_at && f_lseek(&f, restart_at) != FR_OK) {
        f_close(&f);
        sdc_unlock();
        reply(fs, "550 Bad restart offset.");
        return;
    }
    sdc_unlock();

    reply(fs, "150 Opening data connection.");
    int dfd = data_accept(fs);
    if (dfd < 0) {
        sdc_lock();
        f_close(&f);
        sdc_unlock();
        reply(fs, "425 No data connection.");
        return;
    }

    bool ok = true;
    for (;;) {
        UINT br = 0;
        sdc_lock();
        FRESULT rr = f_read(&f, fs->xbuf, XFER_CHUNK, &br);
        sdc_unlock();
        if (rr != FR_OK) {
            ok = false;
            break;
        }
        if (br == 0)
            break;
        if (!send_all(dfd, fs->xbuf, br)) {
            ok = false;
            break;
        }
        if (br < XFER_CHUNK)
            break;
    }

    sdc_lock();
    f_close(&f);
    sdc_unlock();
    lwip_close(dfd);
    reply(fs, ok ? "226 Transfer complete." : "426 Transfer aborted.");
}

static void do_stor(ftps_t *fs, const char *path)
{
    sdc_lock();
    bool mounted = disk_path_mounted(path);
    sdc_unlock();
    if (mounted) {
        reply(fs, "550 File is a mounted disk image; eject it first.");
        return;
    }

    char full[FPATH_MAX];
    full_path(path, full, sizeof(full));

    uint32_t restart_at = fs->restart_at;
    fs->restart_at = 0;                 /* REST only applies to the next transfer */

    /* a resumed upload must keep the existing bytes up to the restart
     * offset; only a fresh upload (no REST) truncates the file */
    FIL f;
    sdc_lock();
    if (f_open(&f, full, restart_at ? (FA_OPEN_EXISTING | FA_WRITE)
                                     : (FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK) {
        sdc_unlock();
        reply(fs, "550 Cannot create file.");
        return;
    }
    if (restart_at && f_lseek(&f, restart_at) != FR_OK) {
        f_close(&f);
        sdc_unlock();
        reply(fs, "550 Bad restart offset.");
        return;
    }
    sdc_unlock();

    reply(fs, "150 Opening data connection.");
    int dfd = data_accept(fs);
    if (dfd < 0) {
        sdc_lock();
        f_close(&f);
        sdc_unlock();
        reply(fs, "425 No data connection.");
        return;
    }

    bool ok = true;
    uint32_t total = restart_at;        /* track offset to pinpoint a bad write */
    uint32_t recv_calls = 0;
    uint32_t write_calls = 0;
    uint32_t max_recv = 0;
    TickType_t recv_ticks = 0;
    TickType_t write_ticks = 0;
#if defined(PICO_RP2040) || defined(PICO_RP2350)
    uint32_t asix_before[ASIX_STAT_COUNT];
    uint32_t asix_after[ASIX_STAT_COUNT];
    tuh_asix_get_stats(asix_before);
#endif
    for (;;) {
        TickType_t t0 = xTaskGetTickCount();
        int r = lwip_recv(dfd, fs->xbuf, XFER_CHUNK, 0);
        recv_ticks += xTaskGetTickCount() - t0;
        recv_calls++;
        if (r == 0)
            break;
        if (r < 0) {
            ok = false;
            break;
        }
        if ((uint32_t)r > max_recv)
            max_recv = (uint32_t)r;
        UINT bw = 0;
        sdc_lock();
        t0 = xTaskGetTickCount();
        FRESULT wr = f_write(&f, fs->xbuf, (UINT)r, &bw);
        write_ticks += xTaskGetTickCount() - t0;
        sdc_unlock();
        write_calls++;
        if (wr != FR_OK || bw != (UINT)r) {
            debugf("FTP: STOR write failed at offset %lu (recv=%d written=%u fr=%d)",
                   (unsigned long)total, r, bw, wr);
            ok = false;               /* volume full or card write failure */
            break;
        }
        total += (uint32_t)bw;
    }

    sdc_lock();
    TickType_t close_start = xTaskGetTickCount();
    f_close(&f);
    TickType_t close_ticks = xTaskGetTickCount() - close_start;
    sdc_unlock();
    lwip_close(dfd);
    reply(fs, ok ? "226 Transfer complete." : "426 Transfer aborted.");
#if defined(PICO_RP2040) || defined(PICO_RP2350)
    tuh_asix_get_stats(asix_after);
#endif
    if (ok) {
        debugf("FTP: STORED %s (%lu bytes, recv calls=%lu max=%lu, recv=%lu ms write=%lu ms close=%lu ms writes=%lu)",
               path, (unsigned long)total, (unsigned long)recv_calls, (unsigned long)max_recv,
               (unsigned long)(recv_ticks * portTICK_PERIOD_MS),
               (unsigned long)(write_ticks * portTICK_PERIOD_MS),
               (unsigned long)(close_ticks * portTICK_PERIOD_MS),
               (unsigned long)write_calls);
#if defined(PICO_RP2040) || defined(PICO_RP2350)
        debugf("FTP: ASIX delta rx_xfer=%lu rx_frame=%lu rx_bad=%lu rx_trunc=%lu rx_pbuf_fail=%lu tx_start=%lu tx_queue=%lu tx_full=%lu tx_fail=%lu tx_done=%lu",
               (unsigned long)(asix_after[ASIX_STAT_RX_XFERS] - asix_before[ASIX_STAT_RX_XFERS]),
               (unsigned long)(asix_after[ASIX_STAT_RX_FRAMES] - asix_before[ASIX_STAT_RX_FRAMES]),
               (unsigned long)(asix_after[ASIX_STAT_RX_BAD_HEADER] - asix_before[ASIX_STAT_RX_BAD_HEADER]),
               (unsigned long)(asix_after[ASIX_STAT_RX_TRUNCATED] - asix_before[ASIX_STAT_RX_TRUNCATED]),
               (unsigned long)(asix_after[ASIX_STAT_RX_PBUF_FAIL] - asix_before[ASIX_STAT_RX_PBUF_FAIL]),
               (unsigned long)(asix_after[ASIX_STAT_TX_STARTED] - asix_before[ASIX_STAT_TX_STARTED]),
               (unsigned long)(asix_after[ASIX_STAT_TX_QUEUED] - asix_before[ASIX_STAT_TX_QUEUED]),
               (unsigned long)(asix_after[ASIX_STAT_TX_QUEUE_FULL] - asix_before[ASIX_STAT_TX_QUEUE_FULL]),
               (unsigned long)(asix_after[ASIX_STAT_TX_START_FAIL] - asix_before[ASIX_STAT_TX_START_FAIL]),
               (unsigned long)(asix_after[ASIX_STAT_TX_DONE] - asix_before[ASIX_STAT_TX_DONE]));
#endif
    }
}

/* ---- command loop ---------------------------------------------------------- */
static void session(ftps_t *fs)
{
    char line[256];
    int  len = 0;
    fs->cwd[0] = 0;
    fs->rnfr[0] = 0;
    fs->restart_at = 0;

    reply(fs, "220 MiSTle FPGA Companion FTP ready.");

    for (;;) {
        char c;
        int r = lwip_recv(fs->ctl, &c, 1, 0);
        if (r <= 0)
            return;
        if (c == '\n') {
            line[len] = 0;
            len = 0;
            /* strip CR */
            char *cr = strchr(line, '\r');
            if (cr)
                *cr = 0;
            if (!line[0])
                continue;

            char *arg = strchr(line, ' ');
            if (arg)
                *arg++ = 0;
            else
                arg = line + strlen(line);
            for (char *p = line; *p; p++)
                *p = (char)toupper((unsigned char)*p);

            char path[CWD_MAX];
            if (!strcmp(line, "USER")) {
                reply(fs, "331 Any password will do.");
            } else if (!strcmp(line, "PASS")) {
                reply(fs, "230 Logged in.");
            } else if (!strcmp(line, "SYST")) {
                reply(fs, "215 UNIX Type: L8");
            } else if (!strcmp(line, "FEAT")) {
                reply(fs, "211-Features:");
                reply(fs, " SIZE");
                reply(fs, " PASV");
                reply(fs, "211 End");
            } else if (!strcmp(line, "TYPE")) {
                reply(fs, "200 Type set.");
            } else if (!strcmp(line, "NOOP")) {
                reply(fs, "200 NOOP.");
            } else if (!strcmp(line, "PWD") || !strcmp(line, "XPWD")) {
                replyf(fs, "257 \"/%s\"", fs->cwd);
            } else if (!strcmp(line, "CWD")) {
                if (resolve(fs, arg, path, sizeof(path))) {
                    bool valid = !path[0];      /* volume root is always valid */
                    if (!valid) {
                        char full[FPATH_MAX];
                        full_path(path, full, sizeof(full));
                        FILINFO fno;
                        sdc_lock();
                        FRESULT r = f_stat(full, &fno);
                        sdc_unlock();
                        valid = (r == FR_OK) && (fno.fattrib & AM_DIR);
                    }
                    if (valid) {
                        snprintf(fs->cwd, sizeof(fs->cwd), "%s", path);
                        replyf(fs, "250 \"/%s\"", fs->cwd);
                    } else {
                        reply(fs, "550 No such directory.");
                    }
                } else {
                    reply(fs, "550 Bad path.");
                }
            } else if (!strcmp(line, "CDUP")) {
                char *ls = strrchr(fs->cwd, '/');
                if (ls)
                    *ls = 0;
                else
                    fs->cwd[0] = 0;
                replyf(fs, "250 \"/%s\"", fs->cwd);
            } else if (!strcmp(line, "PASV")) {
                cmd_pasv(fs);
            } else if (!strcmp(line, "PORT")) {
                reply(fs, "502 Use passive mode.");
            } else if (!strcmp(line, "LIST") || !strcmp(line, "NLST")) {
                bool nl = !strcmp(line, "NLST");
                /* ignore -a style flags; optional path argument */
                const char *a = (arg[0] && arg[0] != '-') ? arg : "";
                if (!resolve(fs, a, path, sizeof(path))) {
                    reply(fs, "550 Bad path.");
                    continue;
                }
                reply(fs, "150 Here comes the directory listing.");
                int dfd = data_accept(fs);
                if (dfd < 0) {
                    reply(fs, "425 No data connection.");
                    continue;
                }
                list_result_t result = send_list(dfd, path, nl, fs->xbuf, sizeof(fs->xbuf));
                lwip_close(dfd);
                if (result == LIST_OK)
                    reply(fs, "226 Directory send OK.");
                else if (result == LIST_DATA_ERROR)
                    reply(fs, "426 Directory transfer aborted.");
                else
                    reply(fs, "550 Directory listing failed.");
            } else if (!strcmp(line, "SIZE")) {
                if (resolve(fs, arg, path, sizeof(path))) {
                    char full[FPATH_MAX];
                    full_path(path, full, sizeof(full));
                    FILINFO fno;
                    sdc_lock();
                    FRESULT r = f_stat(full, &fno);
                    sdc_unlock();
                    if (r == FR_OK && !(fno.fattrib & AM_DIR)) {
                        replyf(fs, "213 %lu", (unsigned long)fno.fsize);
                        continue;
                    }
                }
                reply(fs, "550 Not found.");
            } else if (!strcmp(line, "RETR")) {
                if (resolve(fs, arg, path, sizeof(path)))
                    do_retr(fs, path);
                else
                    reply(fs, "550 Bad path.");
            } else if (!strcmp(line, "STOR")) {
                if (resolve(fs, arg, path, sizeof(path)))
                    do_stor(fs, path);
                else
                    reply(fs, "550 Bad path.");
            } else if (!strcmp(line, "DELE")) {
                if (resolve(fs, arg, path, sizeof(path))) {
                    char full[FPATH_MAX];
                    full_path(path, full, sizeof(full));
                    sdc_lock();
                    bool mounted = disk_path_mounted(path);
                    FRESULT r = mounted ? FR_DENIED : f_unlink(full);
                    sdc_unlock();
                    if (!mounted && r == FR_OK) {
                        reply(fs, "250 Deleted.");
                        continue;
                    }
                }
                reply(fs, "550 Delete failed (mounted image?).");
            } else if (!strcmp(line, "MKD") || !strcmp(line, "XMKD")) {
                if (resolve(fs, arg, path, sizeof(path))) {
                    char full[FPATH_MAX];
                    full_path(path, full, sizeof(full));
                    sdc_lock();
                    FRESULT r = f_mkdir(full);
                    sdc_unlock();
                    if (r == FR_OK) {
                        replyf(fs, "257 \"/%s\" created.", path);
                        continue;
                    }
                }
                reply(fs, "550 Mkdir failed.");
            } else if (!strcmp(line, "RMD") || !strcmp(line, "XRMD")) {
                if (resolve(fs, arg, path, sizeof(path))) {
                    char full[FPATH_MAX];
                    full_path(path, full, sizeof(full));
                    sdc_lock();
                    FRESULT r = f_unlink(full);   /* removes empty dirs too */
                    sdc_unlock();
                    if (r == FR_OK) {
                        reply(fs, "250 Removed.");
                        continue;
                    }
                }
                reply(fs, "550 Rmdir failed (not empty?).");
            } else if (!strcmp(line, "REST")) {
                char *end;
                unsigned long off = strtoul(arg, &end, 10);
                if (*arg && !*end) {
                    fs->restart_at = (uint32_t)off;
                    replyf(fs, "350 Restarting at %lu.", off);
                } else {
                    reply(fs, "501 Invalid restart offset.");
                }
            } else if (!strcmp(line, "RNFR")) {
                if (resolve(fs, arg, fs->rnfr, sizeof(fs->rnfr)))
                    reply(fs, "350 Ready for RNTO.");
                else
                    reply(fs, "550 Bad path.");
            } else if (!strcmp(line, "RNTO")) {
                if (fs->rnfr[0] && resolve(fs, arg, path, sizeof(path))) {
                    char full_from[FPATH_MAX], full_to[FPATH_MAX];
                    full_path(fs->rnfr, full_from, sizeof(full_from));
                    full_path(path, full_to, sizeof(full_to));
                    sdc_lock();
                    bool mounted = disk_path_mounted(fs->rnfr) || disk_path_mounted(path);
                    FRESULT r = mounted ? FR_DENIED : f_rename(full_from, full_to);
                    sdc_unlock();
                    if (!mounted && r == FR_OK) {
                        reply(fs, "250 Renamed.");
                        fs->rnfr[0] = 0;
                        continue;
                    }
                }
                reply(fs, "550 Rename failed.");
                fs->rnfr[0] = 0;
            } else if (!strcmp(line, "QUIT")) {
                reply(fs, "221 Goodbye.");
                return;
            } else {
                reply(fs, "502 Not implemented.");
            }
        } else if (len < (int)sizeof(line) - 1) {
            line[len++] = c;
        }
    }
}

static void session_thread(void *arg)
{
    ftps_t *fs = (ftps_t *)arg;
    session(fs);
    pasv_close(fs);
    lwip_close(fs->ctl);
    fs->ctl = -1;
    fs->in_use = false;
    debugf("FTP: CLIENT DISCONNECTED");
    vTaskDelete(NULL);
}

static void ftpd_thread(void *arg)
{
    (void)arg;
    while (netif_default == NULL)     /* lwIP core init + link (see telnetd) */
        vTaskDelay(pdMS_TO_TICKS(200));

    int lfd = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (lfd < 0)
        return;
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = PP_HTONS(FTP_PORT);
    sa.sin_addr.s_addr = PP_HTONL(INADDR_ANY);
    int one = 1;
    lwip_setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    if (lwip_bind(lfd, (struct sockaddr *)&sa, sizeof(sa)) < 0)
        return;
    lwip_listen(lfd, MAX_SESSIONS);
    debugf("FTP: LISTENING ON PORT 21");

    for (;;) {
        int fd = lwip_accept(lfd, NULL, NULL);
        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        ftps_t *fs = NULL;
        for (int i = 0; i < MAX_SESSIONS; i++) {
            if (!s_sess[i].in_use) {
                fs = &s_sess[i];
                break;
            }
        }
        if (!fs) {
            const char *busy = "421 Too many connections.\r\n";
            lwip_send(fd, busy, (int)strlen(busy), 0);
            lwip_close(fd);
            continue;
        }
        fs->in_use = true;
        fs->ctl = fd;
        fs->pasv = -1;
        BaseType_t created = xTaskCreate(session_thread, "ftps", FTPS_STACK_WORDS, fs,
                configMAX_PRIORITIES - 3, NULL);
        if (created != pdPASS) {
            debugf("FTP: failed to create client task");
            fs->ctl = -1;
            fs->pasv = -1;
            fs->in_use = false;
            lwip_close(fd);
            continue;
        }
        debugf("FTP: CLIENT CONNECTED");
    }
}

void ftpd_init(void)
{
    BaseType_t created = xTaskCreate(ftpd_thread, "ftpd", FTPD_STACK_WORDS, NULL,
                configMAX_PRIORITIES - 3, NULL);
    if (created != pdPASS)
        debugf("FTP: failed to create server task");
}
