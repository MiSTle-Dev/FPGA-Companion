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

#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "usb_osal.h"
#include "usb_config.h"

#include "debug.h"
#include "sdc.h"

#include "ftpd.h"

#define FTP_PORT     21
#define XFER_CHUNK   4096
#define CWD_MAX      128
#define FPATH_MAX    (CWD_MAX + 16)   /* CARD_MOUNTPOINT + '/' + relative path */

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
    if (lwip_bind(fs->pasv, (struct sockaddr *)&sa, sizeof(sa)) < 0 ||
        lwip_listen(fs->pasv, 1) < 0) {
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
    pasv_close(fs);                   /* one transfer per PASV */
    return fd;
}

/* ---- directory listing ---------------------------------------------------- */
static const char *k_mon[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                 "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

static void send_list(int dfd, const char *path, bool names_only)
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
    if (f_opendir(&dir, full) != FR_OK) {
        sdc_unlock();
        return;
    }
    for (;;) {
        if (f_readdir(&dir, &fno) != FR_OK || fno.fname[0] == 0)
            break;
        if (fno.fattrib & (AM_HID | AM_SYS))
            continue;
        char line[128];
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
        lwip_send(dfd, line, n, 0);
    }
    f_closedir(&dir);
    sdc_unlock();
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
    reply(fs, "150 Opening data connection.");
    int dfd = data_accept(fs);
    if (dfd < 0) {
        f_close(&f);
        sdc_unlock();
        reply(fs, "425 No data connection.");
        return;
    }
    bool ok = true;
    for (;;) {
        UINT br = 0;
        if (f_read(&f, fs->xbuf, XFER_CHUNK, &br) != FR_OK) {
            ok = false;
            break;
        }
        if (br == 0)
            break;
        if (lwip_send(dfd, fs->xbuf, (int)br, 0) < 0) {
            ok = false;
            break;
        }
        if (br < XFER_CHUNK)
            break;
    }
    f_close(&f);
    sdc_unlock();
    lwip_close(dfd);
    reply(fs, ok ? "226 Transfer complete." : "426 Transfer aborted.");
}

static void do_stor(ftps_t *fs, const char *path)
{
    sdc_lock();
    if (disk_path_mounted(path)) {
        sdc_unlock();
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
    reply(fs, "150 Opening data connection.");
    int dfd = data_accept(fs);
    if (dfd < 0) {
        f_close(&f);
        sdc_unlock();
        reply(fs, "425 No data connection.");
        return;
    }
    bool ok = true;
    uint32_t total = restart_at;        /* track offset to pinpoint a bad write */
    for (;;) {
        int r = lwip_recv(dfd, fs->xbuf, XFER_CHUNK, 0);
        if (r == 0)
            break;
        if (r < 0) {
            ok = false;
            break;
        }
        UINT bw = 0;
        FRESULT wr = f_write(&f, fs->xbuf, (UINT)r, &bw);
        if (wr != FR_OK || bw != (UINT)r) {
            debugf("FTP: STOR write failed at offset %lu (recv=%d written=%u fr=%d)",
                   (unsigned long)total, r, bw, wr);
            ok = false;               /* volume full or card write failure */
            break;
        }
        total += (uint32_t)bw;
    }
    f_close(&f);
    sdc_unlock();
    lwip_close(dfd);
    reply(fs, ok ? "226 Transfer complete." : "426 Transfer aborted.");
    if (ok)
        debugf("FTP: STORED %s (%lu bytes)", path, (unsigned long)total);
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
                send_list(dfd, path, nl);
                lwip_close(dfd);
                reply(fs, "226 Directory send OK.");
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
    usb_osal_thread_delete(NULL);
}

static void ftpd_thread(void *arg)
{
    (void)arg;
    while (netif_default == NULL)     /* lwIP core init + link (see telnetd) */
        usb_osal_msleep(200);

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
            usb_osal_msleep(500);
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
        debugf("FTP: CLIENT CONNECTED");
        // FF_USE_LFN==2 puts FatFs' LFN working buffer on the stack for every
        // f_opendir/f_readdir/f_stat/... call, so LIST/NLST needs extra room
        usb_osal_thread_create("ftps", 6144, CONFIG_USBHOST_PSC_PRIO + 1,
                               session_thread, fs);
    }
}

void ftpd_init(void)
{
    usb_osal_thread_create("ftpd", 3072, CONFIG_USBHOST_PSC_PRIO + 1,
                           ftpd_thread, NULL);
}
