/*
 * telnetd — console on TCP port 23 (see telnetd.c).
 */
#ifndef _TELNETD_H
#define _TELNETD_H

/* Spawn the server task. Call after tcpip_init(); binding does not require
 * the interface to be up yet. */
void telnetd_init(void);

/* Mirror Bouffalo shell output to telnet while keeping the UART console. */
void telnetd_monitor_shell(void);

/* Mirror a formatted debug line to telnet. Used by debug.h on BL616 only. */
void telnetd_printf(const char *fmt, ...);

#endif
