/*
 * telnetd — remote shell/console on TCP port 23 (see telnetd.c).
 */
#ifndef _TELNETD_H
#define _TELNETD_H

/* Spawn the server task. Call after tcpip_init(); binding does not require
 * the interface to be up yet. */
void telnetd_init(void);

#endif
