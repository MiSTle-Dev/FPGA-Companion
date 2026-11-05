#pragma once
#include "lwip/opt.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/ip.h"
#define LOCK_TCPIP_CORE()
#define UNLOCK_TCPIP_CORE()
typedef void (*tcpip_callback_fn)(void *ctx);
static inline err_t tcpip_try_callback(tcpip_callback_fn f, void *ctx) { f(ctx); return ERR_OK; }
static inline err_t tcpip_input(struct pbuf *p, struct netif *inp) { return ip_input(p, inp); }
