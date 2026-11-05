#pragma once
struct netif; struct pbuf; struct ip4_addr;
struct netif *ppp_port_route(const struct ip4_addr *dest);
int ppp_port_ip4_input(struct pbuf *p, struct netif *inp);
