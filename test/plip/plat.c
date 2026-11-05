#include <stdlib.h>
#include "lwip/sys.h"
sys_prot_t sys_arch_protect(void) { return 0; }
void sys_arch_unprotect(sys_prot_t p) { (void)p; }
unsigned lwip_port_rand(void) { return rand(); }
