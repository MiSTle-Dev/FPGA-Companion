#ifndef _TUSB_ASIX_HOST_H_
#define _TUSB_ASIX_HOST_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "host/usbh_pvt.h"
#include "lwip/netif.h"

//--------------------------------------------------------------------+
// Class Driver Configuration
//--------------------------------------------------------------------+

#ifndef CFG_TUH_ASIX_EP_BUFSIZE
#define CFG_TUH_ASIX_EP_BUFSIZE 1536+4
#endif

typedef struct {
  uint8_t dev_addr;
  struct netif netif;
  
  uint8_t mac[6];     // mac address
  uint16_t phy_id;
  uint8_t embd_phy;   // flag whether phy is embedded
  uint16_t rx_ctl;
  uint16_t medium_status;
  bool link_detected;

  // info from phy
  uint16_t mii_phy_id[2];  // mii oid
  uint16_t bmsr;
  uint16_t bmcr;
  uint16_t advertise;
  
  uint8_t ep[3];
  uint16_t ep_size[3];
  
  uint8_t ep0in_buf[8];
  uint8_t ep1in_buf[CFG_TUH_ASIX_EP_BUFSIZE];
  uint8_t epout_buf[CFG_TUH_ASIX_EP_BUFSIZE];
} asixh_interface_t;

extern usbh_class_driver_t const usbh_asix_driver;

void tuh_asix_umount_cb(asixh_interface_t *asix_itf);
void tuh_asix_mount_cb(asixh_interface_t *asix_itf);
void tuh_asix_transmit(struct netif *netif, uint8_t *data, uint16_t len);
   
#ifdef __cplusplus
}
#endif

#endif /* _TUSB_ASIX_HOST_H_ */
