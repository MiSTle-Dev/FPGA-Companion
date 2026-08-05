/*
  asix_host.c

  TinyUSB host driver for ASIX8877x based USB ethernet adapters

  TODO:
  - test tcp with ethernet
  - sntp
 */

#include <stdio.h>
#include "tusb_option.h"

#if (TUSB_OPT_HOST_ENABLED && CFG_TUH_ASIX)

#include "host/usbh.h"
#include "host/usbh_pvt.h"

#include "asix_host.h"
#include "asix_pvt.h"
#include "mii.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-const-variable"

// list of supported/tested devices
static const struct {
  uint16_t vid;
  uint16_t pid;
} asix_devs[] = {
  { 0x2001, 0x3c05 },  // Silver DLink DUB-E100 H/W Ver B1 Alternate
  { 0x0b95, 0x772b },  // Black DLink DUB-E100, ASIX Electronics Corp. AX88772B
  { 0x2001, 0x1a02 },  // DLink DUB-E100 H/W Ver C1
  { 0x0b95, 0x7720 },  // NoName Wii Adapter
  { 0x05ac, 0x1402 },  // Apple USB Adapter A1277
  { 0, 0 }
};

static asixh_interface_t _devices[CFG_TUH_ASIX];

TU_ATTR_ALWAYS_INLINE static inline asixh_interface_t *get_interface(uint8_t dev_addr) {
  TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX, NULL);

  for(int i=0;i<CFG_TUH_ASIX;i++)
    if(_devices[i].dev_addr == dev_addr)
      return &_devices[i];
    
  return NULL;
}

static bool asix_write_cmd(uint8_t daddr, uint8_t cmd, uint16_t value, uint16_t index,
			   uint8_t *buffer, uint16_t length,
			   tuh_xfer_cb_t complete_cb, uintptr_t user_data) {

  // printf("%s() cmd=0x%02x value=0x%04x index=0x%04x length=%d\n", __FUNCTION__,
  //    cmd, value, index, length);

  tusb_control_request_t const request = {
    .bmRequestType_bit = {
      .recipient = TUSB_REQ_RCPT_DEVICE,
      .type      = TUSB_REQ_TYPE_VENDOR,
      .direction = TUSB_DIR_OUT
    },
    .bRequest = cmd,
    .wValue   = tu_htole16(value),
    .wIndex   = tu_htole16(index),
    .wLength  = tu_htole16(length)
  };
 
  tuh_xfer_t xfer = {
    .daddr       = daddr,
    .ep_addr     = 0,
    .setup       = &request,
    .buffer      = buffer,
    .complete_cb = complete_cb,
    .user_data   = user_data
  };

  return tuh_control_xfer(&xfer);
}

static bool asix_read_cmd(uint8_t daddr, uint8_t cmd, uint16_t value, uint16_t index,
			   uint8_t * buffer, uint16_t length,
			  tuh_xfer_cb_t complete_cb, uintptr_t user_data) {
  //printf("%s() cmd=0x%02x value=0x%04x index=0x%04x length=%d\n", __FUNCTION__,
  //	 cmd, value, index, length);

  tusb_control_request_t const request = {
    .bmRequestType_bit = {
      .recipient = TUSB_REQ_RCPT_DEVICE,
      .type      = TUSB_REQ_TYPE_VENDOR,
      .direction = TUSB_DIR_IN
    },
    .bRequest = cmd,
    .wValue   = tu_htole16(value),
    .wIndex   = tu_htole16(index),
    .wLength  = tu_htole16(length)
  };
 
  tuh_xfer_t xfer = {
    .daddr       = daddr,
    .ep_addr     = 0,
    .setup       = &request,
    .buffer      = buffer,
    .complete_cb = complete_cb,
    .user_data   = user_data
  };

  return tuh_control_xfer(&xfer);
}

//--------------------------------------------------------------------+
// USBH API
//--------------------------------------------------------------------+
bool asixh_init(void) {
    tu_memclr(_devices, sizeof(_devices));
    return true;
}

uint16_t asixh_open(__attribute__((unused)) uint8_t rhport, uint8_t dev_addr,
		    tusb_desc_interface_t const *desc_itf, __attribute__((unused)) uint16_t max_len) {
    TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX, 0);

    TU_VERIFY(0xff == desc_itf->bInterfaceClass, 0);
    TU_VERIFY(0xff == desc_itf->bInterfaceSubClass, 0);
    TU_VERIFY(0x00 == desc_itf->bInterfaceProtocol, 0);
 
    // get device vid/pid
    uint16_t pid, vid;
    tuh_vid_pid_get(dev_addr, &vid, &pid);

    // search in list of asix devices
    int i;
    for(i=0;asix_devs[i].vid;i++)
      if(vid == asix_devs[i].vid && pid == asix_devs[i].pid)
	break;

    // bail out if end of list was reached
    TU_VERIFY(asix_devs[i].vid, 0);

    printf("[%u] ASIX opening Interface %u\r\n", dev_addr, desc_itf->bInterfaceNumber);

    // get an unused entry, bail out of there's none
    asixh_interface_t *itf = get_interface(0);
    TU_VERIFY(itf, 0);

    itf->dev_addr = dev_addr;

    const uint16_t drv_len = (uint16_t)(sizeof(tusb_desc_interface_t) +
					desc_itf->bNumEndpoints * sizeof(tusb_desc_endpoint_t));

    //Parse descriptor for all endpoints and open them
    uint8_t const *p_desc = (uint8_t const *)desc_itf;

    // skip interface descriptor 
    p_desc = tu_desc_next(p_desc);

    // Endpoint Descriptors
    for (uint8_t i = 0; i < desc_itf->bNumEndpoints; i++) {
      const tusb_desc_endpoint_t *desc_ep = (const tusb_desc_endpoint_t *)p_desc;
      TU_ASSERT(TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType, 0);
      TU_ASSERT(tuh_edpt_open(itf->dev_addr, desc_ep), 0);      

      itf->ep[i] = desc_ep->bEndpointAddress;
      itf->ep_size[i] = tu_edpt_packet_size(desc_ep);
      p_desc = tu_desc_next(p_desc);
    }
    
    return drv_len;
}

#define SETUP_DONE  0
#define SETUP_WRITE 1
#define SETUP_READ  2

static void postproc_pyh_id(asixh_interface_t *itf) {
  printf("ASIX: phy is 0x%04x\n", tu_htole16(itf->phy_id));
  itf->embd_phy = (itf->phy_id & 0x1f00) == 0x1000 ? 1 : 0;
  printf("      phy is %sembedded\n", itf->embd_phy?"":"not ");
}

static uint16_t preproc_phy_select(asixh_interface_t *itf) {
  return itf->embd_phy;
}

static uint16_t preproc_phy_id(asixh_interface_t *itf) {
  return itf->phy_id >> 8;
}

static uint16_t preproc_swreset(asixh_interface_t *itf) {
  return itf->embd_phy?AX_SWRESET_IPRL:AX_SWRESET_PRTE;
}

static void postproc_rx_ctl(asixh_interface_t *itf) {
  printf("ASIX: rx ctl is 0x%04x\n", tu_htole16(itf->rx_ctl));
}

static void postproc_medium_status(asixh_interface_t *itf) {
  printf("ASIX: medium status is 0x%04x\n", tu_htole16(itf->medium_status));
}

static void postproc_read_mac(asixh_interface_t *itf) {
  printf("ASIX: MAC is %02x:%02x:%02x:%02x:%02x:%02x\n",
	 itf->mac[0],itf->mac[1],itf->mac[2],itf->mac[3],itf->mac[4],itf->mac[5]);
}

static void postproc_phy_id32(asixh_interface_t *itf) {
  uint32_t oui = (itf->mii_phy_id[0] << 6) + (itf->mii_phy_id[1] >> 10);
  
  printf("ASIX: PHY ID is 0x%04x%04x\n", itf->mii_phy_id[1], itf->mii_phy_id[0] );
  printf("      OUI %02lx:%02lx:%02lx\n", (oui>>16)&0xff,(oui>>8)&0xff,oui&0xff);
  printf("      Manufacturer %0d\n", (itf->mii_phy_id[1]>>4)&0x3f);
  printf("      Revision %d\n", itf->mii_phy_id[1]&0xf);
}

static uint16_t preproc_bmcr_reset(asixh_interface_t *itf) {
  itf->bmcr = BMCR_RESET;
  printf("ASIX: Setting bmcr to 0x%04x\n", itf->bmcr);
  return preproc_phy_id(itf);
}

static uint16_t preproc_bmcr_autonegotiate(asixh_interface_t *itf) {
  if(itf->bmcr & BMCR_ANENABLE)
    itf->bmcr |= BMCR_ANRESTART;
  
  printf("ASIX: Setting bmcr to 0x%04x\n", itf->bmcr);
  return preproc_phy_id(itf);
}

static void postproc_bmcr(asixh_interface_t *itf) {
  printf("ASIX: Basic mode control register is 0x%04x\n", itf->bmcr);
  if(itf->bmcr & BMCR_SPEED1000) printf("      MSB of Speed (1000)\n");
  if(itf->bmcr & BMCR_CTST)      printf("      Collision test\n");
  if(itf->bmcr & BMCR_FULLDPLX)  printf("      Full duplex\n");
  if(itf->bmcr & BMCR_ANRESTART) printf("      Auto negotiation restart\n");
  if(itf->bmcr & BMCR_ISOLATE)   printf("      Disconnect from MII\n");
  if(itf->bmcr & BMCR_PDOWN)     printf("      Powerdown\n");
  if(itf->bmcr & BMCR_ANENABLE)  printf("      Enable auto negotiation\n");
  if(itf->bmcr & BMCR_SPEED100)  printf("      Select 100Mbps\n");
  if(itf->bmcr & BMCR_LOOPBACK)  printf("      TXD loopback bits\n");
  if(itf->bmcr & BMCR_RESET)     printf("      Reset\n");
}

static uint16_t preproc_advertise(asixh_interface_t *itf) {
  itf->advertise = ADVERTISE_ALL | ADVERTISE_CSMA;
  printf("ASIX: Setting advertise to 0x%04x\n", itf->advertise);
  return preproc_phy_id(itf);
}

static void postproc_advertise(asixh_interface_t *itf) {
  printf("ASIX: Advertise register is 0x%04x\n", itf->advertise);
}

static void postproc_bmsr(asixh_interface_t *itf) {
  // Basic mode status register.
  printf("ASIX: Basic mode status register is 0x%04x\n", itf->bmsr);
  if(itf->bmsr & BMSR_ERCAP)        printf("      Ext-reg capability\n");
  if(itf->bmsr & BMSR_JCD)          printf("      Jabber detected\n");
  if(itf->bmsr & BMSR_LSTATUS)      printf("      Link status\n");
  if(itf->bmsr & BMSR_ANEGCAPABLE)  printf("      Able to do auto-negotiation\n");
  if(itf->bmsr & BMSR_RFAULT)       printf("      Remote fault detected\n");
  if(itf->bmsr & BMSR_ANEGCOMPLETE) printf("      Auto-negotiation complete\n");
  if(itf->bmsr & BMSR_ESTATEN)      printf("      Extended Status in R15\n");
  if(itf->bmsr & BMSR_100HALF2)     printf("      Can do 100BASE-T2 HDX\n");
  if(itf->bmsr & BMSR_100FULL2)     printf("      Can do 100BASE-T2 FDX\n");
  if(itf->bmsr & BMSR_10HALF)       printf("      Can do 10mbps, half-duplex\n");
  if(itf->bmsr & BMSR_10FULL)       printf("      Can do 10mbps, full-duplex\n");
  if(itf->bmsr & BMSR_100HALF)      printf("      Can do 100mbps, half-duplex\n");
  if(itf->bmsr & BMSR_100FULL)      printf("      Can do 100mbps, full-duplex\n");
  if(itf->bmsr & BMSR_100BASE4)     printf("      Can do 100mbps, 4k packets\n");
}

#define READ_MII_REG(reg,var,cb) \
  { SETUP_WRITE,  AX_CMD_SET_SW_MII, 0,0, 0, 0, NULL,NULL },  \
  { SETUP_READ,   AX_CMD_READ_MII_REG, 0,reg, offsetof(asixh_interface_t,var), 2, preproc_phy_id,cb }, \
  { SETUP_WRITE,  AX_CMD_SET_HW_MII, 0,0, 0, 0, NULL,NULL }

// structure describing the setup process
static const struct {
  uint8_t type;
  uint8_t cmd;       // USB control command
  uint16_t value;    // USB control command value
  uint16_t index;    // USB control command index
  uint16_t offset;   // command payload offset within interface structure, only used during reads
  uint16_t size;     // command payload length, only used during reads
  uint16_t (*preproc)(asixh_interface_t *);
  void (*postproc)(asixh_interface_t *);
} setup_commands[] = {
  { SETUP_WRITE, AX_CMD_WRITE_GPIOS, AX_GPIO_RSE | AX_GPIO_GPO_2 | AX_GPIO_GPO2EN,0, 0,0, NULL,NULL  /* TODO: sleep 150ms */ },
  { SETUP_READ,  AX_CMD_READ_PHY_ID, 0,0, offsetof(asixh_interface_t, phy_id), 2, NULL, postproc_pyh_id  },
  { SETUP_WRITE, AX_CMD_SW_PHY_SELECT, 0,0, 0,0, preproc_phy_select, NULL },
  { SETUP_WRITE, AX_CMD_SW_RESET, AX_SWRESET_IPPD|AX_SWRESET_PRL,0, 0,0, NULL,NULL },
  { SETUP_WRITE, AX_CMD_SW_RESET, AX_SWRESET_CLEAR,0, 0,0, NULL,NULL },
  { SETUP_WRITE, AX_CMD_SW_RESET, 0,0, 0,0, preproc_swreset,NULL },
  { SETUP_READ,  AX_CMD_READ_RX_CTL, 0,0, offsetof(asixh_interface_t, rx_ctl), 2, NULL,postproc_rx_ctl },
  { SETUP_WRITE, AX_CMD_WRITE_RX_CTL, 0,0, 0, 0, NULL,NULL },
  { SETUP_READ,  AX_CMD_READ_RX_CTL, 0,0, offsetof(asixh_interface_t, rx_ctl), 2, NULL,postproc_rx_ctl },
  { SETUP_READ,  AX_CMD_READ_NODE_ID, 0,0, offsetof(asixh_interface_t, mac), 6, NULL,postproc_read_mac },

  // read both physical id registers
  READ_MII_REG(MII_PHYSID1, mii_phy_id[0], NULL),
  READ_MII_REG(MII_PHYSID2, mii_phy_id[1], postproc_phy_id32),

  { SETUP_WRITE,  AX_CMD_SW_RESET, AX_SWRESET_PRL,0, 0,0, NULL,NULL },
  { SETUP_WRITE,  AX_CMD_SW_RESET, AX_SWRESET_IPRL | AX_SWRESET_PRL,0, 0,0, NULL,NULL },

  READ_MII_REG(MII_BMCR, bmcr, postproc_bmcr),
  READ_MII_REG(MII_BMSR, bmsr, postproc_bmsr),
  READ_MII_REG(MII_ADVERTISE, advertise, postproc_advertise),

  // reset the phy
  { SETUP_WRITE,  AX_CMD_SET_SW_MII, 0,0, 0, 0, NULL,NULL },
  { SETUP_WRITE,  AX_CMD_WRITE_MII_REG, 0,MII_BMCR, offsetof(asixh_interface_t, bmcr), 2, preproc_bmcr_reset,NULL },
  { SETUP_WRITE,  AX_CMD_SET_HW_MII, 0,0, 0, 0, NULL,NULL },

  // set advertise register
  { SETUP_WRITE,  AX_CMD_SET_SW_MII, 0,0, 0, 0, NULL,NULL },
  { SETUP_WRITE,  AX_CMD_WRITE_MII_REG, 0,MII_ADVERTISE, offsetof(asixh_interface_t, advertise), 2, preproc_advertise,NULL },
  { SETUP_WRITE,  AX_CMD_SET_HW_MII, 0,0, 0, 0, NULL,NULL },

  // (re-)enable auto negotiation
  { SETUP_WRITE,  AX_CMD_SET_SW_MII, 0,0, 0, 0, NULL,NULL },
  { SETUP_WRITE,  AX_CMD_WRITE_MII_REG, 0,MII_BMCR, offsetof(asixh_interface_t, bmcr), 2, preproc_bmcr_autonegotiate,NULL },
  { SETUP_WRITE,  AX_CMD_SET_HW_MII, 0,0, 0, 0, NULL,NULL },
  
  { SETUP_WRITE,  AX_CMD_WRITE_MEDIUM_MODE, AX88772_MEDIUM_DEFAULT,0, 0, 0, NULL,NULL },

  { SETUP_WRITE,   AX_CMD_WRITE_IPG0,  AX88772_IPG0_DEFAULT|AX88772_IPG1_DEFAULT, AX88772_IPG2_DEFAULT, 0, 0, NULL,NULL },
  
  /* Set RX_CTL to default values with 2k buffer, and enable cactus */
  { SETUP_WRITE, AX_CMD_WRITE_RX_CTL, AX_DEFAULT_RX_CTL,0, 0, 0, NULL,NULL },
  { SETUP_READ,  AX_CMD_READ_RX_CTL, 0,0, offsetof(asixh_interface_t, rx_ctl), 2, NULL,postproc_rx_ctl },
  
  { SETUP_READ,  AX_CMD_READ_MEDIUM_STATUS, 0,0, offsetof(asixh_interface_t, medium_status), 2, NULL,postproc_medium_status },
  
  { SETUP_DONE,  0, 0,0, 0,0, NULL,NULL }
};

bool tuh_asix_receive(uint8_t dev_addr, uint8_t ep);

static void process_set_config(tuh_xfer_t* xfer) {
  asixh_interface_t *itf = get_interface(xfer->daddr);
  TU_VERIFY(itf, );

  // call post processing function from previous command if present
  if(xfer->user_data && setup_commands[xfer->user_data-1].postproc)
    setup_commands[xfer->user_data-1].postproc(itf);

  // call preprocessing function id present
  uint16_t value = setup_commands[xfer->user_data].value;  
  if(setup_commands[xfer->user_data].preproc)
    value = setup_commands[xfer->user_data].preproc(itf);
  
  if(setup_commands[xfer->user_data].type == SETUP_WRITE) {
    asix_write_cmd(itf->dev_addr, setup_commands[xfer->user_data].cmd,
		   value, setup_commands[xfer->user_data].index,
		   ((uint8_t*)itf)+setup_commands[xfer->user_data].offset,
		   setup_commands[xfer->user_data].size,
		   process_set_config, xfer->user_data+1);
  } else if(setup_commands[xfer->user_data].type == SETUP_READ)  {
    asix_read_cmd(itf->dev_addr, setup_commands[xfer->user_data].cmd,
		  value, setup_commands[xfer->user_data].index,
		  ((uint8_t*)itf)+setup_commands[xfer->user_data].offset,
		  setup_commands[xfer->user_data].size,
		  process_set_config, xfer->user_data+1);    
  } else {
    tuh_asix_mount_cb(itf);
    usbh_driver_set_config_complete(itf->dev_addr, 0);

    // start receiving data
    tuh_asix_receive(itf->dev_addr, 0);
    tuh_asix_receive(itf->dev_addr, 1);
  }
}
  
bool asixh_set_config(uint8_t dev_addr, __attribute__((unused)) uint8_t itf_num) {
  // set interface and trigger setup process
  tuh_interface_set(dev_addr, 0, 0, process_set_config, 0);
  return true;
}

bool tuh_asix_receive(uint8_t dev_addr, uint8_t ep) {
  asixh_interface_t *itf = get_interface(dev_addr);
  TU_VERIFY(itf, 0);
  
  TU_VERIFY(usbh_edpt_claim(dev_addr, itf->ep[ep]), false);
  uint8_t *buf = (ep==0)?itf->ep0in_buf:itf->ep1in_buf;
  uint16_t len = (ep==0)?sizeof(itf->ep0in_buf):sizeof(itf->ep1in_buf);
  
  if(!usbh_edpt_xfer(dev_addr, itf->ep[ep], buf, len)) {
    usbh_edpt_release(dev_addr, itf->ep[ep]);
    return false;
  }
  return true;
}

bool asixh_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
  // printf("%s(%d,%02x,%lu)\n", __FUNCTION__, dev_addr, ep_addr, xferred_bytes);
  
  uint8_t const dir = tu_edpt_dir(ep_addr);
  uint8_t const number = tu_edpt_number(ep_addr);

  asixh_interface_t *itf = get_interface(dev_addr);
  TU_VERIFY(itf, false);

  uint8_t *buf = (number==1)?itf->ep0in_buf:itf->ep1in_buf;
  
  if (result != XFER_RESULT_SUCCESS)
    return false;

  // handle incoming data
  if(dir == TUSB_DIR_IN && xferred_bytes) {  

    if(number == 1) {
      // primary or secondary link detected?
      bool link_detected = ((buf[2] & 3) != 0); 
      
      if(link_detected != itf->link_detected) {
	itf->link_detected = link_detected;
	if(link_detected) netif_set_link_up(&itf->netif);
	else              netif_set_link_down(&itf->netif);
      }

      // receive next interrupt message
      tuh_asix_receive(dev_addr, 0);
    }
    
    // ep 2 is bulk data in
    if(number == 2) {
      // read length field and verify its bit inverse
      uint16_t len = (buf[0] + 256*buf[1]) & 0x7ff;
      if(len == ((0xffff ^ (buf[2] + 256*buf[3])) & 0x7ff)) {
	// check if there's sufficient data inside the packet
	if(xferred_bytes >= (uint32_t)len+4) {
	  // netif up? This may not be the case if USB network card was detected
	  // before the lwip stack was up
	  if(itf->netif.input) {
	    struct pbuf* p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);	
	    pbuf_take(p, buf+4, len);
	    
	    if (itf->netif.input(p, &(itf->netif)) != ERR_OK)
	      pbuf_free(p);
	  }
	}
      }
      // receive next packet
      tuh_asix_receive(dev_addr, 1);
    }
  }
  
  return true;
}

void tuh_asix_transmit(struct netif *netif, uint8_t *data, uint16_t len) {
  // printf("%s(%p,%p,%u)\n", __FUNCTION__, netif, data, len);

  // find asix interface matching this netif
  asixh_interface_t *itf = NULL;
  for(int i=0;i<CFG_TUH_ASIX;i++)
    if(&_devices[i].netif == netif)
      itf = &_devices[i];

  TU_VERIFY(itf, );
  TU_VERIFY(usbh_edpt_claim(itf->dev_addr, itf->ep[2]), );

  *(uint16_t*)&(itf->epout_buf[0]) =  len;
  *(uint16_t*)&(itf->epout_buf[2]) = ~len;
  memcpy(itf->epout_buf+4, data, len);
  
  if(!usbh_edpt_xfer(itf->dev_addr, itf->ep[2], itf->epout_buf, len+4))
    usbh_edpt_release(itf->dev_addr, itf->ep[2]);

  return;
}

void asixh_close(uint8_t dev_addr) {
  asixh_interface_t *itf = get_interface(dev_addr);
  TU_VERIFY(itf, );
  
  tuh_asix_umount_cb(itf);
  
  tu_memclr(itf, sizeof(asixh_interface_t));
}

#ifndef DRIVER_NAME
#if CFG_TUSB_DEBUG >= CFG_TUH_LOG_LEVEL
  #define DRIVER_NAME(_name)    .name = _name,
#else
  #define DRIVER_NAME(_name)
#endif
#endif

usbh_class_driver_t const usbh_asix_driver = {
  DRIVER_NAME("ASIX")
  .init       = asixh_init,
  .open       = asixh_open,
  .set_config = asixh_set_config,
  .xfer_cb    = asixh_xfer_cb,
  .close      = asixh_close
};

#endif
