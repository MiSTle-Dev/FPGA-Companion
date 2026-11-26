#include "board.h"

#ifdef ENABLE_WIFI
#warning "WiFi support enabled"
#else
#warning "WiFi support disabled"
#endif

#ifdef ENABLE_BLUETOOTH
#include "../bluetooth.h"
#endif

#ifdef ENABLE_BLUETOOTH
#ifndef ENABLE_WIFI
#error "Bluetooth needs WiFi to be enabled!"
#endif
#warning "Bluetooth support enabled"
#else
#warning "Bluetooth support disabled"
#endif

extern CFG_TUH_MEM_SECTION struct {
  TUH_EPBUF_TYPE_DEF(tusb_desc_device_t, device);
  TUH_EPBUF_DEF(serial, 64*sizeof(uint16_t));
  TUH_EPBUF_DEF(buf, 128*sizeof(uint16_t));
} desc;

#include "../mcu_hw.h"
#include "../at_wifi.h"
#include "asix_host.h"
#include "../menu.h"
#include "../inifile.h"
#include "../sysctrl.h"
#include "hardware/clocks.h"

// the network connection state
#define NETWORK_STATUS_UNINITIALIZED        (0)
#define NETWORK_STATUS_TCPIP_INIT         (1<<0) // lwip has been initialized
#define NETWORK_STATUS_WIFI               (1<<1) // current setup is wifi based
#define NETWORK_STATUS_UP                 (1<<2) // wifi connected or ethernet up
#define NETWORK_STATUS_HAS_ADDR           (1<<3) // IP address is valid
#define NETWORK_STATUS_SNTP_STARTED       (1<<4) // sntp app is running
#define NETWORK_STATUS_TCP_CONNECTED      (1<<5) // the at wifi tcp connection is established
#define NETWORK_STATUS_WIFI_AUTO          (1<<6) // wifi started from config file (no serial at-wifi IO)

static uint8_t network_status = NETWORK_STATUS_UNINITIALIZED;
static bool asix_unmount_in_progress = false;


/* ======================================================================= */
/* ======                   ASIX ethernet                        ========= */
/* ======================================================================= */

#if CFG_TUH_CDC 
#define ENABLE_PPP
#endif

// LWIP network interface
#include "lwip/etharp.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include "lwip/apps/sntp.h"

static err_t netif_asix_output(struct netif *netif, struct pbuf *p) {
  if(network_status == NETWORK_STATUS_UNINITIALIZED) return ERR_OK;
  
  tuh_asix_transmit(netif, p->payload, p->tot_len);
  return ERR_OK;
}

static err_t netif_asix_low_init(struct netif *netif) {
  netif->linkoutput = netif_asix_output;
  netif->output     = etharp_output;
  netif->mtu        = 1500; 
  netif->flags      = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_IGMP | NETIF_FLAG_MLD6;

  return ERR_OK;
}

static void netif_link_callback(struct netif *netif) {
  usb_debugf("netif link status changed %s", netif_is_link_up(netif) ? "up" : "down");
  if(netif_is_link_up(netif)) {
    network_status |= NETWORK_STATUS_UP;

    // If status callback was missed/raced, still surface IP once link is up.
    if(!ip4_addr_isany_val(*netif_ip4_addr(netif))) {
      if(!(network_status & NETWORK_STATUS_HAS_ADDR)) {
        usb_debugf("ASIX: link up with existing ip address");
        menu_notify_ip(ip4addr_ntoa(netif_ip4_addr(netif)));
      }
      network_status |= NETWORK_STATUS_HAS_ADDR;
    }
  } else {
    network_status &= ~NETWORK_STATUS_UP;
    network_status &= ~NETWORK_STATUS_HAS_ADDR;
    if(!asix_unmount_in_progress)
      menu_notify_network_disconnected();
  }
}

#define SNTP_FALLBACK_SERVER "pool.ntp.org"

static void ntp_setup(struct netif *netif) {  
  if(!(network_status & NETWORK_STATUS_SNTP_STARTED)) {
    sntp_init();
    network_status |= NETWORK_STATUS_SNTP_STARTED;
  }
	
  // set ntp server from config if specified there
  if(inifile_config_has("ntp", "ip")) {
    for(int i=0;i<inifile_config_num_values("ntp", "ip");i++) {
      ip_addr_t sa;
      ip_addr_set_ip4_u32(&sa, htonl(inifile_config_get_ip_n("ntp", "ip", 0, i)));
      sntp_setserver(i, &sa);
    }
  }

  // only request ntp server via dhcp if it has not been set explicitely
  // and if the interface is ppp as that cannot use dhcp
  if(!netif || netif->name[0] != 'p' || netif->name[1] != 'p') 
    sntp_servermode_dhcp(!inifile_config_has("ntp", "ip"));

  // neither config.ini nor DHCP named a server: use the public pool. A server
  // from DHCP later replaces it.
  if(ip_addr_isany(sntp_getserver(0)) && sntp_getservername(0) == NULL)
    sntp_setservername(0, SNTP_FALLBACK_SERVER);
}

// this is actually called by axis _and_ the wifi
static void netif_status_callback(struct netif *netif) {
  usb_debugf("netif status changed %s", ip4addr_ntoa(netif_ip4_addr(netif)));

  // check if we got a valid IP (not ANY, which is 0.0.0.0)
  if(!ip4_addr_isany_val(*netif_ip4_addr(netif))) {
    if(!(network_status & NETWORK_STATUS_HAS_ADDR)) {
      // just got an ip address, start services
      usb_debugf("just got an ip address");
      menu_notify_ip(ip4addr_ntoa(netif_ip4_addr(netif)));

      // display what sntp server is actually set (via DHCP)
      usb_debugf("SNTP server 0 is %s", ip4addr_ntoa(sntp_getserver(0)));
      ntp_setup(NULL);  // re-setup as dhcp may have been overwritten the ntp address
    }
    
    network_status |=  NETWORK_STATUS_HAS_ADDR;
  } else {
    network_status &= ~NETWORK_STATUS_HAS_ADDR;
    if(!asix_unmount_in_progress)
      menu_notify_network_disconnected();
  }
}
  
// re-use parts of the existing PICO/WIFI integration
#include "pico/async_context_freertos.h"
#include "pico/lwip_freertos.h"
#include "pico/cyw43_arch.h"

/* Number of seconds between 1970 and Feb 7, 2036 06:28:16 UTC (epoch 1) */
#define DIFF_SEC_1970_2036          ((u32_t)2085978496L)

void sntp_set_system_time(u32_t sec) {
  debugf("%s(%lu)", __FUNCTION__, sec);

  time_t ut = sec + 3600 * inifile_config_get_int("ntp", "timezone", 0);
  struct tm* timeinfo = gmtime(&ut);

  // time is UTC ...
  debugf(" YEAR:     %u", 1900 + timeinfo->tm_year);
  debugf(" MONTH:    %u", 1 + timeinfo->tm_mon);
  debugf(" DAY:      %u", timeinfo->tm_mday);
  debugf(" WEEK DAY: %u", timeinfo->tm_wday);
  debugf(" HOUR:     %u", timeinfo->tm_hour);
  debugf(" MINUTE:   %u", timeinfo->tm_min);
  debugf(" SECOND:   %u", timeinfo->tm_sec);
  debugf(" DST:      %s", timeinfo->tm_isdst?"true":"false");

  // send time into core
  sys_set_time(SYS_TIME_FLAGS_NTP | ( timeinfo->tm_isdst?SYS_TIME_FLAGS_DST:0),
	       timeinfo->tm_year, timeinfo->tm_mon, timeinfo->tm_mday + (timeinfo->tm_wday << 5),
	       timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
}

// set the name server of a static network config if config.ini names one
#include "lwip/dns.h"
static void netif_set_static_dns(void) {
  if(!inifile_config_has("network", "dns")) return;
  ip_addr_t dns;
  ip_addr_set_ip4_u32(&dns, htonl(inifile_config_get_ip("network", "dns", 0)));
  dns_setserver(0, &dns);
}

static void netif_up(struct netif *netif) {
  usb_debugf("netif_up(%c%c)", netif->name[0], netif->name[1]);
  
  // assign callbacks for link and status
  netif_set_link_callback(netif, netif_link_callback);
  netif_set_status_callback(netif, netif_status_callback);  

  // set the default interface and bring it up
  netif_set_default(netif);
  netif_set_up(netif);

  ntp_setup(netif);
    
  // don't start dhcp on the ppp interface
  if(netif->name[0] != 'p' || netif->name[1] != 'p')  {
    // Start DHCP client unless configured not to do so
    if(inifile_config_get_int("network", "mode", 1) == 1)    
      dhcp_start(netif);
    else {
      // set static network config. Default is 192.168.0.2/24 and gateway 192.168.0.1
      ip_addr_t ipaddr, netmask, gw;
      ip_addr_set_ip4_u32(&ipaddr, htonl(inifile_config_get_ip("network", "ip", 0xc0a80002)));
      ip_addr_set_ip4_u32(&netmask, htonl(inifile_config_get_ip("network", "mask", 0xffffff00)));
      ip_addr_set_ip4_u32(&gw, htonl(inifile_config_get_ip("network", "gw", 0xc0a80001)));
      netif_set_addr(netif, &ipaddr, &netmask, &gw);
      netif_set_static_dns();
    }
  } else
    usb_debugf("Not starting DHCP for PPP");
}

static void asix_net_register(asixh_interface_t *itf) {
  if(!(network_status & NETWORK_STATUS_TCPIP_INIT)) {
    usb_debugf("Ignoring USB network device since TCP stack is not initialized");
    return;
  }

  // if wifi has been connected, then no USB networking device will be accepted
  if((network_status & NETWORK_STATUS_UP) &&
     (network_status & NETWORK_STATUS_WIFI)) {
    usb_debugf("Ignoring USB network device since WIFI is connected");
    return;
  }

  // It is actually possible to register the usb network if wifi is
  // available but not connected. However, from this point on in this session, wifi is not
  // being used anymore, even if the usb network device is being unplugged.  
  network_status &= ~NETWORK_STATUS_WIFI;
  
  memcpy(itf->netif.hwaddr, itf->mac, ETH_HWADDR_LEN);
  itf->netif.hwaddr_len = ETH_HWADDR_LEN;
    
  // initialize the ASIX Ethernet network interface
  netif_add(&itf->netif, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY, NULL,
	    netif_asix_low_init, netif_input);
  
  itf->netif.name[0] = 'e';
  itf->netif.name[1] = '0';

  netif_up(&itf->netif);
}

void tuh_asix_mount_cb(asixh_interface_t *itf) {
  usb_debugf("%s(%d)", __FUNCTION__, itf->dev_addr);
  
  uint8_t xfer_result = tuh_descriptor_get_device_sync(itf->dev_addr, &desc.device, 18);
  if (XFER_RESULT_SUCCESS != xfer_result) {
    usb_debugf("Failed to get device descriptor");
    return;
  }
  
  usb_debugf("ID[%04x:%04x] ASIX Device", desc.device.idVendor, desc.device.idProduct);
  usb_debugf("Device Descriptor:");
  usb_debugf("  bLength             %u", desc.device.bLength);
  usb_debugf("  bDescriptorType     %u", desc.device.bDescriptorType);
  usb_debugf("  bcdUSB              %04x", desc.device.bcdUSB);
  usb_debugf("  bDeviceClass        %u", desc.device.bDeviceClass);
  usb_debugf("  bDeviceSubClass     %u", desc.device.bDeviceSubClass);
  usb_debugf("  bDeviceProtocol     %u", desc.device.bDeviceProtocol);
  usb_debugf("  bMaxPacketSize0     %u", desc.device.bMaxPacketSize0);
  usb_debugf("  idVendor            0x%04x", desc.device.idVendor);
  usb_debugf("  idProduct           0x%04x", desc.device.idProduct);
  usb_debugf("  bcdDevice           %04x", desc.device.bcdDevice);

  asix_net_register(itf);
}

// Invoked when device with asix device is un-mounted
void tuh_asix_umount_cb(asixh_interface_t *itf) {
  usb_debugf("[%u] ASIX Device is unmounted", itf->dev_addr);

  // ignore unmount if tcpip stack isn't even initialzed or
  // if wifi setup is active
  if(!(network_status & NETWORK_STATUS_TCPIP_INIT) ||
     (network_status & NETWORK_STATUS_WIFI)) {

    if(network_status & NETWORK_STATUS_WIFI)
      usb_debugf("Ignoring USB network device since WIFI interface has been detected");
    
    if(!(network_status & NETWORK_STATUS_TCPIP_INIT))
      usb_debugf("Ignoring USB network device since TCP stack is not initialized");

    return;
  }
  
  asix_unmount_in_progress = true;

  netif_set_down(&itf->netif);

  // unregister netif from stack
  netif_remove(&itf->netif);

  asix_unmount_in_progress = false;

  // On USB unplug, always show a network-down OSD message.
  menu_notify(MENU_EVENT_NETWORK_DISCONNECTED);
}

#ifdef ENABLE_PPP

#include "netif/ppp/ppp.h"
#include "netif/ppp/pppos.h"

// only one CDC device is supported, so only one PPP device can
// be detected
static ppp_pcb *ppp;
static struct netif ppp_netif;

static void ppp_status_cb(ppp_pcb *pcb, int err_code, void *ctx) {
  switch (err_code) {
  case PPPERR_NONE:
    usb_debugf("PPP Connected");
    break;
  case PPPERR_AUTHFAIL:
    usb_debugf("Authentication Failed");
    break;
  default:
    usb_debugf("PPP Error: %d", err_code);
  }
}

static int8_t cdc_idx = -1;
u32_t ppp_output_cb(ppp_pcb *pcb, const void *data, u32_t len, void *ctx) {
  // forward data to serial port
  if(cdc_idx < 0) {
    usb_debugf("CDC not available");
    return 0;
  }

  u32_t res = tuh_cdc_write(cdc_idx, data, len);
  if(res != len) usb_debugf("tuh_cdc_write(): unexpected result %d != %d", res, len);
  
  return res;
}

#endif

static void asix_net_task(__attribute__((unused)) void *parms) {
  // setup async context exactly like the cyw43 does it
  async_context_t *context = cyw43_arch_async_context();
  if (!context) {
    context = cyw43_arch_init_default_async_context();
    cyw43_arch_set_async_context(context);
  }

  lwip_freertos_init(context);
  network_status |= NETWORK_STATUS_TCPIP_INIT;

#ifdef ENABLE_PPP
  usb_debugf("Initializing PPP over serial");

  // initialize ppp to be used with the esp32_ppp dongle
  ppp = pppos_create(&ppp_netif, ppp_output_cb, ppp_status_cb, NULL);
#endif
 
  vTaskDelete(NULL);
}  

void mcu_hw_asix_start(void) {      
  // start a init thread
  xTaskCreate(asix_net_task, (char *)"asix_net_task", 2048, NULL, configMAX_PRIORITIES-10, NULL);
}
  
#if CFG_TUH_CDC 
// Invoked when received new data
void tuh_cdc_rx_cb(uint8_t idx) {
  static uint8_t buf[128];

  // forward cdc interfaces -> console
  uint32_t count = tuh_cdc_read(idx, buf, sizeof(buf));

  // Forward the data into lwip/ppp
  pppos_input(ppp, buf, count);
}

static void process_line_state_cb(tuh_xfer_t *xfer) {
  usb_debugf("process_line_state_cb(%d)", xfer->user_data);

  // on esp32: RTS -> EN (reset)
  //           DTR -> GPIO0  
  usb_debugf("DTR (GPIO0): %d", tuh_cdc_get_dtr(cdc_idx));
  usb_debugf("RTS (EN): %d", tuh_cdc_get_rts(cdc_idx));
    
  vTaskDelay(pdMS_TO_TICKS(10));
    
  // DTR = bit 0, RTS = bit 1
  if(xfer->user_data == 0)
    tuh_cdc_set_rts(cdc_idx, 0, process_line_state_cb, 0x1);
  if(xfer->user_data == 1)
    tuh_cdc_set_rts(cdc_idx, 1, process_line_state_cb, 0x2);
  if(xfer->user_data == 2) {  
    ppp_connect(ppp, 0); // `0` for no holdoff delay
    ppp_set_usepeerdns(ppp, 1);

    netif_up(&ppp_netif);
  }
}

void tuh_cdc_mount_cb(uint8_t idx) {
  cdc_idx = idx;
  tuh_itf_info_t itf_info = { 0 };
  tuh_cdc_itf_get_info(idx, &itf_info);

  usb_debugf("CDC%d Interface is mounted: address = %u, itf_num = %u",
	     idx, itf_info.daddr, itf_info.desc.bInterfaceNumber);

  cdc_line_coding_t line_coding = { 0 };
  if (tuh_cdc_get_local_line_coding(idx, &line_coding)) {
    usb_debugf("  Baudrate: %" PRIu32 ", Stop Bits : %u",
	       line_coding.bit_rate, line_coding.stop_bits);
    usb_debugf("  Parity  : %u, Data Width: %u",
	       line_coding.parity, line_coding.data_bits);
  }

  // trigger first transfer
  tuh_xfer_t xfer =  {.user_data=0 };
  process_line_state_cb(&xfer);
}

void tuh_cdc_umount_cb(uint8_t idx) {
  tuh_itf_info_t itf_info = { 0 };
  tuh_cdc_itf_get_info(idx, &itf_info);

  usb_debugf("CDC Interface is unmounted: address = %u, itf_num = %u",
	     itf_info.daddr, itf_info.desc.bInterfaceNumber);

  cdc_idx = -1;
}
#endif


/* ========================================================================= */
/* ======                              WiFi                           ====== */
/* ========================================================================= */

#ifndef ENABLE_WIFI
void mcu_hw_wifi_scan(void) {
  at_wifi_puts("WiFi not available\r\n");
}
bool mcu_hw_wifi_connect(__attribute__((unused)) char *ssid, __attribute__((unused)) char *key) {
  at_wifi_puts("WiFi not available\r\n");
  return true;
}
#else  
#include "pico/cyw43_arch.h"

// The LED hangs on the cyw43 chip, and writing it needs the cyw43 lock, which
// other work can hold for a while. So the timer only sets the state, and the
// cyw43 context writes it: the timer task never waits for that lock.
static volatile bool led_state_w;   // the state the next write sets

static void led_write_w(__attribute__((unused)) async_context_t *context,
                        __attribute__((unused)) async_when_pending_worker_t *worker) {
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state_w);   // runs in the cyw43 context
}

static async_when_pending_worker_t led_worker_w = { .do_work = led_write_w };

static void led_timer_w(__attribute__((unused)) TimerHandle_t pxTimer) {
  static char state = 0;
  switch(inifile_config_get_int("system", "led", DEFAULT_LED_MODE)) {
  case 0:    
    led_state_w = state & 1;   // blink
    break;
  case 1:    
    led_state_w = 1;
    break;
  case 2:    
    led_state_w = 0;
    break;
  }
  // never blocks, the write follows when the cyw43 context is free
  async_context_set_work_pending(cyw43_arch_async_context(), &led_worker_w);
    
  state = !state;
}

// Sets a 32-bit setting (iovar) of the WiFi chip: the name with its NUL, then the value
// in little endian.
static int wifi_set_iovar_u32(const char *name, uint32_t val) {
  uint8_t buf[32];
  size_t len = strlen(name) + 1;
  if(len + 4 > sizeof(buf)) return -1;
  memcpy(buf, name, len);
  for(int i=0;i<4;i++) buf[len+i] = (val >> (8*i)) & 0xff;
  return cyw43_ioctl(&cyw43_state, CYW43_IOCTL_SET_VAR, len + 4, buf, CYW43_ITF_STA);
}

void mcu_hw_wifi_init(void) {
#ifdef PICO_RP2350
  debugf("Detected Pico2-W");
#else
  debugf("Detected Pico-W");
#endif

  if(cyw43_arch_init_with_country(CYW43_COUNTRY_GERMANY)) {
    debugf("WiFi failed to initialise");
    return;
  }
  
  debugf("WiFi initialised");
  network_status |= (NETWORK_STATUS_TCPIP_INIT | NETWORK_STATUS_WIFI);

  sntp_servermode_dhcp(!inifile_config_has("ntp", "ip"));
  
  cyw43_arch_enable_sta_mode();
  debugf("STA mode enabled");

  cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);

  // After a reset the access point may still hold the old association. Some (seen on a
  // Qualcomm based one) then drop the first authentication while they clear it. Phones send
  // it again within 200 ms. The chip does the same once it may retry the join (default 0).
  if(wifi_set_iovar_u32("assoc_retry_max", 3))
    debugf("WiFi: assoc_retry_max not set");

  // the LED timer hands its writes to the cyw43 context, see led_timer_w()
  async_context_add_when_pending_worker(cyw43_arch_async_context(), &led_worker_w);
  TimerHandle_t led_timer_handle =
    xTimerCreate("LED timer (W)", pdMS_TO_TICKS(200), pdTRUE,
		 NULL, led_timer_w);
  xTimerStart(led_timer_handle, 0);

  netif_set_status_callback(netif_default, netif_status_callback);

#ifdef ENABLE_BLUETOOTH
  // bluetooth needs nothing from config.ini, start it right away
  bluetooth_init();
#endif

  // after a cold start com_task reads config.ini only once the FPGA
  // is up, so wait for it before deciding whether to connect. Without
  // an FPGA it waits until com_task detects one and restarts
  if(!inifile_config_is_read()) {
    debugf("WiFi: waiting for the configuration from the card");
    inifile_config_wait();
    debugf("WiFi: configuration is available");
  }

  // connect to wifi immediately if configured through config file
  if(inifile_config_has("wifi", "ssid") && inifile_config_has("wifi", "pass")) {
    network_status |= NETWORK_STATUS_WIFI_AUTO;

    debugf("Connecting to WiFi '%s'", inifile_config_get_str("wifi", "ssid"));

    if(!mcu_hw_wifi_connect(inifile_config_get_str("wifi", "ssid"),
			    inifile_config_get_str("wifi", "pass"))) {
      debugf("failed");      
      network_status &= ~NETWORK_STATUS_WIFI_AUTO;
    }
  } else
    debugf("No WiFi setup");

  //  ntp_setup(NULL);
  
#ifdef ENABLE_BLUETOOTH
  // this will actually never return. But that is no problem
  // as this task is only needed for wifi init
  bluetooth_run();
#endif
}

static const char *auth_mode_str(int authmode) {
  static const struct { int mode; char *str; } mode_str[] = {
    { 0, "OPEN" },
    { 1, "WEP"  },
    { 2, "WPA2 PSK"  },
    { 3, "WPA WPA2 PSK"  },
    { 4, "WPA PSK"  },
    { 5, "ENTERPRISE"  },
    { 6, "WPA3 PSK"  },
    { 7, "WPA2 WPA3 PSK"  },
    { -1, "<unknown>" }
  };

  int i;
  for(i=0;mode_str[i].mode != -1;i++)
    if(mode_str[i].mode == authmode)
      return mode_str[i].str;

  return mode_str[i].str;
}

static mcu_hw_wifi_scan_cb_func wifi_scan_cb = NULL;
static int scan_result(__attribute__((unused)) void *env, const cyw43_ev_scan_result_t *result) {
  if (result) {
    char str[74];
    // the ssid comes with its length and is not terminated when it
    // fills all 32 bytes
    int ssid_len = result->ssid_len < sizeof(result->ssid) ?
      result->ssid_len : sizeof(result->ssid);
    
    debugf("ssid: %.*s rssi: %d chan: %d sec: %u",
	   ssid_len, result->ssid, result->rssi, result->channel,
	   result->auth_mode);

    if(wifi_scan_cb) {
      // allocate reply. It's up to the callee to free this
      mcu_hw_scan_result_t *res = pvPortMalloc(sizeof(mcu_hw_scan_result_t));
      char *ssid = pvPortMalloc(ssid_len+1);
      memcpy(ssid, result->ssid, ssid_len);
      ssid[ssid_len] = '\0';
      res->ssid = ssid;
      res->next = NULL;
			       
      wifi_scan_cb(res);
    } else {     
      snprintf(str, sizeof(str), "SSID %.*s, RSSI %d, CH %d, %s\r\n",
	       ssid_len, result->ssid, result->rssi, result->channel,
	       auth_mode_str(result->auth_mode));
      
      at_wifi_puts(str);
    }
  }
  return 0;
}

char mcu_hw_wifi_scan_start(mcu_hw_wifi_scan_cb_func cb) {
  cyw43_wifi_scan_options_t scan_options = {0};
  wifi_scan_cb = cb;
  int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result);
  if(err) {
    debugf("Scan failed");
    return -1;
  }
  return 0;
}

static bool wifi_available(void) {
  // refuse to scan wifi stack has not been initialized or it's
  // not a wifi setup (but usb ethernet)
  if(!(network_status & NETWORK_STATUS_TCPIP_INIT) ||
     !(network_status & NETWORK_STATUS_WIFI)) {

    if(!(network_status & NETWORK_STATUS_TCPIP_INIT)) {
      debugf("Ignoring WiFi command since TCP stack is not initialized");
      at_wifi_puts("TCPIP not available\r\n");
    } else if(!(network_status & NETWORK_STATUS_WIFI)) {
      debugf("Ignoring WiFi command since network hardware is not wifi");
      at_wifi_puts("WiFi not available\r\n");
    }    
    return false;
  }  
  return true;
}

void mcu_hw_wifi_scan(void) {
  if(!wifi_available()) return;
  
  debugf("WiFi: Performing scan");

  cyw43_wifi_scan_options_t scan_options = {0};
  wifi_scan_cb = NULL;
  int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result);
  if(err) {
    at_wifi_puts("Scan failed\r\n");
    return;
  }

  at_wifi_puts("Scanning...\r\n");  

  while(cyw43_wifi_scan_active(&cyw43_state))
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Joins and waits for the link, a replacement for cyw43_arch_wifi_connect_timeout_ms().
// When the chip retries a failed authentication (assoc_retry_max), the driver reports the
// failure for a few milliseconds before the retry succeeds, and the SDK function takes that
// as final. Here a failure counts only once it has held for a second. Returns PICO_OK or a
// PICO_ERROR_* code like the SDK function.
static int wifi_join(const char *ssid, const char *key, uint32_t timeout_ms) {
  // start the join, the chip and the driver work on it in the background
  int err = cyw43_arch_wifi_connect_async(ssid, key, CYW43_AUTH_WPA2_AES_PSK);
  if(err) return err;

  TickType_t start = xTaskGetTickCount(), fail_since = 0;
  bool failing = false;            // the link status has been negative since fail_since
  for(;;) {
    // the driver's view of the join, polled every 10 ms
    int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    TickType_t now = xTaskGetTickCount();

    // joined, keys exchanged and an IP address from DHCP: done
    if(status == CYW43_LINK_UP)
      return PICO_OK;

    if(status == CYW43_LINK_NONET) {
      // no access point with this SSID found (out of range or not up yet): start the
      // join again, as the SDK function does
      failing = false;
      err = cyw43_arch_wifi_connect_async(ssid, key, CYW43_AUTH_WPA2_AES_PSK);
      if(err) return err;
    } else if(status < 0) {
      // BADAUTH or FAIL. After an unanswered authentication the driver shows BADAUTH while
      // the chip already sends the authentication again, and a good answer turns the state
      // back to joining. Only a failure that lasts for a second is final, e.g. FAIL once
      // the chip has given up.
      if(!failing) { failing = true; fail_since = now; }
      else if(now - fail_since >= pdMS_TO_TICKS(1000))
        return status == CYW43_LINK_BADAUTH ? PICO_ERROR_BADAUTH : PICO_ERROR_CONNECT_FAILED;
    } else
      failing = false;             // still joining, or joined and waiting for DHCP

    // no link within the time limit, e.g. a wrong key: the driver keeps joining and
    // reports no failure state for that
    if(now - start >= pdMS_TO_TICKS(timeout_ms))
      return PICO_ERROR_TIMEOUT;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// The cyw43 driver always starts DHCP on the STA interface. Apply the
// [NETWORK] section of config.ini the same way netif_up() does for usb ethernet.
static void wifi_apply_network_config(void) {
  if(inifile_config_get_int("network", "mode", 1) == 1) return;  // dhcp (default)

  ip_addr_t ipaddr, netmask, gw;
  ip_addr_set_ip4_u32(&ipaddr, htonl(inifile_config_get_ip("network", "ip", 0xc0a80002)));
  ip_addr_set_ip4_u32(&netmask, htonl(inifile_config_get_ip("network", "mask", 0xffffff00)));
  ip_addr_set_ip4_u32(&gw, htonl(inifile_config_get_ip("network", "gw", 0xc0a80001)));

  struct netif *netif = &cyw43_state.netif[CYW43_ITF_STA];
  cyw43_arch_lwip_begin();
  dhcp_stop(netif);
  netif_set_addr(netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
  netif_set_static_dns();
  cyw43_arch_lwip_end();
  debugf("WiFi: static ip %s", ip4addr_ntoa(netif_ip4_addr(netif)));
}

bool mcu_hw_wifi_connect(char *ssid, char *key) {
  if(!wifi_available()) return false;

  wifi_apply_network_config();

  debugf("WiFI: connect to %s/%s", ssid, key);
  
  if(!(network_status & NETWORK_STATUS_WIFI_AUTO))
    at_wifi_puts("Connecting...");
  
  int err = wifi_join(ssid, key, 30000);
  if(err) {
    debugf("WiFI: connect failed, error %d", err);
    // the chip keeps rejoining in the background otherwise (wrong key: every few seconds)
    cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    if(!(network_status & NETWORK_STATUS_WIFI_AUTO))
      at_wifi_puts("\r\nConnection failed!\r\n");

    return false;
  } else {
    if(!(network_status & NETWORK_STATUS_WIFI_AUTO))
      at_wifi_puts("\r\nConnected\r\n");
    
    network_status |= NETWORK_STATUS_UP;
  }
  return true;
}
#endif

#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

static bool network_available(void) {
  // refuse to scan wifi stack has not been initialized or it's
  // not a wifi setup (but usb ethernet)
  if(!(network_status & NETWORK_STATUS_TCPIP_INIT) ||
     !(network_status & NETWORK_STATUS_UP)) {

    if(!(network_status & NETWORK_STATUS_TCPIP_INIT)) {
      debugf("Ignoring command since TCP stack is not initialized");
      at_wifi_puts("TCPIP not available\r\n");
    } else if(!(network_status & NETWORK_STATUS_UP)) {
      debugf("Ignoring command since network is down or disconnected");
      at_wifi_puts("Network not available\r\n");
    }    
    return false;
  }  
  return true;
}

static struct tcp_pcb *tcp_pcb = NULL;

static err_t mcu_tcp_connected( __attribute__((unused)) void *arg, __attribute__((unused)) struct tcp_pcb *tpcb, err_t err) {
  if (err != ERR_OK) {
    debugf("connect failed %d\n", err);
    return ERR_OK;
  }
  
  debugf("Connected");
  at_wifi_puts("Connected\r\n");
  network_status |= NETWORK_STATUS_TCP_CONNECTED;
  return ERR_OK;
}

// when using irq driven wifi, the tcp socket interface needs to be protected
// by a semaphore
static inline void lwip_check(void) {
  if(network_status & NETWORK_STATUS_WIFI)
    cyw43_arch_lwip_check();
}

static inline void lwip_begin(void) {
  if(network_status & NETWORK_STATUS_WIFI)
    cyw43_arch_lwip_begin();
}

static inline void lwip_end(void) {
  if(network_status & NETWORK_STATUS_WIFI)
    cyw43_arch_lwip_end();
}

static void mcu_tcp_err(__attribute__((unused)) void *arg, err_t err) {
  if( err == ERR_RST) {
    debugf("tcp connection reset");
    at_wifi_puts("\r\nNO CARRIER\r\n");
    network_status &= ~NETWORK_STATUS_TCP_CONNECTED;
  } else if (err == ERR_ABRT) {
    debugf("err abort");
    at_wifi_puts("Connection failed\r\n");
    network_status &= ~NETWORK_STATUS_TCP_CONNECTED;
  } else {
    debugf("tcp_err %d", err);
  }
}

err_t mcu_tcp_recv(__attribute__((unused)) void *arg, struct tcp_pcb *tpcb, struct pbuf *p, __attribute__((unused)) err_t err) {
  if (!p) {
    debugf("No data, disconnected?");
    at_wifi_puts("\r\nNO CARRIER\r\n");
    network_status &= ~NETWORK_STATUS_TCP_CONNECTED;
    return ERR_OK;
  }

  // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
  // can use this method to cause an assertion in debug mode, if this method is called when
  // cyw43_arch_lwip_begin IS needed
  lwip_check();
  if (p->tot_len > 0) {
    // debugf("recv %d err %d", p->tot_len, err);

    for (struct pbuf *q = p; q != NULL; q = q->next)
      at_wifi_puts_n(q->payload, q->len);
    
    tcp_recved(tpcb, p->tot_len);
  }
  pbuf_free(p);
  
  return ERR_OK;
}

static void mcu_tcp_connect(const ip_addr_t *ipaddr, int port) {
  if(!network_available()) return;

  debugf("Connecting to IP %s %d", ipaddr_ntoa(ipaddr), port);
  
  // the address was resolved and we can connect
  tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(ipaddr));
  if (!tcp_pcb) {    
    debugf("Unable to create pcb");
    at_wifi_puts("Connection failed!\r\n");
    return;
  }

  tcp_recv(tcp_pcb, mcu_tcp_recv);
  tcp_err(tcp_pcb, mcu_tcp_err);
  
  lwip_begin();
  err_t err = tcp_connect(tcp_pcb, ipaddr, port, mcu_tcp_connected);
  lwip_end();

  if(err) {
    debugf("tcp_connect() failed"); 
    at_wifi_puts("Connection failed!\r\n");
  }
}

void mcu_hw_tcp_disconnect(void) {
  if(!network_available()) return;
  
  if(!(network_status & NETWORK_STATUS_TCP_CONNECTED))
    at_wifi_puts("Not connected!\r\n");
  else
    tcp_close(tcp_pcb);
}

// Call back with a DNS result
static void dns_found(__attribute__((unused)) const char *hostname, const ip_addr_t *ipaddr, void *arg) {
  if (ipaddr) {
    at_wifi_puts("Using address ");
    at_wifi_puts(ipaddr_ntoa(ipaddr));
    at_wifi_puts("\r\n");

    mcu_tcp_connect(ipaddr, *(int*)arg);
  } else
    at_wifi_puts("Cannot resolve host\r\n");
}

void mcu_hw_tcp_connect(char *host, int port) {
  static int lport;
  static ip_addr_t address;

  if(!network_available()) return;

  lport = port;
  debugf("connecting to %s %d", host, lport);
  
  lwip_begin();
  int err = dns_gethostbyname(host, &address, dns_found, &lport);
  lwip_end();

  if(err != ERR_OK && err != ERR_INPROGRESS) {
    debugf("DNS error");
    at_wifi_puts("Cannot resolve host\r\n");
    return;
  }

  if(err == ERR_OK)
    mcu_tcp_connect(&address, port);

  else if(err == ERR_INPROGRESS) 
    debugf("DNS in progress");
}

bool mcu_hw_tcp_data(unsigned char byte) {
  if(network_status & NETWORK_STATUS_TCP_CONNECTED) {
    lwip_begin();
    err_t err = tcp_write(tcp_pcb, &byte, 1, TCP_WRITE_FLAG_COPY);
    lwip_end();
    if (err != ERR_OK) debugf("Failed to write data %d", err);

    return true;
  }
    
  return false;  // data has not been processed (we are not connected)
}
void mcu_hw_wifi_drop(void) {
  
  // Leave the access point before the restart (core switch, reset from the menu), else it
  // still holds the association and drops the first authentication after the restart. The
  // short wait lets the chip send the disassociation.
  if((network_status & NETWORK_STATUS_WIFI) && (network_status & NETWORK_STATUS_UP)) {
    cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    busy_wait_ms(50);
  }
}
