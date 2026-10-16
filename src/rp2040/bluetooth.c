/*
  bluetooth.c

  Raspberry Pi Pico specific bluetooth HID driver for MiSTle

  We are running RTOS in single core mode. The pico btstack uses flash
  to store known bluetooth keys. Thus
     PICO_FLASH_ASSUME_CORE1_SAFE
  has to be set in CMakeList.txt to make sure the flash wrapper know that
  the second core #1 is unused and that the flash routines can ignore the
  second core.

  Based on:
  https://github.com/bluekitchen/btstack/tree/master/example/spp_streamer_client.c
  https://github.com/bluekitchen/btstack/tree/master/example/hid_host_demo.c
*/

//  /opt/pico-sdk/src/rp2_common/pico_btstack/btstack_flash_bank.c
// flash_safe_execute -> PICO_ERROR_NOT_PERMITTED

#include "../bluetooth.h"
#include "../debug.h"
#include "../hid.h"
#include "../hidparser.h"
#include "../menu.h"

#include "btstack_config.h"
#include <btstack.h>

// #define  ENABLE_BTSTACK_LOGGING

#define HID_DESCRIPTOR_STORAGE_SIZE 1024
#define BT_CONNECT_TIMEOUT_MS 15000

#define MAX_BT_HID_DEVICES 2
#define MAX_BT_HID_REPORTS                                                     \
  4 // up to four different resports per device supported

static btstack_timer_source_t connect_timer;
static void connect_next_detected(void);

typedef enum {
  DEV_UNUSED,
  DEV_DETECTED,   // found by inquiry, connection not yet started
  DEV_CONNECTING, // connection (outgoing or incoming) in progress
  DEV_CONNECTED   // connection established
} bt_device_state_t;

static struct bt_hid_device_S {
  bt_device_state_t dev_state;
  bd_addr_t bd_addr;
  uint16_t hid_host_cid;
  hid_state_t state[MAX_BT_HID_REPORTS];
  hid_report_t rep[MAX_BT_HID_REPORTS];
} bt_hid_device[MAX_BT_HID_DEVICES];

static btstack_packet_callback_registration_t hci_event_callback_registration;

// SDP
static uint8_t hid_descriptor_storage[HID_DESCRIPTOR_STORAGE_SIZE];

// static hid_protocol_mode_t hid_host_report_mode =
// HID_PROTOCOL_MODE_REPORT_WITH_FALLBACK_TO_BOOT;
static hid_protocol_mode_t hid_host_report_mode = HID_PROTOCOL_MODE_REPORT;

static struct bt_hid_device_S *get_dev_by_hid_cid(uint16_t cid) {
  for (int i = 0; i < MAX_BT_HID_DEVICES; i++)
    if (bt_hid_device[i].dev_state != DEV_UNUSED &&
        bt_hid_device[i].dev_state !=
            DEV_DETECTED) // detected devices have no cid yet
      if (bt_hid_device[i].hid_host_cid == cid)
        return &bt_hid_device[i];

  return NULL;
}

static void connect_timeout_handler(btstack_timer_source_t *ts) {
  UNUSED(ts);
  for (int i = 0; i < MAX_BT_HID_DEVICES; i++) {
    struct bt_hid_device_S *dev = &bt_hid_device[i];
    if (dev->dev_state != DEV_CONNECTING)
      continue;
    bt_debugf("Connect timeout for %s", bd_addr_to_str(dev->bd_addr));
    hid_host_disconnect(dev->hid_host_cid);
    memset(dev, 0, sizeof(*dev)); // free the slot regardless
  }
  connect_next_detected();
}

// connect to the next freshly detected device, one at a time
static void connect_next_detected(void) {
  // only one connection attempt at a time
  for (int i = 0; i < MAX_BT_HID_DEVICES; i++)
    if (bt_hid_device[i].dev_state == DEV_CONNECTING)
      return;

  for (int i = 0; i < MAX_BT_HID_DEVICES; i++) {
    struct bt_hid_device_S *dev = &bt_hid_device[i];
    if (dev->dev_state != DEV_DETECTED)
      continue;

    bt_debugf("Start to connect and query for HID service: %s",
              bd_addr_to_str(dev->bd_addr));

    uint16_t cid = 0;
    uint8_t status = hid_host_connect(dev->bd_addr, hid_host_report_mode, &cid);
    if (status != ERROR_CODE_SUCCESS) {
      // nothing was set up, so there is nothing to disconnect. Never call
      // hid_host_disconnect() here: the cid may belong to another, working
      // connection
      bt_debugf("HID host connect failed, status 0x%02x.", status);
      memset(dev, 0, sizeof(*dev)); // free this slot only
      continue;                     // try the next detected device
    }

    bt_debugf("HID host connecting, cid 0x%x.", cid);
    dev->hid_host_cid = cid;
    dev->dev_state = DEV_CONNECTING;
    btstack_run_loop_remove_timer(&connect_timer);
    btstack_run_loop_set_timer_handler(&connect_timer, connect_timeout_handler);
    btstack_run_loop_set_timer(&connect_timer, BT_CONNECT_TIMEOUT_MS);
    btstack_run_loop_add_timer(&connect_timer);
    return;
  }
}

static void packet_handler(uint8_t packet_type, uint16_t channel,
                           uint8_t *packet, uint16_t size) {
  UNUSED(channel);
  UNUSED(size);

  // bt_debugf("packet_handler(%d, %d, %d)", packet_type, channel, size);
  //   if(size) hexdump(packet, size);

  bd_addr_t event_addr;
  uint32_t class_of_device;
  uint8_t status;
  uint8_t evt;

  switch (packet_type) {
  case HCI_EVENT_PACKET:
    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
      if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
        bt_debugf("BTStack up and working");
        // Stuff to be done if the stack is fully initialized may go here
        // (e.g. scan for devices)
      }
      break;

    case GAP_EVENT_INQUIRY_RESULT:
      // get class of device, mask Limited Discoverable Mode
      class_of_device =
          gap_event_inquiry_result_get_class_of_device(packet) & 0xdfff;
      gap_event_inquiry_result_get_bd_addr(packet, event_addr);

      // https://bluetooth-pentest.narod.ru/software/bluetooth_class_of_device-service_generator.html
      // COD 504 - Peripheral; Joystick
      // COD 508 - Peripheral; Gamepad
      // COD 540 - Peripheral; Keyboard
      // COD 580 - Peripheral; Pointing Device
      if ((class_of_device == 0x504) || (class_of_device == 0x508) ||
          (class_of_device == 0x540) || (class_of_device == 0x580)) {
        bt_debugf("%s class device found: %s",
                  (class_of_device == 0x504)   ? "Joystick"
                  : (class_of_device == 0x508) ? "Gamepad"
                  : (class_of_device == 0x540) ? "Keyboard"
                                               : "Mouse",
                  bd_addr_to_str(event_addr));

        // ignore devices we already know
        for (int i = 0; i < MAX_BT_HID_DEVICES; i++) {
          if (bt_hid_device[i].dev_state != DEV_UNUSED &&
              !memcmp(bt_hid_device[i].bd_addr, event_addr, 6)) {
            bt_debugf("Device %s already known", bd_addr_to_str(event_addr));
            return;
          }
        }

        // search a free hid slot
        for (int i = 0; i < MAX_BT_HID_DEVICES; i++) {
          if (bt_hid_device[i].dev_state == DEV_UNUSED) {
            memset(&bt_hid_device[i], 0, sizeof(bt_hid_device[i]));
            memcpy(bt_hid_device[i].bd_addr, event_addr, 6);
            bt_hid_device[i].dev_state = DEV_DETECTED;

            gap_inquiry_stop();
            return;
          }
        }
      } else
        bt_debugf("Unsupported device found: %s with COD: 0x%06x",
                  bd_addr_to_str(event_addr), (int)class_of_device);
      break;

    case GAP_EVENT_INQUIRY_COMPLETE:
      bt_debugf("GAP_EVENT_INQUIRY_COMPLETE");
      connect_next_detected();
      break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
      bt_debugf("HCI: disconnected handle 0x%04x, reason 0x%02x",
                hci_event_disconnection_complete_get_connection_handle(packet),
                hci_event_disconnection_complete_get_reason(packet));
      break;

    case HCI_EVENT_AUTHENTICATION_COMPLETE_EVENT:
      bt_debugf("HCI: authentication complete, status 0x%02x", packet[2]);
      break;

    case HCI_EVENT_ENCRYPTION_CHANGE:
      bt_debugf("HCI: encryption change, status 0x%02x", packet[2]);
      break;

    case GAP_EVENT_SECURITY_LEVEL:
      bt_debugf("HCI: security level %d on handle 0x%04x",
                gap_event_security_level_get_security_level(packet),
                gap_event_security_level_get_handle(packet));
      break;

      // case HCI_EVENT_CONNECTION_COMPLETE:          // outer switch: HCI event
      // type
      //   if(hci_event_connection_complete_get_status(packet) ==
      //   ERROR_CODE_SUCCESS) {
      // hci_con_handle_t h =
      // hci_event_connection_complete_get_connection_handle(packet);
      // bt_debugf("HCI: link up, handle 0x%04x, requesting security", h);
      // gap_request_security_level(h, LEVEL_2);
      //   }
      //   break;

    case HCI_EVENT_PIN_CODE_REQUEST:
      // inform about pin code request
      bt_debugf("Pin code request - using '123456'");
      hci_event_pin_code_request_get_bd_addr(packet, event_addr);
      gap_pin_code_response(event_addr, "123456");
      menu_notify(MENU_EVENT_BLUETOOTH_PIN_CODE_REQUEST);
      break;

    case HCI_EVENT_HID_META:
      // bt_debugf("HCI_EVENT_HID_META(0x%02x)",
      // hci_event_hid_meta_get_subevent_code(packet));

      switch (hci_event_hid_meta_get_subevent_code(packet)) {
      case HID_SUBEVENT_SNIFF_SUBRATING_PARAMS:
        bt_debugf("HID_SUBEVENT_SNIFF_SUBRATING_PARAMS");
        break;

      case HID_SUBEVENT_INCOMING_CONNECTION: {
        // There is an incoming connection: we can accept it or decline it.
        // The hid_host_report_mode in the hid_host_accept_connection function
        // allows the application to request a protocol mode.
        // For available protocol modes, see hid_protocol_mode_t in
        // btstack_hid.h file.
        uint16_t hid_cid = hid_subevent_incoming_connection_get_hid_cid(packet);

        struct bt_hid_device_S *dev = NULL;

        // find a free entry
        for (int i = 0; i < MAX_BT_HID_DEVICES && !dev; i++) {
          if (bt_hid_device[i].dev_state == DEV_UNUSED) {
            dev = &bt_hid_device[i];
            memset(dev, 0, sizeof(*dev));
            dev->dev_state = DEV_CONNECTING;
            dev->hid_host_cid = hid_cid;
            hid_subevent_incoming_connection_get_address(packet, dev->bd_addr);
          }
        }

        if (!dev) {
          bt_debugf(
              "HID_SUBEVENT_INCOMING_CONNECTION: No matching device entry");
          hid_host_decline_connection(hid_cid);
          return;
        }

        // Stop any inquiry in progress
        gap_inquiry_stop();

        bt_debugf("Accepting incoming connection from %s",
                  bd_addr_to_str(dev->bd_addr));
        hid_host_accept_connection(hid_cid, hid_host_report_mode);
      } break;

      case HID_SUBEVENT_CONNECTION_OPENED: {
        btstack_run_loop_remove_timer(&connect_timer);
        struct bt_hid_device_S *dev = get_dev_by_hid_cid(
            hid_subevent_connection_opened_get_hid_cid(packet));

        if (!dev)
          bt_debugf("HID_SUBEVENT_CONNECTION_OPENED: No matching device entry");
        else {
          // The status field of this event indicates if the control and
          // interrupt connections were opened successfully.
          status = hid_subevent_connection_opened_get_status(packet);
          if (status != ERROR_CODE_SUCCESS) {
            bt_debugf("Connection failed, status 0x%02x", status);
            memset(dev, 0, sizeof(*dev)); // free the slot
            connect_next_detected();      // any other device waiting?
            return;
          }

          dev->hid_host_cid =
              hid_subevent_connection_opened_get_hid_cid(packet);
          dev->dev_state = DEV_CONNECTED;
          bt_debugf("HID Host connected.");
          connect_next_detected(); // any other device waiting?
        }
      } break;

      case HID_SUBEVENT_DESCRIPTOR_AVAILABLE: {
        struct bt_hid_device_S *dev = get_dev_by_hid_cid(
            hid_subevent_descriptor_available_get_hid_cid(packet));

        if (!dev)
          bt_debugf(
              "HID_SUBEVENT_DESCRIPTOR_AVAILABLE: No matching device entry");
        else {
          bt_debugf("HID_SUBEVENT_DESCRIPTOR_AVAILABLE");
          // This event follows a HID_SUBEVENT_CONNECTION_OPENED event.
          // For incoming connections, i.e. HID Device initiating the
          // connection, the HID_SUBEVENT_DESCRIPTOR_AVAILABLE is delayed, and
          // some HID reports may be received via HID_SUBEVENT_REPORT event. It
          // is up to the application if these reports should be buffered or
          // ignored until the HID descriptor is available.
          status = hid_subevent_descriptor_available_get_status(packet);
          if (status == ERROR_CODE_SUCCESS) {
            const uint8_t *hid_descriptor =
                hid_descriptor_storage_get_descriptor_data(dev->hid_host_cid);
            uint16_t hid_descriptor_len =
                hid_descriptor_storage_get_descriptor_len(dev->hid_host_cid);
            bt_debugf("hid descriptor (%d):", hid_descriptor_len);
            hexdump(hid_descriptor, hid_descriptor_len);

            // Bluetooth devices have a tendency to report mutliple report
            // types. This happens less often with USB devices as these can have
            // multiple interfaces and can expose different capabilities that
            // way.

            // clear all reports
            for (int i = 0; i < MAX_BT_HID_REPORTS; i++)
              dev->rep[i].type = REPORT_TYPE_NONE;

            // parse all report descriptors
            for (int i = 0; (i < MAX_BT_HID_REPORTS) && hid_descriptor_len;
                 i++) {
              uint16_t rbytes;
              if (parse_report_descriptor(hid_descriptor, hid_descriptor_len,
                                          &dev->rep[i], &rbytes)) {
                if (dev->rep[i].type == REPORT_TYPE_JOYSTICK)
                  dev->state[i].joystick.js_index = hid_allocate_joystick();

                // report descriptors successfully parsed, try to parse further
                // descriptors
                hid_descriptor_len -= rbytes;
                hid_descriptor += rbytes;

              } else
                hid_descriptor_len = 0;
            }

            // check if any usable report has been found
            if (dev->rep[0].type == REPORT_TYPE_NONE) {
              // report parsing failed, reject device
              hid_host_disconnect(dev->hid_host_cid);
            } else
              menu_notify(MENU_EVENT_BLUETOOTH_CONNECTED);

          } else {
            fatal_debugf("Cannot handle input report, HID Descriptor is not "
                         "available, status 0x%02x",
                         status);
            hid_host_disconnect(dev->hid_host_cid);
          }
        }
      } break;

      case HID_SUBEVENT_REPORT: {
        struct bt_hid_device_S *dev =
            get_dev_by_hid_cid(hid_subevent_report_get_hid_cid(packet));

        if (!dev)
          bt_debugf("HID_SUBEVENT_REPORT: No matching device entry");
        else {
          uint8_t const *data = hid_subevent_report_get_report(packet);
          uint16_t len = hid_subevent_report_get_report_len(packet);

#if BT_DEBUG_REPORTS
          bt_debugf("HID_SUBEVENT_REPORT");
          printf("Report ");
          hexdump(data, len);
#endif

          if (len && data[0] == 0xa1) { // report type: "input report"
            data++;
            len--; // skip report type

            // If no report id is included in a the report, then only
            // one type of report can exist as otherwise the different
            // reports could not be distinguished.
            if (!dev->rep[0].report_id_present) {
              hid_parse(&dev->rep[0], &dev->state[0], data, len);
            } else {
              // report id's are present, so check which report description
              // the current report matches
              for (int i = 0; i < MAX_BT_HID_REPORTS; i++) {
                // check report id if present
                if (dev->rep[i].report_id_present &&
                    (len - 1 == dev->rep[i].report_size)) {
                  if (data[0] == dev->rep[i].report_id) {
#if BT_DEBUG_REPORTS
                    bt_debugf("report %d matches id %d", i,
                              dev->rep[i].report_id);
#endif
                    hid_parse(&dev->rep[i], &dev->state[i], data, len);
                  }
                }
              }
            }
          }
        }
      } break;

      case HID_SUBEVENT_SET_PROTOCOL_RESPONSE:
        // For incoming connections, the library will set the protocol mode of
        // the HID Device as requested in the call to
        // hid_host_accept_connection. The event reports the result. For
        // connections initiated by calling hid_host_connect, this event will
        // occur only if the established report mode is boot mode.
        status =
            hid_subevent_set_protocol_response_get_handshake_status(packet);
        if (status != HID_HANDSHAKE_PARAM_TYPE_SUCCESSFUL) {
          bt_debugf("Error set protocol, status 0x%02x", status);
          break;
        }
        switch (
            (hid_protocol_mode_t)
                hid_subevent_set_protocol_response_get_protocol_mode(packet)) {
        case HID_PROTOCOL_MODE_BOOT:
          bt_debugf("Protocol mode set: BOOT.");
          break;
        case HID_PROTOCOL_MODE_REPORT:
          bt_debugf("Protocol mode set: REPORT.");
          break;
        default:
          bt_debugf("Unknown protocol mode.");
          break;
        }
        break;

      case HID_SUBEVENT_CONNECTION_CLOSED: {
        struct bt_hid_device_S *dev = get_dev_by_hid_cid(
            hid_subevent_connection_closed_get_hid_cid(packet));

        if (!dev)
          bt_debugf("HID_SUBEVENT_CONNECTION_CLOSED: No matching device entry");
        else {
          bt_debugf("HID Host disconnected.");
          // The connection was closed.

          // release all joysticks
          for (int i = 0; i < MAX_BT_HID_REPORTS; i++)
            if (dev->rep[i].type == REPORT_TYPE_JOYSTICK)
              hid_release_joystick(dev->state[i].joystick.js_index);

          // mark the list entry as free/unused
          memset(dev, 0, sizeof(*dev)); // dev_state = DEV_UNUSED, cid = 0

          menu_notify(MENU_EVENT_BLUETOOTH_DISCONNECTED);
          connect_next_detected(); // any other device waiting?
        }
      } break;
      default:
        break;
      }
      break;

    default:
      evt = hci_event_packet_get_type(packet);
      // skip the noisy ones: command complete/status, completed packets,
      // transport sent
      if (evt != 0x0e && evt != 0x0f && evt != 0x13 && evt != 0x6e)
        // bt_debugf("[%lu] HCI_EVENT(0x%02x)", (unsigned
        // long)btstack_run_loop_get_time_ms(), evt);
        break;
    }
    break;
  }
}

#ifdef ENABLE_BTSTACK_LOGGING
static void bt_log_reset(void) {}

static void bt_log_packet(uint8_t packet_type, uint8_t in, uint8_t *packet,
                          uint16_t len) {
  if (packet_type != HCI_ACL_DATA_PACKET || len < 8)
    return;
  if (little_endian_read_16(packet, 6) != 1)
    return; // L2CAP signaling channel only

  char buf[3 * 24 + 1];
  int n = (len - 8) < 24 ? (len - 8) : 24;
  for (int i = 0; i < n; i++)
    sprintf(buf + 3 * i, "%02x ", packet[8 + i]);
  buf[3 * n] = 0;
  bt_debugf("L2CAP sig %s: %s", in ? "RX" : "TX", buf);
}

static void bt_log_message(int log_level, const char *format, va_list argptr) {
  UNUSED(log_level);
  UNUSED(format);
  UNUSED(argptr); // drop BTstack's own messages
}

static hci_dump_t hci_dump_impl = {.reset = bt_log_reset,
                                   .log_packet = bt_log_packet,
                                   .log_message = bt_log_message};
#endif // ENABLE_BTSTACK_LOGGING

void bluetooth_init(void) {
  bt_debugf("init");
  //   gap_delete_all_link_keys(); // flush all keys, repairs neccessary

  for (int i = 0; i < MAX_BT_HID_DEVICES; i++)
    bt_hid_device[i].dev_state = DEV_UNUSED;

#ifdef ENABLE_BTSTACK_LOGGING
  hci_dump_init(&hci_dump_impl);
#endif

  // ============ setup bluetooth =============
  // Initialize L2CAP
  l2cap_init();
  sdp_init();

  gap_ssp_set_enable(true);
  gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
  gap_ssp_set_auto_accept(true);
  gap_ssp_set_authentication_requirement(
      SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_GENERAL_BONDING);

  gap_set_local_name("MiSTle FPGA Companion");

  // Initialize HID Host
  hid_host_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));
  hid_host_register_packet_handler(packet_handler);

  // Allow sniff mode requests by HID device and support role switch
  gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE |
                                       LM_LINK_POLICY_ENABLE_ROLE_SWITCH);

  // try to become master on incoming connections
  hci_set_master_slave_policy(HCI_ROLE_MASTER);

  // register for HCI events
  hci_event_callback_registration.callback = &packet_handler;
  hci_add_event_handler(&hci_event_callback_registration);

  // make discoverable to allow HID device to initiate connection
  gap_discoverable_control(1);

  // turn on!
  hci_power_control(HCI_POWER_ON);
}

void bluetooth_run(void) {
  bt_debugf("entering main loop");

  // this loop will never terminate
  btstack_run_loop_execute();
}

void bluetooth_scan(void) {
  bt_debugf("Starting 10 second inquiry scan..");
  gap_inquiry_start(10); // start scanning for 10 seconds
}