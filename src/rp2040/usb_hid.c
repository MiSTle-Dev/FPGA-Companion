/*
  rp2040/usb_hid.c
*/

#include "board.h"
#ifdef ENABLE_USB_HID

#include "../hid.h"
#include "../hidparser.h"

#include "usb_hid.h"
#include "../usb_controller_maps_tools.h"

#define MAX_REPORT_PER_IFACE  2

extern CFG_TUH_MEM_SECTION struct {
  TUH_EPBUF_TYPE_DEF(tusb_desc_device_t, device);
  TUH_EPBUF_DEF(serial, 64*sizeof(uint16_t));
  TUH_EPBUF_DEF(buf, 128*sizeof(uint16_t));
} desc;

static struct {
  uint8_t dev_addr;
  uint8_t instance;

  // allow for multiple reports per interface
  struct {  
    hid_state_t state;
    hid_report_t desc;
  } report[MAX_REPORT_PER_IFACE];

} hid_device[MAX_HID_DEVICES];

void usb_hid_init(void) {
  // mark all hid and xbox entries as unused
  for(int i=0;i<MAX_HID_DEVICES;i++)
    hid_device[i].dev_addr = 0xff;
}

int usb_hid_present(int type) {
  int retval = 0;
  
  for(int idx=0;idx<MAX_HID_DEVICES;idx++) {
    if(hid_device[idx].dev_addr != 0xff) {
      for(int i=0;i<MAX_REPORT_PER_IFACE;i++) {      
	if((type && USB_HID_KEYBOARD) &&
	   (hid_device[idx].report[i].desc.type == REPORT_TYPE_KEYBOARD))
	  retval++;;

	if((type && USB_HID_JOYSTICK) &&
	   (hid_device[idx].report[i].desc.type == REPORT_TYPE_JOYSTICK))
	  retval++;;

	if((type && USB_HID_MOUSE)    &&
	   (hid_device[idx].report[i].desc.type == REPORT_TYPE_MOUSE))
	  retval++;;
      }
    }
  }
  return retval;
}


// English
#define LANGUAGE_ID 0x0409

//--------------------------------------------------------------------+
// String Descriptor Helper
//--------------------------------------------------------------------+
static void _convert_utf16le_to_utf8(const uint16_t* utf16, size_t utf16_len, uint8_t* utf8, size_t utf8_len) {
  // TODO: Check for runover.
  (void) utf8_len;
  // Get the UTF-16 length out of the data itself.

  for (size_t i = 0; i < utf16_len; i++) {
    uint16_t chr = utf16[i];
    if (chr < 0x80) {
      *utf8++ = chr & 0xffu;
    } else if (chr < 0x800) {
      *utf8++ = (uint8_t) (0xC0 | (chr >> 6 & 0x1F));
      *utf8++ = (uint8_t) (0x80 | (chr >> 0 & 0x3F));
    } else {
      // TODO: Verify surrogate.
      *utf8++ = (uint8_t) (0xE0 | (chr >> 12 & 0x0F));
      *utf8++ = (uint8_t) (0x80 | (chr >> 6 & 0x3F));
      *utf8++ = (uint8_t) (0x80 | (chr >> 0 & 0x3F));
    }
    // TODO: Handle UTF-16 code points that take two entries.
  }
} 

// Count how many bytes a utf-16-le encoded string will take in utf-8.
static int _count_utf8_bytes(const uint16_t* buf, size_t len) {
  size_t total_bytes = 0;
  for (size_t i = 0; i < len; i++) {
    uint16_t chr = buf[i];
    if (chr < 0x80) {
      total_bytes += 1;
    } else if (chr < 0x800) {
      total_bytes += 2;
    } else {
      total_bytes += 3;
    }
    // TODO: Handle UTF-16 code points that take two entries.
  }
  return (int) total_bytes;
}

static void print_utf16(uint16_t* temp_buf, size_t buf_len) {
  if ((temp_buf[0] & 0xff) == 0) return;  // empty
  size_t utf16_len = ((temp_buf[0] & 0xff) - 2) / sizeof(uint16_t);
  size_t utf8_len = (size_t) _count_utf8_bytes(temp_buf + 1, utf16_len);
  _convert_utf16le_to_utf8(temp_buf + 1, utf16_len, (uint8_t*) temp_buf, sizeof(uint16_t) * buf_len);
  ((uint8_t*) temp_buf)[utf8_len] = '\0';

  usb_debugf("%s", (char*) temp_buf);
}

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len) {
  // Interface protocol (hid_interface_protocol_enum_t)
  const char* protocol_str[] = { "None", "Keyboard", "Mouse" };
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);
  
  // check for Sony PS3 / Speedlink Competition Pro V3 (054c:0268)
  if (vid == 0x054c && pid == 0x0268) {
    static uint8_t const magic_init[] = { 0x42, 0x0c, 0x00, 0x00 };
    // We send a Set_Report (Feature) to activate the controller
    tuh_hid_set_report(dev_addr, instance, 0xf4, HID_REPORT_TYPE_FEATURE, (void*)magic_init, sizeof(magic_init));
    usb_debugf("PS3-Mode Joystick activated!\n");
   }

  uint8_t xfer_result = tuh_descriptor_get_device_sync(dev_addr, &desc.device, 18);
  if (XFER_RESULT_SUCCESS != xfer_result) {
    usb_debugf("Failed to get device descriptor");
  }

  usb_debugf("ID[%04x:%04x] [%u] HID Interface%u, Protocol = %s", desc.device.idVendor, desc.device.idProduct, dev_addr, instance, protocol_str[itf_protocol]);
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

  xfer_result = XFER_RESULT_FAILED;
  if (desc.device.iSerialNumber != 0) {
    xfer_result = tuh_descriptor_get_serial_string_sync(dev_addr, LANGUAGE_ID, desc.serial, sizeof(desc.serial));
  }
  if (XFER_RESULT_SUCCESS != xfer_result) {
    uint16_t* serial = (uint16_t*)(uintptr_t) desc.serial;

    serial[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * 1 + 2));
    serial[1] = '0'; // simply 0
    serial[2] = 0;
  }
  print_utf16((uint16_t*)(uintptr_t) desc.serial, sizeof(desc.serial)/2);

  usb_debugf("  iManufacturer       %u     ", desc.device.iManufacturer);
  if (desc.device.iManufacturer != 0) {
    xfer_result = tuh_descriptor_get_manufacturer_string_sync(dev_addr, LANGUAGE_ID, desc.buf, sizeof(desc.buf));
    if (XFER_RESULT_SUCCESS == xfer_result) {
      print_utf16((uint16_t*)(uintptr_t) desc.buf, sizeof(desc.buf)/2);
    }
  }

  usb_debugf("  iProduct            %u     ", desc.device.iProduct);
  if (desc.device.iProduct != 0) {
    xfer_result = tuh_descriptor_get_product_string_sync(dev_addr, LANGUAGE_ID, desc.buf, sizeof(desc.buf));
    if (XFER_RESULT_SUCCESS == xfer_result) {
      print_utf16((uint16_t*)(uintptr_t) desc.buf, sizeof(desc.buf)/2);
    }
  }
  usb_debugf("  iSerialNumber       %u     ", desc.device.iSerialNumber);
  usb_debugf("%s", (char*)desc.serial); // serial is already to UTF-8
  usb_debugf("  bNumConfigurations  %u", desc.device.bNumConfigurations);

  // search for a free hid entry
  int idx;
  for(idx=0;idx<MAX_HID_DEVICES && (hid_device[idx].dev_addr != 0xff);idx++);
  if(idx != MAX_HID_DEVICES) {
    usb_debugf("Using HID entry %d", idx);

    // Some device return broken hid descriptor reports. Just like the Linux
    // kernel we replace these.
    fix_report_descriptor(desc.device.idVendor, desc.device.idProduct, desc.device.bcdDevice, (uint8_t*)desc_report, desc_len);
    
    uint16_t rbytes = 0;
    if(parse_report_descriptor(desc_report, desc_len, &hid_device[idx].report[0].desc, &rbytes)) {
      hid_device[idx].dev_addr = dev_addr;
      hid_device[idx].instance = instance;

      // One report descriptor was successfully parsed. So accept the device and
      // try to search for more usable descriptors
      
      // mark all remaining entries as unused
      for(int i=1;i<MAX_REPORT_PER_IFACE;i++)
	hid_device[idx].report[i].desc.type = REPORT_TYPE_NONE;

      if(hid_device[idx].report[0].desc.type == REPORT_TYPE_JOYSTICK)
	hid_device[idx].report[0].state.joystick.js_index = hid_allocate_joystick();
      
      // Check if there's significant unused data in the report desciptor.
      // This likely means that there's annother report to be parsed.
      int rep_idx = 1;
      while((desc_len - rbytes > 0) && (rep_idx < MAX_REPORT_PER_IFACE)) {
	desc_report += rbytes;
	desc_len -= rbytes;

	usb_debugf("Parse remaining %d bytes to report %d", desc_len, rep_idx);
	
	if(parse_report_descriptor(desc_report, desc_len, &hid_device[idx].report[rep_idx].desc, &rbytes)) {
	  if(hid_device[idx].report[rep_idx].desc.type == REPORT_TYPE_JOYSTICK)
	    hid_device[idx].report[rep_idx].state.joystick.js_index = hid_allocate_joystick();

	  rep_idx++;
	}
      }      
    } else
      usb_debugf("Ignoring device");
  } else
    usb_debugf("Error, no more free HID entries");

  /* check controller map for all joysticks found */
  for(int i=0;i<MAX_REPORT_PER_IFACE;i++) {
    if(hid_device[idx].report[i].desc.type == REPORT_TYPE_JOYSTICK) {
      // lookup for VID/PID/Version
      const void *map = usb_controller_maps_find(desc.device.idVendor, desc.device.idProduct, desc.device.bcdDevice);
      
      if (map) {
	// Check of this is a data frog pc07 and force our custom map then. 
	if((hid_device[idx].report[0].desc.type == REPORT_TYPE_JOYSTICK) &&
	   (hid_device[idx].report[1].desc.type == REPORT_TYPE_JOYSTICK))
	  map = usb_controller_maps_get_data_frog_pc07(map);

	usb_controller_maps_show(map);
	
	hid_device[idx].report[i].desc.map = map;
	hid_device[idx].report[i].desc.map_found = 1;
	hid_device[idx].report[i].desc.map_checked = 1;
      } else {
	usb_debugf("No map for VID=%04x PID=%04x, VERSION=%04x",desc.device.idVendor, desc.device.idProduct, desc.device.bcdDevice);
	hid_device[idx].report[i].desc.map = NULL;
	hid_device[idx].report[i].desc.map_found = 0;
	hid_device[idx].report[i].desc.map_checked = 1;
      }
    }
  }

  // tuh_hid_report_received_cb() will be invoked when report is available
  if (!tuh_hid_receive_report(dev_addr, instance) ) 
    usb_debugf("Error: cannot request report");

  mcu_hw_usb_check_devices();
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  usb_debugf("[%u] HID Interface%u is unmounted", dev_addr, instance);

  // find matching hid report
  for(int idx=0;idx<MAX_HID_DEVICES;idx++) {
    if(hid_device[idx].dev_addr == dev_addr && hid_device[idx].instance == instance) {
      usb_debugf("releasing %d", idx);
      hid_device[idx].dev_addr = 0xff;
      for(int i=0;i<MAX_REPORT_PER_IFACE;i++) {
	if(hid_device[idx].report[i].desc.type == REPORT_TYPE_KEYBOARD)
	  kbd_release_all(&hid_device[idx].report[i].state.kbd);

	if(hid_device[idx].report[i].desc.type == REPORT_TYPE_JOYSTICK) {
	  if(hid_device[idx].report[i].state.joystick.js_index != NO_JOYSTICK)	
	    hid_release_joystick(hid_device[idx].report[i].state.joystick.js_index);
	}
      }
    }
  }
  mcu_hw_usb_check_devices();
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
  //usb_debugf("[%u] HID Interface%u %p/%d", dev_addr, instance, report, len);
  //hexdump(report, len);
  
  // find matching hid report
  for(int idx=0;idx<MAX_HID_DEVICES;idx++)
    if(hid_device[idx].dev_addr == dev_addr && hid_device[idx].instance == instance) {
      const hid_report_t *report_desc = &hid_device[idx].report[0].desc;
      // if there's no report id at all, then just use the first entry as these are
      // indistinguishable without report id
      if(!report_desc->report_id_present)      
	hid_parse(report_desc, &hid_device[idx].report[0].state, report, len);
      else {
	// there is a report id, so choose the right report descriptor for the
	// hid layer to parse against
	for(int i=0;i<MAX_REPORT_PER_IFACE;i++) {
	  const hid_report_t *report_desc = &hid_device[idx].report[i].desc;	
	  if(report_desc->report_id_present && report[0] == report_desc->report_id) {
	    //usb_debugf("parsing report %d, id %d", i, report[0]);
	    //hexdump(report, len);
	    hid_parse(report_desc, &hid_device[idx].report[i].state, report, len);
	  }
	}
      }
    }
  
  // continue to request to receive report
  if ( report && !tuh_hid_receive_report(dev_addr, instance) )
    usb_debugf("Error: cannot request report");
}


#endif
