#include "bsp/board_api.h"
#include "tusb.h"

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0x0403,
#if JTAG_CHANNELS == 2
    .idProduct          = 0x6010,
    .bcdDevice          = 0x0500,
#else
    .idProduct          = 0x6014,
    .bcdDevice          = 0x0900,
#endif

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const* tud_descriptor_device_cb(void) {
  return (uint8_t const*) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

#define TUD_MPSSE_IF_DESCRIPTOR(_itfnum) \
  0x09, TUSB_DESC_INTERFACE, _itfnum, 0x00, 2, 0xff, 0xff, 0xff, 2

#define TUD_MPSSE_IF_DESCRIPTOR_LEN 9u

#define TUD_MPSSE_BULK_DESCRIPTORS(_epin, _epout)	\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0u, \
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0u

#define TUD_MPSSE_BULK_DESCRIPTORS_LEN (7u+7u)

#if JTAG_CHANNELS == 2
#define TUD_MPSSE_DESC \
  TUD_MPSSE_IF_DESCRIPTOR(0), TUD_MPSSE_BULK_DESCRIPTORS(0x81, 0x02), \
  TUD_MPSSE_IF_DESCRIPTOR(1), TUD_MPSSE_BULK_DESCRIPTORS(0x83, 0x04)
  
#define TUD_MPSSE_DESC_LEN (2*(TUD_MPSSE_IF_DESCRIPTOR_LEN + TUD_MPSSE_BULK_DESCRIPTORS_LEN))
#else
#define TUD_MPSSE_DESC \
  TUD_MPSSE_IF_DESCRIPTOR(0), TUD_MPSSE_BULK_DESCRIPTORS(0x81, 0x02)
  
#define TUD_MPSSE_DESC_LEN (TUD_MPSSE_IF_DESCRIPTOR_LEN + TUD_MPSSE_BULK_DESCRIPTORS_LEN)
#endif

// full speed configuration
static uint8_t const desc_fs_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_MPSSE_DESC_LEN, 0x00, 100),
    TUD_MPSSE_DESC
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
  // (void) index; // for multiple configurations
  return desc_fs_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// String Descriptor Index
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL
};

// array of pointer to string descriptors
char const* string_desc_arr[] = {
    (const char[]) {0x09, 0x04}, // 0: is supported language is English (0x0409)
    "MiSTle Project",              // 1: Manufacturer
    "FPGA Companion",              // 2: Product
    NULL                           // 3: Serials will use unique ID if possible
};

static uint16_t _desc_str[32 + 1];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void) langid;
  size_t chr_count;

  switch (index) {
    case STRID_LANGID:
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
      break;

    case STRID_SERIAL:
      chr_count = board_usb_get_serial(_desc_str + 1, 32);
      break;

    default:
      // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
      // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

      if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) return NULL;

      const char* str = string_desc_arr[index];

      // Cap at max char
      chr_count = strlen(str);
      size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1; // -1 for string type
      if (chr_count > max_count) chr_count = max_count;

      // Convert ASCII string into UTF-16
      for (size_t i = 0; i < chr_count; i++) {
        _desc_str[1 + i] = str[i];
      }
      break;
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

  return _desc_str;
}
