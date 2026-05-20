#ifndef _BTSTACK_CONFIG_H
#define _BTSTACK_CONFIG_H

// BTstack features that can be enabled
#define ENABLE_LOG_ERROR
// #define ENABLE_LOG_INFO
// #define ENABLE_LOG_DEBUG
#define ENABLE_PRINTF_HEXDUMP

// BTstack configuration. buffers, sizes, ...
#define HCI_INCOMING_PRE_BUFFER_SIZE 4
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4

#define NVM_NUM_LINK_KEYS 16

#define MAX_NR_HCI_CONNECTIONS 4
#define MAX_NR_HID_HOST_CONNECTIONS 4

#define MAX_NR_L2CAP_SERVICES  4
#define MAX_NR_L2CAP_CHANNELS  4

#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4
#define HCI_ACL_PAYLOAD_SIZE (1691 + 4)

// We don't give btstack a malloc, so use a fixed-size ATT DB.
// #define MAX_ATT_DB_SIZE 512

#endif // _BTSTACK_CONFIG_H
