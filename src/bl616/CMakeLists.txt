cmake_minimum_required(VERSION 3.15)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Components
set(CONFIG_FREERTOS 1)
set(CONFIG_CHERRYUSB 1)
set(CONFIG_CHERRYUSB_HOST 1)
set(CONFIG_CHERRYUSB_DEVICE 1)
set(CONFIG_CHERRYUSB_DEVICE_HID 1)
set(CONFIG_PSRAM 0)
set(CONFIG_FATFS 1)
set(CONFIG_BFLOG 0)
set(CONFIG_SHELL 1)
set(CONFIG_CHERRYUSB_OSAL "freertos")
set(CONFIG_CHERRYUSB_HOST_HCD "ehci_bouffalo")
set(CONFIG_CHERRYUSB_HOST_HID 1)
set(CONFIG_CHERRYMP 1)
set(CONFIG_CHERRYRB 1)
set(CONFIG_VSNPRINTF_FLOAT      1)
set(CONFIG_VSNPRINTF_FLOAT_EX   1)
set(CONFIG_VSNPRINTF_LONG_LONG  1)
set(CONFIG_CLI_CMD_ENABLE 1)

#include(proj.conf)

# wifi support
set(CONFIG_LWIP 1)
set(CONFIG_WIFI6 1)
set(CONFIG_RF 1)
set(CONFIG_MBEDTLS 1)
set(CONFIG_DHCPD 1)
set(CONFIG_POSIX 1)
set(CONFIG_TLSF 1)
set(CONFIG_PING 1)
# mbedtls
set(CONFIG_MBEDTLS_AES_USE_HW 1)
set(CONFIG_MBEDTLS_BIGNUM_USE_HW 1)
set(CONFIG_MBEDTLS_ECC_USE_HW 1)
set(CONFIG_MBEDTLS_SHA1_USE_HW 1)
set(CONFIG_MBEDTLS_SHA256_USE_HW 1)
set(CONFIG_MBEDTLS_SHA512_USE_HW 1)
# wifi
set(CONFIG_VIF_MAX 2)
set(CONFIG_STA_MAX 4)
set(CONFIG_MAC_TXQ_DEPTH 32)
set(CONFIG_MAC_RXQ_DEPTH 12)
#set(CONFIG_WIFI_IPERF 1)

if(TANG_BOARD STREQUAL "mega60k")
    add_definitions(-DTANG_MEGA60K)
elseif(TANG_BOARD STREQUAL "mega138kpro")
    add_definitions(-DTANG_MEGA138KPRO)
elseif(TANG_BOARD STREQUAL "console60k") 
    add_definitions(-DTANG_CONSOLE60K)
elseif(TANG_BOARD STREQUAL "console138k")
    add_definitions(-DTANG_CONSOLE138K)
elseif(TANG_BOARD STREQUAL "primer25k")
    add_definitions(-DTANG_PRIMER25K)
elseif(TANG_BOARD STREQUAL "nano20k")
    add_definitions(-DTANG_NANO20K)
elseif(TANG_BOARD STREQUAL "m0sdock")
    add_definitions(-DM0S_DOCK)
endif()

find_package(bouffalo_sdk REQUIRED HINTS $ENV{BL_SDK_BASE})

sdk_add_include_directories(. ../u8g2/csrc)

target_sources(app PRIVATE
   	mcu_hw.c
   	../core.c
   	../core_atarist.c
   	../core_c64.c
   	../core_vic20.c
   	../core_amiga.c
   	../core_atari2600.c
   	../hidparser.c
   	../hid.c
   	../inifile.c
   	../menu.c
   	../sdc.c
   	../osd_u8g2.c
   	../sysctrl.c
   	../config.c
   	../xml.c
    ../at_wifi.c
    ../puff.c
    hid_keyboard_template.c
)

file(GLOB COMPONENT_SRCS ../u8g2/csrc/*.c  ../u8g2/sys/bitmap/common/*.c)
target_sources(app PRIVATE ${COMPONENT_SRCS})

sdk_set_main_file(../main.c)

project(fpga_companion)
#project(fpga_companion CXX)