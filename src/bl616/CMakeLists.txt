cmake_minimum_required(VERSION 3.15)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile commands for tooling" FORCE)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED TANG_BOARD OR "${TANG_BOARD}" STREQUAL "")
    set(TANG_BOARD "m0sdock")
endif()

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
elseif(TANG_BOARD STREQUAL "nano20k_v3923")
    add_definitions(-DTANG_NANO20K_V3923)
elseif(TANG_BOARD STREQUAL "m0sdock")
    add_definitions(-DM0S_DOCK)
else()
   message(FATAL_ERROR "TANG_BOARD must be mega60k, mega138kpro, console60k, primer25k, nano20k, nano20k_v3923 or m0sdock")
endif()

find_package(bouffalo_sdk REQUIRED HINTS $ENV{BL_SDK_BASE})
sdk_add_include_directories($ENV{BL_SDK_BASE}/components/net/lwip/lwip/src/include)
sdk_add_include_directories(. ../u8g2/csrc)

sdk_add_link_options(-Wl,--wrap=bflb_console_write)

# User
sdk_add_compile_definitions(-DCONFIG_CLI_CMD_ENABLE)

target_sources(app PRIVATE
   	mcu_hw.c
    $ENV{BL_SDK_BASE}/components/net/lwip/lwip/src/apps/sntp/sntp.c
    ../ftpd.c
    ../telnetd.c
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
    ../jtag.c
    hid_keyboard_template.c
    ../gowin.c
    sdc_direct.c
    ../ps2helper.c
)

file(GLOB COMPONENT_SRCS ../u8g2/csrc/*.c  ../u8g2/sys/bitmap/common/*.c)
target_sources(app PRIVATE ${COMPONENT_SRCS})

sdk_set_main_file(../main.c)

project(fpga_companion)
#project(fpga_companion CXX)
