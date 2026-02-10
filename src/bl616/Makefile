# Makefile for the MiSTeryNano firmware

COMX?=/dev/m0s_debugger
BAUDRATE?=2000000

SDK_DEMO_PATH ?= .
BL_SDK_BASE ?= $(SDK_DEMO_PATH)/../../..

CHIP ?= bl616
BOARD ?= bl616dk
TANG_BOARD ?= m0sdock
CROSS_COMPILE ?= riscv64-unknown-elf-
cmake_definition+=-DTANG_BOARD=$(TANG_BOARD)

include $(BL_SDK_BASE)/project.build

