# to build for waveshare rp2040-zero do
# cmake -DWS2040_ZERO=ON ..
# to build for Pico2 or Pico2-W do
# cmake -DPICO2=ON ..
# You might need to delete CMakeCache.txt when changing these

cmake_minimum_required(VERSION 3.13)

set(PROJECT fpga_companion)

option(PICO2 "Build for Pico2 and Pico2-W" OFF) # Regular Pico-W by default

option(SH20KLITE "Build for MiSTeryShield20k Lite" OFF)

# This will also work for the regular Pico. The
# firmware detects it and acts accordingly
if(PICO2)
set(PICO_BOARD pico2_w)
set(TARGET rp2350)
else(PICO2)
set(PICO_BOARD pico_w)
set(TARGET rp2040)
endif(PICO2)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(${PROJECT} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Pull in FreeRTOS
if(PICO2)
include(./FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)
else(PICO2)
include(../FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
endif(PICO2)

# u8g2
file(GLOB U8G2_SRC ../u8g2/csrc/*.c)
add_library(u8g2 ${U8G2_SRC})

add_executable(${PROJECT}
	../main.c
	mcu_hw.c
	../sysctrl.c
	../hidparser.c
	../hid.c
	../sdc.c
	../osd_u8g2.c
	../menu.c
	../inifile.c
	../at_wifi.c
	../puff.c
	../fatfs/source/ff.c
	../fatfs/source/ffunicode.c
	../u8g2/sys/bitmap/common/u8x8_d_bitmap.c
	../tusb_xinput/xinput_host.c
	../config.c
	../freertos_callbacks.c
	../xml.c
)

family_add_pico_pio_usb(${PROJECT})

target_compile_definitions(${PROJECT} PRIVATE
        PIO_USB_DP_PIN_DEFAULT=2
        PICO_DEFAULT_PIO_USB_DP_PIN=2
        )
	
if(SH20KLITE)
  add_compile_definitions(${PROJECT} PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64 SH20KLITE=1)
endif(SH20KLITE)
	
option(WS2040_ZERO "Build for Waveshare RP2040-Zero" OFF) # Regular Pico by default
if(WS2040_ZERO)
        add_compile_definitions(${PROJECT} PRIVATE WAVESHARE_RP2040_ZERO=1)
        add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "Firmware has been built for Waveshare RP2040-Zero.")
else(WS2040_ZERO)
	if(PICO2)
        add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "Firmware has been built for Raspberry Pi Pico2 or Pico2-W.")
	else(PICO2)
		if(SH20KLITE)
			add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "Firmware has been built for MiSTeryShield20k-Lite.")
		else(SH20KLITE)
		        add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "Firmware has been built for Raspberry Pi Pico or Pico-W.")
		endif(SH20KLITE)
	endif(PICO2)
endif(WS2040_ZERO)

target_include_directories(${PROJECT} PRIVATE
      ${CMAKE_CURRENT_LIST_DIR}
      ${CMAKE_CURRENT_LIST_DIR}/../fatfs/source
      ${CMAKE_CURRENT_LIST_DIR}/../u8g2/csrc
      ${CMAKE_CURRENT_LIST_DIR}/../tusb_xinput
)

string(APPEND CMAKE_C_FLAGS " -Os -fdata-sections -ffunction-sections -Wno-error=incompatible-pointer-types" )

target_link_libraries(${PROJECT} PRIVATE pico_stdlib pico_multicore)
target_link_libraries(${PROJECT} PRIVATE FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)
target_link_libraries(${PROJECT} PRIVATE hardware_pio hardware_dma hardware_spi hardware_adc)
target_link_libraries(${PROJECT} PRIVATE pico_cyw43_arch_lwip_sys_freertos)
target_link_libraries(${PROJECT} PRIVATE tinyusb_host tinyusb_board u8g2)

pico_add_extra_outputs(${PROJECT})
pico_enable_stdio_usb(${PROJECT} 0)
pico_enable_stdio_uart(${PROJECT} 1)

add_custom_target(flash
    COMMAND echo "Flashing ${PROJECT} ..."
    COMMAND openocd -f interface/cmsis-dap.cfg -f target/${TARGET}.cfg -c "adapter speed 5000" -c "program ${PROJECT}.elf verify reset exit"
    DEPENDS "${PROJECT}"
    COMMENT "Flash target using openocd"
)

target_link_options(${PROJECT} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${PROJECT} PRIVATE -Wall -Wextra)

add_custom_target(reset
    COMMAND echo "Reseting ${PROJECT} for ${TARGET} ..."
    COMMAND openocd -f interface/cmsis-dap.cfg -f target/${TARGET}.cfg -c init -c reset -c exit
    COMMENT "Reset target using openocd"
)

if(WS2040_ZERO)
add_custom_target(term COMMAND term.sh /dev/ttyACM0 460800)
else(WS2040_ZERO)
add_custom_target(term COMMAND term.sh /dev/ttyACM0 921600)
endif(WS2040_ZERO)
