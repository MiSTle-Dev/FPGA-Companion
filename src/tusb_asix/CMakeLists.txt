cmake_minimum_required(VERSION 3.13)

add_library(asix_host INTERFACE)
target_sources(asix_host INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/asix_host.c
)
target_include_directories(asix_host INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(asix_host INTERFACE pico_stdlib tinyusb_host)
