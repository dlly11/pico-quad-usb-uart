cmake_minimum_required(VERSION 3.13)

add_executable(quad_usb_uart)

pico_generate_pio_header(quad_usb_uart ${CMAKE_CURRENT_LIST_DIR}/uart_rx.pio)
pico_generate_pio_header(quad_usb_uart ${CMAKE_CURRENT_LIST_DIR}/uart_tx.pio)

target_include_directories(quad_usb_uart PUBLIC ${PROJECT_SOURCE_DIR}/include)

file(GLOB PROJ_SRC_FILES ${CMAKE_CURRENT_LIST_DIR}/*.c)

set_property(SOURCE ${PROJ_SRC_FILES} PROPERTY COMPILE_FLAGS 
    "-Wall -Wextra -Wconversion -Wuninitialized -Wunused -Wcast-qual -Wcast-align")

target_sources(quad_usb_uart PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/main.c
    ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
)

# In addition to pico_stdlib required for common PicoSDK functionality, add dependency on tinyusb_device
# for TinyUSB device support and tinyusb_board for the additional board support library used by the example
target_link_libraries(
    quad_usb_uart PUBLIC 
    pico_stdlib 
    tinyusb_device 
    tinyusb_board 
    hardware_pio 
    pico_multicore
)

# Uncomment this line to enable fix for Errata RP2040-E5 (the fix requires use of GPIO 15)
#target_compile_definitions(quad_usb_uart PUBLIC PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1)

pico_add_extra_outputs(quad_usb_uart)

