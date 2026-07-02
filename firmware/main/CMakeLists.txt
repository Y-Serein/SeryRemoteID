set(MAIN_SRCS
    "app_main.c"
    "cfg.c"
    "protocol/rid_state.c"
    "transport/rid_mavlink.c"
    "net/rid_wifi.c"
    "net/rid_ble.c"
    "web/rid_web.c"
    "web/rid_firmware_check.c"
    "board/rid_led.c"
    "security/rid_secure.c"
    "transport/rid_dronecan.c"
)

idf_component_register(
    SRCS ${MAIN_SRCS}
    INCLUDE_DIRS
        "."
    REQUIRES
        app_update
        bt
        driver
        dronecan_generated
        esp_event
        esp_driver_usb_serial_jtag
        esp_http_server
        esp_netif
        esp_timer
        esp_wifi
        monocypher
        nvs_flash
        opendroneid
        mavlink
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
)
