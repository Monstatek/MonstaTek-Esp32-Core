# mtek_wifi_hal_esp32.c (the real ESP-IDF-backed HAL) is added here once
# the target build wiring lands; host tests link mtek_wifi_hal_fake.c
# directly instead (host_tests/CMakeLists.txt), bypassing this file.
idf_component_register(
    SRCS "mtek_wifi_logic.c" "mtek_wifi_hal_esp32.c"
    INCLUDE_DIRS "include"
    REQUIRES mtek_hal_common mtek_schema mtek_core mtek_arbiter mtek_router
    PRIV_REQUIRES esp_wifi esp_netif esp_event esp_http_server lwip
)
