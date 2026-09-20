idf_component_register(
    SRCS "mtek_espnow_logic.c" "mtek_espnow_hal_esp32.c"
    INCLUDE_DIRS "include"
    REQUIRES mtek_hal_common mtek_schema mtek_core mtek_arbiter mtek_router
    PRIV_REQUIRES esp_wifi esp_event
)
