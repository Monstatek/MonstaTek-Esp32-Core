idf_component_register(
    SRCS "mtek_spi_native_frame.c" "mtek_spi_native_dispatch.c"
    INCLUDE_DIRS "include"
    REQUIRES mtek_espnow_service mtek_schema mtek_core mtek_router mtek_capture_service mtek_wifi_service mtek_ble_service
)
