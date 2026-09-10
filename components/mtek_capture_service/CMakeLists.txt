idf_component_register(
    SRCS "mtek_capture_logic.c"
    INCLUDE_DIRS "include"
    REQUIRES mtek_hal_common mtek_schema mtek_core mtek_arbiter mtek_router mtek_wifi_service
)
