idf_component_register(
    SRCS "mtek_uart_adapter.c"
    INCLUDE_DIRS "include"
    REQUIRES mtek_schema mtek_core mtek_router mtek_ble_service
)
