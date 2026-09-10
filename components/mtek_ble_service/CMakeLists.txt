idf_component_register(
    SRCS "mtek_ble_logic.c" "mtek_ble_hal_esp32.c" "mtk_ble_op_generation.c" "mtk_ble_op_lifecycle.c"
    INCLUDE_DIRS "include"
    REQUIRES mtek_hal_common mtek_schema mtek_core mtek_arbiter mtek_router
    PRIV_REQUIRES bt
)
