# mtek_ble_hal_esp32.c is compiled unconditionally and gates its own contents
# on CONFIG_BT_ENABLED. Conditioning SRCS/PRIV_REQUIRES here instead does not
# work: component requirement expansion runs before per-component CONFIG_ values
# are available, so the file would be compiled without the bt component's
# include paths. With the controller compiled out the translation unit is empty
# and every BLE opcode refuses through the service's existing "no HAL installed"
# guard.
idf_component_register(
    SRCS "mtek_ble_logic.c" "mtek_ble_hal_esp32.c" "mtk_ble_op_generation.c" "mtk_ble_op_lifecycle.c"
    INCLUDE_DIRS "include"
    REQUIRES mtek_hal_common mtek_schema mtek_core mtek_arbiter mtek_router
    PRIV_REQUIRES bt
)
