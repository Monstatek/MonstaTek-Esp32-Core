# The portable service logic always compiles and is what host tests link. The
# ESP-IDF-backed HAL gates its own contents on CONFIG_MTEK_IEEE802154_ENABLED,
# for the same two-pass reason the BLE HAL does: component requirement
# expansion runs before per-component CONFIG_ values are available, so the
# file must stay in SRCS and guard itself.
idf_component_register(
    SRCS "mtek_ieee802154_logic.c" "mtek_ieee802154_hal_esp32.c"
    INCLUDE_DIRS "include"
    REQUIRES mtek_hal_common mtek_schema mtek_core mtek_arbiter mtek_router
    PRIV_REQUIRES ieee802154 esp_timer
)
