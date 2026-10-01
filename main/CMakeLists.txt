# The universal build compiles all three transport adapters;
# CONFIG_MTEK_ADAPTER_* gates their *use* in app_main.c (task/creation, header inclusion) via
# preprocessor #if, so a product-specific build that disables one still
# excludes its behavior/footprint without forking the canonical core, even
# though this component-level REQUIRES list stays unconditional (component
# requirement expansion runs before per-file conditional compilation would
# let us cheaply prune the dependency graph itself).
idf_component_register(
    SRCS "app_main.c" "mtek_spi_runtime.c"
    INCLUDE_DIRS "."
    REQUIRES
        mtek_schema mtek_core mtek_arbiter mtek_router mtek_hal_common
        mtek_system_service mtek_wifi_service mtek_ble_service mtek_capture_service
        mtek_transport_uart mtek_transport_spi_native mtek_transport_spi_compat mtek_transport_select
        mtek_espnow_service mtek_ieee802154_service
    PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_driver_uart esp_driver_spi esp_driver_gpio esp_timer
)

# Build identity, derived from real build inputs and consumed by
# app_main.c's mtk_system_build_info_t so a target's GET_VERSION response
# and the release notes' candidate manifest agree exactly.
#
# MTK_RELEASE_CANDIDATE carries the release identity embedded in the
# firmware's own GET_VERSION response. It is wire-visible, so leaving it stale
# misidentifies the binary: set it in the same change that finalizes a
# release's source. This is the product release version, 0.8.1.0; the earlier
# rc13/rc14 candidate identities are historical and stay immutable.
#
# This is deliberately separate from the Core host API version
# (MTK_CORE_API_MAJOR/MINOR, components/mtek_schema). The product version
# moving to 0.8.1.0 does not change the API contract, which stays at 1.0
# because no opcode was removed, renumbered, or changed shape.
#
# The release epoch below is a fixed, explicit constant
# (SOURCE_DATE_EPOCH-style), bumped by hand alongside the candidate rather
# than derived from wall-clock build time, so every build of an identical
# source tree embeds an identical epoch. app_main.c's
# build_epoch_reproducible flag is set on the strength of an actual check --
# two clean `idf.py build` runs from independent build directories producing
# byte-identical `mtkcore.bin` -- not on the mere existence of this constant.
set(MTK_RELEASE_CANDIDATE "0.8.1.0" CACHE STRING
    "Release identifier embedded in the firmware's own GET_VERSION response; set alongside release/RELEASE_NOTES.md for each release")
set(MTK_RELEASE_EPOCH_S "1790856000" CACHE STRING
    "Fixed SOURCE_DATE_EPOCH-style release epoch (Unix seconds UTC, 2026-10-01T12:00:00Z -- the date this release's source was frozen) -- set by hand alongside MTK_RELEASE_CANDIDATE, never derived from wall-clock build time")
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    MTK_BUILD_ID="${MTK_RELEASE_CANDIDATE}"
    MTK_BUILD_EPOCH_S=${MTK_RELEASE_EPOCH_S}
)
