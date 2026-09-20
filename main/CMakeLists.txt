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
        mtek_espnow_service
    PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_driver_uart esp_driver_spi esp_driver_gpio esp_timer
)

# Build identity, derived from real build inputs and consumed by
# app_main.c's mtk_system_build_info_t so a target's GET_VERSION response
# and the release notes' candidate manifest agree exactly.
#
# MTK_RELEASE_CANDIDATE is a short, manually maintained string, bumped
# once per candidate alongside the release notes. It is wire-visible via
# GET_VERSION, so leaving it stale misidentifies the binary: bump it in
# the same change that finalizes a candidate's source.
#
# MTK_RELEASE_EPOCH_S is a fixed, explicit release epoch
# (SOURCE_DATE_EPOCH-style), bumped by hand alongside
# MTK_RELEASE_CANDIDATE rather than derived from wall-clock build time,
# so every build of an identical source tree embeds an identical epoch.
# Deriving it from the time of compilation would make the value differ on
# every rebuild and would not support reproducibility. app_main.c's
# build_epoch_reproducible flag is set to 1 on the strength of an actual
# check -- two clean `idf.py build` runs from independent build
# directories producing byte-identical `mtkcore.bin` -- not on the mere
# existence of this constant.
set(MTK_RELEASE_CANDIDATE "rc12" CACHE STRING
    "Release candidate identifier embedded in the firmware's own GET_VERSION response; bump alongside release/RELEASE_NOTES.md for each new candidate")
set(MTK_RELEASE_EPOCH_S "1788739200" CACHE STRING
    "Fixed SOURCE_DATE_EPOCH-style release epoch (Unix seconds UTC, 2026-09-07T00:00:00Z -- the date this candidate's source was finalized) for this candidate -- bump by hand alongside MTK_RELEASE_CANDIDATE, never derived from wall-clock build time")
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    MTK_BUILD_ID="${MTK_RELEASE_CANDIDATE}"
    MTK_BUILD_EPOCH_S=${MTK_RELEASE_EPOCH_S}
)
