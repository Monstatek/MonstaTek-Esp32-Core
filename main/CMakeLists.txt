# The universal build compiles all three transport adapters
# (TASK003_IMPLEMENTATION.md requirement); CONFIG_MTEK_ADAPTER_* gates
# their *use* in app_main.c (task/creation, header inclusion) via
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
    PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_driver_uart esp_driver_spi esp_driver_gpio esp_timer
)

# RC5 independent audit P1 "Release identity is not real": a genuine
# build identity derived from actual build inputs, consumed by
# app_main.c's mtk_system_build_info_t so a real target's GET_VERSION
# response and release/RELEASE_NOTES.md's own candidate manifest can be
# made to agree exactly. There is no git in this clean-room tree to
# derive a commit hash from (the binding no-git constraint recorded in
# docs/DECISION_LOG.md) -- MTK_RELEASE_CANDIDATE is a short, manually-
# maintained string bumped once per candidate alongside the release notes
# (matching this project's own external RCn numbering).
#
# RC7 independent audit P1 "Release identity is not reproducible": RC6
# used `string(TIMESTAMP ... "%s" UTC)` here -- the real wall-clock time
# THIS SPECIFIC COMPILE happened, which is by definition different on
# every rebuild, then set app_main.c's build_epoch_reproducible=1 by
# redefining "reproducible" to mean merely "genuinely captured" rather
# than "bit-for-bit identical across builds" -- exactly the audit's own
# cited defect. MTK_BUILD_EPOCH_S is now a fixed, explicit release-epoch
# constant (SOURCE_DATE_EPOCH-style): a value bumped by hand alongside
# MTK_RELEASE_CANDIDATE (once per candidate, not once per compile), so
# every build of the IDENTICAL source tree embeds the IDENTICAL epoch --
# a real, verifiable input to build reproducibility, not a timestamp of
# the act of compiling. app_main.c's own build_epoch_reproducible flag is
# set to 1 only because this was independently verified this round: two
# separate clean `idf.py build` runs from independent build directories
# produced byte-identical `mtkcore.bin` output (see docs/DECISION_LOG.md
# for the exact verification and its result) -- not asserted from this
# constant's mere existence.
# RC11 release-finalization correction (independent audit): this
# was left at "rc7" through every RC8-RC11 correction round -- a real,
# wire-visible staleness (GET_VERSION exposes this exact string) the
# audit correctly flagged as making the whole release package's own
# identity untrustworthy regardless of how much source-level work had
# actually landed since rc7 was set. Bumped to "rc11" to match the real
# state of the tree this candidate is packaged from.
# RC12 final blocker correction (read-only audit): the deferred
# Compatibility response/event state machine and the cross-operation-token
# family validation landed since rc11 was set; bumped to "rc12" so the
# GET_VERSION-exposed identity matches the tree this candidate is
# packaged from (a stale rc11 here would misidentify the binary).
set(MTK_RELEASE_CANDIDATE "rc12-pcap1" CACHE STRING
    "Release candidate identifier embedded in the firmware's own GET_VERSION response; bump alongside release/RELEASE_NOTES.md for each new candidate")
set(MTK_RELEASE_EPOCH_S "1788998400" CACHE STRING
    "Fixed SOURCE_DATE_EPOCH-style release epoch (Unix seconds UTC, 2026-09-10T00:00:00Z -- the date this candidate's source was finalized) for this candidate -- bump by hand alongside MTK_RELEASE_CANDIDATE, never derived from wall-clock build time")
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    MTK_BUILD_ID="${MTK_RELEASE_CANDIDATE}"
    MTK_BUILD_EPOCH_S=${MTK_RELEASE_EPOCH_S}
)
