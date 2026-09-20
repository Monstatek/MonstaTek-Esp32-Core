# Deterministic host tests: transport framing, schema encoding,
# lifecycle/state transitions, resource conflicts, adapter translation,
# factory response strings, and malformed inputs -- built and run on the
# development machine, entirely independent of the ESP-IDF toolchain/build.
#
# Usage:
#   cmake -S host_tests -B /tmp/mtek-host-build
#   cmake --build /tmp/mtek-host-build
#   MTK_RELEASE_PRE_PACKAGING=1 ctest --test-dir /tmp/mtek-host-build --output-on-failure
#
# MTK_RELEASE_PRE_PACKAGING=1 skips deep release-package validation only when
# release artifacts are absent. It does not validate a candidate package.
cmake_minimum_required(VERSION 3.16)
project(mtek_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -g)

# Enabled by default: ASan+UBSan give an active memory/UB check on every
# host test, not just a manual lifetime argument -- in particular for
# test_router_async_safety, which exercises heap-allocated sink lifetimes
# across a deferred ("async") dispatch. Disable with -DMTK_SANITIZE=OFF if
# the local toolchain lacks sanitizer runtime support.
option(MTK_SANITIZE "Build host tests with AddressSanitizer + UndefinedBehaviorSanitizer" ON)
if(MTK_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_library(mtek_host_lib STATIC
    ${ROOT}/components/mtek_schema/mtek_schema_codec.c
    ${ROOT}/components/mtek_schema/mtek_opcode_overlay.c
    ${ROOT}/components/mtek_schema/generated/mtek_schema_field_tables.c
    ${ROOT}/components/mtek_schema/generated/mtek_opcode_registry.c
    ${ROOT}/components/mtek_schema/generated/mtek_arbiter_tables.c
    ${ROOT}/components/mtek_core/mtek_core.c
    ${ROOT}/components/mtek_core/mtek_async_queue.c
    ${ROOT}/components/mtek_arbiter/mtek_arbiter.c
    ${ROOT}/components/mtek_transport_spi_native/mtek_spi_native_frame.c
    ${ROOT}/components/mtek_transport_spi_native/mtek_spi_native_dispatch.c
    ${ROOT}/components/mtek_transport_uart/mtek_uart_adapter.c
    ${ROOT}/components/mtek_transport_spi_compat/mtek_compat_opcode_map.c
    ${ROOT}/components/mtek_transport_spi_compat/mtek_compat_frame.c
    ${ROOT}/components/mtek_transport_spi_compat/mtek_compat_dispatch.c
    ${ROOT}/components/mtek_transport_spi_compat/generated/mtek_compat_opcode_table.c
    ${ROOT}/components/mtek_transport_select/mtek_transport_select.c
    ${ROOT}/components/mtek_router/mtek_router.c
    ${ROOT}/components/mtek_wifi_service/mtek_wifi_logic.c
    ${ROOT}/components/mtek_ble_service/mtek_ble_logic.c
    ${ROOT}/components/mtek_ble_service/mtk_ble_op_generation.c
    ${ROOT}/components/mtek_ble_service/mtk_ble_op_lifecycle.c
    ${ROOT}/components/mtek_capture_service/mtek_capture_logic.c
    ${ROOT}/components/mtek_system_service/mtek_system_logic.c
    ${ROOT}/components/mtek_espnow_service/mtek_espnow_logic.c
)
target_include_directories(mtek_host_lib PUBLIC
    ${ROOT}/components/mtek_schema/include
    ${ROOT}/components/mtek_core/include
    ${ROOT}/components/mtek_arbiter/include
    ${ROOT}/components/mtek_router/include
    ${ROOT}/components/mtek_hal_common/include
    ${ROOT}/components/mtek_transport_spi_native/include
    ${ROOT}/components/mtek_transport_uart/include
    ${ROOT}/components/mtek_transport_spi_compat/include
    ${ROOT}/components/mtek_transport_select/include
    ${ROOT}/components/mtek_wifi_service/include
    ${ROOT}/components/mtek_ble_service/include
    ${ROOT}/components/mtek_capture_service/include
    ${ROOT}/components/mtek_system_service/include
    ${ROOT}/components/mtek_espnow_service/include
    ${ROOT}/host_tests/support
)

# Compile the test-only opcode routing (the
# 0x00F0 async-fixture START and 0x00F1 generic STOP in mtek_system_dispatch)
# ONLY into the host-test build. The ESP32 target never defines this, so
# those opcodes are genuinely absent from the shipped firmware.
target_compile_definitions(mtek_host_lib PUBLIC MTK_ENABLE_TEST_OPCODES)

# Compile the optional capability modules the shipped universal build also
# enables (main/Kconfig.projbuild defaults them all to y), so host tests
# exercise the same dispatch configuration the target actually ships rather
# than a reduced one in which module opcodes are unreachable.
target_compile_definitions(mtek_host_lib PUBLIC
    CONFIG_MTEK_MODULE_CAPTIVE_PORTAL=1
    CONFIG_MTEK_MODULE_KARMA=1
    CONFIG_MTEK_MODULE_PROBE_FLOOD=1
    CONFIG_MTEK_MODULE_BEACON_VARIANTS=1)

enable_testing()

function(mtk_add_test name)
    add_executable(${name} ${name}.c)
    target_link_libraries(${name} PRIVATE mtek_host_lib)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

mtk_add_test(test_schema_codec)
mtk_add_test(test_codec_bounds)
mtk_add_test(test_architecture_contracts)
mtk_add_test(test_spi_runtime_reliability)
target_include_directories(test_spi_runtime_reliability BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/support/spi_runtime_stubs)
mtk_add_test(test_opcode_registry)
mtk_add_test(test_arbiter)
mtk_add_test(test_spi_native_transport)
mtk_add_test(test_spi_native_fragmentation)
mtk_add_test(test_core_lifecycle)
mtk_add_test(test_uart_factory_parity)
mtk_add_test(test_uart_pre_radio_readiness)
mtk_add_test(test_min_one_tick_policy)
mtk_add_test(test_ble_op_lifecycle)
mtk_add_test(test_transport_counters_concurrency)
mtk_add_test(test_op_family_validation)
mtk_add_test(test_compat_deferred_scans_gatt)
mtk_add_test(test_uart_ble_detail_and_discovery)
mtk_add_test(test_wifi_prior_state_restore)
mtk_add_test(test_uart_boot_banner_help)
mtk_add_test(test_sta_scan_lifecycle)
mtk_add_test(test_handshake_monitor_entry_failure)
mtk_add_test(test_wifi_prior_state_capture_failure)
mtk_add_test(test_sta_scan_rc10)
mtk_add_test(test_radio_restore_quarantine)
mtk_add_test(test_promisc_audit_followups)
mtk_add_test(test_ap_scan_rc11)
mtk_add_test(test_malformed_inputs)
mtk_add_test(test_deauth_selected)
mtk_add_test(test_deauth_selected_set)
mtk_add_test(test_deauth_all_scanned)
mtk_add_test(test_deauth_broadcast)
mtk_add_test(test_raw_tx_channel)
mtk_add_test(test_softap_lifecycle)
mtk_add_test(test_captive_portal)
mtk_add_test(test_espnow_lifecycle)
mtk_add_test(test_handshake_capture)
mtk_add_test(test_monstashark_capture)
mtk_add_test(test_wifi_scan_connect)
mtk_add_test(test_ble_scan_adv)
mtk_add_test(test_gatt_rw_sub)
mtk_add_test(test_signal_meter)
mtk_add_test(test_release_artifact)
# Release validation must validate the actual release pair: bake the real,
# absolute project root
# (CMake always knows this precisely, regardless of where the build
# directory itself lives -- in-tree or fully out-of-tree) into the test
# binary, so it can resolve release/ correctly instead of a hard-coded
# `../../release` relative path that only happened to work when ctest's own
# working directory was exactly two levels under the project root.
get_filename_component(MTK_PROJECT_ROOT_ABS "${CMAKE_SOURCE_DIR}/.." ABSOLUTE)
target_compile_definitions(test_release_artifact PRIVATE MTK_PROJECT_ROOT="${MTK_PROJECT_ROOT_ABS}")
mtk_add_test(test_compat_framing)
mtk_add_test(test_compat_coverage)
mtk_add_test(test_compat_fragmentation)
mtk_add_test(test_spi_native_dispatch)
mtk_add_test(test_transport_select)
mtk_add_test(test_ble_op_generation)
mtk_add_test(test_ble_disc_status)
mtk_add_test(test_native_cellsize_negotiator)
mtk_add_test(test_router_async_safety)
mtk_add_test(test_uart_async_delivery)

find_package(Threads REQUIRED)
add_executable(test_async_queue test_async_queue.c)
target_link_libraries(test_async_queue PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_async_queue COMMAND test_async_queue)

add_executable(test_compat_async_deauth test_compat_async_deauth.c)
target_link_libraries(test_compat_async_deauth PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_compat_async_deauth COMMAND test_compat_async_deauth)

add_executable(test_compat_async_multi test_compat_async_multi.c)
target_link_libraries(test_compat_async_multi PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_compat_async_multi COMMAND test_compat_async_multi)

add_executable(test_spi_native_async test_spi_native_async.c)
target_link_libraries(test_spi_native_async PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_spi_native_async COMMAND test_spi_native_async)

add_executable(test_spi_native_dup_cache test_spi_native_dup_cache.c)
target_link_libraries(test_spi_native_dup_cache PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_spi_native_dup_cache COMMAND test_spi_native_dup_cache)

add_executable(test_spi_native_4inflight test_spi_native_4inflight.c)
target_link_libraries(test_spi_native_4inflight PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_spi_native_4inflight COMMAND test_spi_native_4inflight)

add_executable(test_spi_native_credit_cancel test_spi_native_credit_cancel.c)
target_link_libraries(test_spi_native_credit_cancel PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_spi_native_credit_cancel COMMAND test_spi_native_credit_cancel)

add_executable(test_spi_native_epoch_separation test_spi_native_epoch_separation.c)
target_link_libraries(test_spi_native_epoch_separation PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_spi_native_epoch_separation COMMAND test_spi_native_epoch_separation)

add_executable(test_op_alloc_aba_service test_op_alloc_aba_service.c)
target_link_libraries(test_op_alloc_aba_service PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_op_alloc_aba_service COMMAND test_op_alloc_aba_service)

add_executable(test_peer_session_invalidation_services test_peer_session_invalidation_services.c)
target_link_libraries(test_peer_session_invalidation_services PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_peer_session_invalidation_services COMMAND test_peer_session_invalidation_services)

add_executable(test_spi_native_deferred_and_retained test_spi_native_deferred_and_retained.c)
target_link_libraries(test_spi_native_deferred_and_retained PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_spi_native_deferred_and_retained COMMAND test_spi_native_deferred_and_retained)

add_executable(test_peer_reset_concurrency_round4 test_peer_reset_concurrency_round4.c)
target_link_libraries(test_peer_reset_concurrency_round4 PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_peer_reset_concurrency_round4 COMMAND test_peer_reset_concurrency_round4)

add_executable(test_p0_concurrency_closure_round5 test_p0_concurrency_closure_round5.c)
target_link_libraries(test_p0_concurrency_closure_round5 PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_p0_concurrency_closure_round5 COMMAND test_p0_concurrency_closure_round5)

add_executable(test_p0_session_publication_closure_round7 test_p0_session_publication_closure_round7.c)
target_link_libraries(test_p0_session_publication_closure_round7 PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_p0_session_publication_closure_round7 COMMAND test_p0_session_publication_closure_round7)

add_executable(test_p0_round8_final_closure test_p0_round8_final_closure.c)
target_link_libraries(test_p0_round8_final_closure PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_p0_round8_final_closure COMMAND test_p0_round8_final_closure)

add_executable(test_p0_round9_final_closure test_p0_round9_final_closure.c)
target_link_libraries(test_p0_round9_final_closure PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_p0_round9_final_closure COMMAND test_p0_round9_final_closure)

add_executable(test_p0_round10_final_closure test_p0_round10_final_closure.c)
target_link_libraries(test_p0_round10_final_closure PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_p0_round10_final_closure COMMAND test_p0_round10_final_closure)

mtk_add_test(test_spi_native_events_streams)
mtk_add_test(test_spi_native_discovery)

add_executable(test_deauth_continuous test_deauth_continuous.c)
target_link_libraries(test_deauth_continuous PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_deauth_continuous COMMAND test_deauth_continuous)

add_executable(test_core_concurrency test_core_concurrency.c)
target_link_libraries(test_core_concurrency PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_core_concurrency COMMAND test_core_concurrency)

add_executable(test_uart_async_safety test_uart_async_safety.c)
target_link_libraries(test_uart_async_safety PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_uart_async_safety COMMAND test_uart_async_safety)

add_executable(test_transport_claim test_transport_claim.c)
target_link_libraries(test_transport_claim PRIVATE mtek_host_lib Threads::Threads)
target_link_libraries(test_uart_pre_radio_readiness PRIVATE mtek_host_lib Threads::Threads)
target_link_libraries(test_ble_op_lifecycle PRIVATE mtek_host_lib Threads::Threads)
target_link_libraries(test_transport_counters_concurrency PRIVATE mtek_host_lib Threads::Threads)
target_link_libraries(test_compat_deferred_scans_gatt PRIVATE mtek_host_lib Threads::Threads)
target_link_libraries(test_sta_scan_lifecycle PRIVATE mtek_host_lib Threads::Threads)
target_link_libraries(test_sta_scan_rc10 PRIVATE mtek_host_lib Threads::Threads)
target_link_libraries(test_ap_scan_rc11 PRIVATE mtek_host_lib Threads::Threads)
target_link_libraries(test_handshake_monitor_entry_failure PRIVATE mtek_host_lib Threads::Threads)
add_test(NAME test_transport_claim COMMAND test_transport_claim)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
    # Passes the real, absolute project
    # root (CMake's own robust resolution, correct regardless of build
    # directory location) so this script can ALSO cross-check the real
    # release/MtkCore.bin + MtkCore.md5 pair with a second, independent MD5
    # implementation (Python's hashlib) -- agreeing with test_release_
    # artifact.c's own hand-rolled RFC 1321 implementation, not merely
    # exercising validate_pair() against synthetic temp files.
    add_test(NAME test_release_negative COMMAND ${Python3_EXECUTABLE} ${ROOT}/tools/validate_release_negative.py --root ${ROOT})
endif()
