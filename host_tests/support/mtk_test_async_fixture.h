/* RC12 hardening round, item 5 (P1): the test-only asynchronous fixture.
 *
 * Several host tests need a GENERIC arbiter-free ACCEPTED_ASYNC opcode as a
 * vehicle to exercise the router/core/native-dispatch async machinery
 * (deferred-worker pool budgeting, NO_MEMORY-on-overflow, eviction/ABA,
 * cancel, publish-guard/reset races). They historically borrowed the real
 * TIME_SYNC_START opcode for this, which forced its production capability
 * to stay SUPPORTED despite this candidate having no SNTP client. This
 * fixture provides an equivalent opcode that lives ONLY in the test-only
 * opcode overlay (mtek_opcode_overlay.h) -- never in the production table
 * or GET_CAPABILITIES -- so TIME_SYNC_START can now advertise the truthful
 * UNSUPPORTED.
 *
 * The fixture opcode is deliberately identical in every dispatch-relevant
 * dimension to what TIME_SYNC_START was (ACCEPTED_ASYNC, MTK_ARB_NONE,
 * no_radio_lease, the same request/response descriptors and deadline
 * envelope) and is routed by mtek_system_dispatch to the SAME production
 * handler (handle_time_sync_start), so a test using it drives the exact
 * production async path -- not a divergent reimplementation. It lives in
 * system service 0x0000 under a reserved test opcode number (0x00F0) that
 * no production opcode uses.
 *
 * Header-only (like mtk_test_bootstrap.h): the entry is a translation-unit-
 * local static in each test that includes this, and register/unregister are
 * inline. mtk_test_bootstrap() (mtk_test_bootstrap.h) is unchanged; a test
 * that wants the fixture calls mtk_test_async_fixture_install() after its
 * own core/router/service setup and mtk_test_async_fixture_uninstall() (or
 * relies on process exit) in teardown. */
#pragma once
#include "mtek_opcode_registry.h"
#include "mtek_opcode_overlay.h"
#include "mtek_schema_message_descs.h"

/* service 0x0000 (system), reserved test opcode 0x00F0 -- mirror of the
 * former TIME_SYNC_START metadata: ACCEPTED_ASYNC, MTK_ARB_NONE,
 * no_radio_lease=1, SUPPORTED on every profile (it only ever exists in the
 * overlay, so this never reaches GET_CAPABILITIES), same req/resp
 * descriptors and 15000/1000/60000 deadline envelope, cancellable. */
static const mtk_opcode_entry_t g_mtk_test_async_op = {
    "TEST_ASYNC_NOARB", 0x0000, 0x00F0, MTK_LC_ACCEPTED_ASYNC, MTK_ARB_NONE, 1,
    MTK_CAP_SUPPORTED, MTK_CAP_SUPPORTED, MTK_CAP_SUPPORTED,
    15000, 1000, 60000, 1, 0,
    &mtk_time_sync_start_req_t_desc, &mtk_time_sync_start_resp_t_desc,
};

/* RC12 blocker round, item 1: the paired test-only generic SYNCHRONOUS STOP
 * (0x00F1), routed by mtek_system_dispatch (under MTK_ENABLE_TEST_OPCODES,
 * never compiled into the ESP32 target) to the SAME production
 * handle_time_sync_stop. Provided so tests that need a generic token-
 * addressed STOP no longer depend on the real TIME_SYNC_STOP opcode, which
 * is now UNSUPPORTED on native. Reuses the TIME_SYNC_STOP descriptors, so a
 * test encoding a mtk_time_sync_stop_req_t against op->req_desc is
 * unchanged. handle_time_sync_stop family-validates a 0x00F1 STOP against
 * the test START family (0x00F0), so it correctly stops tokens minted by
 * g_mtk_test_async_op above. */
static const mtk_opcode_entry_t g_mtk_test_async_stop_op = {
    "TEST_ASYNC_NOARB_STOP", 0x0000, 0x00F1, MTK_LC_SYNCHRONOUS, MTK_ARB_NONE, 1,
    MTK_CAP_SUPPORTED, MTK_CAP_SUPPORTED, MTK_CAP_SUPPORTED,
    1000, 100, 1000, 0, 1,
    &mtk_time_sync_stop_req_t_desc, &mtk_time_sync_stop_resp_t_desc,
};

/* Registers the fixture opcode in the test-only overlay so mtk_opcode_find
 * (and thus the router and native-SPI dispatch) resolves it. Returns the
 * opcode entry to use in place of a former mtk_test_find_op("TIME_SYNC_
 * START"). The request/response wire encoding is byte-identical to
 * TIME_SYNC_START (same descriptors), so a test encoding a
 * mtk_time_sync_start_req_t against op->req_desc is unchanged. */
static inline const mtk_opcode_entry_t *mtk_test_async_fixture_install(void) {
    mtk_opcode_overlay_register(&g_mtk_test_async_op);
    return &g_mtk_test_async_op;
}

/* Registers AND returns the paired generic test-only STOP opcode (0x00F1).
 * Idempotently also registers the START op, so a test can call this alone. */
static inline const mtk_opcode_entry_t *mtk_test_async_fixture_stop_install(void) {
    mtk_opcode_overlay_register(&g_mtk_test_async_op);
    mtk_opcode_overlay_register(&g_mtk_test_async_stop_op);
    return &g_mtk_test_async_stop_op;
}

static inline void mtk_test_async_fixture_uninstall(void) {
    mtk_opcode_overlay_clear();
}
