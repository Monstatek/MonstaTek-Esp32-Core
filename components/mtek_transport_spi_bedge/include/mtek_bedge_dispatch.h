/* Clean-room implementation from MonstaTek contract. Translates parsed
 * Bedge/C3 `m1_link` REQUEST frames (mtek_bedge_frame.h) into canonical
 * router calls (mtek_router.h) and formats the result back into a
 * RESP/NAK/FRAG frame sequence. Portable: no ESP-IDF dependency,
 * host-testable. */
#pragma once
#include "mtek_bedge_frame.h"
#include "mtek_async_queue.h"
#include "mtek_schema_structs.h"
#include <stdint.h>

/* RC8 independent audit P0-1 "Eliminate target stack overflow paths":
 * moved here from mtek_bedge_dispatch.c (where it was previously declared
 * only as a same-file-static-function-signature type) so it can be a
 * NAMED, dctx-owned field below instead of a stack-local -- see that
 * struct's own doc comment for the full before/after accounting. Holds
 * one canonical response capture (status/body/body_len) plus a few
 * opcode-family-specific scratch fields threaded through the dispatch
 * helper functions that build a Bedge-shaped reply from one or more
 * canonical dispatch calls. */
#define BEDGE_CAP_BODY_MAX MTK_BEDGE_MAX_REASSEMBLY_PAYLOAD
typedef struct {
    uint8_t status;
    uint8_t body[BEDGE_CAP_BODY_MAX];
    size_t body_len;
    uint8_t got_scan_generation;
    uint32_t scan_generation;
    uint8_t got_connection_token;
    uint32_t connection_token;
    /* RC12 Astra-closure item 2 "honest GATT terminal-failure semantics":
     * the terminal event's own canonical status (MTK_STATUS_OK on a real
     * connect, MTK_STATUS_TIMEOUT on a failed one -- mtek_ble_logic.c
     * handle_gatt_connect's only two terminal outcomes). Lets the GATT
     * translation emit a bare RESP on success but the mapped NAK on a
     * terminal failure, instead of an unconditional OK. Meaningfully
     * non-OK only for GATT; AP/STA scan-complete extracts set it OK. */
    uint8_t event_status;
    uint8_t deferred;
} bedge_capture_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Staged outbound response too large for one 502-byte cell: emitted as
 * FRAG cells (matching mtk_bedge_reassembly_feed's own inbound
 * convention exactly, so the same framing is symmetric in both
 * directions) followed by one terminal RESP/NAK cell carrying the final
 * chunk. Drained one cell per poll via mtek_bedge_dispatch_poll_outbound,
 * matching the Bedge transport's own poll-driven, one-outstanding-
 * message model. */
typedef struct {
    uint8_t active;
    uint16_t msg_id;
    uint8_t final_msg_type; /* MTK_BEDGE_MSG_RESP or MTK_BEDGE_MSG_NAK */
    uint16_t total_len;
    uint16_t sent_offset;
    uint8_t data[MTK_BEDGE_MAX_REASSEMBLY_PAYLOAD];
} mtk_bedge_outbound_t;

typedef struct {
    uint32_t boot_epoch;
    uint32_t next_correlation;
    /* Per-family operation tokens: minted by this dispatch layer's own
     * successful *_START/CONNECT calls, consumed (and cleared) by the
     * matching *_STOP; each family can have at most one Bedge-originated
     * session active at a time, matching the arbiter's own single-active-
     * class-per-resource model. */
    uint32_t capture_token;
    uint32_t deauth_token;
    uint32_t handshake_token;
    uint32_t karma_token;
    uint32_t beacon_token;
    uint32_t probe_token;
    uint32_t softap_token;
    uint32_t captive_token;
    uint32_t gatt_conn_token;
    uint32_t ble_adv_token;
    /* Populated from AP_SCAN_COMPLETE/STA_SCAN_COMPLETE terminal events
     * (captured via the dispatch sink's emit_event), consumed by
     * AP_SCAN_RESULTS_PAGE/STA_SCAN_RESULTS_PAGE. */
    uint32_t ap_scan_generation;
    uint32_t sta_scan_generation;
    uint8_t ap_scan_has_generation;
    uint8_t sta_scan_has_generation;

    mtk_bedge_outbound_t outbound;

    /* Coverage/test instrumentation only (never read by any adapter/
     * production code path): the (service_id, opcode) pair the most
     * recent mtek_bedge_dispatch_request call actually routed to the
     * canonical router, or (0xFFFF, 0xFFFF) if the call never reached the
     * router at all (an opcode this dispatch layer does not recognize on
     * the wire). Lets a host test prove every one of the 44 bedge_c3-
     * SUPPORTED canonical opcodes is reachable through its real,
     * documented Bedge wire opcode -- not merely present in a registry. */
    uint16_t last_dispatched_service_id;
    uint16_t last_dispatched_opcode;

    /* Persistent, adapter-owned async delivery (mtek_router.h's SAFETY
     * CONTRACT): when a registered async runner defers an ACCEPTED_ASYNC
     * opcode's handler to a background task, that task's own
     * emit_response call lands here instead of a stack-local capture --
     * dctx (and this queue inside it) outlives the synchronous
     * mtek_bedge_dispatch_request call that kicked it off. Bedge is
     * strictly single-outstanding-request, so at most one deferred
     * accept is ever pending: `pending_start_msg_id` (0 = none) records
     * which original REQUEST's RESP/NAK is still owed once the worker
     * task's real emit_response call reaches this queue. */
    mtk_async_queue_t event_queue;
    uint16_t pending_start_msg_id;
    /* Where to harvest the deferred ACCEPTED response's operation_token
     * once it arrives (a pointer into this same dctx's own token field,
     * always valid for dctx's whole lifetime), and which canonical
     * opcode's resp_desc to decode it with. NULL/0 when nothing pending. */
    uint32_t *pending_token_field;
    uint16_t pending_service_id, pending_opcode;
    /* CAPTURE_START's confirmed Bedge response format (raw 4-byte LE
     * esp_err_t) is not the generic bare-status convention every other
     * deferred opcode uses -- set when the still-pending operation needs
     * that special formatting once its real response is delivered. */
    uint8_t pending_is_capture_raw_errno;

    /* RC12 blocker round, item 2 "real deferred Community execution":
     * AP_SCAN_START/STA_SCAN_START/GATT_CONNECT produce their real Bedge
     * response only from a TERMINAL EVENT (AP/STA result_generation, GATT
     * connection_token) that the deferred worker emits AFTER its ACCEPTED
     * response, possibly several polls later. The bare pending_start_msg_id
     * machinery above only ever relayed the ACCEPTED status and DISCARDED
     * that event -- silently losing the scan list / generation / connection
     * token on the real (async-runner) target. This continuation retains
     * the owed request kind, consumes the ACCEPTED response and the terminal
     * event internally across polls (waiting, emitting IDLE, until BOTH have
     * arrived), captures the needed field, and only then produces the
     * confirmed Community response (for AP scan, by issuing the canonical
     * AP_SCAN_RESULTS_PAGE and building the same network-list bytes the
     * synchronous path builds). No new Bedge wire event or opcode is
     * introduced. pending_continuation==0 leaves every other deferred opcode
     * (DEAUTH/HANDSHAKE/CAPTURE bare-status async) on its existing path,
     * unchanged. */
    uint8_t pending_continuation;      /* 0=none; MTK_BEDGE_CONT_* otherwise */
    char pending_event_name[MTK_ASYNC_EVENT_NAME_MAX]; /* terminal event to await */
    uint8_t pending_have_response;     /* the ACCEPTED response frame has arrived */
    uint8_t pending_response_status;   /* its canonical status */
    uint8_t pending_have_event;        /* the awaited terminal event has arrived */
    uint8_t pending_event_ok;          /* that event's own status was OK (GATT) */
    uint8_t pending_event_status;      /* RC12 Astra-closure item 2: the terminal
                                        * event's canonical status, so a deferred
                                        * GATT failure maps to the correct NAK
                                        * instead of an unconditional OK */
    uint32_t pending_event_generation; /* harvested AP/STA result_generation */
    uint32_t pending_event_conn_token; /* harvested GATT connection_token */

    /* RC8 independent audit P0-1 "Eliminate target stack overflow paths":
     * real ELF disassembly showed mtek_bedge_dispatch_request's own frame
     * (~8,272 bytes, dominated by a stack-local `bedge_capture_t cap` --
     * ~4.1KB -- plus a second, inline ~4KB scratch buffer for one opcode
     * case) and its nested handle_ap_scan_start path (~10,496 bytes,
     * dominated by ITS OWN stack-local `bedge_capture_t page_cap` plus a
     * third ~4KB scratch buffer) together reachable well past the
     * 12,288-byte configured task stack -- at least ~18.7KB just from
     * these two frames' declared locals, before any deeper call's own
     * frame. `cap`/`scratch_cap`/`async_complete_cap` below replace every
     * such stack-local `bedge_capture_t` across this whole file (dispatch_
     * request's own primary capture, every handler needing a SECOND
     * capture for an intermediate canonical call -- GET_STATUS's
     * capabilities probe, AP_SCAN_START's own results-page follow-up --
     * and mtek_bedge_dispatch_poll_outbound's own async-completion
     * capture) with dctx-owned fields; every one of the three ~4KB
     * per-handler `uint8_t out[BEDGE_CAP_BODY_MAX]` scratch buffers this
     * audit also found (AP_SCAN_START, STA_SCAN_RESULTS_PAGE,
     * CAPTURE_POLL_READ) was eliminated entirely rather than relocated --
     * each already had `cap->body` itself (now dctx-owned) as its exact
     * final destination, so building the result directly into `cap->body`
     * (via mtk_encode-shaped writes or memmove for the one genuinely
     * in-place transform, CAPTURE_POLL_READ) needs no scratch copy at
     * all. `ap_scan_page` (~2.1KB, `mtk_ap_scan_results_page_resp_t`'s own
     * 50-record array) replaces AP_SCAN_START's own stack-local decode
     * target for the same reason -- STA_SCAN_RESULTS_PAGE's equivalent
     * (32 7-byte station records, ~264 bytes) is small enough to stay a
     * stack local. Safe as dctx-owned fields for the identical reason
     * `sync_capture`/`outbound` already are (mtek_spi_native_dispatch.h):
     * Bedge dispatch is strictly single-outstanding-request, one
     * synchronous call at a time on one physical transaction loop's own
     * thread of control -- never concurrent with itself. */
    bedge_capture_t cap;
    bedge_capture_t scratch_cap;
    bedge_capture_t async_complete_cap;
    mtk_ap_scan_results_page_resp_t ap_scan_page;
} mtk_bedge_dispatch_ctx_t;

/* RC6 independent audit gate #2 -- see mtek_spi_native_dispatch.h's own
 * matching _Static_assert for the full rationale. This struct (~12.3KB,
 * dominated by outbound.data's 4082-byte reassembly buffer) is static in
 * main/mtek_spi_runtime.c, not stack-local. RC8 independent audit P0-1
 * raised this ceiling (20000 -> 30000) to accommodate the three
 * dctx-owned `bedge_capture_t` fields plus `ap_scan_page` added above --
 * a deliberate, measured trade of DIRAM for eliminated stack risk (see
 * docs/RESOURCE_BUDGET.md for the real `idf.py size` before/after). */
#include <assert.h>
_Static_assert(sizeof(mtk_bedge_dispatch_ctx_t) < 30000,
                "mtk_bedge_dispatch_ctx_t grew past its documented RESOURCE_BUDGET.md ceiling -- "
                "re-measure before proceeding");

void mtek_bedge_dispatch_init(mtk_bedge_dispatch_ctx_t *dctx, uint32_t boot_epoch);

/* `resp_payload` must have capacity >= MTK_BEDGE_SINGLE_CELL_PAYLOAD_MAX
 * (one wire cell's worth); a response over that size is staged into
 * dctx->outbound and drained via mtek_bedge_dispatch_poll_outbound. */
void mtek_bedge_dispatch_request(mtk_bedge_dispatch_ctx_t *dctx, const mtk_bedge_header_t *hdr, const uint8_t *payload,
                                  mtk_bedge_header_t *resp_hdr, uint8_t *resp_payload, uint16_t *resp_payload_len);

/* Called when the transport loop receives an IDLE poll from the peer
 * while dctx->outbound.active: emits the next FRAG cell, or the final
 * RESP/NAK cell that clears dctx->outbound.active. If nothing is
 * outbound, emits a well-formed IDLE cell (msg_type=MTK_BEDGE_MSG_IDLE,
 * payload_len=0) rather than leaving the caller's buffers untouched. */
void mtek_bedge_dispatch_poll_outbound(mtk_bedge_dispatch_ctx_t *dctx,
                                        mtk_bedge_header_t *resp_hdr, uint8_t *resp_payload, uint16_t *resp_payload_len);

#ifdef __cplusplus
}
#endif
