/* Clean-room implementation from MonstaTek contract (SPI_PROTOCOL_V1.md).
 * Translates parsed native SPI v1 cells into canonical router calls and
 * back, including real multi-cell request reassembly and response
 * fragmentation (mtek_spi_native_frame.h). Portable: no ESP-IDF
 * dependency, host-testable. */
#pragma once
#include "mtek_spi_native_frame.h"
#include "mtek_async_queue.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RC5 independent audit P0 "Native in-flight request guarantee is not
 * met": the router's own async execution pool (mtek_router.c's
 * MTK_ROUTER_ASYNC_POOL_SIZE) is exactly 4, matching the core contract's
 * declared 4-in-flight-request budget (002-canonical-core-contract.md
 * Sec 6) -- a 5th concurrent ACCEPTED_ASYNC dispatch already gets an
 * immediate synchronous NO_MEMORY from the router itself, before this
 * dispatch layer ever needs to remember it. So this transport-level
 * pending table only ever needs to hold at most 4 entries at once; the
 * bug was collapsing that to exactly 1 (a second deferred request would
 * silently overwrite the first's identity, permanently orphaning its
 * eventual response). */
#define MTK_SPI_NATIVE_MAX_IN_FLIGHT 4

typedef struct {
    uint8_t active;
    uint16_t service, opcode;
    uint32_t request_id, packet_seq;
    /* Content identity (SPI_PROTOCOL_V1.md "Correlation and duplicate
     * safety": "A retry uses the same request ID and byte-identical
     * logical request"): payload_len plus a CRC32C of the payload bytes
     * (the same algorithm already used for the wire's own frame integrity
     * check, mtk_crc32c) -- a disclosed engineering choice standing in for
     * a full byte-for-byte payload retention, which would cost another
     * full MTK_SPI_NATIVE_MAX_PAYLOAD-sized buffer per in-flight slot for
     * no observable benefit at this control protocol's request rate and
     * bounded size: any hash collision would at worst answer a genuinely
     * different request from a stale in-flight slot as "still processing"
     * (an honest, recoverable IDLE) rather than corrupt or misdirect any
     * side effect. */
    uint16_t payload_len;
    uint32_t payload_crc;
} mtk_spi_native_pending_t;

/* RC7 independent audit item 3 "RETRY/latest-eight duplicate cache"
 * (SPI_PROTOCOL_V1.md "Correlation and duplicate safety": "ESP caches the
 * identity and completed response of the latest eight side-effecting
 * requests. A valid duplicate is answered without re-execution.";
 * 002-canonical-core-contract.md Sec 6: "Duplicate-response cache depth |
 * 8 side-effecting requests"). A "side-effecting" request is one whose
 * opcode table entry has `idempotent == 0` -- re-executing it would have
 * an observably different effect than a single execution (e.g.
 * DEAUTH_START restarting its own send cadence), unlike a genuinely
 * idempotent query (GET_VERSION, PING) that this cache does not need to
 * bother with. FIFO ring of the eight MOST RECENTLY COMPLETED such
 * requests' identity + final response, so a retry whose original response
 * never reached the peer (lost on the wire) is answered from here instead
 * of being re-executed -- see dispatch_complete_message's own doc comment
 * for the full three-stage lookup (still-active pending[] slot, then this
 * cache, then a genuinely new dispatch). */
#define MTK_SPI_NATIVE_DUP_CACHE_SIZE 8
typedef struct {
    uint8_t used;
    uint32_t request_id;
    uint16_t service, opcode;
    uint16_t payload_len;
    uint32_t payload_crc;
    uint8_t status;
    uint8_t body[MTK_SPI_NATIVE_MAX_PAYLOAD];
    uint16_t body_len;
} mtk_spi_native_dup_entry_t;

/* RC6 independent audit P0 "Target stack usage is catastrophically larger
 * than the configured stacks": the SYNCHRONOUS-lifecycle dispatch capture
 * (large enough, MTK_SPI_NATIVE_MAX_MESSAGE, to carry any response up to
 * the reassembly ceiling) is a named field of the dispatch context, not a
 * stack-local inside dispatch_complete_message -- dctx itself is static/
 * heap-owned by every real caller (main/mtek_spi_runtime.c's
 * spi_runtime_task, which never returns for the whole boot session; every
 * host test allocates its own dctx on that test function's stack, which is
 * fine since host stacks are orders of magnitude larger and this is not a
 * target-constrained context). Safe as a single shared field: SYNCHRONOUS
 * opcodes are dispatched directly on the physical transaction loop's own
 * single thread of control, one at a time, never concurrently with
 * themselves. */
typedef struct {
    uint8_t status;
    uint8_t body[MTK_SPI_NATIVE_MAX_MESSAGE];
    size_t body_len;
} mtk_spi_native_sync_capture_t;

typedef struct {
    /* Release-tooling-round P0 correction (independent audit, "Native
     * SPI confuses the STM32 and ESP boot epochs"): this field tracks ONLY
     * the currently-recognized PEER (STM32) boot epoch -- adopted from an
     * inbound HELLO's own header field (SPI_PROTOCOL_V1.md: "boot_epoch:
     * random nonzero value regenerated on every SENDER boot"), used solely
     * to (a) detect a peer reboot (an inbound HELLO whose boot_epoch
     * differs from this) and (b) reject a CREDIT/CANCEL cell that names a
     * stale peer session. It must NEVER be used to populate the canonical
     * request context's boot_epoch field (002-canonical-core-contract.md
     * §2/§3.4: that is always the ESP's OWN in-memory epoch, mtk_core_
     * boot_epoch() -- one core-owned value per ESP boot, stamped into
     * every operation token at mint time) nor to stamp any ESP-originated
     * outbound wire cell (HELLO_ACK/RESPONSE/EVENT/STREAM/LINK_ERROR/IDLE
     * all carry the ESP's OWN epoch as "sender", per the same protocol
     * line -- never an echo of whatever epoch the peer last reported).
     * Before this fix, a single field served both roles: adopting the
     * peer's own random epoch here on HELLO, then using that SAME
     * (now peer-owned) value both for ctx.boot_epoch and for outbound wire
     * stamping -- harmless only because every host test happens to
     * initialize the peer's HELLO epoch, this dctx's own seed, and
     * mtk_core's own epoch to the identical value (0x1234 or
     * MTK_TEST_BOOT_EPOCH), which a real STM32 peer (an independent device
     * with its own random RNG) never would. On real hardware this made
     * every operation-token lookup after the first real HELLO fail with
     * NOT_FOUND (START stamps a new record with mtk_core_boot_epoch(), but
     * STATUS/STOP dispatched ctx.boot_epoch = the peer's own epoch). */
    uint32_t peer_boot_epoch;
    mtk_spi_native_reassembly_t inbound;
    mtk_spi_native_outbound_t outbound;
    /* Persistent, adapter-owned async delivery (mtek_router.h's SAFETY
     * CONTRACT): when a registered async runner defers an ACCEPTED_ASYNC
     * opcode's handler to a background task, that task's own
     * emit_response call lands here instead of a stack-local capture.
     * Frames carry their originating request's correlation (= request_id)
     * so mtek_spi_native_dispatch_poll_outbound can match a delivered
     * frame back to the correct pending[] slot regardless of completion
     * order across up to 4 genuinely concurrent deferred operations. */
    mtk_async_queue_t event_queue;
    mtk_spi_native_pending_t pending[MTK_SPI_NATIVE_MAX_IN_FLIGHT];
    mtk_spi_native_sync_capture_t sync_capture;
    /* RC7 independent audit item 3 "RETRY/latest-eight duplicate cache". */
    mtk_spi_native_dup_entry_t dup_cache[MTK_SPI_NATIVE_DUP_CACHE_SIZE];
    uint32_t dup_cache_next; /* ring insertion index -- FIFO eviction of the OLDEST of the latest eight */
    /* RC7 independent audit item 3 "packet-sequence diagnostics". */
    mtk_spi_native_packet_seq_tracker_t packet_seq_tracker;
} mtk_spi_native_dispatch_ctx_t;

/* RC6 independent audit gate #2 "The exact target stack/RAM/heap budget is
 * published and mechanically checked in CI/build scripts": this dctx is
 * the single largest static allocation in the whole firmware (dominated by
 * inbound.data/outbound.data/sync_capture.body, three independent
 * 65,536-byte arrays -- see docs/RESOURCE_BUDGET.md's own "RC6 measured
 * SRAM conflict" section for the full accounting and the contract
 * exception it documents: the accepted 4-independent-reassembly-context
 * guarantee does not fit this chip's published 512KB total SRAM, so this
 * dispatcher keeps exactly ONE full-size inbound context, serialized
 * rather than duplicated -- MTK_SPI_REASM_BUSY, mtek_spi_native_frame.h).
 * This bound is deliberately generous (fails loud at link/compile time
 * long before any real budget ceiling) -- its only job is to catch a
 * future change that silently multiplies this struct's size (e.g. bumping
 * MTK_SPI_NATIVE_MAX_MESSAGE, or adding a second full-size buffer) before
 * it ever reaches a build, not to assert this size is itself safe against
 * the chip's real free-heap budget (which needs hardware measurement --
 * see docs/RESOURCE_BUDGET.md). */
#include <assert.h>
_Static_assert(sizeof(mtk_spi_native_dispatch_ctx_t) < 60000,
                "mtk_spi_native_dispatch_ctx_t grew past its documented RESOURCE_BUDGET.md ceiling -- "
                "re-measure the RC6 SRAM conflict (idf.py size) before proceeding");

void mtek_spi_native_dispatch_init(mtk_spi_native_dispatch_ctx_t *dctx, uint32_t boot_epoch);

/* Local inactivity policy, not a negotiated wire value: two seconds since
 * the last accepted fragment. Owner task calls tick even without traffic.
 * now_ms is the low 32 bits of monotonic milliseconds (wrap-safe). */
#define MTK_SPI_NATIVE_REASM_TIMEOUT_MS 2000u
void mtek_spi_native_dispatch_tick(mtk_spi_native_dispatch_ctx_t *dctx, uint32_t now_ms);

/* Feeds one already-parsed, already-CRC-valid physical cell. `now_ms` is
 * caller-supplied monotonic milliseconds, not a transaction counter,
 * used only to drive inbound-reassembly timeout tracking
 * (mtk_spi_native_reassembly_timed_out is checked internally before
 * accepting a continuation fragment for an already-abandoned message).
 * Writes this transaction's immediate reply into resp_hdr/resp_payload
 * (capacity >= MTK_SPI_NATIVE_MAX_PAYLOAD)/resp_payload_len: IDLE while a
 * multi-cell request is still being accumulated, HELLO_ACK for a HELLO,
 * LINK_ERROR for a genuine reassembly protocol violation (gap/duplicate/
 * orphan fragment/overflow/timeout), or the real dispatched RESPONSE
 * (single-cell, or the first cell of a staged multi-cell one -- see
 * mtek_spi_native_dispatch_poll_outbound). */
void mtek_spi_native_dispatch_feed_cell(mtk_spi_native_dispatch_ctx_t *dctx, const mtk_spi_native_header_t *hdr, const uint8_t *payload,
                                         uint32_t now_ms, mtk_spi_native_header_t *resp_hdr, uint8_t *resp_payload, uint16_t *resp_payload_len);

/* Called when the physical loop receives an IDLE poll from the peer
 * while dctx->outbound.active: emits the next FRAG-equivalent
 * (FIRST/LAST-flagged) cell of a staged multi-cell response. If nothing
 * is outbound, emits a well-formed IDLE cell. */
void mtek_spi_native_dispatch_poll_outbound(mtk_spi_native_dispatch_ctx_t *dctx,
                                             mtk_spi_native_header_t *resp_hdr, uint8_t *resp_payload, uint16_t *resp_payload_len);

#ifdef __cplusplus
}
#endif
