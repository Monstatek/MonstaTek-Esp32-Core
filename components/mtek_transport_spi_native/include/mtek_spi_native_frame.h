/* Clean-room implementation from MonstaTek contract (SPI_PROTOCOL_V1.md). */
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MTK_SPI_NATIVE_MAGIC 0x3153314Du /* literal bytes 4D 31 53 31 ("M1S1"), read little-endian */
#define MTK_SPI_NATIVE_CELL_SIZE 1024
#define MTK_SPI_NATIVE_HEADER_SIZE 40
#define MTK_SPI_NATIVE_MAX_PAYLOAD 984
#define MTK_SPI_NATIVE_MAJOR 1
#define MTK_SPI_NATIVE_MINOR 0

typedef enum {
    MTK_SPI_CLASS_IDLE = 0,
    MTK_SPI_CLASS_HELLO = 1,
    MTK_SPI_CLASS_HELLO_ACK = 2,
    MTK_SPI_CLASS_REQUEST = 3,
    MTK_SPI_CLASS_RESPONSE = 4,
    MTK_SPI_CLASS_EVENT = 5,
    MTK_SPI_CLASS_STREAM = 6,
    MTK_SPI_CLASS_CREDIT = 7,
    MTK_SPI_CLASS_CANCEL = 8,
    MTK_SPI_CLASS_LINK_ERROR = 9,
} mtk_spi_class_t;

#define MTK_SPI_FLAG_FIRST 0x01
#define MTK_SPI_FLAG_LAST 0x02
#define MTK_SPI_FLAG_RETRY 0x04
#define MTK_SPI_FLAG_ACK_REQUIRED 0x08
#define MTK_SPI_FLAG_RESERVED_MASK 0xF0

typedef struct {
    uint32_t magic;
    uint8_t major;
    uint8_t minor;
    uint8_t msg_class;   /* mtk_spi_class_t */
    uint8_t flags;
    uint16_t service;
    uint16_t opcode;
    uint16_t status;
    uint16_t payload_len;   /* 0..984 */
    uint32_t request_id;
    uint32_t packet_seq;
    uint32_t boot_epoch;
    uint32_t message_len;
    uint32_t fragment_offset;
    uint32_t crc32c;
} mtk_spi_native_header_t;

typedef enum {
    MTK_SPI_PARSE_OK = 0,
    MTK_SPI_PARSE_TRUNCATED,
    MTK_SPI_PARSE_BAD_MAGIC,
    MTK_SPI_PARSE_BAD_VERSION,
    MTK_SPI_PARSE_BAD_FLAGS,
    MTK_SPI_PARSE_BAD_LENGTH,
    MTK_SPI_PARSE_BAD_CLASS,
    MTK_SPI_PARSE_BAD_CRC,
} mtk_spi_parse_result_t;

uint32_t mtk_crc32c(const uint8_t *data, size_t len);

/* Multi-cell fragmentation/reassembly ----------------- this was previously
 * 65536, matching the canonical core contract's own logical control-message
 * ceiling (`MTK_BUDGET_MAX_CONTROL_PAYLOAD_BYTES`, max_control_payload_bytes)
 * "by design" (docs/RESOURCE_BUDGET.md). Real `idf.py size` measurement against
 * an actual ESP32-C6 build (this SRAM region -- ESP-IDF's own "DIRAM" -- totals
 * 452,112 bytes on this target/sdkconfig) showed three such buffers (inbound
 * reassembly, outbound staging, and the SYNCHRONOUS-opcode sync_capture added by
 * this same audit round, mtek_spi_native_dispatch.h) alone consumed 441,154
 * bytes of DIRAM (97.6%), leaving only 10,958 bytes for EVERY runtime heap need
 * (FreeRTOS task stacks, esp_wifi_init, the NimBLE BLE stack, lwIP, NVS) -- not
 * a claim this would boot, a measured demonstration it almost certainly could
 * not. This is exactly the documented contract exception scenario the RC7
 * correction order itself anticipated ("If the accepted 4x65,536-byte reassembly
 * guarantee cannot fit, report the measured conflict and proposed contract
 * exception before changing the protocol") -- native SPI v1 is this tree's own
 * List B / future-facing transport (docs/ARCHITECTURE.md's four-tier module
 * boundary), not a frozen List A shipped-parity surface, so this transport's OWN
 * reassembly/outbound/sync-capture ceiling has design freedom independent of the
 * canonical schema's field-level MTK_BUDGET_MAX_CONTROL_PAYLOAD_ BYTES=65536
 * bound (unchanged, still governs individual schema field lengths elsewhere --
 * e.g. GATT read's bytes(max=512), the capture record's bytes(max=1000)).
 * Reduced to 8192 bytes: independently checked against every real message this
 * codebase actually constructs on this transport (AP_SCAN_RESULTS_PAGE's
 * 50-record page is ~2.2KB; the largest EVENT/STREAM payload relayed through the
 * same outbound path is ~985 bytes; nothing in this tree ever legitimately needs
 * more) -- with this change, the same three-buffer group measures 24,672 bytes
 * total (three 8192-byte arrays) instead of 196,864, freeing ~172KB of DIRAM
 * back to runtime heap. See docs/RESOURCE_BUDGET.md's "RC6 measured SRAM
 * conflict" section for the full before/after `idf.py size` accounting; still
 * not a claim of hardware-verified boot success (no target access), only that
 * the prior number was a measured, near-certain boot failure and this one is
 * not. */
#define MTK_SPI_NATIVE_MAX_MESSAGE 8192

typedef struct {
    uint8_t active;
    uint32_t request_id;      /* correlates every fragment of one logical message */
    uint16_t service, opcode; /* from the first (FLAG_FIRST) fragment */
    uint32_t boot_epoch;
    uint32_t message_len;     /* total logical message length, from the first fragment */
    uint32_t received_len;    /* contiguous bytes received so far -- next expected fragment_offset */
    uint32_t last_seen_ms;   /* low 32 bits of caller-supplied monotonic milliseconds */
    uint8_t data[MTK_SPI_NATIVE_MAX_MESSAGE];
} mtk_spi_native_reassembly_t;

typedef enum {
    MTK_SPI_REASM_IN_PROGRESS = 0,
    MTK_SPI_REASM_COMPLETE,
    MTK_SPI_REASM_OVERFLOW,        /* message_len exceeds MTK_SPI_NATIVE_MAX_MESSAGE */
    MTK_SPI_REASM_GAP,             /* fragment_offset > received_len: a fragment was skipped */
    MTK_SPI_REASM_DUPLICATE,       /* fragment_offset < received_len: an already-received range was resent */
    MTK_SPI_REASM_ORPHAN_FRAGMENT, /* a non-FIRST fragment arrived with no matching in-progress context */
    /* A FLAG_FIRST fragment for a DIFFERENT request_id arrived while another
     * logical message is still actively being reassembled. This dispatcher owns
     * exactly one full-size (65,536-byte) inbound reassembly context -- a
     * measured, documented ESP32-C6 SRAM budget exception from the accepted
     * contract's 4-context guarantee (docs/RESOURCE_BUDGET.md,
     * docs/DECISION_LOG.md: 4 such contexts would be 262,144 bytes, over half
     * the chip's total 512KB SRAM, before any WiFi/BLE/FreeRTOS allocation). The
     * prior behavior silently reset and overwrote the in-progress context with
     * the new message -- permanently corrupting/losing the first message's
     * already- received bytes with no signal to either peer. This result lets
     * the caller reject the NEW fragment explicitly (LINK_ERROR) while leaving
     * the in-progress reassembly completely untouched, matching a single
     * physical SPI bus's own realistic capability: this transport serializes
     * inbound multi-cell reassembly to one logical message at a time rather than
     * silently corrupting a second one. */
    MTK_SPI_REASM_BUSY,
} mtk_spi_reasm_result_t;

void mtk_spi_native_reassembly_reset(mtk_spi_native_reassembly_t *ctx);

/* Feeds one parsed, already-CRC-valid fragment. `now_ms` is
 * caller-supplied monotonic milliseconds, recorded as `last_seen_ms`
 * on progress, letting the
 * caller implement its own timeout policy (mtk_spi_native_reassembly_
 * timed_out) without this portable component depending on a wall clock. */
mtk_spi_reasm_result_t mtk_spi_native_reassembly_feed(mtk_spi_native_reassembly_t *ctx, const mtk_spi_native_header_t *hdr,
                                                       const uint8_t *payload, uint32_t now_ms);

/* True if `ctx` is active and `now_ms - ctx->last_seen_ms >= timeout_ms`
 * (milliseconds) -- an abandoned in-flight reassembly the caller should
 * mtk_spi_native_reassembly_reset and reject. */
int mtk_spi_native_reassembly_timed_out(const mtk_spi_native_reassembly_t *ctx, uint32_t now_ms, uint32_t timeout_ms);

/* Outbound multi-cell fragmentation ----------------- Stages a
 * response/event/stream body too large for one cell; drained one cell per call
 * to mtk_spi_native_outbound_next, mirroring the inbound reassembly's own
 * FIRST/LAST flag convention exactly (symmetric wire behavior in both
 * directions). */
typedef struct {
    uint8_t active;
    mtk_spi_native_header_t hdr; /* template: msg_class/service/opcode/status/request_id/boot_epoch/packet_seq already set by the caller */
    uint32_t total_len;
    uint32_t sent_offset;
    uint8_t data[MTK_SPI_NATIVE_MAX_MESSAGE];
} mtk_spi_native_outbound_t;

void mtk_spi_native_outbound_start(mtk_spi_native_outbound_t *ob, const mtk_spi_native_header_t *hdr_template,
                                    const uint8_t *body, uint32_t body_len);
/* Builds the next cell (FLAG_FIRST on the first call, FLAG_LAST on the
 * last, both if the whole message fits in one cell) into `out`
 * (MTK_SPI_NATIVE_CELL_SIZE bytes). Returns 1 while more cells remain
 * active after this call, 0 once this was the last cell (ob->active
 * cleared). Caller must not call this when !ob->active. */
int mtk_spi_native_outbound_next(mtk_spi_native_outbound_t *ob, uint8_t *out);

/* Builds one 1024-byte cell (header + payload + zero-filled tail) into
 * `out` (must be exactly MTK_SPI_NATIVE_CELL_SIZE bytes). CRC32C is
 * computed over header bytes 0..35 followed by `payload_len` payload
 * bytes, per spec -- the unused tail is never covered. */
void mtk_spi_native_build_cell(const mtk_spi_native_header_t *hdr, const uint8_t *payload, uint8_t *out);

/* Validates and parses exactly one MTK_SPI_NATIVE_CELL_SIZE-byte cell.
 * Never partially executes an invalid message: on any result other than
 * MTK_SPI_PARSE_OK, `*hdr` is not a trustworthy request to act on. */
mtk_spi_parse_result_t mtk_spi_native_parse_cell(const uint8_t *in, mtk_spi_native_header_t *hdr,
                                                  const uint8_t **payload_out);

/* Discovery-phase variant for a buffer shorter than the full 1024-byte
 * steady-state cell (e.g. the 512-byte AUTO discovery transaction). See
 * mtek_spi_native_frame.c for the exact bound checked. */
mtk_spi_parse_result_t mtk_spi_native_parse_bounded(const uint8_t *in, size_t len, mtk_spi_native_header_t *hdr,
                                                     const uint8_t **payload_out);

/* (SPI_PROTOCOL_V1.md "Reset and resynchronization": "Packet-sequence gaps
 * increment diagnostics but do not alone reset the link"): `packet_seq`
 * increments for every transaction the sender clocks, including IDLE -- a gap
 * here means at least one physical transaction's cell was lost/corrupted/skipped
 * between two the receiver actually saw, a real (if not by itself link-fatal)
 * signal worth counting. Portable, host-testable; the real caller
 * (main/mtek_spi_runtime.c) notes every successfully-parsed cell's packet_seq
 * here regardless of class, including IDLE (which
 * mtek_spi_native_dispatch_feed_cell itself never sees -- intercepted at the
 * runtime layer before that call). */
typedef struct {
    uint8_t known;
    uint32_t last_seq;
} mtk_spi_native_packet_seq_tracker_t;

void mtk_spi_native_packet_seq_tracker_init(mtk_spi_native_packet_seq_tracker_t *t);

/* Records `seq` as the latest observed packet_seq and returns 1 if it is a
 * gap relative to the previously recorded value (anything other than
 * exactly one more, including a wraparound-tolerant check via unsigned
 * arithmetic -- 0xFFFFFFFF -> 0 is NOT a gap), 0 otherwise. The very first
 * call always returns 0 (nothing to compare against yet) and merely
 * establishes the baseline. Never itself resets the link (per spec) --
 * tracking continues normally after a gap is reported, it is not sticky. */
int mtk_spi_native_packet_seq_tracker_note(mtk_spi_native_packet_seq_tracker_t *t, uint32_t seq);

#ifdef __cplusplus
}
#endif
