/* Clean-room implementation from MonstaTek contract. Wires the ESP32-C6
 * `spi_slave` peripheral onto the production board's confirmed pins and
 * runs the AUTO-discovery + boot-exclusive dispatch loop for the native
 * SPI v1 and Bedge/C3 SPI adapters.
 *
 * Confirmed facts this file implements against (both profiles, one
 * shared physical bus -- AUTO discovery could not work at all if the two
 * profiles disagreed on pins/mode, so a single shared spi_slave
 * configuration is a structural requirement, not a guess):
 *  - ESP32 acts as SPI SLAVE; SPI mode 1 (CPOL=0, CPHA=1), MSB-first
 *    (001-profile-bootstrap-feasibility.md Sec 1.1, citing the pinned
 *    Bedge m1_link.c `#define SPI_MODE 1` and its header comment).
 *  - Confirmed production pins: SCLK=GPIO7, MOSI=GPIO12, MISO=GPIO13,
 *    CS=GPIO15, HANDSHAKE=GPIO14 (same doc, cross-confirmed against
 *    HARDWARE_EVIDENCE.md's independent production-board pin table).
 *  - HANDSHAKE timing: driven HIGH in the SPI slave's post_setup_cb
 *    ("armed & safe to clock") and LOW in post_trans_cb ("transfer done;
 *    re-arm before next") -- exact confirmed ESP-side semantic, mirrored
 *    here via spi_slave_interface_config_t's own callback hooks (same
 *    doc Sec 1.2).
 *  - DATAREADY (GPIO6, HARDWARE_EVIDENCE.md/001-profile-bootstrap-
 *    feasibility.md Sec 1.1) is profile-specific, per-profile confirmed
 *    behavior: for Bedge/C3, the pinned reference build leaves this
 *    pin's ESP-side driving DISABLED by default (`PIN_DATA_READY (-1)`
 *    unless a Kconfig option most Bedge builds do not set) -- HANDSHAKE
 *    alone gates clocking in that confirmed default configuration, so
 *    this implementation matches it (DATAREADY held low/unused whenever
 *    Bedge is the locked profile). For native SPI v1, SPI_PROTOCOL_V1.md
 *    itself specifies the semantic exactly ("GPIO state machine": high
 *    when the armed MISO cell is useful or useful outbound data is
 *    queued, low otherwise) -- implemented per-transaction below,
 *    driven high whenever the cell about to be armed is a real
 *    RESPONSE/EVENT/etc (not IDLE).
 *  - Discovery phase: 512-byte non-dispatching transactions
 *    (SPI_PROTOCOL_V1.md); a valid Bedge/C3 frame or native HELLO locks
 *    that profile for the remainder of the boot session; the losing
 *    adapter never dispatches again until reset (same doc, "Runtime
 *    transport selection"). Cross-profile rejection and the discovery
 *    parse itself reuse mtek_transport_select, already exhaustively
 *    tested on host (test_transport_select.c) -- not reimplemented here.
 *  - Once locked, native negotiates up to the 1024-byte steady-state
 *    cell (same doc); Bedge's cell size is always the fixed 512 bytes
 *    (mtek_bedge_frame.h).
 *  - Clock rate is driven by the SPI *master* (the STM32/M1 side) in
 *    this slave role; the confirmed 4.6875 MHz figure only bounds what
 *    the master should assert, and is out of this slave-mode driver's
 *    own configuration surface (spi_slave_interface_config_t carries no
 *    clock-rate field -- there is nothing to guess here).
 */
#include "mtek_spi_runtime.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "mtek_core.h"
#include "mtek_arbiter.h"
#include "mtek_router.h"
#include "mtek_transport_select.h"
#include "mtek_spi_native_frame.h"
#include "mtek_spi_native_dispatch.h"
#include "mtek_bedge_frame.h"
#include "mtek_bedge_dispatch.h"
#include <string.h>

/* ---- Shared-state locking (RC7 independent audit P0 "Shared operation/
 * session state remains data-racy") --------------------------------------
 * The real mutex, async runner, and core/arbiter/router lock registration
 * are now installed once by app_main.c BEFORE any adapter task starts
 * (including this one) -- see its own "Shared cross-adapter
 * infrastructure" doc comment. This file only needs its own copy of the
 * queue-lock trampoline (matching mtk_async_queue_set_lock's `void
 * *lock_ctx`-taking signature, distinct from mtk_core_lock_fn/
 * mtk_arbiter_lock_fn's plain `void(*)(void)`) to lock native_dctx.
 * event_queue/bedge_dctx.event_queue against the SAME shared mutex,
 * passed in as `shared_mutex` (mtek_spi_runtime_start). */
/* P0 correction (this round, requirement 7): `ctx` is app_main.c's own
 * s_shared_mutex, which can legitimately be NULL if xSemaphoreCreateMutex
 * failed there -- null-check before taking/giving it, matching every
 * *_lock_v / *_unlock_v wrapper in app_main.c. */
static void queue_lock(void *ctx) { if (ctx) xSemaphoreTake((SemaphoreHandle_t)ctx, portMAX_DELAY); }
static void queue_unlock(void *ctx) { if (ctx) xSemaphoreGive((SemaphoreHandle_t)ctx); }

#define MTK_SPI_HOST SPI2_HOST
#define PIN_SCLK 7
#define PIN_MOSI 12
#define PIN_MISO 13
#define PIN_CS 15
#define PIN_HANDSHAKE 14
#define PIN_DATAREADY 6

static const char *TAG = "mtek_spi";

static void IRAM_ATTR spi_post_setup_cb(spi_slave_transaction_t *t) {
    (void)t;
    gpio_set_level(PIN_HANDSHAKE, 1); /* raise HANDSHAKE: peer may now clock this transaction */
}
static void IRAM_ATTR spi_post_trans_cb(spi_slave_transaction_t *t) {
    (void)t;
    gpio_set_level(PIN_HANDSHAKE, 0); /* drop HANDSHAKE now that the transfer finished; ready for the next one */
}

/* RC12 hardening round, item 4 (P1/P2): returns 0 only if every GPIO
 * configuration/level call genuinely succeeded. A failure here (e.g. an
 * invalid pin or a peripheral conflict) previously went entirely
 * unchecked, so a SPI adapter could "start" against pins that were never
 * actually configured. The real result is now propagated up to app_main
 * via the startup handshake (see mtek_spi_runtime_start). */
static int configure_gpios(void) {
    gpio_config_t hs_cfg = {
        .pin_bit_mask = 1ULL << PIN_HANDSHAKE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t e = gpio_config(&hs_cfg);
    if (e != ESP_OK) { ESP_LOGE(TAG, "gpio_config(HANDSHAKE) failed: %d", (int)e); return -1; }
    e = gpio_set_level(PIN_HANDSHAKE, 0);
    if (e != ESP_OK) { ESP_LOGE(TAG, "gpio_set_level(HANDSHAKE) failed: %d", (int)e); return -1; }

    /* DATAREADY: initialized low. For the Bedge/C3 profile it is left at
     * this confirmed pinned-build default (HANDSHAKE alone gates
     * clocking there, see file header doc). For the native profile it is
     * actively driven per-transaction (see the native dispatch branch
     * below) to reflect SPI_PROTOCOL_V1.md's own "GPIO state machine"
     * semantic. Configured as a driven output rather than left floating
     * either way, so its electrical state is always well-defined. */
    gpio_config_t dr_cfg = hs_cfg;
    dr_cfg.pin_bit_mask = 1ULL << PIN_DATAREADY;
    e = gpio_config(&dr_cfg);
    if (e != ESP_OK) { ESP_LOGE(TAG, "gpio_config(DATAREADY) failed: %d", (int)e); return -1; }
    e = gpio_set_level(PIN_DATAREADY, 0);
    if (e != ESP_OK) { ESP_LOGE(TAG, "gpio_set_level(DATAREADY) failed: %d", (int)e); return -1; }
    return 0;
}

/* RC12 hardening round, item 4 (P1/P2): the startup handshake between the
 * SPI runtime task and app_main. The task performs GPIO configuration and
 * spi_slave_initialize FIRST, records the real result here, signals `done`,
 * and only then enters its transaction loop -- so mtek_spi_runtime_start
 * (below) returns the actual peripheral/GPIO init outcome to app_main
 * rather than merely "the task was created". One SPI task exists for the
 * whole boot session, so this is a one-shot static: `done` is never
 * deleted (a late give after a start()-side timeout is therefore always
 * safe). */
typedef struct {
    SemaphoreHandle_t done;
    int result; /* 0 = GPIO + spi_slave_initialize both succeeded; -1 = failed */
} spi_startup_handshake_t;
static spi_startup_handshake_t s_spi_startup;

static void spi_runtime_task(void *arg) {
    SemaphoreHandle_t shared_mutex = (SemaphoreHandle_t)arg;
    int init_rc = configure_gpios();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MTK_SPI_NATIVE_CELL_SIZE,
    };
    spi_slave_interface_config_t slave_cfg = {
        .spics_io_num = PIN_CS,
        .flags = 0,
        .queue_size = 1, /* the pinned reference model is strictly sequential -- one transaction in flight */
        .mode = 1,       /* CPOL=0, CPHA=1, confirmed */
        .post_setup_cb = spi_post_setup_cb,
        .post_trans_cb = spi_post_trans_cb,
    };
    esp_err_t err = ESP_OK;
    if (init_rc == 0) {
        err = spi_slave_initialize(MTK_SPI_HOST, &bus_cfg, &slave_cfg, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_slave_initialize failed: %d -- SPI adapters will not dispatch this boot", (int)err);
            init_rc = -1;
        }
    }
    /* Report the real GPIO+peripheral init outcome to app_main BEFORE
     * touching the loop, then bail if it failed. A SPI-only build whose
     * init failed thus never lets app_main announce normal readiness (the
     * task exits, s_spi_startup.result is -1, and mtek_spi_runtime_start
     * returns -1 -> app_main's any_transport_usable gate refuses). */
    s_spi_startup.result = init_rc;
    if (s_spi_startup.done) xSemaphoreGive(s_spi_startup.done);
    if (init_rc != 0) {
        vTaskDelete(NULL);
        return;
    }

    mtk_transport_lock_t profile = MTK_TRANSPORT_AUTO;
    size_t cell_size = MTK_BEDGE_CELL_SIZE; /* 512, the shared discovery-phase size */
    /* RC7 independent audit P0 "The release artifact starts the wrong
     * transport for shipped M1 compatibility": set true the moment this
     * SPI runtime wins the cross-transport claim (mtk_transport_claim_try,
     * right after `profile` first locks below) -- gates every actual
     * dispatch call for the rest of this boot session. Before that (still
     * MTK_TRANSPORT_AUTO) or if a different adapter (the UART REPL task)
     * already won, this stays 0 and both the native and Bedge/C3 branches
     * below answer well-formed IDLE only, never reaching
     * mtk_router_dispatch -- see each branch's own comment. */
    int can_dispatch = 0;

#if CONFIG_MTEK_ADAPTER_BEDGE_C3
    /* RC6 independent audit P0 "Target stack usage is catastrophically
     * larger than the configured stacks": bedge_dctx (~12KB, mostly
     * bedge_dctx.outbound's 4082-byte reassembly buffer plus token/
     * generation bookkeeping) is `static`, not stack-local. This task
     * never returns (infinite loop below) for the whole boot session, so
     * static storage has exactly the same lifetime a stack-local here
     * would have needed anyway -- the difference is it no longer counts
     * against spi_runtime_task's configured stack size at all. A
     * background worker task holding a pointer into it (via
     * bedge_dctx.event_queue) is exactly the persistent-adapter-owned-
     * queue shape the router's async SAFETY CONTRACT requires, and static
     * storage satisfies that at least as well as a never-returning stack
     * frame did. */
    static mtk_bedge_dispatch_ctx_t bedge_dctx;
    mtek_bedge_dispatch_init(&bedge_dctx, mtk_core_boot_epoch());
    mtk_async_queue_set_lock(&bedge_dctx.event_queue, queue_lock, queue_unlock, shared_mutex);
#endif
#if CONFIG_MTEK_ADAPTER_NATIVE_SPI
    /* Same fix, much larger magnitude: mtk_spi_native_dispatch_ctx_t is
     * dominated by its inbound reassembly buffer (65,536 bytes),
     * outbound staging buffer (65,536 bytes), and sync_capture (65,536+
     * bytes, RC6's own new field -- see mtek_spi_native_dispatch.h) --
     * independently measured at ~139KB total before this fix, the single
     * largest contributor to the audit's reported 152,688-byte overflowed
     * stack frame. `static`, not stack-local, for the identical reason as
     * bedge_dctx above. */
    static mtk_spi_native_dispatch_ctx_t native_dctx;
    mtek_spi_native_dispatch_init(&native_dctx, mtk_core_boot_epoch());
    mtk_async_queue_set_lock(&native_dctx.event_queue, queue_lock, queue_unlock, shared_mutex);
    uint32_t native_seq = 0; /* monotonic transaction counter, drives reassembly timeout tracking */
#endif

    static WORD_ALIGNED_ATTR uint8_t txbuf[MTK_SPI_NATIVE_CELL_SIZE];
    static WORD_ALIGNED_ATTR uint8_t rxbuf[MTK_SPI_NATIVE_CELL_SIZE];
    memset(txbuf, 0, sizeof(txbuf)); /* neutral first-MISO cell (all-zero), per the facts' own discovery-phase behavior */

    ESP_LOGI(TAG, "SPI slave runtime up on host %d (SCLK=%d MOSI=%d MISO=%d CS=%d HANDSHAKE=%d), AUTO discovery active",
             (int)MTK_SPI_HOST, PIN_SCLK, PIN_MOSI, PIN_MISO, PIN_CS, PIN_HANDSHAKE);

    /* RC7 independent audit P0 "Native 512-to-1024 discovery transition is
     * still wrong": the real, PROVEN (test_native_cellsize_negotiator.c)
     * transaction-timing state machine -- see its own doc comment
     * (mtek_transport_select.h) for the exact defect this replaces (a
     * prior round's single-flag version upgraded the cell size exactly
     * one transaction too early, the very transaction that carries the
     * HELLO_ACK response itself). */
    mtk_native_cellsize_negotiator_t native_negotiator;
    mtk_native_cellsize_negotiator_init(&native_negotiator);

    while (1) {
        cell_size = mtk_native_cellsize_negotiator_tick(&native_negotiator, cell_size, MTK_SPI_NATIVE_CELL_SIZE);
        spi_slave_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = cell_size * 8;
        t.tx_buffer = txbuf;
        t.rx_buffer = rxbuf;
        err = spi_slave_transmit(MTK_SPI_HOST, &t, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "spi_slave_transmit error %d", (int)err);
            continue;
        }

        memset(txbuf, 0, sizeof(txbuf)); /* default next MISO: neutral, overwritten below if there is a real response */

        if (profile == MTK_TRANSPORT_AUTO) {
            profile = mtk_transport_try_recognize_discovery(rxbuf, cell_size);
            if (profile == MTK_TRANSPORT_NATIVE_SPI) {
                ESP_LOGI(TAG, "AUTO discovery locked: native SPI v1");
                /* This transaction's HELLO is answered with a real
                 * HELLO_ACK below (the dispatch block right after this
                 * one, which now runs in the SAME iteration since
                 * `profile` was just updated) -- but that ACK is the MISO
                 * data for the transaction RIGHT AFTER this one (call it
                 * N+1), which the master -- having only just sent its
                 * HELLO in THIS transaction and not yet seen any
                 * acknowledgement that native was chosen or that a larger
                 * cell is coming -- necessarily still drives at the
                 * 512-byte discovery cadence. The upgrade must therefore
                 * take effect starting transaction N+2 (two transactions
                 * from now), not N+1 -- see
                 * mtk_native_cellsize_negotiator_hello_recognized's own
                 * doc comment (mtek_transport_select.h) for the RC7 audit
                 * finding this corrects (RC6's own single-flag version
                 * upgraded at N+1, the exact transaction carrying the ACK
                 * itself). */
                mtk_native_cellsize_negotiator_hello_recognized(&native_negotiator);
            } else if (profile == MTK_TRANSPORT_BEDGE_C3_SPI) {
                ESP_LOGI(TAG, "AUTO discovery locked: Bedge/C3 SPI");
            }
            /* Still MTK_TRANSPORT_AUTO: keep polling at the 512-byte
             * discovery size with a neutral all-zero MISO cell -- never
             * runs two adapters simultaneously, since neither is locked
             * yet. */

            /* RC7 independent audit P0 "The release artifact starts the
             * wrong transport for shipped M1 compatibility": the SPI-bus-
             * internal profile lock above (native vs. Bedge/C3) is a
             * separate question from the CROSS-transport race against the
             * UART REPL task -- this is that race's own attempt, made
             * exactly once, right when this bus first recognizes either
             * profile. If a different adapter already won (the UART REPL
             * task saw a genuine command line first), can_dispatch stays
             * 0 for the rest of this boot session: this loop keeps
             * running the real wire-level state machine (HELLO_ACK,
             * cell-size negotiation) so the peer on this physical bus is
             * never left hanging mid-handshake, but every dispatch branch
             * below only ever answers IDLE once locked-out -- see
             * mtk_transport_claim_try's own doc comment
             * (mtek_transport_select.h). */
            if (profile == MTK_TRANSPORT_NATIVE_SPI) {
                can_dispatch = mtk_transport_claim_try(MTK_PUBLIC_ADAPTER_NATIVE_SPI);
            } else if (profile == MTK_TRANSPORT_BEDGE_C3_SPI) {
                can_dispatch = mtk_transport_claim_try(MTK_PUBLIC_ADAPTER_BEDGE_C3_SPI);
            }
        }

#if CONFIG_MTEK_ADAPTER_BEDGE_C3
        if (profile == MTK_TRANSPORT_BEDGE_C3_SPI) {
            mtk_bedge_header_t resp_hdr; uint8_t resp_payload[MTK_BEDGE_SINGLE_CELL_PAYLOAD_MAX]; uint16_t resp_len = 0;
            if (!can_dispatch) {
                /* RC7 independent audit P0 "The release artifact starts
                 * the wrong transport for shipped M1 compatibility": a
                 * different adapter already won this boot session's
                 * cross-transport claim -- never reach
                 * mtek_bedge_dispatch_request/mtk_router_dispatch again.
                 * A well-formed IDLE reply (this tree's "never silence"
                 * discipline), not a hang. */
                memset(&resp_hdr, 0, sizeof(resp_hdr));
                resp_hdr.magic = MTK_BEDGE_MAGIC; resp_hdr.version = MTK_BEDGE_VERSION; resp_hdr.msg_type = MTK_BEDGE_MSG_IDLE;
                mtk_bedge_build_cell(&resp_hdr, NULL, txbuf);
            } else {
            mtk_bedge_header_t bhdr; const uint8_t *bpayload;
            mtk_bedge_parse_result_t pr = mtk_bedge_parse_bounded(rxbuf, cell_size, &bhdr, &bpayload);
            if (pr == MTK_BEDGE_PARSE_OK && bhdr.msg_type == MTK_BEDGE_MSG_REQ) {
                mtek_bedge_dispatch_request(&bedge_dctx, &bhdr, bpayload, &resp_hdr, resp_payload, &resp_len);
                mtk_bedge_build_cell(&resp_hdr, resp_payload, txbuf);
            } else if (pr == MTK_BEDGE_PARSE_OK && bhdr.msg_type == MTK_BEDGE_MSG_IDLE) {
                /* Peer polling: drain a staged fragmented response if any
                 * is outstanding, else a well-formed IDLE. */
                mtek_bedge_dispatch_poll_outbound(&bedge_dctx, &resp_hdr, resp_payload, &resp_len);
                mtk_bedge_build_cell(&resp_hdr, resp_payload, txbuf);
            } else {
                /* Malformed/unexpected cell from the peer: a well-formed
                 * IDLE reply, matching the confirmed "every cell is
                 * always a complete, parseable, magic-bearing frame"
                 * guarantee -- never silence, never a guessed response. */
                mtk_bedge_header_t idle_hdr;
                memset(&idle_hdr, 0, sizeof(idle_hdr));
                idle_hdr.magic = MTK_BEDGE_MAGIC; idle_hdr.version = MTK_BEDGE_VERSION; idle_hdr.msg_type = MTK_BEDGE_MSG_IDLE;
                mtk_bedge_build_cell(&idle_hdr, NULL, txbuf);
            }
            }
        } else
#endif
#if CONFIG_MTEK_ADAPTER_NATIVE_SPI
        if (profile == MTK_TRANSPORT_NATIVE_SPI) {
            native_seq++;
            mtk_spi_native_header_t resp_hdr; uint8_t resp_payload[MTK_SPI_NATIVE_MAX_PAYLOAD]; uint16_t resp_len = 0;
            if (!can_dispatch) {
                /* Same cross-transport lockout as the Bedge/C3 branch
                 * above -- HELLO/HELLO_ACK link-level handshaking is not
                 * repeated for a losing peer; it simply sees IDLE from
                 * here on, an honest "this bus is not the winner" signal
                 * rather than a silent stall. */
                memset(&resp_hdr, 0, sizeof(resp_hdr));
                resp_hdr.magic = MTK_SPI_NATIVE_MAGIC; resp_hdr.major = MTK_SPI_NATIVE_MAJOR; resp_hdr.minor = MTK_SPI_NATIVE_MINOR;
                resp_hdr.msg_class = MTK_SPI_CLASS_IDLE;
                mtk_spi_native_build_cell(&resp_hdr, NULL, txbuf);
            } else {
            mtk_spi_native_header_t nhdr; const uint8_t *npayload;
            mtk_spi_parse_result_t pr = mtk_spi_native_parse_cell(rxbuf, &nhdr, &npayload);
            if (pr == MTK_SPI_PARSE_OK && nhdr.msg_class == MTK_SPI_CLASS_IDLE) {
                /* RC7 independent audit item 3 "packet-sequence
                 * diagnostics": feed_cell itself never sees an IDLE cell
                 * (intercepted right here), so its own packet_seq must be
                 * noted at this call site instead, or IDLE polls would be
                 * invisible to gap detection entirely. */
                if (mtk_spi_native_packet_seq_tracker_note(&native_dctx.packet_seq_tracker, nhdr.packet_seq)) {
                    mtk_transport_counters_add_packet_seq_gap();
                }
                /* Peer polling: drain a staged multi-cell response if any
                 * is outstanding, else a well-formed IDLE. */
                mtek_spi_native_dispatch_poll_outbound(&native_dctx, &resp_hdr, resp_payload, &resp_len);
                mtk_spi_native_build_cell(&resp_hdr, resp_payload, txbuf);
            } else if (pr == MTK_SPI_PARSE_OK) {
                mtek_spi_native_dispatch_feed_cell(&native_dctx, &nhdr, npayload, native_seq, &resp_hdr, resp_payload, &resp_len);
                mtk_spi_native_build_cell(&resp_hdr, resp_payload, txbuf);
            } else {
                /* RC7 independent audit item 3 "packet-sequence
                 * diagnostics" / SPI_PROTOCOL_V1.md "Parser requirements":
                 * "Integrity failures are counted and dropped; do not
                 * trust a corrupted request ID enough to act on it." A
                 * failed parse (bad magic/version/CRC/length/flags/class)
                 * cannot be safely fed to anything downstream -- counted
                 * here, then answered with an honest IDLE (never a
                 * fabricated response for a message this session could not
                 * actually validate). */
                mtk_transport_counters_add_integrity_failure();
                mtk_spi_native_header_t idle_hdr;
                memset(&idle_hdr, 0, sizeof(idle_hdr));
                idle_hdr.magic = MTK_SPI_NATIVE_MAGIC; idle_hdr.major = MTK_SPI_NATIVE_MAJOR; idle_hdr.minor = MTK_SPI_NATIVE_MINOR;
                idle_hdr.msg_class = MTK_SPI_CLASS_IDLE;
                resp_hdr = idle_hdr;
                mtk_spi_native_build_cell(&idle_hdr, NULL, txbuf);
            }
            }
            /* DATAREADY per SPI_PROTOCOL_V1.md "GPIO state machine": high
             * exactly when the now-armed next MISO cell is useful (a real
             * RESPONSE/EVENT/etc, not IDLE) OR useful outbound data is
             * still queued (a staged multi-cell response with more
             * FRAG-equivalent cells left to drain) -- either condition
             * means the peer should keep clocking. Set for the
             * transaction about to be armed (the next spi_slave_transmit
             * call below). */
            gpio_set_level(PIN_DATAREADY, (resp_hdr.msg_class != MTK_SPI_CLASS_IDLE || native_dctx.outbound.active) ? 1 : 0);
        } else
#endif
        {
            /* profile == MTK_TRANSPORT_AUTO (still discovering), or the
             * peer locked a profile this product build has compiled out
             * of dispatch -- txbuf is already the neutral all-zero cell
             * set above; nothing further to do this transaction. */
        }
    }
}

int mtek_spi_runtime_start(SemaphoreHandle_t shared_mutex) {
    /* RC6 independent audit P0 "Target stack usage is catastrophically
     * larger than the configured stacks": bedge_dctx/native_dctx (the
     * dominant contributors, ~139KB+12KB) are now static, not stack-local
     * (see their own doc comments above), so this configured stack no
     * longer needs to hold them. What remains is txbuf/rxbuf (already
     * static), the `spi_slave_transaction_t t` local, and whatever
     * per-call-frame depth the native/Bedge dispatch call chain and
     * ESP-IDF's own spi_slave/router/service/HAL calls need beneath them --
     * bumped from the prior 8192 to 12288 bytes as a measured-safer
     * starting point given the removed footprint, but the real per-
     * transaction high-water mark (uxTaskGetStackHighWaterMark) has not
     * been measured on real hardware this session (no target access) and
     * remains a disclosed gap, not a claimed-safe number -- see
     * docs/RESOURCE_BUDGET.md. */
    /* P0 correction (Round 8, item 4 "check ... SPI runtime task
     * creation"): previously discarded entirely -- see this function's own
     * header doc comment (mtek_spi_runtime.h) for what the caller does
     * with a nonzero return.
     *
     * RC12 hardening round, item 4 (P1/P2): task creation succeeding is no
     * longer reported as readiness. A bounded startup handshake now waits
     * for the task to actually configure its GPIOs and initialize the
     * spi_slave peripheral, and returns THAT result -- so a GPIO/peripheral
     * init failure (or a task that never reaches readiness within the
     * bound) is reported to app_main as a real failure, not masked behind
     * "the task was created". The transaction loop and transport-selection
     * timing are unchanged: the task signals `done` and immediately
     * proceeds into its loop, so a successful handshake returns within a
     * few milliseconds and adds no wire-visible delay. */
    s_spi_startup.done = xSemaphoreCreateBinary();
    if (!s_spi_startup.done) {
        ESP_LOGE(TAG, "mtek_spi_runtime_start: xSemaphoreCreateBinary(startup) failed");
        return -1;
    }
    s_spi_startup.result = -1;
    if (xTaskCreate(spi_runtime_task, "mtek_spi_runtime", 12288, (void *)shared_mutex, 6, NULL) != pdPASS) {
        return -1;
    }
    /* Bounded wait for the task's real GPIO + spi_slave_initialize result.
     * 5s is far beyond the millisecond-scale cost of those calls; a timeout
     * here is itself treated as "not ready" rather than a false success.
     * s_spi_startup.done is intentionally never deleted (one SPI task per
     * boot), so a late give after a timeout can never touch freed memory. */
    if (xSemaphoreTake(s_spi_startup.done, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "mtek_spi_runtime_start: SPI task did not report startup readiness within 5s");
        return -1;
    }
    return s_spi_startup.result;
}
