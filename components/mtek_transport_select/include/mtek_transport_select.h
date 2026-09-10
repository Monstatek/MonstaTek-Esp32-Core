/* Clean-room implementation from MonstaTek contract
 * (SPI_PROTOCOL_V1.md "Runtime transport selection"): the ESP application
 * starts in AUTO; the first valid complete recognized operational input
 * locks the boot session to exactly one adapter. Portable: no ESP-IDF
 * dependency, host-testable against synthetic discovery buffers. */
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MTK_TRANSPORT_AUTO = 0,
    MTK_TRANSPORT_NATIVE_SPI,
    MTK_TRANSPORT_COMPAT_SPI,
} mtk_transport_lock_t;

/* Examines one discovery-phase transaction buffer (SPI_PROTOCOL_V1.md:
 * "a non-dispatching 512-byte discovery phase") and returns which profile
 * it recognizes, or MTK_TRANSPORT_AUTO if neither parser accepts it (stay
 * in AUTO and keep polling). Cross-profile rejection is guaranteed by
 * construction: native's fixed 4-byte magic ("M1S1", wire bytes
 * 4D 31 53 31) and Legacy SPI Compatibility's fixed 2-byte magic (0x4D31 LE, wire bytes
 * 31 4D) are disjoint at byte 0 (0x4D vs 0x31)
 * (001-profile-bootstrap-feasibility.md Sec 2), so at most one of the two
 * bounded parsers below can ever accept the same buffer -- never both. */
mtk_transport_lock_t mtk_transport_try_recognize_discovery(const uint8_t *buf, size_t len);

/* RC7 independent audit P0 "The release artifact starts the wrong
 * transport for shipped M1 compatibility": SPI_PROTOCOL_V1.md's own
 * "Runtime transport selection" section requires AUTO discovery across a
 * native SPI HELLO, a Legacy SPI Compatibility discovery frame, OR a valid legacy UART
 * command -- three independent physical listeners (UART0 and the shared
 * SPI bus are genuinely different buses, so both can and must run
 * concurrently before selection) that may all be live at boot, but
 * "only canonical dispatch must become exclusive after the first valid
 * operational input" (the audit's own words). This is that cross-
 * transport exclusivity latch: separate from (and layered on top of)
 * mtk_transport_try_recognize_discovery above, which only arbitrates
 * between the two SPI-bus profiles sharing one physical bus. Portable
 * (no ESP-IDF dependency, host-testable); thread-safe via caller-supplied
 * lock hooks, matching every other cross-task primitive in this tree
 * (mtek_core.h, mtek_arbiter.h, mtek_async_queue.h) -- with no hooks
 * registered, safe only from a single thread of control. */
typedef enum {
    MTK_PUBLIC_ADAPTER_NONE = 0,
    MTK_PUBLIC_ADAPTER_FACTORY_UART,
    MTK_PUBLIC_ADAPTER_NATIVE_SPI,
    MTK_PUBLIC_ADAPTER_COMPAT_SPI,
} mtk_public_adapter_t;

typedef void (*mtk_transport_claim_lock_fn)(void);

/* Resets the claim to NONE -- call once at boot, before any adapter task
 * starts trying to claim. */
void mtk_transport_claim_reset(void);

void mtk_transport_claim_set_lock(mtk_transport_claim_lock_fn lock, mtk_transport_claim_lock_fn unlock);

/* Attempts to claim exclusivity for `which`. Returns 1 if `which` now
 * owns (or already owned) the claim -- this caller may dispatch through
 * the canonical router for the rest of the boot session. Returns 0 if a
 * DIFFERENT adapter already won first -- this caller must never dispatch
 * through the canonical router again this boot session (it may still run
 * its own physical transaction loop and answer well-formed neutral/IDLE
 * traffic, matching this tree's "never silence" discipline, but must not
 * reach mtk_router_dispatch). Idempotent for repeated calls with the same
 * `which` that already won. */
int mtk_transport_claim_try(mtk_public_adapter_t which);

mtk_public_adapter_t mtk_transport_claim_get(void);

/* RC7 independent audit P0 "Native 512-to-1024 discovery transition is
 * still wrong": the audit's own required correction was "extract a
 * host-testable physical transaction state machine. Prove the cold-boot
 * sequence byte-for-byte and length-for-length: neutral/HELLO at 512,
 * HELLO_ACK at 512, then 1024 only after the ACK transaction completes."
 * A prior round's own single-flag version (`if (upgrade_pending) {
 * cell_size = 1024; ...}` applied starting the very next transaction)
 * upgraded exactly one transaction too early -- the very transaction that
 * carries the HELLO_ACK response itself, which the master (having only
 * just sent its own HELLO and seen no acknowledgement yet) necessarily
 * still clocks at the 512-byte discovery cadence. This pure, portable
 * state machine is that fix, extracted so the exact transaction-by-
 * transaction sequence can be proven on host, not only trusted by
 * inspection of target-only code (main/mtek_spi_runtime.c, which is not
 * itself host-testable -- see mtek_spi_native_discovery.c's own doc
 * comment on that boundary). */
typedef struct {
    int countdown; /* 0 = no upgrade pending; N = apply once N more transactions have started */
} mtk_native_cellsize_negotiator_t;

void mtk_native_cellsize_negotiator_init(mtk_native_cellsize_negotiator_t *n);

/* Call once per physical transaction, at the very top of the loop,
 * BEFORE computing/using the cell size for the transaction about to
 * start. `current_cell_size` is the size that was in effect for the
 * PREVIOUS transaction (512, the shared discovery size, before any HELLO
 * is recognized). Returns the cell size to actually use for the
 * transaction now starting -- unchanged unless the countdown armed by
 * mtk_native_cellsize_negotiator_hello_recognized below has just reached
 * zero. */
size_t mtk_native_cellsize_negotiator_tick(mtk_native_cellsize_negotiator_t *n, size_t current_cell_size, size_t upgraded_cell_size);

/* Call when a HELLO is recognized during the CURRENT transaction (the one
 * whose response, HELLO_ACK, will be staged and clocked out during the
 * NEXT transaction, still at the OLD cell size the master is still
 * using). Arms the upgrade for exactly two transactions from now: the
 * next call to the tick function above only decrements the countdown;
 * the call after that applies the upgrade. */
void mtk_native_cellsize_negotiator_hello_recognized(mtk_native_cellsize_negotiator_t *n);

#ifdef __cplusplus
}
#endif
