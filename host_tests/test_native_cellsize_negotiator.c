/* RC7 independent audit P0 "Native 512-to-1024 discovery transition is
 * still wrong": proves the exact transaction-by-transaction cold-boot
 * sequence the audit itself demanded -- "neutral/HELLO at 512, HELLO_ACK
 * at 512, then 1024 only after the ACK transaction completes." A prior
 * round's own single-flag version upgraded exactly one transaction too
 * early (the very transaction carrying the HELLO_ACK response itself);
 * this proves the fix transaction-by-transaction, not by inspection of
 * mtek_spi_runtime.c (which is target-only and not itself host-testable
 * -- see mtek_transport_select.h's own doc comment on that boundary). */
#include "mtk_test.h"
#include "mtek_transport_select.h"

#define DISCOVERY_SIZE 512u
#define NATIVE_SIZE 1024u

MTK_TEST_MAIN_BEGIN

    mtk_native_cellsize_negotiator_t n;
    mtk_native_cellsize_negotiator_init(&n);

    /* Transaction 0 (pre-HELLO, still AUTO): stays at 512, HELLO not yet
     * recognized. */
    size_t cell = DISCOVERY_SIZE;
    cell = mtk_native_cellsize_negotiator_tick(&n, cell, NATIVE_SIZE);
    MTK_CHECK_EQ(cell, DISCOVERY_SIZE);

    /* Transaction N: this transaction's rx buffer IS the HELLO -- the
     * caller recognizes it only AFTER this transaction's tick call (the
     * tick for N already happened above using the pre-HELLO state), so it
     * calls hello_recognized() now, arming the upgrade. */
    mtk_native_cellsize_negotiator_hello_recognized(&n);

    /* Transaction N+1: this is the transaction whose MISO carries the
     * real HELLO_ACK -- the master, having only just clocked out its
     * HELLO in transaction N and seen no acknowledgement yet, is still
     * driving the OLD (512) cadence. The negotiator must NOT upgrade yet
     * -- this is exactly the one-transaction-too-early defect the prior
     * round shipped. */
    cell = mtk_native_cellsize_negotiator_tick(&n, cell, NATIVE_SIZE);
    MTK_CHECK_EQ(cell, DISCOVERY_SIZE);

    /* Transaction N+2: only now, after the HELLO_ACK transaction (N+1)
     * has fully completed, does the master switch cadence -- the
     * negotiator must upgrade starting exactly this transaction. */
    cell = mtk_native_cellsize_negotiator_tick(&n, cell, NATIVE_SIZE);
    MTK_CHECK_EQ(cell, NATIVE_SIZE);

    /* Transaction N+3 and beyond: stays upgraded, no further countdown
     * effect, idempotent regardless of how many more ticks happen. */
    for (int i = 0; i < 5; i++) {
        cell = mtk_native_cellsize_negotiator_tick(&n, cell, NATIVE_SIZE);
        MTK_CHECK_EQ(cell, NATIVE_SIZE);
    }

    /* A fresh negotiator (new boot session) that never sees a HELLO stays
     * at the discovery size forever -- never upgrades spuriously. */
    mtk_native_cellsize_negotiator_t n2;
    mtk_native_cellsize_negotiator_init(&n2);
    size_t cell2 = DISCOVERY_SIZE;
    for (int i = 0; i < 10; i++) {
        cell2 = mtk_native_cellsize_negotiator_tick(&n2, cell2, NATIVE_SIZE);
        MTK_CHECK_EQ(cell2, DISCOVERY_SIZE);
    }

MTK_TEST_MAIN_END
