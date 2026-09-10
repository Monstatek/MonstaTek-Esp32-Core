/* RC8 independent audit P0-4 "Fix NimBLE timeout/late-callback ABA
 * hazards": proves mtk_ble_op_generation_t's own contract directly --
 * the exact scenario the audit's own required test describes ("operation
 * A times out, operation B starts immediately, then A's delayed callback
 * arrives; B must not be completed or corrupted"), portably, without
 * needing real NimBLE/FreeRTOS. See mtek_ble_hal_esp32.c for how this
 * primitive is actually wired into each of its seven static-context
 * callback sites (scan, connect, GATT discover/read/write, CCCD
 * discovery, characteristic-bound search) -- target-only integration,
 * not itself host-testable, matching this tree's established
 * portable-primitive/target-glue split (e.g. mtk_native_cellsize_
 * negotiator_t, mtk_spi_native_packet_seq_tracker_t). */
#include "mtk_test.h"
#include "mtk_ble_op_generation.h"

MTK_TEST_MAIN_BEGIN

    mtk_ble_op_generation_t g;
    mtk_ble_op_generation_init(&g);

    /* Nothing armed yet: even a zero/garbage candidate is never current. */
    MTK_CHECK_EQ(mtk_ble_op_generation_is_current(&g, 0), 0);
    MTK_CHECK_EQ(mtk_ble_op_generation_is_current(&g, 12345), 0);

    /* Operation A arms. Its own generation is current; nothing else is. */
    uint32_t gen_a = mtk_ble_op_generation_arm(&g);
    MTK_CHECK(gen_a != 0);
    MTK_CHECK_EQ(mtk_ble_op_generation_is_current(&g, gen_a), 1);

    /* A times out (the caller gives up waiting) WITHOUT clearing the
     * generation via a natural completion path -- exactly the real HAL's
     * own shape (it just stops waiting and tears down its own context;
     * it does not tell this primitive "A is done" separately, since arm()
     * itself is what invalidates A once B calls it). Operation B starts
     * immediately, re-arming the SAME underlying context storage in the
     * real HAL (simulated here by simply calling arm() again). */
    uint32_t gen_b = mtk_ble_op_generation_arm(&g);
    MTK_CHECK(gen_b != gen_a); /* never reissued */
    MTK_CHECK_EQ(mtk_ble_op_generation_is_current(&g, gen_b), 1);

    /* A's delayed callback finally arrives, carrying A's OWN generation
     * (NimBLE's own dispatch guarantees this -- the `arg` a callback
     * receives is whatever was registered for THAT procedure, unaffected
     * by what a later procedure registered). It must be rejected: B's
     * context must never be touched by A's stale callback. */
    MTK_CHECK_EQ(mtk_ble_op_generation_is_current(&g, gen_a), 0);
    /* B's own callback, arriving normally, is correctly accepted. */
    MTK_CHECK_EQ(mtk_ble_op_generation_is_current(&g, gen_b), 1);

    /* B completes normally and clears -- a THIRD, even-later stray
     * callback (e.g. a duplicate/retried NimBLE event) carrying either
     * old generation is rejected, and nothing is "current" until a new
     * operation arms again. */
    mtk_ble_op_generation_clear(&g);
    MTK_CHECK_EQ(mtk_ble_op_generation_is_current(&g, gen_a), 0);
    MTK_CHECK_EQ(mtk_ble_op_generation_is_current(&g, gen_b), 0);

    /* Many sequential arm/clear cycles: every generation is unique, and
     * only ever the most recently armed one is current. */
    uint32_t seen[64];
    for (int i = 0; i < 64; i++) {
        uint32_t g_i = mtk_ble_op_generation_arm(&g);
        MTK_CHECK(g_i != 0);
        for (int j = 0; j < i; j++) MTK_CHECK(seen[j] != g_i);
        seen[i] = g_i;
        MTK_CHECK_EQ(mtk_ble_op_generation_is_current(&g, g_i), 1);
        if (i > 0) MTK_CHECK_EQ(mtk_ble_op_generation_is_current(&g, seen[i - 1]), 0);
    }

MTK_TEST_MAIN_END
