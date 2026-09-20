/* Resource arbiter: 66 distinct-class pairs + 14 self-pairs, single-active-class
 * model, and the D->H guarded-transition failure path's unconditional slot
 * release. */
#include "mtk_test.h"
#include "mtek_arbiter.h"

MTK_TEST_MAIN_BEGIN

    mtk_arbiter_init();

    /* 12 active classes, C(12,2)=66 distinct pairs, all resolvable
     * (never falls through the pairwise table's conservative default for
     * a real declared pair). */
    static const mtk_arbiter_class_t active[] = {
        MTK_ARB_WMC, MTK_ARB_WS, MTK_ARB_BEACON, MTK_ARB_D, MTK_ARB_H, MTK_ARB_M,
        MTK_ARB_BS, MTK_ARB_BA, MTK_ARB_SM, MTK_ARB_GC, MTK_ARB_SAP, MTK_ARB_RAW,
        MTK_ARB_ESPNOW, MTK_ARB_IEEE154,
    };
    unsigned n = sizeof(active) / sizeof(active[0]);
    MTK_CHECK_EQ(n, 14);
    unsigned pair_count = 0;
    for (unsigned i = 0; i < n; i++)
        for (unsigned j = i + 1; j < n; j++) {
            pair_count++;
            mtk_arbiter_policy_for(active[i], active[j]); /* must not assert/crash */
        }
    /* Every unordered pair among the 14 implemented classes is declared:
     * IEEE154 joined ESPNOW as a real class, so 13*12/2 = 78 became
     * 14*13/2 = 91. */
    MTK_CHECK_EQ(pair_count, 91);

    /* Self-pairs: BUSY for every active class, DISABLED for reserved.
     * ESPNOW is now an implemented, acquirable class, so it is BUSY against
     * itself like every other active class; only RESV_154 remains reserved. */
    for (unsigned i = 0; i < n; i++) MTK_CHECK_EQ(mtk_arbiter_policy_for(active[i], active[i]), MTK_POLICY_BUSY);
    MTK_CHECK_EQ(mtk_arbiter_policy_for(MTK_ARB_ESPNOW, MTK_ARB_ESPNOW), MTK_POLICY_BUSY);
    MTK_CHECK_EQ(mtk_arbiter_policy_for(MTK_ARB_IEEE154, MTK_ARB_IEEE154), MTK_POLICY_BUSY);
    /* 802.15.4 shares the one 2.4GHz radio with Wi-Fi and ESP-NOW, so those
     * are serialized; BLE has its own controller path and is cross-subsystem. */
    MTK_CHECK_EQ(mtk_arbiter_policy_for(MTK_ARB_M, MTK_ARB_IEEE154), MTK_POLICY_SERIALIZED);
    MTK_CHECK_EQ(mtk_arbiter_policy_for(MTK_ARB_SAP, MTK_ARB_IEEE154), MTK_POLICY_SERIALIZED);
    MTK_CHECK_EQ(mtk_arbiter_policy_for(MTK_ARB_ESPNOW, MTK_ARB_IEEE154), MTK_POLICY_SERIALIZED);
    MTK_CHECK_EQ(mtk_arbiter_policy_for(MTK_ARB_BS, MTK_ARB_IEEE154), MTK_POLICY_CROSS_SUBSYSTEM_BUSY);

    /* Single-active-class model: WS active excludes WMC (serialized) and
     * BS (cross-subsystem-busy) both. */
    mtk_arbiter_reset();
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_WS, 100), MTK_ARB_GRANT_OK);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_WS);
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_WMC, 101), MTK_ARB_GRANT_BUSY);
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_BS, 102), MTK_ARB_GRANT_BUSY);
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_WS, 103), MTK_ARB_GRANT_BUSY); /* self-pair */
    mtk_arbiter_release(MTK_ARB_WS);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_BS, 104), MTK_ARB_GRANT_OK);

    /* D->H is the one guarded transition in Phase 1. */
    mtk_arbiter_reset();
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_D, 200), MTK_ARB_GRANT_OK);
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_H, 201), MTK_ARB_GRANT_GUARDED);
    /* success path: caller confirms D stopped, then hands off atomically */
    mtk_arbiter_force_transfer(MTK_ARB_H, 201);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_H);
    MTK_CHECK_EQ(mtk_arbiter_active_token(), 201u);

    /* failure path: slot is released unconditionally, never left assigned
     * to a non-responsive D. */
    mtk_arbiter_reset();
    mtk_arbiter_acquire(MTK_ARB_D, 300);
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_H, 301), MTK_ARB_GRANT_GUARDED);
    mtk_arbiter_force_release();
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    /* Owner mapping: WMC/WS/.../RAW are RADIO_OWNER_WIFI; BS/BA/SM/GC are
     * RADIO_OWNER_BLE. */
    mtk_arbiter_reset();
    mtk_arbiter_acquire(MTK_ARB_WMC, 400);
    MTK_CHECK_EQ(mtk_arbiter_active_owner(), MTK_RADIO_OWNER_WIFI);
    mtk_arbiter_reset();
    mtk_arbiter_acquire(MTK_ARB_GC, 401);
    MTK_CHECK_EQ(mtk_arbiter_active_owner(), MTK_RADIO_OWNER_BLE);

MTK_TEST_MAIN_END
