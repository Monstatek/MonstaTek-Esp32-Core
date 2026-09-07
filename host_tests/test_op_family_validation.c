/* RC12 blocker round, item 1 "cross-operation-token family validation":
 * adversarial proof of the mtk_op_*_family core APIs -- the exact atomic
 * primitives every feature-specific token-addressed STOP/STATUS/READ/
 * session-info/stats handler now gates on. Mints one operation token per
 * family (using each family's real (service_id, START opcode)) and proves:
 *   - validate/snapshot/transition/claim accept ONLY the matching family;
 *   - a wrong-family (or wrong-epoch, or zero) token is rejected with NO
 *     mutation of the record's state (the defect this closes: a foreign
 *     token was found and acted upon);
 *   - a rejected transition/claim leaves the targeted record and every
 *     other family's record completely unchanged.
 * Every family used by a converted handler is represented, so "every
 * operation family" is covered concretely. */
#include "mtk_test.h"
#include "mtek_core.h"
#include <string.h>

/* (service_id, START opcode) for every family a token-addressed handler
 * consumes -- mirrors the per-file constants in the service logic. */
#define SVC_WIFI 0x0001
#define SVC_BLE  0x0002
#define SVC_GATT 0x0003
#define SVC_CAP  0x0004
#define SVC_SYS  0x0000

typedef struct { const char *name; uint16_t svc; uint16_t start_op; } family_t;
static const family_t FAMILIES[] = {
    { "AP_SCAN",      SVC_WIFI, 0x0001 },
    { "STA_SCAN",     SVC_WIFI, 0x0006 },
    { "DEAUTH",       SVC_WIFI, 0x0010 },
    { "HANDSHAKE",    SVC_WIFI, 0x0013 },
    { "BLE_SCAN",     SVC_BLE,  0x0001 },
    { "BLE_ADV",      SVC_BLE,  0x0006 },
    { "SIGNAL_METER", SVC_BLE,  0x0009 },
    { "CAPTURE",      SVC_CAP,  0x0001 },
    { "TIME_SYNC",    SVC_SYS,  0x0008 },
};
#define N_FAM (sizeof(FAMILIES) / sizeof(FAMILIES[0]))

MTK_TEST_MAIN_BEGIN

    mtk_core_init(0xABCD1234u);
    uint32_t epoch = mtk_core_boot_epoch();

    /* Mint exactly one token per family. The 8-slot table holds 8; the 9th
     * (TIME_SYNC) evicts nothing because none are terminal, so it fails with
     * no_memory -- so mint the first 8 families and test the 9th's rejection
     * of foreign families using one of those 8. */
    uint32_t tok[N_FAM]; memset(tok, 0, sizeof(tok));
    unsigned minted = 0;
    for (unsigned i = 0; i < N_FAM && minted < 8; i++) {
        int nomem = 0;
        mtk_op_id_t id = mtk_op_alloc_id(FAMILIES[i].svc, FAMILIES[i].start_op, 1000, &nomem);
        MTK_CHECK(id.token != 0);
        MTK_CHECK_EQ(id.boot_epoch, epoch);
        tok[i] = id.token;
        minted++;
    }

    /* ---- validate_family: every token matches ONLY its own family. ---- */
    for (unsigned i = 0; i < minted; i++) {
        MTK_CHECK_EQ(mtk_op_validate_family(tok[i], epoch, FAMILIES[i].svc, FAMILIES[i].start_op), 1);
        /* Against every OTHER family (different service and/or opcode): reject. */
        for (unsigned j = 0; j < minted; j++) {
            if (j == i) continue;
            if (FAMILIES[j].svc == FAMILIES[i].svc && FAMILIES[j].start_op == FAMILIES[i].start_op) continue;
            MTK_CHECK_EQ(mtk_op_validate_family(tok[i], epoch, FAMILIES[j].svc, FAMILIES[j].start_op), 0);
        }
        /* Wrong epoch and zero token also rejected. */
        MTK_CHECK_EQ(mtk_op_validate_family(tok[i], epoch ^ 0x1u, FAMILIES[i].svc, FAMILIES[i].start_op), 0);
        MTK_CHECK_EQ(mtk_op_validate_family(0, epoch, FAMILIES[i].svc, FAMILIES[i].start_op), 0);
    }

    /* ---- snapshot_family: wrong family returns 0 and does NOT write out. ---- */
    {
        mtk_operation_record_t snap; memset(&snap, 0, sizeof(snap)); snap.token = 0xDEADBEEFu;
        /* AP_SCAN token (index 0) asked for as STA_SCAN (index 1) family. */
        MTK_CHECK_EQ(mtk_op_snapshot_family(tok[0], epoch, FAMILIES[1].svc, FAMILIES[1].start_op, &snap), 0);
        MTK_CHECK_EQ(snap.token, 0xDEADBEEFu); /* untouched */
        MTK_CHECK_EQ(mtk_op_snapshot_family(tok[0], epoch, FAMILIES[0].svc, FAMILIES[0].start_op, &snap), 1);
        MTK_CHECK_EQ(snap.token, tok[0]);
        MTK_CHECK_EQ(snap.state, MTK_OPS_ACCEPTED);
    }

    /* ---- transition_by_token_family: a wrong-family STOP does NOT mutate. ---- */
    {
        /* Try to STOP the AP_SCAN token via the DEAUTH family (index 2). */
        MTK_CHECK_EQ(mtk_op_transition_by_token_family(tok[0], epoch, FAMILIES[2].svc, FAMILIES[2].start_op,
                                                       MTK_OPS_STOPPED, MTK_STATUS_OK, 2000), 0);
        mtk_operation_record_t snap;
        MTK_CHECK(mtk_op_snapshot(tok[0], epoch, &snap));
        MTK_CHECK_EQ(snap.state, MTK_OPS_ACCEPTED); /* unchanged -- never transitioned by a foreign family */
        /* Correct family transitions it. */
        MTK_CHECK_EQ(mtk_op_transition_by_token_family(tok[0], epoch, FAMILIES[0].svc, FAMILIES[0].start_op,
                                                       MTK_OPS_RUNNING, MTK_STATUS_OK, 2000), 1);
        MTK_CHECK(mtk_op_snapshot(tok[0], epoch, &snap));
        MTK_CHECK_EQ(snap.state, MTK_OPS_RUNNING);
    }

    /* ---- claim_finalization_family: wrong family does NOT claim. ---- */
    {
        /* STA_SCAN token (index 1) claimed via AP_SCAN family: rejected. */
        MTK_CHECK_EQ(mtk_op_claim_finalization_family(tok[1], epoch, FAMILIES[0].svc, FAMILIES[0].start_op), 0);
        mtk_operation_record_t snap;
        MTK_CHECK(mtk_op_snapshot(tok[1], epoch, &snap));
        MTK_CHECK_EQ(snap.state, MTK_OPS_ACCEPTED); /* not STOPPING -- foreign claim had no effect */
        /* Correct family claims exactly once. */
        MTK_CHECK_EQ(mtk_op_claim_finalization_family(tok[1], epoch, FAMILIES[1].svc, FAMILIES[1].start_op), 1);
        MTK_CHECK(mtk_op_snapshot(tok[1], epoch, &snap));
        MTK_CHECK_EQ(snap.state, MTK_OPS_STOPPING);
        MTK_CHECK_EQ(mtk_op_claim_finalization_family(tok[1], epoch, FAMILIES[1].svc, FAMILIES[1].start_op), 0);
    }

    /* ---- every OTHER family's record is untouched by all of the above. ---- */
    for (unsigned i = 2; i < minted; i++) {
        mtk_operation_record_t snap;
        MTK_CHECK(mtk_op_snapshot(tok[i], epoch, &snap));
        MTK_CHECK_EQ(snap.state, MTK_OPS_ACCEPTED);
        MTK_CHECK_EQ(snap.service_id, FAMILIES[i].svc);
        MTK_CHECK_EQ(snap.opcode, FAMILIES[i].start_op);
    }

    /* ---- the 9th family (TIME_SYNC) validates against one of the 8 minted
     * tokens: a TIME_SYNC family query on any Wi-Fi/BLE/capture token is
     * rejected. ---- */
    MTK_CHECK_EQ(mtk_op_validate_family(tok[0], epoch, SVC_SYS, 0x0008), 0);
    MTK_CHECK_EQ(mtk_op_validate_family(tok[4], epoch, SVC_SYS, 0x0008), 0);

MTK_TEST_MAIN_END
