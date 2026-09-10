/* Clean-room implementation from MonstaTek contract. */
#include "mtek_transport_select.h"
#include "mtek_spi_native_frame.h"
#include "mtek_compat_frame.h"
#include <stddef.h>

mtk_transport_lock_t mtk_transport_try_recognize_discovery(const uint8_t *buf, size_t len) {
    mtk_spi_native_header_t nhdr;
    const uint8_t *npayload;
    /* "A valid native HELLO locks this profile" -- specifically the HELLO
     * class, not any well-formed native frame (SPI_PROTOCOL_V1.md). */
    if (mtk_spi_native_parse_bounded(buf, len, &nhdr, &npayload) == MTK_SPI_PARSE_OK &&
        nhdr.msg_class == MTK_SPI_CLASS_HELLO) {
        return MTK_TRANSPORT_NATIVE_SPI;
    }

    mtk_compat_header_t bhdr;
    const uint8_t *bpayload;
    /* "A valid Legacy SPI Compatibility frame locks that profile" -- any well-formed
     * frame (the pinned reference always sends a well-formed IDLE frame
     * even when idle, per 001-profile-bootstrap-feasibility.md Sec 1.2,
     * so IDLE itself is a legitimate, expected lock trigger). */
    if (mtk_compat_parse_bounded(buf, len, &bhdr, &bpayload) == MTK_COMPAT_PARSE_OK) {
        return MTK_TRANSPORT_COMPAT_SPI;
    }

    return MTK_TRANSPORT_AUTO;
}

static mtk_public_adapter_t s_claimed = MTK_PUBLIC_ADAPTER_NONE;
static mtk_transport_claim_lock_fn s_claim_lock, s_claim_unlock;

static void claim_lock(void) { if (s_claim_lock) s_claim_lock(); }
static void claim_unlock(void) { if (s_claim_unlock) s_claim_unlock(); }

void mtk_transport_claim_reset(void) {
    claim_lock();
    s_claimed = MTK_PUBLIC_ADAPTER_NONE;
    claim_unlock();
}

void mtk_transport_claim_set_lock(mtk_transport_claim_lock_fn lock, mtk_transport_claim_lock_fn unlock) {
    s_claim_lock = lock;
    s_claim_unlock = unlock;
}

int mtk_transport_claim_try(mtk_public_adapter_t which) {
    claim_lock();
    int won;
    if (s_claimed == MTK_PUBLIC_ADAPTER_NONE) {
        s_claimed = which;
        won = 1;
    } else {
        won = (s_claimed == which);
    }
    claim_unlock();
    return won;
}

mtk_public_adapter_t mtk_transport_claim_get(void) {
    claim_lock();
    mtk_public_adapter_t v = s_claimed;
    claim_unlock();
    return v;
}

void mtk_native_cellsize_negotiator_init(mtk_native_cellsize_negotiator_t *n) {
    n->countdown = 0;
}

size_t mtk_native_cellsize_negotiator_tick(mtk_native_cellsize_negotiator_t *n, size_t current_cell_size, size_t upgraded_cell_size) {
    if (n->countdown > 0) {
        n->countdown--;
        if (n->countdown == 0) {
            return upgraded_cell_size;
        }
    }
    return current_cell_size;
}

void mtk_native_cellsize_negotiator_hello_recognized(mtk_native_cellsize_negotiator_t *n) {
    n->countdown = 2;
}
