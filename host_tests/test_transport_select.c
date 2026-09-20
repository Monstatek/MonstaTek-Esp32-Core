/* AUTO transport selection (SPI_PROTOCOL_V1.md "Runtime transport selection"):
 * exactly one profile locks per boot, cross-profile rejection holds across an
 * exhaustive 2-byte magic-region sweep (mirroring corroboration sweep), and
 * noise/corruption never locks anything. */
#include "mtk_test.h"
#include "mtek_transport_select.h"
#include "mtek_spi_native_frame.h"
#include "mtek_compat_frame.h"
#include <string.h>

static void build_native_hello(uint8_t out[512]) {
    mtk_spi_native_header_t h = {0};
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = 1; h.minor = 0;
    h.msg_class = MTK_SPI_CLASS_HELLO; h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST;
    h.service = 0; h.opcode = 0; h.payload_len = 0; h.request_id = 1;
    h.packet_seq = 1; h.boot_epoch = 0x1111; h.message_len = 0;
    uint8_t cell[MTK_SPI_NATIVE_CELL_SIZE];
    mtk_spi_native_build_cell(&h, NULL, cell);
    memcpy(out, cell, 512); /* discovery transaction is 512 bytes */
}

static void build_compat_idle(uint8_t out[512]) {
    mtk_compat_header_t h = {0};
    h.magic = MTK_COMPAT_MAGIC; h.version = MTK_COMPAT_VERSION; h.msg_type = MTK_COMPAT_MSG_IDLE;
    mtk_compat_build_cell(&h, NULL, out);
}

MTK_TEST_MAIN_BEGIN

    uint8_t native_buf[512], compat_buf[512];
    build_native_hello(native_buf);
    build_compat_idle(compat_buf);

    MTK_CHECK_EQ(mtk_transport_try_recognize_discovery(native_buf, 512), MTK_TRANSPORT_NATIVE_SPI);
    MTK_CHECK_EQ(mtk_transport_try_recognize_discovery(compat_buf, 512), MTK_TRANSPORT_COMPAT_SPI);

    /* All-zero / all-0xFF neutral input locks nothing. */
    uint8_t zeros[512]; memset(zeros, 0, sizeof(zeros));
    MTK_CHECK_EQ(mtk_transport_try_recognize_discovery(zeros, 512), MTK_TRANSPORT_AUTO);
    uint8_t ones[512]; memset(ones, 0xFF, sizeof(ones));
    MTK_CHECK_EQ(mtk_transport_try_recognize_discovery(ones, 512), MTK_TRANSPORT_AUTO);

    /* Truncated buffers lock nothing. */
    MTK_CHECK_EQ(mtk_transport_try_recognize_discovery(native_buf, 20), MTK_TRANSPORT_AUTO);
    MTK_CHECK_EQ(mtk_transport_try_recognize_discovery(compat_buf, 5), MTK_TRANSPORT_AUTO);

    /* Exhaustive 2-byte magic-region sweep: holding each real frame fixed
     * except its first two bytes, no combination ever locks the *other*
     * profile (65,536 probes x 2 base frames). */
    unsigned false_accepts = 0;
    for (unsigned v = 0; v < 65536; v++) {
        uint8_t probe[512];
        memcpy(probe, native_buf, 512);
        probe[0] = (uint8_t)v; probe[1] = (uint8_t)(v >> 8);
        if (mtk_transport_try_recognize_discovery(probe, 512) == MTK_TRANSPORT_COMPAT_SPI) false_accepts++;

        memcpy(probe, compat_buf, 512);
        probe[0] = (uint8_t)v; probe[1] = (uint8_t)(v >> 8);
        if (mtk_transport_try_recognize_discovery(probe, 512) == MTK_TRANSPORT_NATIVE_SPI) false_accepts++;
    }
    MTK_CHECK_EQ(false_accepts, 0);

    /* A non-HELLO native class (e.g. IDLE) never locks native during
     * discovery -- only HELLO does, per spec. */
    mtk_spi_native_header_t h = {0};
    h.magic = MTK_SPI_NATIVE_MAGIC; h.major = 1; h.msg_class = MTK_SPI_CLASS_IDLE;
    h.flags = MTK_SPI_FLAG_FIRST | MTK_SPI_FLAG_LAST; h.boot_epoch = 1;
    uint8_t idle_cell[MTK_SPI_NATIVE_CELL_SIZE];
    mtk_spi_native_build_cell(&h, NULL, idle_cell);
    MTK_CHECK_EQ(mtk_transport_try_recognize_discovery(idle_cell, 512), MTK_TRANSPORT_AUTO);

MTK_TEST_MAIN_END
