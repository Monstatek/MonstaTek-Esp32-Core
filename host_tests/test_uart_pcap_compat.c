#include "mtk_test.h"
#include "mtk_test_bootstrap.h"
#include "mtek_uart_adapter.h"
#include "mtek_uart_pcap.h"

#include <string.h>

enum {
    EV_ACK = 1,
    EV_BAUD_CAPTURE,
    EV_READY,
    EV_BATCH,
    EV_STOPPED,
    EV_BAUD_CONSOLE,
    EV_DONE,
};

typedef struct {
    uint32_t baud;
    uint64_t now;
    uint8_t rx[256];
    size_t rx_len;
    unsigned events[32];
    uint64_t event_ms[32];
    unsigned event_count;
    unsigned ready_count;
    unsigned batch_count;
    unsigned stopped_count;
    unsigned logging_on;
    unsigned logging_off;
    uint32_t grant_amount;
    int stop_after_batch;
    unsigned write_calls;
    unsigned wait_calls;
    unsigned set_baud_calls;
    unsigned flush_calls;
    unsigned read_calls;
    unsigned fail_write_call;
    unsigned fail_wait_call;
    unsigned fail_set_baud_call;
    unsigned fail_flush_call;
    unsigned fail_read_call;
    uint8_t batch_payload[MTEK_UART_PCAP_MAX_PAYLOAD];
    uint16_t batch_payload_len;
    uint32_t batch_wire_bytes;
    char text[512];
    size_t text_len;
} fake_io_t;

static void note(fake_io_t *f, unsigned ev) {
    if (f->event_count < sizeof(f->events) / sizeof(f->events[0])) {
        f->events[f->event_count] = ev;
        f->event_ms[f->event_count] = f->now;
        f->event_count++;
    }
}

static uint64_t event_time(const fake_io_t *f, unsigned ev) {
    for (unsigned i = 0; i < f->event_count; i++)
        if (f->events[i] == ev) return f->event_ms[i];
    return UINT64_MAX;
}

static int contains_bytes(const uint8_t *data, size_t len, const char *needle) {
    size_t n = strlen(needle);
    if (n > len) return 0;
    for (size_t i = 0; i + n <= len; i++)
        if (memcmp(data + i, needle, n) == 0) return 1;
    return 0;
}

static int event_before(const fake_io_t *f, unsigned a, unsigned b) {
    int ai = -1, bi = -1;
    for (unsigned i = 0; i < f->event_count; i++) {
        if (f->events[i] == a && ai < 0) ai = (int)i;
        if (f->events[i] == b && bi < 0) bi = (int)i;
    }
    return ai >= 0 && bi >= 0 && ai < bi;
}

static void stage_control(fake_io_t *f, uint8_t type, const uint8_t *payload,
                          uint16_t payload_len) {
    uint8_t wire[MTEK_UART_PCAP_MAX_WIRE];
    size_t n = mtek_uart_pcap_frame_encode(type, MTEK_UART_PCAP_SESSION_ID,
                                            1, payload, payload_len, wire,
                                            sizeof(wire));
    MTK_CHECK(n != 0);
    MTK_CHECK(f->rx_len + n <= sizeof(f->rx));
    if (f->rx_len + n <= sizeof(f->rx)) {
        memcpy(f->rx + f->rx_len, wire, n);
        f->rx_len += n;
    }
}

static int fake_write(void *ctx, const uint8_t *data, size_t len) {
    fake_io_t *f = (fake_io_t *)ctx;
    f->write_calls++;
    if (f->fail_write_call == f->write_calls) return -1;
    if (f->baud == MTEK_UART_PCAP_CONSOLE_BAUD) {
        size_t room = sizeof(f->text) - f->text_len - 1u;
        size_t copy = len < room ? len : room;
        memcpy(f->text + f->text_len, data, copy);
        f->text_len += copy;
        f->text[f->text_len] = 0;
        if (contains_bytes(data, len, "PCAP ACK")) note(f, EV_ACK);
        if (contains_bytes(data, len, "PCAP DONE")) note(f, EV_DONE);
        return (int)len;
    }
    if (len < 2 || data[len - 1] != 0) return -1;
    uint8_t decoded[MTEK_UART_PCAP_MAX_ENVELOPE];
    mtek_uart_pcap_msg_t msg;
    if (mtek_uart_pcap_frame_decode(data, len - 1, decoded,
                                    sizeof(decoded), &msg) !=
        MTEK_UART_PCAP_DECODE_OK) return -1;
    MTK_CHECK_EQ(msg.session_id, MTEK_UART_PCAP_SESSION_ID);
    if (msg.msg_type == MTEK_UART_PCAP_READY) {
        note(f, EV_READY);
        f->ready_count++;
        if (f->ready_count == 1) {
            uint8_t amount[4] = {
                (uint8_t)f->grant_amount,
                (uint8_t)(f->grant_amount >> 8),
                (uint8_t)(f->grant_amount >> 16),
                (uint8_t)(f->grant_amount >> 24),
            };
            stage_control(f, MTEK_UART_PCAP_CREDIT, amount, sizeof(amount));
            mtk_fake_wifi_deliver_frames();
        }
    } else if (msg.msg_type == MTEK_UART_PCAP_FRAME_BATCH) {
        note(f, EV_BATCH);
        f->batch_count++;
        f->batch_wire_bytes += (uint32_t)len;
        f->batch_payload_len = msg.payload_len;
        memcpy(f->batch_payload, msg.payload, msg.payload_len);
        if (f->stop_after_batch)
            stage_control(f, MTEK_UART_PCAP_STOP, NULL, 0);
    } else if (msg.msg_type == MTEK_UART_PCAP_STOPPED) {
        note(f, EV_STOPPED);
        f->stopped_count++;
        MTK_CHECK_EQ(msg.payload_len, 28);
    }
    return (int)len;
}

static int fake_wait(void *ctx, uint32_t timeout_ms) {
    fake_io_t *f = (fake_io_t *)ctx;
    (void)timeout_ms;
    f->wait_calls++;
    if (f->fail_wait_call == f->wait_calls) return -1;
    return 0;
}

static int fake_set_baud(void *ctx, uint32_t baud) {
    fake_io_t *f = (fake_io_t *)ctx;
    f->set_baud_calls++;
    if (baud == MTEK_UART_PCAP_CAPTURE_BAUD) {
        note(f, EV_BAUD_CAPTURE);
    } else if (baud == MTEK_UART_PCAP_CONSOLE_BAUD) {
        note(f, EV_BAUD_CONSOLE);
    }
    if (f->fail_set_baud_call == f->set_baud_calls) return -1;
    f->baud = baud;
    return 0;
}

static int fake_flush(void *ctx) {
    fake_io_t *f = (fake_io_t *)ctx;
    f->flush_calls++;
    if (f->fail_flush_call == f->flush_calls) return -1;
    return 0;
}

static int fake_read(void *ctx, uint8_t *data, size_t cap,
                     uint32_t timeout_ms) {
    fake_io_t *f = (fake_io_t *)ctx;
    f->read_calls++;
    if (f->fail_read_call == f->read_calls) return -1;
    f->now += timeout_ms ? timeout_ms : 1u;
    size_t n = f->rx_len < cap ? f->rx_len : cap;
    memcpy(data, f->rx, n);
    memmove(f->rx, f->rx + n, f->rx_len - n);
    f->rx_len -= n;
    return (int)n;
}

static uint64_t fake_now(void *ctx) { return ((fake_io_t *)ctx)->now; }
static void fake_delay(void *ctx, uint32_t ms) { ((fake_io_t *)ctx)->now += ms; }
static void fake_logging(void *ctx, int active) {
    fake_io_t *f = (fake_io_t *)ctx;
    if (active) f->logging_on++;
    else f->logging_off++;
}

static mtek_uart_pcap_io_t make_io(fake_io_t *f) {
    mtek_uart_pcap_io_t io = {
        .ctx = f,
        .write = fake_write,
        .wait_tx_done = fake_wait,
        .set_baud = fake_set_baud,
        .flush_input = fake_flush,
        .read = fake_read,
        .now_ms = fake_now,
        .delay_ms = fake_delay,
        .set_binary_logging = fake_logging,
    };
    return io;
}

static void stage_one_frame(void) {
    mtk_fake_wifi_reset();
    g_fake_wifi.defer_frames = 1;
    g_fake_wifi.frame_count = 1;
    g_fake_wifi.frames[0].len = 24;
    g_fake_wifi.frames[0].channel = 6;
    g_fake_wifi.frames[0].rssi = -42;
    memset(g_fake_wifi.frames[0].data, 0, 24);
    g_fake_wifi.frames[0].data[0] = 0x80; /* beacon: management class */
    for (unsigned i = 1; i < 24; i++) g_fake_wifi.frames[0].data[i] = (uint8_t)i;
}

MTK_TEST_MAIN_BEGIN
    mtk_test_bootstrap();

    /* Exact console grammar and claim recognition. */
    uint8_t channel = 0;
    uint32_t duration = 0;
    MTK_CHECK(mtek_uart_pcap_parse_start("PCAP_START 6 15000", &channel,
                                         &duration));
    MTK_CHECK_EQ(channel, 6);
    MTK_CHECK_EQ(duration, 15000);
    MTK_CHECK(!mtek_uart_pcap_parse_start("PCAP_START 6", NULL, NULL));
    MTK_CHECK(!mtek_uart_pcap_parse_start("PCAP_START 0 15000", NULL, NULL));
    MTK_CHECK(!mtek_uart_pcap_parse_start("PCAP_START 6 15000 extra", NULL,
                                          NULL));
    mtk_uart_adapter_state_t uart;
    mtek_uart_adapter_init(&uart, MTK_TEST_BOOT_EPOCH);
    MTK_CHECK(mtek_uart_adapter_line_is_recognized(&uart,
                                                    "PCAP_START 6 15000"));

    /* Golden framing vector independently generated from the STM32's
     * m1_capture_transport implementation (LE header + CRC32C + COBS). */
    static const uint8_t golden_payload[] = {0, 1, 2, 0xff, 0, 0x88, 0x8e};
    static const uint8_t golden_wire[] = {
        0x0a,0x01,0x04,0xef,0xbe,0x04,0x03,0x02,0x01,0x07,0x01,0x04,
        0x01,0x02,0xff,0x07,0x88,0x8e,0x41,0x1f,0xa3,0x44,0x00
    };
    uint8_t wire[MTEK_UART_PCAP_MAX_WIRE];
    size_t wire_len = mtek_uart_pcap_frame_encode(
        MTEK_UART_PCAP_FRAME_BATCH, 0xbeef, 0x01020304, golden_payload,
        sizeof(golden_payload), wire, sizeof(wire));
    MTK_CHECK_EQ(wire_len, sizeof(golden_wire));
    MTK_CHECK(memcmp(wire, golden_wire, sizeof(golden_wire)) == 0);

    /* Full success path: ACK at 115200, switch to 3M, READY, credit-gated
     * exact record, STOP/STOPPED, restore 115200, then DONE. */
    stage_one_frame();
    fake_io_t f = { .baud = MTEK_UART_PCAP_CONSOLE_BAUD,
                    .grant_amount = 32768, .stop_after_batch = 1 };
    mtek_uart_pcap_io_t io = make_io(&f);
    mtek_uart_pcap_state_t pcap;
    mtek_uart_pcap_init(&pcap, MTK_TEST_BOOT_EPOCH);
    MTK_CHECK_EQ(mtek_uart_pcap_run(&pcap, 6, 15000, &io),
                 MTEK_UART_PCAP_RUN_OK);
    MTK_CHECK(strstr(f.text, "PCAP ACK") != NULL);
    MTK_CHECK(strstr(f.text, "PCAP DONE") != NULL);
    MTK_CHECK(event_before(&f, EV_ACK, EV_BAUD_CAPTURE));
    MTK_CHECK(event_before(&f, EV_BAUD_CAPTURE, EV_READY));
    MTK_CHECK(event_before(&f, EV_READY, EV_BATCH));
    MTK_CHECK(event_before(&f, EV_BATCH, EV_STOPPED));
    MTK_CHECK(event_before(&f, EV_STOPPED, EV_BAUD_CONSOLE));
    MTK_CHECK(event_before(&f, EV_BAUD_CONSOLE, EV_DONE));
    MTK_CHECK(event_time(&f, EV_READY) - event_time(&f, EV_BAUD_CAPTURE) <=
              1000);
    MTK_CHECK(event_time(&f, EV_STOPPED) - event_time(&f, EV_BATCH) <= 400);
    MTK_CHECK(event_time(&f, EV_DONE) - event_time(&f, EV_BAUD_CONSOLE) <=
              800);
    MTK_CHECK_EQ(f.ready_count, 1);
    MTK_CHECK_EQ(f.batch_count, 1);
    MTK_CHECK_EQ(f.stopped_count, 1);
    MTK_CHECK(f.batch_wire_bytes <= f.grant_amount);
    MTK_CHECK_EQ(f.batch_payload_len, 48);
    MTK_CHECK_EQ(f.batch_payload[0], 1);       /* record version */
    MTK_CHECK_EQ(f.batch_payload[1], 0);       /* management class */
    MTK_CHECK_EQ(f.batch_payload[2], 6);       /* primary channel */
    MTK_CHECK_EQ(f.batch_payload[3], 0);       /* no secondary channel */
    MTK_CHECK_EQ(f.batch_payload[4], (uint8_t)-42);
    MTK_CHECK_EQ(f.batch_payload[8], 24);      /* original length LE */
    MTK_CHECK_EQ(f.batch_payload[10], 24);     /* captured length LE */
    MTK_CHECK(memcmp(f.batch_payload + 24, g_fake_wifi.frames[0].data, 24) == 0);
    MTK_CHECK_EQ(g_fake_wifi.promisc_start_count, 1);
    MTK_CHECK_EQ(g_fake_wifi.promisc_stop_count, 1);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    MTK_CHECK_EQ(f.logging_on, 1);
    MTK_CHECK_EQ(f.logging_off, 1);

    /* A grant smaller than the encoded batch plus the 128-byte control
     * reserve must never permit a frame write. */
    stage_one_frame();
    memset(&f, 0, sizeof(f));
    f.baud = MTEK_UART_PCAP_CONSOLE_BAUD;
    f.grant_amount = 140;
    io = make_io(&f);
    mtek_uart_pcap_init(&pcap, MTK_TEST_BOOT_EPOCH);
    MTK_CHECK_EQ(mtek_uart_pcap_run(&pcap, 6, 25, &io),
                 MTEK_UART_PCAP_RUN_OK);
    MTK_CHECK_EQ(f.batch_count, 0);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);

    /* A BLE-owned radio is rejected explicitly at console speed. */
    MTK_CHECK_EQ(mtk_arbiter_acquire(MTK_ARB_BS, 0x1234), MTK_ARB_GRANT_OK);
    memset(&f, 0, sizeof(f));
    f.baud = MTEK_UART_PCAP_CONSOLE_BAUD;
    io = make_io(&f);
    mtek_uart_pcap_init(&pcap, MTK_TEST_BOOT_EPOCH);
    MTK_CHECK_EQ(mtek_uart_pcap_run(&pcap, 6, 1000, &io),
                 MTEK_UART_PCAP_RUN_RADIO_BUSY);
    MTK_CHECK(strstr(f.text, "Radio busy") != NULL);
    MTK_CHECK_EQ(f.event_count, 0);
    mtk_arbiter_release(MTK_ARB_BS);

    /* Failure after ownership is acquired still stops capture, restores
     * radio ownership, and attempts the normal console baud. */
    stage_one_frame();
    memset(&f, 0, sizeof(f));
    f.baud = MTEK_UART_PCAP_CONSOLE_BAUD;
    f.fail_set_baud_call = 1;
    io = make_io(&f);
    mtek_uart_pcap_init(&pcap, MTK_TEST_BOOT_EPOCH);
    MTK_CHECK_EQ(mtek_uart_pcap_run(&pcap, 6, 1000, &io),
                 MTEK_UART_PCAP_RUN_IO_FAILED);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    MTK_CHECK(event_before(&f, EV_BAUD_CAPTURE, EV_BAUD_CONSOLE));
    MTK_CHECK(strstr(f.text, "PCAP DONE") != NULL);

    /* Force each meaningful transport failure boundary after capture owns
     * the radio. Every case must stop canonical capture, release ownership,
     * and run the console-restoration cleanup sequence. */
    for (unsigned stage = 0; stage < 8; stage++) {
        stage_one_frame();
        memset(&f, 0, sizeof(f));
        f.baud = MTEK_UART_PCAP_CONSOLE_BAUD;
        f.grant_amount = 32768;
        f.stop_after_batch = 1;
        switch (stage) {
        case 0: f.fail_write_call = 1; break; /* ACK write */
        case 1: f.fail_wait_call = 1; break;  /* ACK drain */
        case 2: f.fail_flush_call = 1; break; /* capture-baud flush */
        case 3: f.fail_read_call = 1; break;  /* binary receive */
        case 4: f.fail_write_call = 2; break; /* READY */
        case 5: f.fail_write_call = 3; break; /* FRAME_BATCH */
        case 6: f.fail_wait_call = 2; break;  /* STOPPED drain */
        case 7: f.fail_flush_call = 2; break; /* console-baud flush */
        }
        io = make_io(&f);
        mtek_uart_pcap_init(&pcap, MTK_TEST_BOOT_EPOCH);
        MTK_CHECK_EQ(mtek_uart_pcap_run(&pcap, 6, 1000, &io),
                     MTEK_UART_PCAP_RUN_IO_FAILED);
        MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
        MTK_CHECK(f.set_baud_calls >= 1);
        MTK_CHECK_EQ(f.logging_on, 1);
        MTK_CHECK_EQ(f.logging_off, 1);
    }

    /* A failed physical console-baud restore cannot be hidden as success,
     * but all later cleanup is still attempted. */
    stage_one_frame();
    memset(&f, 0, sizeof(f));
    f.baud = MTEK_UART_PCAP_CONSOLE_BAUD;
    f.grant_amount = 32768;
    f.stop_after_batch = 1;
    f.fail_set_baud_call = 2;
    io = make_io(&f);
    mtek_uart_pcap_init(&pcap, MTK_TEST_BOOT_EPOCH);
    MTK_CHECK_EQ(mtek_uart_pcap_run(&pcap, 6, 1000, &io),
                 MTEK_UART_PCAP_RUN_IO_FAILED);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    MTK_CHECK_EQ(f.set_baud_calls, 2);
    MTK_CHECK_EQ(f.logging_off, 1);

    /* Canonical monitor-entry failure never changes baud and never leaks
     * the radio owner acquired before the failing HAL call. */
    stage_one_frame();
    g_fake_wifi.promisc_start_rc = -1;
    memset(&f, 0, sizeof(f));
    f.baud = MTEK_UART_PCAP_CONSOLE_BAUD;
    io = make_io(&f);
    mtek_uart_pcap_init(&pcap, MTK_TEST_BOOT_EPOCH);
    MTK_CHECK_EQ(mtek_uart_pcap_run(&pcap, 6, 1000, &io),
                 MTEK_UART_PCAP_RUN_START_FAILED);
    MTK_CHECK_EQ(mtk_arbiter_active_class(), MTK_ARB_NONE);
    MTK_CHECK_EQ(f.set_baud_calls, 0);
    MTK_CHECK(strstr(f.text, "PCAP start failed") != NULL);

MTK_TEST_MAIN_END
