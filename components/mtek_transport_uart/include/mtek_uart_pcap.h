/* MonstaShark factory-UART compatibility endpoint.
 *
 * This is the ESP32 side of the shipped STM32 capture contract.  It keeps
 * the canonical capture service as the radio/capture owner and translates
 * that service to the version-1 COBS/CRC32C UART stream consumed by the M1.
 * No ESP-IDF type appears here, so parsing, framing, credit handling, and the
 * complete rate-switch lifecycle remain host-testable. */
#pragma once

#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MTEK_UART_PCAP_CONSOLE_BAUD 115200u
#define MTEK_UART_PCAP_CAPTURE_BAUD 3000000u
#define MTEK_UART_PCAP_SESSION_ID 0xCA07u
#define MTEK_UART_PCAP_PROTO_VERSION 1u
#define MTEK_UART_PCAP_MAX_PAYLOAD 1024u
#define MTEK_UART_PCAP_HDR_LEN 10u
#define MTEK_UART_PCAP_CRC_LEN 4u
#define MTEK_UART_PCAP_MAX_ENVELOPE \
    (MTEK_UART_PCAP_HDR_LEN + MTEK_UART_PCAP_MAX_PAYLOAD + MTEK_UART_PCAP_CRC_LEN)
#define MTEK_UART_PCAP_MAX_WIRE \
    (MTEK_UART_PCAP_MAX_ENVELOPE + (MTEK_UART_PCAP_MAX_ENVELOPE / 254u) + 2u)
#define MTEK_UART_PCAP_RECORD_HDR_LEN 24u

typedef enum {
    MTEK_UART_PCAP_OPEN = 1,
    MTEK_UART_PCAP_READY = 2,
    MTEK_UART_PCAP_START = 3,
    MTEK_UART_PCAP_FRAME_BATCH = 4,
    MTEK_UART_PCAP_STATS = 5,
    MTEK_UART_PCAP_MARK = 6,
    MTEK_UART_PCAP_CHANNEL_EVENT = 7,
    MTEK_UART_PCAP_CREDIT = 8,
    MTEK_UART_PCAP_STOP = 9,
    MTEK_UART_PCAP_STOPPED = 10,
    MTEK_UART_PCAP_ERROR = 11,
} mtek_uart_pcap_msg_type_t;

typedef struct {
    uint8_t version;
    uint8_t msg_type;
    uint16_t session_id;
    uint32_t sequence;
    const uint8_t *payload;
    uint16_t payload_len;
} mtek_uart_pcap_msg_t;

typedef enum {
    MTEK_UART_PCAP_DECODE_OK = 0,
    MTEK_UART_PCAP_DECODE_CRC,
    MTEK_UART_PCAP_DECODE_FORMAT,
    MTEK_UART_PCAP_DECODE_VERSION,
} mtek_uart_pcap_decode_result_t;

typedef enum {
    MTEK_UART_PCAP_RUN_OK = 0,
    MTEK_UART_PCAP_RUN_INVALID,
    MTEK_UART_PCAP_RUN_RADIO_BUSY,
    MTEK_UART_PCAP_RUN_START_FAILED,
    MTEK_UART_PCAP_RUN_IO_FAILED,
} mtek_uart_pcap_run_result_t;

typedef void (*mtek_uart_pcap_lock_fn)(void *ctx);

typedef struct {
    void *ctx;
    int (*write)(void *ctx, const uint8_t *data, size_t len);
    int (*wait_tx_done)(void *ctx, uint32_t timeout_ms);
    int (*set_baud)(void *ctx, uint32_t baud);
    int (*flush_input)(void *ctx);
    int (*read)(void *ctx, uint8_t *data, size_t cap, uint32_t timeout_ms);
    uint64_t (*now_ms)(void *ctx);
    void (*delay_ms)(void *ctx, uint32_t delay_ms);
    void (*set_binary_logging)(void *ctx, int binary_active);
} mtek_uart_pcap_io_t;

typedef struct {
    uint32_t boot_epoch;
    uint32_t next_correlation;
    uint32_t operation_token;
    uint32_t requested_duration_ms;
    uint8_t channel;
    uint8_t active;

    mtek_uart_pcap_lock_fn lock;
    mtek_uart_pcap_lock_fn unlock;
    void *lock_ctx;

    uint8_t start_response_seen;
    uint8_t start_status;
    uint8_t terminal_seen;
    uint8_t terminal_status;
    uint32_t terminal_frames;
    uint32_t terminal_drops;
    uint32_t terminal_truncated;

    uint32_t granted_bytes;
    uint32_t consumed_bytes;
    uint32_t grants;
    uint32_t starved;
    uint32_t wire_sequence;
    uint32_t batches;
    uint32_t wire_bytes;

    union {
        uint8_t rx_block[MTEK_UART_PCAP_MAX_WIRE];
        uint8_t payload[MTEK_UART_PCAP_MAX_PAYLOAD];
    } record_or_rx;
    size_t rx_len;
    uint8_t rx_overrun;
    union {
        uint8_t decode_buf[MTEK_UART_PCAP_MAX_ENVELOPE];
        uint8_t poll_body[1u + 25u + 1000u];
    } decode_or_poll;
    size_t poll_body_len;
    uint8_t wire[MTEK_UART_PCAP_MAX_WIRE];
    size_t pending_wire_len;
} mtek_uart_pcap_state_t;

void mtek_uart_pcap_init(mtek_uart_pcap_state_t *st, uint32_t boot_epoch);
void mtek_uart_pcap_set_lock(mtek_uart_pcap_state_t *st,
                             mtek_uart_pcap_lock_fn lock,
                             mtek_uart_pcap_lock_fn unlock,
                             void *lock_ctx);

/* Exact accepted grammar: PCAP_START <channel> <duration_ms>.
 * Channel is 1..14 and duration is 1..600000 ms. */
int mtek_uart_pcap_parse_start(const char *line, uint8_t *channel,
                               uint32_t *duration_ms);

size_t mtek_uart_pcap_frame_encode(uint8_t msg_type, uint16_t session_id,
                                   uint32_t sequence, const uint8_t *payload,
                                   uint16_t payload_len, uint8_t *dst,
                                   size_t dst_cap);
mtek_uart_pcap_decode_result_t mtek_uart_pcap_frame_decode(
    const uint8_t *cobs_block, size_t block_len, uint8_t *decode_buf,
    size_t decode_cap, mtek_uart_pcap_msg_t *out);

/* Runs one complete blocking capture session and always attempts to restore
 * the normal console transport before returning. */
mtek_uart_pcap_run_result_t mtek_uart_pcap_run(
    mtek_uart_pcap_state_t *st, uint8_t channel, uint32_t duration_ms,
    const mtek_uart_pcap_io_t *io);

#ifdef __cplusplus
}
#endif
