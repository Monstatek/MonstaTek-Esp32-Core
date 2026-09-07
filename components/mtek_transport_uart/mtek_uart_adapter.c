/* Clean-room implementation from MonstaTek contract
 * (001-command-behavior-matrix.md Sec A/B/C). Portable: no ESP-IDF
 * dependency, host-testable.
 *
 * Scope note (see docs/PROVENANCE.md): this adapter wires the List A
 * release-blocking surface (Wi-Fi AP/station scan, select, deauth with
 * real cached-table targeting, BLE scan/advertise/signal/GATT) plus the
 * List B console verbs the matrix documents (beacon, handshake, stop).
 * `list -h` (handshake hex dump) is reduced to a summary line -- the
 * underlying canonical data is available (HANDSHAKE_READ) but the exact
 * field-by-field text layout was not fully specified in the accepted
 * contract package. BLE `list <id>`/`list -d` (full per-AD-type detail
 * block, RC8 P0-6) and `connect`/`services` (nested service/
 * characteristic/descriptor discovery, RC8 P0-6) are now implemented in
 * full against `001-command-behavior-matrix.md`'s own evidence -- see
 * `format_ble_device_detail` and `gatt_discover_tree` below for exactly
 * what is a confirmed field vs. a disclosed, reasonable engineering
 * choice.
 */
#include "mtek_uart_adapter.h"
#include "mtek_router.h"
#include "mtek_schema_structs.h"
#include "mtek_schema_message_descs.h"
#include "mtek_codec_api.h"
#include "mtek_ble_service.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#define UART_AP_MAX 50
#define UART_STA_MAX 32
#define UART_BLE_MAX 40

typedef struct {
    uint8_t status;
    uint8_t body[2048];
    size_t body_len;
    char event_name[40];
    uint8_t event_body[2048];
    size_t event_body_len;
    int have_event;
} uart_capture_t;

/* RC6 independent audit P0 "Target stack usage is catastrophically larger
 * than the configured stacks" (fixed) + RC7 independent audit P0 "The
 * release artifact starts the wrong transport for shipped M1
 * compatibility" (this file's OWN follow-on measurement): every command
 * handler below uses one or two `uart_capture_t` values (~4.1KB each) to
 * capture a synchronous mtk_router_dispatch call's response. RC6 made
 * each of those ~25 call sites' own local `static uart_capture_t
 * cap`/`cap2`/`ev` -- safe (this REPL is strictly single-threaded,
 * processing one command line to completion, including every nested
 * dispatch call, before the next is read -- no concurrent or re-entrant
 * caller ever observes a torn or unexpectedly-shared value), but a real
 * `idf.py size` measurement after RC7 made the UART REPL task start
 * unconditionally (previously it was `#if`'d out entirely whenever SPI
 * was the build-time "primary transport", so RC6's own measurement never
 * actually counted this) showed those ~25 independently-named statics
 * cost ~103KB of DIRAM by themselves -- on a chip already tight on SRAM
 * (docs/RESOURCE_BUDGET.md's own "RC6 measured SRAM conflict" section).
 * Since the same single-threaded-non-reentrant safety argument that
 * justified `static` in the first place applies equally to SHARING one
 * instance across every call site (nothing here is ever concurrent with
 * itself), this file now declares exactly THREE shared file-scope
 * instances below (matching the real maximum ever simultaneously live in
 * one call -- no handler needs more than two, e.g. handle_ble_signal's
 * `cap`+`ev`) and every handler below simply reuses them by NOT
 * redeclaring a local `cap`/`cap2`/`ev` of its own -- plain C scoping
 * means an undeclared identifier used inside a function resolves to the
 * file-scope one. This cuts the same real memory from ~103KB to ~12KB
 * (three instances instead of twenty-five) with no change in behavior:
 * each handler still fully owns and resets "its" capture(s) for the
 * duration of its own synchronous call, exactly as before. */
static uart_capture_t cap, cap2, ev;

static void cap_resp(void *user, uint32_t correlation, uint8_t status, const void *body, const mtk_struct_desc_t *desc) {
    (void)correlation;
    uart_capture_t *c = (uart_capture_t *)user;
    c->status = status;
    c->body_len = 0;
    if (body && desc) mtk_encode(desc, body, c->body, sizeof(c->body), &c->body_len);
}
static void cap_resp_raw(void *user, uint32_t correlation, uint8_t status, const uint8_t *body, size_t len) {
    (void)correlation;
    uart_capture_t *c = (uart_capture_t *)user;
    c->status = status;
    c->body_len = len > sizeof(c->body) ? sizeof(c->body) : len;
    if (body) memcpy(c->body, body, c->body_len);
}
static void cap_event(void *user, uint32_t correlation_or_zero, const char *name, const void *body, const mtk_struct_desc_t *desc) {
    (void)correlation_or_zero;
    uart_capture_t *c = (uart_capture_t *)user;
    strncpy(c->event_name, name, sizeof(c->event_name) - 1);
    c->event_body_len = 0;
    if (body && desc) mtk_encode(desc, body, c->event_body, sizeof(c->event_body), &c->event_body_len);
    c->have_event = 1;
}
static void cap_stream(void *user, uint32_t t, uint32_t s, const uint8_t *c, size_t l) { (void)user; (void)t; (void)s; (void)c; (void)l; }

static mtk_request_ctx_t make_ctx(mtk_uart_adapter_state_t *st, uart_capture_t *cap) {
    mtk_request_ctx_t ctx;
    ctx.profile = MTK_PROFILE_FACTORY_UART;
    ctx.correlation = ++st->next_correlation;
    ctx.boot_epoch = st->boot_epoch;
    ctx.session_generation = 0; /* factory UART has no peer-reboot concept of its own (mtek_core.h's own doc comment) -- never fenced */
    ctx.authorization_level = 0;
    ctx.sink.user = cap;
    ctx.sink.emit_response = cap_resp;
    ctx.sink.emit_response_raw = cap_resp_raw;
    ctx.sink.emit_event = cap_event;
    ctx.sink.emit_stream = cap_stream;
    return ctx;
}

/* ---- Persistent, queue-backed sink (RC5 independent audit P0) --------
 * Used only for the three commands whose canonical operation retains its
 * sink across the synchronous call for later background HAL-callback
 * delivery (see mtek_uart_adapter.h's session_queue doc comment). `user`
 * is `&st->session_queue` -- adapter-owned, boot-session-persistent --
 * never a stack-local object, so it stays valid for as long as the
 * canonical service's own session state (handshake_session_t, s_sig,
 * s_gatt) chooses to keep calling through it, exactly the same pattern
 * proven safe for native/Bedge SPI's own event_queue. */
static void qcap_resp(void *user, uint32_t correlation, uint8_t status, const void *body, const mtk_struct_desc_t *desc) {
    mtk_async_frame_t f; memset(&f, 0, sizeof(f));
    f.kind = MTK_ASYNC_FRAME_RESPONSE;
    f.correlation = correlation;
    f.seq_or_status = status;
    if (body && desc) mtk_encode(desc, body, f.body, sizeof(f.body), &f.body_len);
    mtk_async_queue_push((mtk_async_queue_t *)user, &f);
}
static void qcap_event(void *user, uint32_t correlation_or_zero, const char *name, const void *body, const mtk_struct_desc_t *desc) {
    mtk_async_frame_t f; memset(&f, 0, sizeof(f));
    f.kind = MTK_ASYNC_FRAME_EVENT;
    f.correlation = correlation_or_zero;
    if (name) { size_t n = strlen(name); if (n >= sizeof(f.event_name)) n = sizeof(f.event_name) - 1; memcpy(f.event_name, name, n); }
    if (body && desc) mtk_encode(desc, body, f.body, sizeof(f.body), &f.body_len);
    mtk_async_queue_push((mtk_async_queue_t *)user, &f);
}
static void qcap_stream(void *user, uint32_t session_token, uint32_t seq, const uint8_t *chunk, size_t len) {
    mtk_async_frame_t f; memset(&f, 0, sizeof(f));
    f.kind = MTK_ASYNC_FRAME_STREAM;
    f.correlation = session_token;
    f.seq_or_status = seq;
    f.body_len = len > sizeof(f.body) ? sizeof(f.body) : len;
    if (chunk) memcpy(f.body, chunk, f.body_len);
    mtk_async_queue_push((mtk_async_queue_t *)user, &f);
}

static mtk_request_ctx_t make_session_ctx(mtk_uart_adapter_state_t *st) {
    mtk_request_ctx_t ctx;
    ctx.profile = MTK_PROFILE_FACTORY_UART;
    ctx.correlation = ++st->next_correlation;
    ctx.boot_epoch = st->boot_epoch;
    ctx.session_generation = 0; /* factory UART has no peer-reboot concept of its own (mtek_core.h's own doc comment) -- never fenced */
    ctx.authorization_level = 0;
    ctx.sink.user = &st->session_queue;
    ctx.sink.emit_response = qcap_resp;
    ctx.sink.emit_response_raw = NULL; /* none of the three session-queue opcodes use the raw-response escape hatch */
    ctx.sink.emit_event = qcap_event;
    ctx.sink.emit_stream = qcap_stream;
    return ctx;
}

/* Drains st->session_queue immediately after a synchronous
 * make_session_ctx dispatch: since the router never defers a UART
 * dispatch (mtek_router.c), the ONLY frames that can possibly be in the
 * queue at this point are the ones the handler itself pushed during this
 * same synchronous call -- its one required response, and (only for
 * handlers whose canonical HAL call is itself blocking, e.g.
 * GATT_CONNECT's gatt_connect or the signal meter's first synchronous
 * sample) one same-batch terminal/update event. Any later, genuinely
 * asynchronous delivery (a mid-capture HANDSHAKE_EVENT, a later
 * SIGNAL_METER_UPDATE, a GATT_VALUE_EVENT notification) happens strictly
 * after this function has already returned and the queue is empty again
 * -- that traffic is what mtek_uart_adapter_poll_background is for, and
 * is never touched here. */
static uint8_t drain_session_response(mtk_uart_adapter_state_t *st, uart_capture_t *out_cap,
                                       const char *want_event_name, uart_capture_t *out_event) {
    mtk_async_frame_t f;
    uint8_t got_resp = 0;
    memset(out_cap, 0, sizeof(*out_cap));
    if (out_event) memset(out_event, 0, sizeof(*out_event));
    while (mtk_async_queue_pop(&st->session_queue, &f)) {
        if (f.kind == MTK_ASYNC_FRAME_RESPONSE && !got_resp) {
            out_cap->status = (uint8_t)f.seq_or_status;
            out_cap->body_len = f.body_len > sizeof(out_cap->body) ? sizeof(out_cap->body) : f.body_len;
            memcpy(out_cap->body, f.body, out_cap->body_len);
            got_resp = 1;
        } else if (f.kind == MTK_ASYNC_FRAME_EVENT && want_event_name && out_event &&
                   strcmp(f.event_name, want_event_name) == 0 && !out_event->have_event) {
            strncpy(out_event->event_name, f.event_name, sizeof(out_event->event_name) - 1);
            out_event->event_body_len = f.body_len > sizeof(out_event->event_body) ? sizeof(out_event->event_body) : f.body_len;
            memcpy(out_event->event_body, f.body, out_event->event_body_len);
            out_event->have_event = 1;
        }
    }
    return got_resp;
}

static size_t emit(char *out, size_t out_cap, size_t used, const char *fmt, ...) {
    if (used >= out_cap) return used;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(out + used, out_cap - used, fmt, ap);
    va_end(ap);
    if (n < 0) return used;
    return used + (size_t)n;
}

/* RC9 independent correction order P0 "strict List A UART parity is
 * still knowingly incomplete": the exact boot warning + full command
 * reference, reconstructed via clean-room OBSERVABLE INTERFACE
 * reimplementation from the accepted factory image (an external factory
 * compatibility artifact, `MtkEsp32-monstashark.bin`, held outside this
 * source tree) -- read-only `strings -a` / raw byte inspection of that
 * binary's own rodata (a behavioral-evidence
 * artifact, never opened as source, never copied into this tree or any
 * future git history), which stores each line below as its own separate
 * NUL-terminated string literal (confirmed by inspecting the raw bytes
 * around each string, not merely `strings`' line-per-string display --
 * i.e. this is genuinely one `puts`/`printf("%s\n", ...)` call per line
 * in the original firmware, not one giant multi-line literal with an
 * unconfirmed internal blank-line count). Every character, including
 * leading-space indentation and punctuation, is copied verbatim from
 * those bytes. Printed identically at boot (before the first `>> `
 * prompt) and by the `help` command -- both are observed, in the
 * accepted command-behavior matrix's own words, to print "before the
 * first prompt" / "on request", never independently confirmed to differ
 * in content between the two call sites. */
static const char *const MTK_HELP_BLOCK_LINES[] = {
    "Warning! This program is designed solely for educational and ethical security research purposes.",
    "Please familiarize yourself with local laws and always obtain appropriate permissions before conducting network tests.",
    "Available commands:",
    "  mode -w                   - Switch to WIFI mode",
    "  mode -b                   - Switch to BLE mode",
    "  mode                      - Show current mode",
    "  WIFI mode commands:",
    "    beacon \"s1\" \"s2\" ...      - Broadcast fake beacon frames for given SSIDs(Max.11)",
    "    scan -a                   - Scan for access points",
    "    list -a                   - List scanned access points",
    "    select -a <id>            - Select access point by ID",
    "    scan -s                   - Scan for stations on selected AP",
    "    list -s                   - List scanned stations",
    "    select -s <id>            - Select station by ID",
    "    select -l                 - Show selected AP and station",
    "    deauth [set <id,id,...>|all|broadcast] - Start deauth on selected target(s)",
    "    handshake                 - Start handshake capture on selected AP",
    "    list -h                   - List captured handshake packets",
    "  BLE mode commands:",
    "    scan [-a]                 - Active scan (aggressive, finds more devices)",
    "    scan -p                   - Passive scan (conservative, less power)",
    "    scan -t <sec>             - Set scan duration in seconds",
    "    scan -n <name>            - Filter devices by name substring",
    "    list                      - Show all scanned devices",
    "    list <id>                 - Show detailed ADV fields for scan result ID",
    "    list -d                   - Show detailed ADV fields for all scanned devices",
    "    signal <id>               - Signal Meter: live RSSI for a device (no connect)",
    "    signal stop | status      - Stop / query the Signal Meter",
    "    advertise                 - Start BLE advertising 'ESP32C6-M1-BLE'",
    "    advertise -n <name>       - Set advertising name and start",
    "    connect <id>              - Connect to scanned device; auto-discover GATT",
    "    services                  - List discovered services/chars (props)/descriptors",
    "    read <handle>             - Read a characteristic value",
    "    write <handle> <hex>      - Stage write WITH response (needs 'confirm')",
    "    writenr <handle> <hex>    - Stage write WITHOUT response (needs 'confirm')",
    "    confirm | cancel          - Send or discard the staged write",
    "    subscribe <handle>        - Enable notifications; indicate for indications",
    "    unsubscribe <handle>      - Disable notifications/indications",
    "    status                    - Connection + dropped-notification status",
    "    disconnect                - Disconnect the active BLE connection",
    "    stop                      - Stop BLE advertising/scan/connection",
    "  stop (or press Enter key) - Stop the running command",
    "  reboot                    - Rebooting system",
    "  help                      - Show this help message",
};
#define MTK_HELP_BLOCK_LINE_COUNT (sizeof(MTK_HELP_BLOCK_LINES) / sizeof(MTK_HELP_BLOCK_LINES[0]))
static size_t format_help_block(char *out, size_t out_cap) {
    size_t used = 0;
    for (unsigned i = 0; i < MTK_HELP_BLOCK_LINE_COUNT; i++) {
        used = emit(out, out_cap, used, "%s\n", MTK_HELP_BLOCK_LINES[i]);
    }
    return used;
}
/* Public (mtek_uart_adapter.h): the target REPL loop (app_main.c) calls
 * this exactly once, before writing the first `>> ` prompt. */
size_t mtek_uart_adapter_boot_banner(char *out, size_t out_cap) { return format_help_block(out, out_cap); }

static int starts_with(const char *s, const char *prefix) { return strncmp(s, prefix, strlen(prefix)) == 0; }
static void mac_to_str(const uint8_t b[6], char *out) {
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", b[0], b[1], b[2], b[3], b[4], b[5]);
}
static void hex_to_bytes(const char *hex, uint8_t *out, uint8_t *out_len, uint8_t max_len) {
    uint8_t n = 0;
    int hi = -1;
    for (const char *p = hex; *p && n < max_len; p++) {
        char c = *p;
        int v;
        if (c >= '0' && c <= '9') v = c - '0';
        else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
        else continue; /* spaces and other separators ignored */
        if (hi < 0) hi = v;
        else { out[n++] = (uint8_t)((hi << 4) | v); hi = -1; }
    }
    *out_len = n;
}
static void bytes_to_hex(const uint8_t *data, uint16_t len, char *out, size_t out_cap) {
    size_t o = 0;
    for (uint16_t i = 0; i < len && o + 2 < out_cap; i++) o += (size_t)snprintf(out + o, out_cap - o, "%02X", data[i]);
    out[o] = 0;
}

void mtek_uart_adapter_init(mtk_uart_adapter_state_t *st, uint32_t boot_epoch) {
    memset(st, 0, sizeof(*st));
    st->boot_epoch = boot_epoch;
    st->ap_selected = -1;
    st->sta_selected = -1;
    st->ble_selected = -1;
    mtk_async_queue_init(&st->session_queue);
}

/* ===================== Wi-Fi mode ===================================== */

static size_t handle_scan_a(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    size_t used = 0;
    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t ctx = make_ctx(st, &cap);
    mtk_ap_scan_start_req_t req; memset(&req, 0, sizeof(req));
    req.band = 2; req.channel_plan.mode = 1; req.channel_plan.band = 2;
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0001, 0x0001);
    uint8_t buf[64]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    used = emit(out, out_cap, used, "[*] Starting AP scan...\n");
    mtk_router_dispatch(&ctx, 0x0001, 0x0001, buf, blen);
    if (cap.status != MTK_STATUS_ACCEPTED || !cap.have_event) return emit(out, out_cap, used, "[!] Scan failed.\n");
    mtk_ap_scan_complete_ev_t done; memset(&done, 0, sizeof(done));
    mtk_decode(&mtk_ap_scan_complete_ev_t_desc, &done, cap.event_body, cap.event_body_len, NULL);

    memset(&cap2, 0, sizeof(cap2));
    mtk_request_ctx_t ctx2 = make_ctx(st, &cap2);
    mtk_ap_scan_results_page_req_t preq = {0};
    preq.result_generation = done.result_generation; preq.max_items = UART_AP_MAX;
    const mtk_opcode_entry_t *page_op = mtk_opcode_find(0x0001, 0x0003);
    uint8_t pbuf[16]; size_t plen = 0;
    mtk_encode(page_op->req_desc, &preq, pbuf, sizeof(pbuf), &plen);
    mtk_router_dispatch(&ctx2, 0x0001, 0x0003, pbuf, plen);
    mtk_ap_scan_results_page_resp_t page; memset(&page, 0, sizeof(page));
    if (cap2.status == MTK_STATUS_OK) mtk_decode(page_op->resp_desc, &page, cap2.body, cap2.body_len, NULL);

    st->ap_count = page.items.count > UART_AP_MAX ? UART_AP_MAX : (uint16_t)page.items.count;
    for (uint32_t i = 0; i < st->ap_count; i++) st->ap_table[i] = page.items.items[i];
    st->ap_scan_valid = 1;
    st->ap_selected = -1;

    static const char *auth_names[] = {"OPEN","WEP","WPA_PSK","WPA2_PSK","WPA_WPA2","WPA2_ENT","WPA3_PSK","WPA2_WPA3"};
    for (uint16_t i = 0; i < st->ap_count; i++) {
        mtk_aprecord_t *r = &st->ap_table[i];
        char ssid[33]; memcpy(ssid, r->ssid.data, r->ssid.len); ssid[r->ssid.len] = 0;
        char bssid[18]; mac_to_str(r->bssid.b, bssid);
        const char *auth = r->authmode < 8 ? auth_names[r->authmode] : "UNKNOWN";
        used = emit(out, out_cap, used, "[%02u] %-34s %-4d %-5d %-18s %-8s\n", i, ssid, r->channel, r->rssi, bssid, auth);
    }
    return emit(out, out_cap, used, "[+] Scan complete. %u AP(s) found.\n", (unsigned)st->ap_count);
}

static size_t handle_list_a(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    if (!st->ap_scan_valid) return emit(out, out_cap, 0, "[!] No AP scan results. Run 'scan -a' first.\n");
    size_t used = 0;
    static const char *auth_names[] = {"OPEN","WEP","WPA_PSK","WPA2_PSK","WPA_WPA2","WPA2_ENT","WPA3_PSK","WPA2_WPA3"};
    for (uint16_t i = 0; i < st->ap_count; i++) {
        mtk_aprecord_t *r = &st->ap_table[i];
        char ssid[33]; memcpy(ssid, r->ssid.data, r->ssid.len); ssid[r->ssid.len] = 0;
        char bssid[18]; mac_to_str(r->bssid.b, bssid);
        const char *auth = r->authmode < 8 ? auth_names[r->authmode] : "UNKNOWN";
        used = emit(out, out_cap, used, "[%02u] %-34s %-4d %-5d %-18s %-8s\n", i, ssid, r->channel, r->rssi, bssid, auth);
    }
    return used;
}

static size_t handle_select_a(mtk_uart_adapter_state_t *st, const char *arg, char *out, size_t out_cap) {
    if (!st->ap_scan_valid) return emit(out, out_cap, 0, "[!] Invalid AP ID\n");
    int id = -1; sscanf(arg, "%d", &id);
    if (id < 0 || (unsigned)id >= st->ap_count) return emit(out, out_cap, 0, "[!] Invalid AP ID\n");
    st->ap_selected = id;
    char ssid[33]; memcpy(ssid, st->ap_table[id].ssid.data, st->ap_table[id].ssid.len); ssid[st->ap_table[id].ssid.len] = 0;
    return emit(out, out_cap, 0, "[*] Selected AP %d: %s\n", id, ssid);
}

static size_t handle_scan_s(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    if (st->ap_selected < 0) return emit(out, out_cap, 0, "[!] Invalid AP ID\n");
    size_t used = 0;
    mtk_aprecord_t *ap = &st->ap_table[st->ap_selected];
    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t ctx = make_ctx(st, &cap);
    mtk_sta_scan_start_req_t req = {0};
    req.target_bssid = ap->bssid; req.channel = ap->channel; req.duration_ms = 5000;
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0001, 0x0006);
    uint8_t buf[16]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    used = emit(out, out_cap, used, "[*] Starting station scan...\n");
    mtk_router_dispatch(&ctx, 0x0001, 0x0006, buf, blen);
    if (cap.status != MTK_STATUS_ACCEPTED || !cap.have_event) return emit(out, out_cap, used, "[!] Station scan failed.\n");
    mtk_sta_scan_complete_ev_t done = {0};
    mtk_decode(&mtk_sta_scan_complete_ev_t_desc, &done, cap.event_body, cap.event_body_len, NULL);

    memset(&cap2, 0, sizeof(cap2));
    mtk_request_ctx_t ctx2 = make_ctx(st, &cap2);
    mtk_sta_scan_results_page_req_t preq = {0};
    preq.result_generation = done.result_generation; preq.max_items = UART_STA_MAX;
    const mtk_opcode_entry_t *page_op = mtk_opcode_find(0x0001, 0x0008);
    uint8_t pbuf[16]; size_t plen = 0;
    mtk_encode(page_op->req_desc, &preq, pbuf, sizeof(pbuf), &plen);
    mtk_router_dispatch(&ctx2, 0x0001, 0x0008, pbuf, plen);
    mtk_sta_scan_results_page_resp_t page = {0};
    if (cap2.status == MTK_STATUS_OK) mtk_decode(page_op->resp_desc, &page, cap2.body, cap2.body_len, NULL);

    st->sta_count = page.items.count > UART_STA_MAX ? UART_STA_MAX : (uint16_t)page.items.count;
    for (uint32_t i = 0; i < st->sta_count; i++) st->sta_table[i] = page.items.items[i];
    st->sta_scan_valid = 1;
    st->sta_scan_generation = done.result_generation;
    st->sta_selected = -1;

    for (uint16_t i = 0; i < st->sta_count; i++) {
        char mac[18]; mac_to_str(st->sta_table[i].mac.b, mac);
        used = emit(out, out_cap, used, "[%02u] %-18s %-5d\n", i, mac, st->sta_table[i].rssi);
    }
    return emit(out, out_cap, used, "[+] Station scan complete. %u station(s) found.\n", (unsigned)st->sta_count);
}

static size_t handle_list_s(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    if (!st->sta_scan_valid) return emit(out, out_cap, 0, "[!] No station scan results. Run 'scan -s' first.\n");
    size_t used = 0;
    for (uint16_t i = 0; i < st->sta_count; i++) {
        char mac[18]; mac_to_str(st->sta_table[i].mac.b, mac);
        used = emit(out, out_cap, used, "[%02u] %-18s %-5d\n", i, mac, st->sta_table[i].rssi);
    }
    return used;
}

static size_t handle_select_s(mtk_uart_adapter_state_t *st, const char *arg, char *out, size_t out_cap) {
    if (!st->sta_scan_valid) return emit(out, out_cap, 0, "[!] Invalid\n");
    int id = -1; sscanf(arg, "%d", &id);
    if (id < 0 || (unsigned)id >= st->sta_count) return emit(out, out_cap, 0, "[!] Invalid\n");
    st->sta_selected = id;
    char mac[18]; mac_to_str(st->sta_table[id].mac.b, mac);
    return emit(out, out_cap, 0, "[*] Selected station %d: %s\n", id, mac);
}

static size_t handle_select_l(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    size_t used = 0;
    if (st->ap_selected >= 0) {
        char ssid[33]; mtk_aprecord_t *ap = &st->ap_table[st->ap_selected];
        memcpy(ssid, ap->ssid.data, ap->ssid.len); ssid[ap->ssid.len] = 0;
        used = emit(out, out_cap, used, "[*] Selected AP: %d (%s)\n", st->ap_selected, ssid);
    } else {
        used = emit(out, out_cap, used, "[*] Selected AP: none\n");
    }
    if (st->sta_selected >= 0) {
        char mac[18]; mac_to_str(st->sta_table[st->sta_selected].mac.b, mac);
        used = emit(out, out_cap, used, "[*] Selected station: %d (%s)\n", st->sta_selected, mac);
    } else {
        used = emit(out, out_cap, used, "[*] Selected station: none\n");
    }
    return used;
}

/* Real cached-table-driven deauth targeting (not a precondition-only
 * stub): bare deauth uses the selected AP+station; `set` uses the
 * comma-listed station-table ids against the selected AP; `all` uses the
 * full retained station-scan snapshot (ALL_SCANNED); `broadcast` targets
 * the selected AP's broadcast address. Each dispatches the real canonical
 * DEAUTH_START, reaching the Wi-Fi HAL. */
static size_t handle_deauth(mtk_uart_adapter_state_t *st, const char *rest, char *out, size_t out_cap) {
    while (*rest == ' ') rest++;

    mtk_deauth_start_req_t req; memset(&req, 0, sizeof(req));

    if (*rest == 0) {
        if (st->ap_selected < 0 || st->sta_selected < 0)
            return emit(out, out_cap, 0, "[!] Invalid deauth target selection. Scan/select targets first.\n");
        req.target_mode = 0;
        req.ap_bssid = st->ap_table[st->ap_selected].bssid;
        req.channel = st->ap_table[st->ap_selected].channel;
        req.targets.count = 1;
        req.targets.items[0] = st->sta_table[st->sta_selected].mac;
    } else if (starts_with(rest, "set ")) {
        /* Syntax is validated before preconditions: a malformed id list is
         * always a Usage error, regardless of whether an AP/scan is
         * selected yet. */
        const char *p = rest + 4;
        int ok = (*p != 0);
        uint32_t ids[64]; unsigned id_count = 0;
        while (*p && ok) {
            char *endp; long id = strtol(p, &endp, 10);
            if (endp == p || id < 0 || id >= 64) { ok = 0; break; }
            if (id_count < 64) ids[id_count++] = (uint32_t)id;
            p = endp;
            if (*p == ',') p++;
            else if (*p != 0) { ok = 0; break; }
        }
        if (!ok || id_count == 0) return emit(out, out_cap, 0, "[!] Usage: deauth [set <id,id,...>|all|broadcast]\n");
        if (st->ap_selected < 0 || !st->sta_scan_valid)
            return emit(out, out_cap, 0, "[!] Invalid deauth target selection. Scan/select targets first.\n");
        req.target_mode = 0;
        req.ap_bssid = st->ap_table[st->ap_selected].bssid;
        req.channel = st->ap_table[st->ap_selected].channel;
        for (unsigned i = 0; i < id_count; i++) {
            if (ids[i] >= st->sta_count) return emit(out, out_cap, 0, "[!] Usage: deauth [set <id,id,...>|all|broadcast]\n");
            req.targets.items[req.targets.count++] = st->sta_table[ids[i]].mac;
        }
    } else if (strcmp(rest, "all") == 0) {
        if (!st->sta_scan_valid) return emit(out, out_cap, 0, "[!] Invalid deauth target selection. Scan/select targets first.\n");
        req.target_mode = 1;
        req.station_scan_token = st->sta_scan_generation;
    } else if (strcmp(rest, "broadcast") == 0) {
        if (st->ap_selected < 0) return emit(out, out_cap, 0, "[!] Invalid deauth target selection. Scan/select targets first.\n");
        req.target_mode = 2;
        req.ap_bssid = st->ap_table[st->ap_selected].bssid;
        req.channel = st->ap_table[st->ap_selected].channel;
    } else {
        return emit(out, out_cap, 0, "[!] Usage: deauth [set <id,id,...>|all|broadcast]\n");
    }

    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t ctx = make_ctx(st, &cap);
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0001, 0x0010);
    uint8_t buf[256]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0001, 0x0010, buf, blen);
    if (cap.status != MTK_STATUS_ACCEPTED) return emit(out, out_cap, 0, "[!] Invalid deauth target selection. Scan/select targets first.\n");
    st->deauth_running = 1;
    mtk_deauth_start_resp_t started = {0};
    mtk_decode(op->resp_desc, &started, cap.body, cap.body_len, NULL);
    st->deauth_token = started.operation_token;
    return emit(out, out_cap, 0, "[*] Deauth started.\n");
}

static size_t handle_beacon(mtk_uart_adapter_state_t *st, const char *rest, char *out, size_t out_cap) {
    mtk_beacon_start_req_t req; memset(&req, 0, sizeof(req));
    const char *p = rest;
    while (*p && req.ssids.count < 32) {
        while (*p == ' ') p++;
        if (*p != '"') break;
        p++;
        const char *start = p;
        while (*p && *p != '"') p++;
        size_t n = (size_t)(p - start);
        if (n > 32) n = 32;
        req.ssids.items[req.ssids.count].len = (uint16_t)n;
        memcpy(req.ssids.items[req.ssids.count].data, start, n);
        req.ssids.count++;
        if (*p == '"') p++;
    }
    if (req.ssids.count == 0) return emit(out, out_cap, 0, "[!] Usage: beacon \"ssid1\" \"ssid2\" ...\n");
    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t ctx = make_ctx(st, &cap);
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0001, 0x000D);
    uint8_t buf[1200]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0001, 0x000D, buf, blen);
    if (cap.status != MTK_STATUS_ACCEPTED) return emit(out, out_cap, 0, "[!] Beacon start failed.\n");
    st->beacon_running = 1;
    mtk_beacon_start_resp_t started = {0};
    mtk_decode(op->resp_desc, &started, cap.body, cap.body_len, NULL);
    st->beacon_token = started.operation_token;
    return emit(out, out_cap, 0, "[*] Beacon started.\n");
}

static size_t handle_handshake(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    if (st->ap_selected < 0) return emit(out, out_cap, 0, "[!] Invalid AP ID\n");
    mtk_handshake_start_req_t req = {0};
    req.target_bssid = st->ap_table[st->ap_selected].bssid;
    req.channel = st->ap_table[st->ap_selected].channel;
    req.deauth_count = 0;
    /* Persistent, queue-backed sink (RC5 independent audit P0): the real
     * target's promiscuous-mode callback delivers HANDSHAKE_EVENT frames
     * from the Wi-Fi driver's own task, well after this dispatch returns
     * -- mtek_wifi_logic.c's handshake_session_t retains a copy of
     * ctx.sink for that whole capture's lifetime, so it must point at
     * boot-session-persistent storage, never this function's own stack. */
    mtk_request_ctx_t ctx = make_session_ctx(st);
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0001, 0x0013);
    uint8_t buf[32]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0001, 0x0013, buf, blen);
    drain_session_response(st, &cap, NULL, NULL);
    if (cap.status != MTK_STATUS_ACCEPTED) return emit(out, out_cap, 0, "[!] Handshake capture failed to start.\n");
    mtk_handshake_start_resp_t started = {0};
    mtk_decode(op->resp_desc, &started, cap.body, cap.body_len, NULL);
    st->handshake_running = 1;
    st->handshake_token = started.operation_token;
    /* Progress events (HANDSHAKE_EVENT phase transitions, "[FOUND
     * EAPOL]"/"[CAPTURED]"/">>> [SUCCESS]"-equivalent lines) arrive later,
     * asynchronously, via mtek_uart_adapter_poll_background -- not
     * re-derived here from a single synchronously-available event, since
     * a real capture's whole point is that most of them arrive after this
     * call returns. HANDSHAKE_STATUS/HANDSHAKE_READ remain available for
     * polling the real outcome on demand regardless. */
    return emit(out, out_cap, 0, "[*] Handshake capture started.\n");
}

static size_t handle_stop(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    size_t used = emit(out, out_cap, 0, "[!] Stopping...\n");
    if (st->deauth_running) {
        mtk_deauth_stop_req_t req = {0}; req.operation_token = st->deauth_token;
        memset(&cap, 0, sizeof(cap));
        mtk_request_ctx_t ctx = make_ctx(st, &cap);
        const mtk_opcode_entry_t *op = mtk_opcode_find(0x0001, 0x0011);
        uint8_t buf[8]; size_t blen = 0;
        mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
        mtk_router_dispatch(&ctx, 0x0001, 0x0011, buf, blen);
        st->deauth_running = 0;
    }
    if (st->handshake_running) {
        mtk_handshake_stop_req_t req = {0}; req.operation_token = st->handshake_token;
        memset(&cap, 0, sizeof(cap));
        mtk_request_ctx_t ctx = make_ctx(st, &cap);
        const mtk_opcode_entry_t *op = mtk_opcode_find(0x0001, 0x0016);
        uint8_t buf[8]; size_t blen = 0;
        mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
        mtk_router_dispatch(&ctx, 0x0001, 0x0016, buf, blen);
        st->handshake_running = 0;
    }
    if (st->beacon_running) {
        mtk_beacon_stop_req_t req = {0}; req.operation_token = st->beacon_token;
        memset(&cap, 0, sizeof(cap));
        mtk_request_ctx_t ctx = make_ctx(st, &cap);
        const mtk_opcode_entry_t *op = mtk_opcode_find(0x0001, 0x000E);
        uint8_t buf[8]; size_t blen = 0;
        mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
        mtk_router_dispatch(&ctx, 0x0001, 0x000E, buf, blen);
        st->beacon_running = 0;
    }
    if (st->ble_adv_running) {
        mtk_ble_adv_stop_req_t req = {0}; req.operation_token = st->ble_adv_token;
        memset(&cap, 0, sizeof(cap));
        mtk_request_ctx_t ctx = make_ctx(st, &cap);
        const mtk_opcode_entry_t *op = mtk_opcode_find(0x0002, 0x0007);
        uint8_t buf[8]; size_t blen = 0;
        mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
        mtk_router_dispatch(&ctx, 0x0002, 0x0007, buf, blen);
        st->ble_adv_running = 0;
        used = emit(out, out_cap, used, "[+] BLE advertising stopped.\n");
    }
    if (st->ble_signal_running) {
        mtk_signal_meter_stop_req_t req = {0}; req.operation_token = st->ble_signal_token;
        memset(&cap, 0, sizeof(cap));
        mtk_request_ctx_t ctx = make_ctx(st, &cap);
        const mtk_opcode_entry_t *op = mtk_opcode_find(0x0002, 0x000B);
        uint8_t buf[8]; size_t blen = 0;
        mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
        mtk_router_dispatch(&ctx, 0x0002, 0x000B, buf, blen);
        st->ble_signal_running = 0;
    }
    return emit(out, out_cap, used, "Attack stopped\n");
}

/* ===================== BLE mode ======================================== */

/* RC8 independent audit P0-6 "Preserve exact shipped UART behavior": the
 * accepted baseline's own `scan` row -- "combinable flags -t <sec>,
 * -n <name>" -- alongside the existing -a/-p mode flag. A single
 * space-delimited token for -n's own name argument (no quoting grammar
 * is documented for this flag, unlike beacon's own quoted-SSID list). */
static uint32_t parse_scan_duration_ms(const char *rest) {
    const char *p = strstr(rest, "-t ");
    if (!p) return 5000;
    long sec = strtol(p + 3, NULL, 10);
    return sec > 0 ? (uint32_t)sec * 1000u : 5000u;
}
static const char *parse_scan_name_filter(const char *rest, char *buf, size_t buf_cap) {
    const char *p = strstr(rest, "-n ");
    if (!p) return NULL;
    p += 3;
    size_t n = 0;
    while (p[n] && p[n] != ' ' && n + 1 < buf_cap) { buf[n] = p[n]; n++; }
    buf[n] = 0;
    return n ? buf : NULL;
}

static size_t handle_ble_scan(mtk_uart_adapter_state_t *st, uint8_t passive, const char *rest, char *out, size_t out_cap) {
    uint32_t duration_ms = parse_scan_duration_ms(rest);
    char name_buf[33]; const char *name_filter = parse_scan_name_filter(rest, name_buf, sizeof(name_buf));
    size_t used = emit(out, out_cap, 0, "[*] Starting BLE scan: mode=%s, duration=%lums\n", passive ? "passive" : "active", (unsigned long)duration_ms);
    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t ctx = make_ctx(st, &cap);
    mtk_ble_scan_start_req_t req = {0}; req.mode = passive; req.duration_ms = duration_ms;
    if (name_filter) { size_t nl = strlen(name_filter); if (nl > 32) nl = 32; req.name_filter.len = (uint16_t)nl; memcpy(req.name_filter.data, name_filter, nl); }
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0002, 0x0001);
    uint8_t buf[64]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0002, 0x0001, buf, blen);
    if (cap.status != MTK_STATUS_ACCEPTED || !cap.have_event) return emit(out, out_cap, used, "[!] BLE scan failed.\n");
    mtk_ble_scan_complete_ev_t done = {0};
    mtk_decode(&mtk_ble_scan_complete_ev_t_desc, &done, cap.event_body, cap.event_body_len, NULL);

    memset(&cap2, 0, sizeof(cap2));
    mtk_request_ctx_t ctx2 = make_ctx(st, &cap2);
    mtk_ble_scan_results_page_req_t preq = {0}; preq.result_generation = done.result_generation; preq.max_items = UART_BLE_MAX;
    const mtk_opcode_entry_t *page_op = mtk_opcode_find(0x0002, 0x0004);
    uint8_t pbuf[16]; size_t plen = 0;
    mtk_encode(page_op->req_desc, &preq, pbuf, sizeof(pbuf), &plen);
    mtk_router_dispatch(&ctx2, 0x0002, 0x0004, pbuf, plen);
    mtk_ble_scan_results_page_resp_t page = {0};
    if (cap2.status == MTK_STATUS_OK) mtk_decode(page_op->resp_desc, &page, cap2.body, cap2.body_len, NULL);

    st->ble_count = page.items.count > UART_BLE_MAX ? UART_BLE_MAX : (uint16_t)page.items.count;
    for (uint32_t i = 0; i < st->ble_count; i++) {
        st->ble_addr[i] = page.items.items[i].addr;
        st->ble_addr_type[i] = page.items.items[i].addr_type;
        st->ble_rssi[i] = page.items.items[i].rssi;
        st->ble_name_len[i] = (uint8_t)page.items.items[i].name.len;
        memcpy(st->ble_name[i], page.items.items[i].name.data, page.items.items[i].name.len);
    }
    st->ble_generation = done.result_generation;
    st->ble_scan_valid = 1;
    st->ble_selected = -1;
    for (uint16_t i = 0; i < st->ble_count; i++) {
        char mac[18]; mac_to_str(st->ble_addr[i].b, mac);
        char name[33]; memcpy(name, st->ble_name[i], st->ble_name_len[i]); name[st->ble_name_len[i]] = 0;
        used = emit(out, out_cap, used, "[BLE] %s RSSI=%d type=0 name=%s len=%u\n", mac, st->ble_rssi[i], name, st->ble_name_len[i]);
    }
    return emit(out, out_cap, used, "[+] BLE scan complete. %u device(s) found.\n", (unsigned)st->ble_count);
}

static size_t handle_ble_list(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    if (!st->ble_scan_valid) return emit(out, out_cap, 0, "[!] No BLE scan results. Run 'scan' first.\n");
    size_t used = 0;
    for (uint16_t i = 0; i < st->ble_count; i++) {
        char mac[18]; mac_to_str(st->ble_addr[i].b, mac);
        char name[33]; memcpy(name, st->ble_name[i], st->ble_name_len[i]); name[st->ble_name_len[i]] = 0;
        used = emit(out, out_cap, used, "[%02u] %s RSSI=%d NAME=%s\n", i, mac, st->ble_rssi[i], name);
    }
    return used;
}

/* RC8 independent audit P0-6 "Preserve exact shipped UART behavior",
 * evidence `001-command-behavior-matrix.md` line 68: "detail block: RSSI,
 * Flags, Shortened/Complete Local Name, Service UUID16/128, TX Power,
 * Service Data UUID16, Manufacturer Data, unknown-AD-type fallback,
 * `Raw ADV data :`, then additive `Raw SCAN_RSP data :`". The canonical
 * `mtk_ble_device_details_resp_t` schema (accepted Task 002 contract)
 * has no dedicated Service UUID16/128 or Service Data UUID16 fields --
 * but this is NOT an unfixable data-model gap: the response already
 * carries the complete raw AD byte streams (`raw_adv`/`raw_scan_rsp`),
 * which is exactly where a real console sources these fields too. This
 * UART layer therefore parses the standard length-prefixed BT AD
 * structures (Core Spec Vol 3 Part C Sec 11) directly out of both
 * buffers itself, closing the gap without any canonical-schema change.
 * Flags/Name-text/TxPower/Manufacturer-Data are NOT re-derived here --
 * they stay sourced from the HAL's own already-parsed schema fields (one
 * value, one source of truth); the AD walk below is used only for (a)
 * the Shortened-vs-Complete Local Name distinction (AD type 0x08 vs
 * 0x09, a bit the schema does not carry), (b) Service UUID16/128 lists
 * (0x02/0x03, 0x06/0x07), (c) Service Data 16-bit UUID (0x16), and (d)
 * the unknown-AD-type fallback for every other type byte encountered.
 * ADV and SCAN_RSP are walked and merged (duplicate UUIDs suppressed) --
 * a disclosed, reasonable choice for what "the detail block" aggregates
 * across both PDUs, not an independently confirmed byte-exact match.
 * The exact per-field label text below is likewise this session's own
 * reasonable choice, not confirmed against `m1_console.c` itself (out of
 * reach under this task's clean-room constraint) -- matches this tree's
 * established "disclosed engineering choice" pattern elsewhere (e.g.
 * HELLO_ACK's empty payload, MTK_SIGNAL_METER_MISS_TOLERANCE's value). */
#define BLE_AD_UUID16_MAX 8
#define BLE_AD_UUID128_MAX 4
typedef struct {
    uint8_t uuid16_count; uint16_t uuid16[BLE_AD_UUID16_MAX];
    uint8_t uuid128_count; uint8_t uuid128[BLE_AD_UUID128_MAX][16];
    uint8_t have_svc_data16; uint16_t svc_data16_uuid;
    uint8_t svc_data16[27]; uint8_t svc_data16_len;
    uint8_t name_seen_shortened, name_seen_complete;
    char unknown_lines[256]; size_t unknown_used;
} ble_ad_info_t;
static void ble_ad_add_uuid16(ble_ad_info_t *info, uint16_t u) {
    for (uint8_t i = 0; i < info->uuid16_count; i++) if (info->uuid16[i] == u) return;
    if (info->uuid16_count < BLE_AD_UUID16_MAX) info->uuid16[info->uuid16_count++] = u;
}
static void ble_ad_add_uuid128(ble_ad_info_t *info, const uint8_t *u) {
    for (uint8_t i = 0; i < info->uuid128_count; i++) if (memcmp(info->uuid128[i], u, 16) == 0) return;
    if (info->uuid128_count < BLE_AD_UUID128_MAX) memcpy(info->uuid128[info->uuid128_count++], u, 16);
}
static void ble_ad_walk(ble_ad_info_t *info, const uint8_t *buf, uint8_t len) {
    uint8_t i = 0;
    while ((uint16_t)(i + 1) < (uint16_t)len) {
        uint8_t seg_len = buf[i];
        if (seg_len == 0) break;
        if ((uint16_t)(i + 1 + seg_len) > len) break; /* truncated -- stop, do not read past the buffer */
        uint8_t type = buf[i + 1];
        const uint8_t *data = &buf[i + 2];
        uint8_t data_len = (uint8_t)(seg_len - 1);
        switch (type) {
            case 0x01: case 0x0A: case 0xFF:
                break; /* Flags/TxPower/MfgData: already sourced from the schema's own fields */
            case 0x08: info->name_seen_shortened = 1; break;
            case 0x09: info->name_seen_complete = 1; break;
            case 0x02: case 0x03:
                for (uint8_t k = 0; (uint16_t)(k + 2) <= data_len; k = (uint8_t)(k + 2)) {
                    ble_ad_add_uuid16(info, (uint16_t)data[k] | ((uint16_t)data[k + 1] << 8));
                }
                break;
            case 0x06: case 0x07:
                for (uint8_t k = 0; (uint16_t)(k + 16) <= data_len; k = (uint8_t)(k + 16)) {
                    ble_ad_add_uuid128(info, &data[k]);
                }
                break;
            case 0x16:
                if (data_len >= 2 && !info->have_svc_data16) {
                    info->have_svc_data16 = 1;
                    info->svc_data16_uuid = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
                    uint8_t n = (uint8_t)(data_len - 2);
                    if (n > sizeof(info->svc_data16)) n = sizeof(info->svc_data16);
                    memcpy(info->svc_data16, data + 2, n);
                    info->svc_data16_len = n;
                }
                break;
            default: {
                char hex[64]; bytes_to_hex(data, data_len, hex, sizeof(hex));
                info->unknown_used = emit(info->unknown_lines, sizeof(info->unknown_lines), info->unknown_used,
                                           "  Unknown AD type 0x%02X : %s\n", type, hex);
                break;
            }
        }
        i = (uint8_t)(i + 1 + seg_len);
    }
}
static void format_uuid128_str(const uint8_t u[16], char *out /* >=37 bytes */) {
    /* AD structures store 128-bit UUIDs little-endian; the standard textual
     * form is big-endian, so the byte order is reversed here. */
    snprintf(out, 37, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
             u[15], u[14], u[13], u[12], u[11], u[10], u[9], u[8],
             u[7], u[6], u[5], u[4], u[3], u[2], u[1], u[0]);
}
static size_t format_ble_device_detail(mtk_uart_adapter_state_t *st, uint16_t id, char *out, size_t out_cap, size_t used) {
    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t ctx = make_ctx(st, &cap);
    mtk_ble_device_details_req_t req = {0}; req.result_generation = st->ble_generation; req.index = (uint8_t)id;
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0002, 0x0005);
    uint8_t buf[8]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0002, 0x0005, buf, blen);
    if (cap.status != MTK_STATUS_OK) return emit(out, out_cap, used, "[!] Invalid BLE ID: %u\n", id);
    mtk_ble_device_details_resp_t d = {0};
    mtk_decode(op->resp_desc, &d, cap.body, cap.body_len, NULL);
    char mac[18]; mac_to_str(d.addr.b, mac);
    char name[33]; memcpy(name, d.name.data, d.name.len); name[d.name.len] = 0;

    ble_ad_info_t info; memset(&info, 0, sizeof(info));
    ble_ad_walk(&info, d.raw_adv.data, (uint8_t)d.raw_adv.len);
    ble_ad_walk(&info, d.raw_scan_rsp.data, (uint8_t)d.raw_scan_rsp.len);

    used = emit(out, out_cap, used, "[%02u] %s\n", id, mac);
    used = emit(out, out_cap, used, "  RSSI=%d\n", d.rssi);
    used = emit(out, out_cap, used, "  Flags=0x%02X\n", d.flags);
    if (d.name.len) {
        const char *label = info.name_seen_complete ? "Complete Local Name" :
                             info.name_seen_shortened ? "Shortened Local Name" : "Local Name";
        used = emit(out, out_cap, used, "  %s : %s\n", label, name);
    }
    if (info.uuid16_count) {
        used = emit(out, out_cap, used, "  Service UUID16 :");
        for (uint8_t i = 0; i < info.uuid16_count; i++) used = emit(out, out_cap, used, " 0x%04X", info.uuid16[i]);
        used = emit(out, out_cap, used, "\n");
    }
    for (uint8_t i = 0; i < info.uuid128_count; i++) {
        char us[37]; format_uuid128_str(info.uuid128[i], us);
        used = emit(out, out_cap, used, "  Service UUID128 : %s\n", us);
    }
    used = emit(out, out_cap, used, "  TxPower=%d\n", d.tx_power);
    if (info.have_svc_data16) {
        char hex[64]; bytes_to_hex(info.svc_data16, info.svc_data16_len, hex, sizeof(hex));
        used = emit(out, out_cap, used, "  Service Data (UUID16 0x%04X) : %s\n", info.svc_data16_uuid, hex);
    }
    if (d.mfg_data.len) {
        char hex[64]; bytes_to_hex(d.mfg_data.data, d.mfg_data.len, hex, sizeof(hex));
        used = emit(out, out_cap, used, "  Manufacturer Data : %s\n", hex);
    }
    if (info.unknown_used) used = emit(out, out_cap, used, "%s", info.unknown_lines);
    char raw[64]; bytes_to_hex(d.raw_adv.data, d.raw_adv.len, raw, sizeof(raw));
    used = emit(out, out_cap, used, "  Raw ADV data : %s\n", raw);
    if (d.raw_scan_rsp.len) {
        char rawrsp[64]; bytes_to_hex(d.raw_scan_rsp.data, d.raw_scan_rsp.len, rawrsp, sizeof(rawrsp));
        used = emit(out, out_cap, used, "  Raw SCAN_RSP data : %s\n", rawrsp);
    }
    return used;
}
static size_t handle_ble_list_detail(mtk_uart_adapter_state_t *st, const char *arg, char *out, size_t out_cap) {
    if (!st->ble_scan_valid) return emit(out, out_cap, 0, "[!] No BLE scan results. Run 'scan' first.\n");
    char *end = NULL;
    long id = strtol(arg, &end, 10);
    if (end == arg || id < 0) return emit(out, out_cap, 0, "[!] Invalid BLE ID format: %s\n", arg);
    if ((uint16_t)id >= st->ble_count) return emit(out, out_cap, 0, "[!] Invalid BLE ID: %ld\n", id);
    return format_ble_device_detail(st, (uint16_t)id, out, out_cap, 0);
}
static size_t handle_ble_list_all_detail(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    if (!st->ble_scan_valid) return emit(out, out_cap, 0, "[!] No BLE scan results. Run 'scan' first.\n");
    size_t used = 0;
    for (uint16_t i = 0; i < st->ble_count; i++) used = format_ble_device_detail(st, i, out, out_cap, used);
    return used;
}

static size_t handle_ble_advertise(mtk_uart_adapter_state_t *st, const char *name, char *out, size_t out_cap) {
    if (st->ble_adv_running) return emit(out, out_cap, 0, "[*] BLE advertising is already running. Use 'stop' first.\n");
    mtk_ble_adv_start_req_t req = {0};
    const char *n = (name && *name) ? name : "ESP32C6-M1-BLE";
    size_t nlen = strlen(n); if (nlen > 31) nlen = 31;
    req.name.len = (uint16_t)nlen; memcpy(req.name.data, n, nlen);
    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t ctx = make_ctx(st, &cap);
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0002, 0x0006);
    uint8_t buf[64]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0002, 0x0006, buf, blen);
    if (cap.status != MTK_STATUS_ACCEPTED) return emit(out, out_cap, 0, "[!] BLE advertising failed to start.\n");
    mtk_ble_adv_start_resp_t started = {0};
    mtk_decode(op->resp_desc, &started, cap.body, cap.body_len, NULL);
    st->ble_adv_running = 1; st->ble_adv_token = started.operation_token;
    return emit(out, out_cap, 0, "[+] BLE advertising started. name=%s\n", n);
}

static size_t handle_ble_signal(mtk_uart_adapter_state_t *st, const char *arg, char *out, size_t out_cap) {
    int id = -1; sscanf(arg, "%d", &id);
    if (!st->ble_scan_valid || id < 0 || (unsigned)id >= st->ble_count) return emit(out, out_cap, 0, "[!] Invalid BLE ID: %s\n", arg);
    mtk_signal_meter_start_req_t req = {0};
    req.target.addr = st->ble_addr[id]; req.target.addr_type = st->ble_addr_type[id];
    /* Persistent, queue-backed sink (RC5 independent audit P0): every
     * SIGNAL_METER_UPDATE/LOST after the first sample arrives from the
     * periodic mtek_ble_signal_meter_tick() background task, not this
     * call. */
    mtk_request_ctx_t ctx = make_session_ctx(st);
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0002, 0x0009);
    uint8_t buf[16]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0002, 0x0009, buf, blen);
    drain_session_response(st, &cap, "SIGNAL_METER_UPDATE", &ev);
    if (cap.status != MTK_STATUS_ACCEPTED) return emit(out, out_cap, 0, "[!] Invalid BLE ID: %s\n", arg);
    mtk_signal_meter_start_resp_t started = {0};
    mtk_decode(op->resp_desc, &started, cap.body, cap.body_len, NULL);
    st->ble_signal_running = 1; st->ble_signal_token = started.operation_token;
    size_t used = emit(out, out_cap, 0, "[BLE:SIG:START]\n");
    if (ev.have_event) {
        mtk_signal_meter_update_ev_t upd = {0};
        mtk_decode(&mtk_signal_meter_update_ev_t_desc, &upd, ev.event_body, ev.event_body_len, NULL);
        used = emit(out, out_cap, used, "[BLE:SIG] raw=%d avg=%d cat=%u age=%u\n", upd.raw_rssi, upd.avg_rssi, upd.category, upd.age_ms);
    }
    return used;
}

/* RC8 independent audit P0-6, evidence `001-command-behavior-matrix.md`
 * lines 74-75: `connect <id>` auto-discovery completes with
 * "[BLE:DISC] complete: S service(s), C characteristic(s), D
 * descriptor(s)]"; `services` prints nested `[SVC]` / `[CHR]` / `[DSC]`
 * rows. GATT_DISCOVER (0x0003/0x0004) only ever enumerated services --
 * closing this gap needed genuinely new characteristic/descriptor
 * discovery, added as the new, purely additive GATT_DISCOVER_CHARS
 * (0x0003/0x0009) and GATT_DISCOVER_DESCS (0x0003/0x000A) opcodes (see
 * schemas.json / mtek_ble_logic.c / mtek_ble_hal_esp32.c) -- no existing
 * opcode's command ID, framing, or response shape changed.
 *
 * `gatt_discover_tree` below performs the full services->chars->descs
 * walk once and both callers (connect's auto-discovery, `services`'s own
 * listing) reuse it. The result tree is a file-static scratch object
 * (this session's own P0-1 "large aggregate objects must never live in a
 * task's call stack" rule applies here too -- ~6.7KB, far past what
 * belongs in any UART command handler's own frame), not a per-adapter-
 * instance field: the whole UART REPL is single-threaded and processes
 * one command to completion before the next, so one shared static is
 * safe, matching this file's own established `cap`/`cap2`/`ev` pattern.
 * Bounded to GATT_TREE_MAX_SVC/_CHR/_DSC (8/8/4) -- a disclosed,
 * reasonable practical limit for a console-grade tool, not a claim that
 * every real peripheral's GATT database is always this small; the
 * canonical opcodes themselves support pagination (`next_index`) that
 * this bounded, single-page tree view does not use. Per-characteristic
 * descriptor search range mirrors gatt_subscribe's own established
 * contract: (val_handle+1 .. next characteristic's def_handle-1, or the
 * containing service's end_handle if this is the last characteristic). */
#define GATT_TREE_MAX_SVC 8
#define GATT_TREE_MAX_CHR 8
#define GATT_TREE_MAX_DSC 4
typedef struct { mtk_uuid_t uuid; uint16_t handle; } gatt_tree_dsc_t;
typedef struct {
    mtk_uuid_t uuid; uint16_t def_handle, val_handle; uint8_t properties;
    gatt_tree_dsc_t dscs[GATT_TREE_MAX_DSC]; uint8_t dsc_count;
} gatt_tree_chr_t;
typedef struct {
    mtk_uuid_t uuid; uint16_t start_handle, end_handle;
    gatt_tree_chr_t chrs[GATT_TREE_MAX_CHR]; uint8_t chr_count;
} gatt_tree_svc_t;
typedef struct { gatt_tree_svc_t svcs[GATT_TREE_MAX_SVC]; uint8_t svc_count; uint32_t total_chrs, total_dscs; } gatt_tree_t;
static gatt_tree_t s_gatt_tree;

static void gatt_discover_tree(mtk_uart_adapter_state_t *st) {
    gatt_tree_t *t = &s_gatt_tree;
    memset(t, 0, sizeof(*t));

    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t sctx = make_ctx(st, &cap);
    mtk_gatt_discover_req_t sreq = {0}; sreq.connection_token = st->gatt_conn_token; sreq.max_items = GATT_TREE_MAX_SVC;
    const mtk_opcode_entry_t *svc_op = mtk_opcode_find(0x0003, 0x0004);
    uint8_t sbuf[16]; size_t sblen = 0;
    mtk_encode(svc_op->req_desc, &sreq, sbuf, sizeof(sbuf), &sblen);
    mtk_router_dispatch(&sctx, 0x0003, 0x0004, sbuf, sblen);
    if (cap.status != MTK_STATUS_OK) return;
    mtk_gatt_discover_resp_t svc_resp = {0};
    mtk_decode(svc_op->resp_desc, &svc_resp, cap.body, cap.body_len, NULL);
    t->svc_count = (uint8_t)(svc_resp.services.count > GATT_TREE_MAX_SVC ? GATT_TREE_MAX_SVC : svc_resp.services.count);

    const mtk_opcode_entry_t *chr_op = mtk_opcode_find(0x0003, 0x0009);
    const mtk_opcode_entry_t *dsc_op = mtk_opcode_find(0x0003, 0x000A);

    for (uint8_t si = 0; si < t->svc_count; si++) {
        gatt_tree_svc_t *sv = &t->svcs[si];
        sv->uuid = svc_resp.services.items[si].uuid;
        sv->start_handle = svc_resp.services.items[si].start_handle;
        sv->end_handle = svc_resp.services.items[si].end_handle;

        memset(&cap2, 0, sizeof(cap2));
        mtk_request_ctx_t cctx = make_ctx(st, &cap2);
        mtk_gatt_discover_chars_req_t creq = {0};
        creq.connection_token = st->gatt_conn_token; creq.start_handle = sv->start_handle; creq.end_handle = sv->end_handle;
        creq.max_items = GATT_TREE_MAX_CHR;
        uint8_t cbuf[16]; size_t cblen = 0;
        mtk_encode(chr_op->req_desc, &creq, cbuf, sizeof(cbuf), &cblen);
        mtk_router_dispatch(&cctx, 0x0003, 0x0009, cbuf, cblen);
        if (cap2.status != MTK_STATUS_OK) continue;
        mtk_gatt_discover_chars_resp_t chr_resp = {0};
        mtk_decode(chr_op->resp_desc, &chr_resp, cap2.body, cap2.body_len, NULL);
        sv->chr_count = (uint8_t)(chr_resp.items.count > GATT_TREE_MAX_CHR ? GATT_TREE_MAX_CHR : chr_resp.items.count);
        t->total_chrs += sv->chr_count;

        for (uint8_t ci = 0; ci < sv->chr_count; ci++) {
            gatt_tree_chr_t *ch = &sv->chrs[ci];
            ch->uuid = chr_resp.items.items[ci].uuid;
            ch->def_handle = chr_resp.items.items[ci].def_handle;
            ch->val_handle = chr_resp.items.items[ci].val_handle;
            ch->properties = chr_resp.items.items[ci].properties;

            uint16_t dsc_end = ((uint32_t)ci + 1 < chr_resp.items.count) ? (uint16_t)(chr_resp.items.items[ci + 1].def_handle - 1) : sv->end_handle;
            if (dsc_end <= ch->val_handle) continue;

            memset(&cap, 0, sizeof(cap));
            mtk_request_ctx_t dctx = make_ctx(st, &cap);
            mtk_gatt_discover_descs_req_t dreq = {0};
            dreq.connection_token = st->gatt_conn_token;
            dreq.start_handle = (uint16_t)(ch->val_handle + 1); dreq.end_handle = dsc_end;
            dreq.max_items = GATT_TREE_MAX_DSC;
            uint8_t dbuf[16]; size_t dblen = 0;
            mtk_encode(dsc_op->req_desc, &dreq, dbuf, sizeof(dbuf), &dblen);
            mtk_router_dispatch(&dctx, 0x0003, 0x000A, dbuf, dblen);
            if (cap.status != MTK_STATUS_OK) continue;
            mtk_gatt_discover_descs_resp_t dsc_resp = {0};
            mtk_decode(dsc_op->resp_desc, &dsc_resp, cap.body, cap.body_len, NULL);
            ch->dsc_count = (uint8_t)(dsc_resp.items.count > GATT_TREE_MAX_DSC ? GATT_TREE_MAX_DSC : dsc_resp.items.count);
            t->total_dscs += ch->dsc_count;
            for (uint8_t di = 0; di < ch->dsc_count; di++) {
                ch->dscs[di].uuid = dsc_resp.items.items[di].uuid;
                ch->dscs[di].handle = dsc_resp.items.items[di].handle;
            }
        }
    }
}

static size_t handle_ble_connect(mtk_uart_adapter_state_t *st, const char *arg, char *out, size_t out_cap) {
    int id = -1; sscanf(arg, "%d", &id);
    if (!st->ble_scan_valid || id < 0 || (unsigned)id >= st->ble_count) return emit(out, out_cap, 0, "[!] Invalid BLE ID: %s\n", arg);
    char mac[18]; mac_to_str(st->ble_addr[id].b, mac);
    size_t used = emit(out, out_cap, 0, "[*] Connecting to %s (30s timeout)...\n", mac);
    mtk_gatt_connect_req_t req = {0};
    req.target.addr = st->ble_addr[id]; req.target.addr_type = st->ble_addr_type[id];
    /* Persistent, queue-backed sink (RC5 independent audit P0): GATT_VALUE_EVENT
     * notifications/indications after a successful connect arrive from
     * mtek_ble_gatt_tick()'s background polling, not this call. */
    mtk_request_ctx_t ctx = make_session_ctx(st);
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0003, 0x0001);
    uint8_t buf[16]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0003, 0x0001, buf, blen);
    drain_session_response(st, &cap, "GATT_CONNECT_COMPLETE", &ev);
    if (cap.status != MTK_STATUS_ACCEPTED || !ev.have_event) return emit(out, out_cap, used, "[BLE:ERR] connect failed status=%u\n", cap.status);
    mtk_gatt_connect_complete_ev_t done = {0};
    mtk_decode(&mtk_gatt_connect_complete_ev_t_desc, &done, ev.event_body, ev.event_body_len, NULL);
    if (done.status != MTK_STATUS_OK) return emit(out, out_cap, used, "[BLE:ERR] connect failed status=%u\n", done.status);
    st->gatt_connected = 1; st->gatt_conn_token = done.connection_token;
    used = emit(out, out_cap, used, "[BLE:CONN] connected handle=%u\n", done.connection_token);
    gatt_discover_tree(st);
    return emit(out, out_cap, used, "[BLE:DISC] complete: %u service(s), %lu characteristic(s), %lu descriptor(s)\n",
                s_gatt_tree.svc_count, (unsigned long)s_gatt_tree.total_chrs, (unsigned long)s_gatt_tree.total_dscs);
}

static const char *gatt_props_str(uint8_t properties, char *buf, size_t buf_cap) {
    /* Evidence order: `[R W WNR N I]`. Broadcast/AuthSignedWrite/
     * ExtendedProps have no shipped-observed letter in the accepted
     * matrix -- disclosed, not fabricated: omitted rather than guessed. */
    size_t used = 0;
    if (properties & 0x02) used += (size_t)snprintf(buf + used, buf_cap - used, "R ");
    if (properties & 0x08) used += (size_t)snprintf(buf + used, buf_cap - used, "W ");
    if (properties & 0x04) used += (size_t)snprintf(buf + used, buf_cap - used, "WNR ");
    if (properties & 0x10) used += (size_t)snprintf(buf + used, buf_cap - used, "N ");
    if (properties & 0x20) used += (size_t)snprintf(buf + used, buf_cap - used, "I ");
    if (used > 0 && buf[used - 1] == ' ') buf[used - 1] = 0;
    else buf[0] = 0;
    return buf;
}

static size_t handle_ble_services(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    if (!st->gatt_connected) return emit(out, out_cap, 0, "[!] Not connected.\n");
    gatt_discover_tree(st);
    size_t used = 0;
    for (uint8_t si = 0; si < s_gatt_tree.svc_count; si++) {
        gatt_tree_svc_t *sv = &s_gatt_tree.svcs[si];
        if (sv->uuid.width == 0) used = emit(out, out_cap, used, "[SVC %u] UUID16 0x%02X%02X handles %u-%u\n", si, sv->uuid.value[1], sv->uuid.value[0], sv->start_handle, sv->end_handle);
        else used = emit(out, out_cap, used, "[SVC %u] UUID128 handles %u-%u\n", si, sv->start_handle, sv->end_handle);
        for (uint8_t ci = 0; ci < sv->chr_count; ci++) {
            gatt_tree_chr_t *ch = &sv->chrs[ci];
            char props[24]; gatt_props_str(ch->properties, props, sizeof(props));
            if (ch->uuid.width == 0) used = emit(out, out_cap, used, "  [CHR] UUID16 0x%02X%02X val=%u props=0x%02X [%s]\n", ch->uuid.value[1], ch->uuid.value[0], ch->val_handle, ch->properties, props);
            else used = emit(out, out_cap, used, "  [CHR] UUID128 val=%u props=0x%02X [%s]\n", ch->val_handle, ch->properties, props);
            for (uint8_t di = 0; di < ch->dsc_count; di++) {
                gatt_tree_dsc_t *ds = &ch->dscs[di];
                if (ds->uuid.width == 0) used = emit(out, out_cap, used, "    [DSC] UUID16 0x%02X%02X handle=%u\n", ds->uuid.value[1], ds->uuid.value[0], ds->handle);
                else used = emit(out, out_cap, used, "    [DSC] UUID128 handle=%u\n", ds->handle);
            }
        }
    }
    return used;
}

static size_t handle_ble_read(mtk_uart_adapter_state_t *st, const char *arg, char *out, size_t out_cap) {
    if (!st->gatt_connected) return emit(out, out_cap, 0, "[!] Not connected.\n");
    long handle = strtol(arg, NULL, 0);
    mtk_gatt_read_req_t req = {0}; req.connection_token = st->gatt_conn_token; req.handle = (uint16_t)handle;
    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t ctx = make_ctx(st, &cap);
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0003, 0x0005);
    uint8_t buf[8]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0003, 0x0005, buf, blen);
    if (cap.status != MTK_STATUS_OK) return emit(out, out_cap, 0, "[BLE:ERR] read handle failed status=%u\n", cap.status);
    mtk_gatt_read_resp_t resp = {0};
    mtk_decode(op->resp_desc, &resp, cap.body, cap.body_len, NULL);
    char hex[1030]; bytes_to_hex(resp.data.data, resp.data.len, hex, sizeof(hex));
    return emit(out, out_cap, 0, "[BLE:READ] handle=%ld len=%u data=%s\n", handle, resp.data.len, hex);
}

static size_t handle_ble_write_stage(mtk_uart_adapter_state_t *st, const char *args, uint8_t with_response, char *out, size_t out_cap) {
    if (!st->gatt_connected) return emit(out, out_cap, 0, "[!] Not connected.\n");
    char handle_str[16]; unsigned hi = 0;
    while (args[hi] && args[hi] != ' ' && hi < sizeof(handle_str) - 1) { handle_str[hi] = args[hi]; hi++; }
    handle_str[hi] = 0;
    const char *hexpart = args + hi; while (*hexpart == ' ') hexpart++;
    st->write_handle = (uint16_t)strtol(handle_str, NULL, 0);
    hex_to_bytes(hexpart, st->write_data, &st->write_len, sizeof(st->write_data));
    st->write_with_response = with_response;
    st->write_staged = 1;
    char hex[130]; bytes_to_hex(st->write_data, st->write_len, hex, sizeof(hex));
    size_t used = emit(out, out_cap, 0, "[*] PENDING WRITE (%s) handle=%u len=%u data=%s\n",
                        with_response ? "with-response" : "no-response", st->write_handle, st->write_len, hex);
    return emit(out, out_cap, used, "-> 'confirm' to send, 'cancel' to abort.\n");
}

static size_t handle_ble_confirm(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    if (!st->write_staged) return emit(out, out_cap, 0, "[*] No write pending.\n");
    mtk_gatt_write_req_t req = {0};
    req.connection_token = st->gatt_conn_token; req.handle = st->write_handle;
    req.data.len = st->write_len; memcpy(req.data.data, st->write_data, st->write_len);
    req.with_response = st->write_with_response;
    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t ctx = make_ctx(st, &cap);
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0003, 0x0006);
    uint8_t buf[80]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0003, 0x0006, buf, blen);
    st->write_staged = 0;
    if (cap.status != MTK_STATUS_OK) return emit(out, out_cap, 0, "[BLE:ERR] write failed status=%u\n", cap.status);
    return emit(out, out_cap, 0, req.with_response ? "[BLE:WRITE] ok\n" : "[BLE:WRITE] sent (no response requested)\n");
}
static size_t handle_ble_cancel(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    if (!st->write_staged) return emit(out, out_cap, 0, "[*] No write pending.\n");
    st->write_staged = 0;
    return 0;
}

static size_t handle_ble_subscribe(mtk_uart_adapter_state_t *st, const char *arg, uint8_t mode, char *out, size_t out_cap) {
    if (!st->gatt_connected) return emit(out, out_cap, 0, "[!] Not connected.\n");
    long handle = strtol(arg, NULL, 0);
    mtk_gatt_subscribe_req_t req = {0}; req.connection_token = st->gatt_conn_token; req.handle = (uint16_t)handle; req.mode = mode;
    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t ctx = make_ctx(st, &cap);
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0003, 0x0007);
    uint8_t buf[8]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0003, 0x0007, buf, blen);
    /* RC8 independent audit P0-6 "Preserve exact shipped UART behavior":
     * the accepted baseline's own `subscribe`/`indicate` rows: a distinct
     * "[!] Characteristic <h> has no CCCD (not subscribable)." error
     * (MTK_STATUS_NOT_FOUND, mtek_ble_logic.c's own new distinction), and
     * BOTH subscribe and indicate ack with "[BLE:SUB] ok" -- indicate's
     * own ack column is explicitly "as above" (subscribe's), not a
     * separate "[BLE:IND] ok"; "[BLE:IND]" is reserved for the later
     * per-notification STREAM tag (format_background_frame below), a
     * genuinely different message this ack was previously, incorrectly,
     * reusing. */
    if (cap.status == MTK_STATUS_NOT_FOUND) return emit(out, out_cap, 0, "[!] Characteristic %ld has no CCCD (not subscribable).\n", handle);
    if (cap.status != MTK_STATUS_OK) return emit(out, out_cap, 0, "[BLE:ERR] subscribe failed status=%u\n", cap.status);
    return emit(out, out_cap, 0, "[BLE:SUB] ok\n");
}
static size_t handle_ble_unsubscribe(mtk_uart_adapter_state_t *st, const char *arg, char *out, size_t out_cap) {
    if (!st->gatt_connected) return emit(out, out_cap, 0, "[!] Not connected.\n");
    long handle = strtol(arg, NULL, 0);
    mtk_gatt_unsubscribe_req_t req = {0}; req.connection_token = st->gatt_conn_token; req.handle = (uint16_t)handle;
    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t ctx = make_ctx(st, &cap);
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0003, 0x0008);
    uint8_t buf[8]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0003, 0x0008, buf, blen);
    (void)cap;
    return 0;
}

static size_t handle_ble_status(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    if (!st->gatt_connected) return emit(out, out_cap, 0, "[BLE:STATUS] connected=0 dropped_notifications=0\n");
    mtk_gatt_status_req_t req = {0}; req.connection_token = st->gatt_conn_token;
    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t ctx = make_ctx(st, &cap);
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0003, 0x0003);
    uint8_t buf[8]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0003, 0x0003, buf, blen);
    mtk_gatt_status_resp_t resp = {0};
    if (cap.status == MTK_STATUS_OK) mtk_decode(op->resp_desc, &resp, cap.body, cap.body_len, NULL);
    if (st->write_staged) return emit(out, out_cap, 0, "[BLE:STATUS] connected=%u dropped_notifications=%u (write pending: 'confirm'/'cancel')\n", resp.connected, resp.dropped_notification_count);
    return emit(out, out_cap, 0, "[BLE:STATUS] connected=%u dropped_notifications=%u\n", resp.connected, resp.dropped_notification_count);
}

static size_t handle_ble_disconnect(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    if (!st->gatt_connected) return emit(out, out_cap, 0, "[!] Not connected.\n");
    mtk_gatt_disconnect_req_t req = {0}; req.connection_token = st->gatt_conn_token;
    memset(&cap, 0, sizeof(cap));
    mtk_request_ctx_t ctx = make_ctx(st, &cap);
    const mtk_opcode_entry_t *op = mtk_opcode_find(0x0003, 0x0002);
    uint8_t buf[8]; size_t blen = 0;
    mtk_encode(op->req_desc, &req, buf, sizeof(buf), &blen);
    mtk_router_dispatch(&ctx, 0x0003, 0x0002, buf, blen);
    st->gatt_connected = 0;
    return emit(out, out_cap, 0, "[BLE:CONN] disconnected reason=0\n");
}

/* ===================== top-level line dispatch ========================= */

/* RC8 independent audit P0-7 "Claim AUTO transport only after valid
 * grammar recognition": "AUTO mode currently claims UART on any non-
 * empty input, despite its own contract saying the first valid legacy
 * command wins. Noise, partial input, boot chatter, or an unknown
 * command can permanently lock out SPI for that boot." A side-effect-
 * free recognizer -- checked by the REPL loop (app_main.c) BEFORE ever
 * attempting mtk_transport_claim_try, mirroring how the SPI side of this
 * same cross-transport race only ever claims after recognizing a real
 * protocol frame (mtk_transport_try_recognize_discovery), never on
 * arbitrary bytes. Mirrors mtek_uart_process_line's own dispatch grammar
 * exactly (mode-independent global commands, then the current mode's own
 * table) -- MUST be kept in sync with that function below if its own
 * command set ever changes; deliberately does not call into any handler
 * (no side effect, no radio/state mutation), so it is safe to call
 * speculatively before the cross-transport race is even decided. */
static int mtek_uart_line_is_recognized(const mtk_uart_adapter_state_t *st, const char *line) {
    if (line[0] == 0) return 0; /* an empty line is never itself a "valid legacy command" signal for the claim race */
    if (strcmp(line, "help") == 0 || strcmp(line, "version") == 0 || strcmp(line, "reboot") == 0) return 1;
    if (strcmp(line, "mode") == 0 || strcmp(line, "mode -w") == 0 || strcmp(line, "mode -b") == 0) return 1;
    if (st->mode == MTK_UART_MODE_WIFI) {
        if (strcmp(line, "scan -a") == 0 || strcmp(line, "scan -s") == 0) return 1;
        if (strcmp(line, "list -a") == 0 || strcmp(line, "list -s") == 0) return 1;
        if (strcmp(line, "select -l") == 0) return 1;
        if (starts_with(line, "select -a") || starts_with(line, "select -s")) return 1;
        if (starts_with(line, "beacon ")) return 1;
        if (strcmp(line, "handshake") == 0) return 1;
        if (starts_with(line, "deauth")) return 1;
        if (strcmp(line, "stop") == 0) return 1;
        return 0;
    }
    if (strcmp(line, "scan") == 0 || starts_with(line, "scan ")) return 1;
    if (strcmp(line, "list") == 0 || strcmp(line, "list -d") == 0 || starts_with(line, "list ")) return 1;
    if (strcmp(line, "advertise") == 0 || starts_with(line, "advertise -n ")) return 1;
    if (starts_with(line, "signal ") || starts_with(line, "connect ")) return 1;
    if (strcmp(line, "services") == 0) return 1;
    if (starts_with(line, "read ") || starts_with(line, "write ") || starts_with(line, "writenr ")) return 1;
    if (strcmp(line, "confirm") == 0 || strcmp(line, "cancel") == 0) return 1;
    if (starts_with(line, "subscribe ") || starts_with(line, "indicate ") || starts_with(line, "unsubscribe ")) return 1;
    if (strcmp(line, "status") == 0 || strcmp(line, "disconnect") == 0 || strcmp(line, "stop") == 0) return 1;
    return 0;
}
int mtek_uart_adapter_line_is_recognized(const mtk_uart_adapter_state_t *st, const char *line) {
    return mtek_uart_line_is_recognized(st, line);
}

size_t mtek_uart_process_line(mtk_uart_adapter_state_t *st, const char *line, char *out, size_t out_cap) {
    if (out_cap) out[0] = 0;
    /* RC5 independent audit P1 "Factory UART parity is incomplete": bare
     * Enter (an empty line) stops a running List B operation, matching
     * the shipped command-behavior matrix's documented bare-Enter attack
     * stop -- a no-op otherwise. The REPL loop (app_main.c) must call
     * this with an empty string on a bare Enter for this to take effect;
     * previously it silently discarded a bare Enter before ever reaching
     * this function at all. */
    if (line[0] == 0) {
        if (st->deauth_running || st->handshake_running || st->beacon_running || st->ble_adv_running || st->ble_signal_running)
            return handle_stop(st, out, out_cap);
        return 0;
    }
    if (strcmp(line, "help") == 0) return format_help_block(out, out_cap);
    if (strcmp(line, "version") == 0) {
        memset(&cap, 0, sizeof(cap));
        mtk_request_ctx_t ctx = make_ctx(st, &cap);
        mtk_router_dispatch(&ctx, 0x0000, 0x0002, NULL, 0);
        if (cap.status != MTK_STATUS_OK) return emit(out, out_cap, 0, "");
        mtk_get_version_resp_t v; memset(&v, 0, sizeof(v));
        const mtk_opcode_entry_t *op = mtk_opcode_find(0x0000, 0x0002);
        mtk_decode(op->resp_desc, &v, cap.body, cap.body_len, NULL);
        return emit(out, out_cap, 0, "MonstaTek M1 ESP32-C6 v%u.%u.%u\n", v.product_major, v.product_minor, v.product_patch);
    }
    if (strcmp(line, "mode") == 0) return emit(out, out_cap, 0, "[*] Current mode: %s\n", st->mode == MTK_UART_MODE_WIFI ? "WIFI" : "BLE");
    if (strcmp(line, "mode -w") == 0) { st->mode = MTK_UART_MODE_WIFI; return emit(out, out_cap, 0, "[*] Current mode: WIFI\n"); }
    if (strcmp(line, "mode -b") == 0) { st->mode = MTK_UART_MODE_BLE; return emit(out, out_cap, 0, "[*] Current mode: BLE\n"); }
    if (strcmp(line, "reboot") == 0) { st->reboot_requested = 1; return emit(out, out_cap, 0, "[*] Rebooting system...\n"); }

    if (st->mode == MTK_UART_MODE_WIFI) {
        if (strcmp(line, "scan -a") == 0) return handle_scan_a(st, out, out_cap);
        if (strcmp(line, "scan -s") == 0) return handle_scan_s(st, out, out_cap);
        if (strcmp(line, "list -a") == 0) return handle_list_a(st, out, out_cap);
        if (strcmp(line, "list -s") == 0) return handle_list_s(st, out, out_cap);
        if (strcmp(line, "select -l") == 0) return handle_select_l(st, out, out_cap);
        if (starts_with(line, "select -a")) return handle_select_a(st, line + 9, out, out_cap);
        if (starts_with(line, "select -s")) return handle_select_s(st, line + 9, out, out_cap);
        if (starts_with(line, "beacon ")) return handle_beacon(st, line + 7, out, out_cap);
        if (strcmp(line, "handshake") == 0) return handle_handshake(st, out, out_cap);
        if (starts_with(line, "deauth")) return handle_deauth(st, line + 6, out, out_cap);
        if (strcmp(line, "stop") == 0) return handle_stop(st, out, out_cap);
        return emit(out, out_cap, 0, "[!] WIFI mode supports only WIFI commands. Type 'help' for available commands.\n");
    } else {
        if (strcmp(line, "scan") == 0) return handle_ble_scan(st, 0, "", out, out_cap);
        if (starts_with(line, "scan ")) {
            const char *rest = line + 5;
            uint8_t passive = (strstr(rest, "-p") != NULL) ? 1 : 0;
            return handle_ble_scan(st, passive, rest, out, out_cap);
        }
        if (strcmp(line, "list") == 0) return handle_ble_list(st, out, out_cap);
        if (strcmp(line, "list -d") == 0) return handle_ble_list_all_detail(st, out, out_cap);
        if (starts_with(line, "list ")) return handle_ble_list_detail(st, line + 5, out, out_cap);
        if (strcmp(line, "advertise") == 0) return handle_ble_advertise(st, NULL, out, out_cap);
        if (starts_with(line, "advertise -n ")) return handle_ble_advertise(st, line + 13, out, out_cap);
        if (starts_with(line, "signal ")) return handle_ble_signal(st, line + 7, out, out_cap);
        if (starts_with(line, "connect ")) return handle_ble_connect(st, line + 8, out, out_cap);
        if (strcmp(line, "services") == 0) return handle_ble_services(st, out, out_cap);
        if (starts_with(line, "read ")) return handle_ble_read(st, line + 5, out, out_cap);
        if (starts_with(line, "write ")) return handle_ble_write_stage(st, line + 6, 1, out, out_cap);
        if (starts_with(line, "writenr ")) return handle_ble_write_stage(st, line + 8, 0, out, out_cap);
        if (strcmp(line, "confirm") == 0) return handle_ble_confirm(st, out, out_cap);
        if (strcmp(line, "cancel") == 0) return handle_ble_cancel(st, out, out_cap);
        if (starts_with(line, "subscribe ")) return handle_ble_subscribe(st, line + 10, 0, out, out_cap);
        if (starts_with(line, "indicate ")) return handle_ble_subscribe(st, line + 9, 1, out, out_cap);
        if (starts_with(line, "unsubscribe ")) return handle_ble_unsubscribe(st, line + 12, out, out_cap);
        if (strcmp(line, "status") == 0) return handle_ble_status(st, out, out_cap);
        if (strcmp(line, "disconnect") == 0) return handle_ble_disconnect(st, out, out_cap);
        if (strcmp(line, "stop") == 0) return handle_stop(st, out, out_cap);
        return emit(out, out_cap, 0,
            "[!] BLE mode supports 'scan', 'list <id>|all', 'advertise', and 'stop' commands. Type 'help' for available commands.\n");
    }
}

/* ===================== background (unsolicited) delivery =============== */

/* RC5 independent audit P0/P1: real asynchronous delivery for the three
 * long-lived sessions (handshake capture, signal meter, GATT notify) that
 * can produce output while the REPL is idle at the prompt, not just at
 * the moment their `handshake`/`signal`/`connect` command was issued. The
 * exact console line formatting reuses the accepted baseline's own
 * confirmed tags (`[BLE:SIG]`/`[BLE:NTF]`/`[BLE:IND]`/handshake phase
 * tags, 001-command-behavior-matrix.md), not an invented format. */
static size_t format_background_frame(const mtk_async_frame_t *f, char *out, size_t out_cap) {
    if (f->kind == MTK_ASYNC_FRAME_EVENT && strcmp(f->event_name, "HANDSHAKE_EVENT") == 0) {
        mtk_handshake_event_ev_t ev = {0};
        mtk_decode(&mtk_handshake_event_ev_t_desc, &ev, f->body, f->body_len, NULL);
        static const char *phase_tag[] = {"[FOUND EAPOL]", "[CAPTURED]", ">>> [SUCCESS]"};
        const char *tag = ev.phase < 3 ? phase_tag[ev.phase] : "[HANDSHAKE]";
        return emit(out, out_cap, 0, "%s key_frame=M%u\n", tag, ev.key_frame);
    }
    if (f->kind == MTK_ASYNC_FRAME_EVENT && strcmp(f->event_name, "HANDSHAKE_STOPPED") == 0) {
        mtk_handshake_stopped_ev_t ev = {0};
        mtk_decode(&mtk_handshake_stopped_ev_t_desc, &ev, f->body, f->body_len, NULL);
        return emit(out, out_cap, 0, "[HANDSHAKE:STOPPED] status=%u captured_len=%u\n", ev.status, ev.captured_total_len);
    }
    if (f->kind == MTK_ASYNC_FRAME_EVENT && strcmp(f->event_name, "SIGNAL_METER_UPDATE") == 0) {
        mtk_signal_meter_update_ev_t ev = {0};
        mtk_decode(&mtk_signal_meter_update_ev_t_desc, &ev, f->body, f->body_len, NULL);
        return emit(out, out_cap, 0, "[BLE:SIG] raw=%d avg=%d cat=%u age=%u\n", ev.raw_rssi, ev.avg_rssi, ev.category, ev.age_ms);
    }
    if (f->kind == MTK_ASYNC_FRAME_EVENT && strcmp(f->event_name, "SIGNAL_METER_LOST") == 0) {
        return emit(out, out_cap, 0, "[BLE:SIG:LOST]\n");
    }
    if (f->kind == MTK_ASYNC_FRAME_EVENT && strcmp(f->event_name, "GATT_VALUE_EVENT") == 0) {
        mtk_gatt_value_event_ev_t ev = {0};
        mtk_decode(&mtk_gatt_value_event_ev_t_desc, &ev, f->body, f->body_len, NULL);
        /* RC8 independent audit P0-6 "Preserve exact shipped UART
         * behavior": the accepted baseline's own subscribe/indicate rows
         * require "[BLE:NTF] handle=H len=N data=<hex>" /
         * "[BLE:IND] handle=H len=N data=<hex>" verbatim -- this
         * previously used "[BLE:NOTIFY]"/"[BLE:INDICATE]" (invented
         * names, not the shipped ones) and omitted len=N entirely. */
        char hex[130]; bytes_to_hex(ev.data.data, ev.data.len, hex, sizeof(hex));
        return emit(out, out_cap, 0, "[BLE:%s] handle=%u len=%u data=%s\n", ev.mode == 0 ? "NTF" : "IND", ev.handle, ev.data.len, hex);
    }
    /* An unrecognized/unhandled frame kind (e.g. a STREAM -- none of the
     * three session-queue opcodes emit one) is silently consumed rather
     * than ever misformatted -- never fabricate a console line for
     * traffic this adapter does not have a confirmed printed format for. */
    return 0;
}

size_t mtek_uart_adapter_poll_background(mtk_uart_adapter_state_t *st, char *out, size_t out_cap) {
    if (out_cap) out[0] = 0;
    /* RC7 independent audit item 9 "remote disconnect clears state but
     * emits no frozen '[BLE:CONN] disconnected reason=R' output" -- see
     * mtek_ble_service.h's own doc comment on this accessor for why it
     * is polled directly here instead of flowing through session_queue
     * like every other background delivery below. Checked first so it
     * is never starved by other pending traffic.
     * RC8 independent audit P0-6 "Remote disconnect currently loses the
     * real reason and emits a hard-coded reason": the real HCI-level
     * reason, threaded all the way from the HAL's own GAP callback. */
    uint8_t disc_reason = 0;
    if (mtek_ble_gatt_take_remote_disconnect_notice(&disc_reason)) {
        return emit(out, out_cap, 0, "[BLE:CONN] disconnected reason=%u\n", disc_reason);
    }
    mtk_async_frame_t f;
    while (mtk_async_queue_pop(&st->session_queue, &f)) {
        size_t n = format_background_frame(&f, out, out_cap);
        if (n) return n; /* one event per call, matching the REPL's own one-line-at-a-time idle poll */
    }
    return 0;
}
