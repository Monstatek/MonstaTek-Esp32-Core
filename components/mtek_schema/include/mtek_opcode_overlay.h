/* Clean-room implementation from MonstaTek contract.
 *
 * RC12 hardening round, item 5 (P1): a test-only opcode overlay seam. The
 * generated production opcode table (mtek_opcode_registry.c) is immutable
 * and every entry in it is a genuinely shipped opcode advertised by
 * GET_CAPABILITIES. Host tests that need a generic arbiter-free
 * ACCEPTED_ASYNC vehicle previously borrowed the real TIME_SYNC_START
 * opcode for that purpose, which forced TIME_SYNC_START to keep advertising
 * itself SUPPORTED even though this candidate has no SNTP implementation
 * (it always fails IO_ERROR) -- a dishonest capability claim kept alive
 * purely to satisfy tests.
 *
 * This overlay decouples the two. A test can register a synthetic opcode
 * entry that mtk_opcode_find will return, WITHOUT it ever appearing in the
 * production table or GET_CAPABILITIES. Production code never calls
 * register/clear, so in a shipped build the overlay is permanently empty
 * and mtk_opcode_find behaves EXACTLY as the generated table alone (the
 * generated find consults an empty overlay first, a zero-iteration loop).
 * This mirrors the existing test-seam precedent in this tree (mtk_op_set_
 * won_hook in mtek_core.c): an inert hook in production, meaningful only
 * when a test wires it up. */
#pragma once
#include "mtek_opcode_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MTK_OPCODE_OVERLAY_MAX 4

/* Registers a caller-owned opcode entry (its storage must outlive every
 * dispatch that could look it up -- a file-scope static in the test).
 * Ignored if NULL, already registered, or the overlay is full. The
 * entry's (service_id, opcode) MUST NOT collide with any production table
 * entry; the overlay is consulted first, so a collision would shadow the
 * production opcode. */
void mtk_opcode_overlay_register(const mtk_opcode_entry_t *entry);

/* Empties the overlay. Tests that install an overlay opcode should call
 * this in teardown so one test's synthetic opcode never leaks into
 * another that runs in the same process. */
void mtk_opcode_overlay_clear(void);

/* Returns the registered overlay entry matching (service_id, opcode), or
 * NULL. Called by the generated mtk_opcode_find before it scans the
 * production table. */
const mtk_opcode_entry_t *mtk_opcode_overlay_find(uint16_t service_id, uint16_t opcode);

#ifdef __cplusplus
}
#endif
