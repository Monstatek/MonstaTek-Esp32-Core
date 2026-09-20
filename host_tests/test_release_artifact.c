/* This test previously (a) only validated the MD5 SIDECAR'S OWN TEXT FORMAT (32
 * uppercase hex characters) -- it never actually hashed `MtkCore.bin` at all, so
 * a stale or simply wrong sidecar passed as long as it merely looked like an MD5
 * string; (b) resolved the release directory via a hard-coded `../../release`
 * relative path, which silently resolves to the wrong location (or nowhere)
 * whenever ctest's own working directory isn't exactly two levels under the
 * project root -- an out-of-tree build directory made this gate trivially (and
 * silently) skip, "passing" without validating anything. Both are fixed: the
 * release directory is now resolved via (in priority order) an `MTK_RELEASE_DIR`
 * environment variable, a build-time-baked absolute project root
 * (`MTK_PROJECT_ROOT`, set by host_tests/CMakeLists.txt from CMake's own
 * `CMAKE_SOURCE_DIR` -- correct regardless of where the build directory actually
 * lives), or the historical relative path as a last resort (kept only for a
 * plain `cc` compile with neither of the above); and a real, self-contained MD5
 * (RFC 1321 -- no external command, no OpenSSL link dependency, matching this
 * whole suite's own dependency-free design) is computed over the actual bytes of
 * `MtkCore.bin` and compared byte-for-byte against the actual sidecar text. A
 * prior stale `release/MtkCore.bin` left over from an earlier candidate
 * therefore cannot make this gate look current: either its own real MD5 already
 * matches its own sidecar (nothing to catch -- this test cannot know intent,
 * only byte-consistency) or, far more usefully, a NEWLY regenerated `.bin` with
 * the OLD sidecar left in place is caught immediately as a mismatch. */
#include "mtk_test.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* RFC 1321 MD5, compact reference implementation (public domain algorithm;
 * written directly against the RFC's own pseudocode, not copied from any
 * existing codebase) -- verified below against the RFC's own test vectors before
 * this test trusts it on the real artifact. */
typedef struct { uint32_t a, b, c, d; uint64_t len; uint8_t buf[64]; unsigned buf_len; } md5_ctx_t;

static uint32_t md5_left_rotate(uint32_t x, uint32_t c) { return (x << c) | (x >> (32 - c)); }

static const uint32_t MD5_K[64] = {
    0xd76aa478,0xe8c7b756,0x242070db,0xc1bdceee,0xf57c0faf,0x4787c62a,0xa8304613,0xfd469501,
    0x698098d8,0x8b44f7af,0xffff5bb1,0x895cd7be,0x6b901122,0xfd987193,0xa679438e,0x49b40821,
    0xf61e2562,0xc040b340,0x265e5a51,0xe9b6c7aa,0xd62f105d,0x02441453,0xd8a1e681,0xe7d3fbc8,
    0x21e1cde6,0xc33707d6,0xf4d50d87,0x455a14ed,0xa9e3e905,0xfcefa3f8,0x676f02d9,0x8d2a4c8a,
    0xfffa3942,0x8771f681,0x6d9d6122,0xfde5380c,0xa4beea44,0x4bdecfa9,0xf6bb4b60,0xbebfbc70,
    0x289b7ec6,0xeaa127fa,0xd4ef3085,0x04881d05,0xd9d4d039,0xe6db99e5,0x1fa27cf8,0xc4ac5665,
    0xf4292244,0x432aff97,0xab9423a7,0xfc93a039,0x655b59c3,0x8f0ccc92,0xffeff47d,0x85845dd1,
    0x6fa87e4f,0xfe2ce6e0,0xa3014314,0x4e0811a1,0xf7537e82,0xbd3af235,0x2ad7d2bb,0xeb86d391,
};
static const uint32_t MD5_S[64] = {
    7,12,17,22, 7,12,17,22, 7,12,17,22, 7,12,17,22,
    5, 9,14,20, 5, 9,14,20, 5, 9,14,20, 5, 9,14,20,
    4,11,16,23, 4,11,16,23, 4,11,16,23, 4,11,16,23,
    6,10,15,21, 6,10,15,21, 6,10,15,21, 6,10,15,21,
};

static void md5_process_block(md5_ctx_t *ctx, const uint8_t *block) {
    uint32_t m[16];
    for (int i = 0; i < 16; i++) {
        m[i] = (uint32_t)block[i*4] | ((uint32_t)block[i*4+1] << 8) |
               ((uint32_t)block[i*4+2] << 16) | ((uint32_t)block[i*4+3] << 24);
    }
    uint32_t a = ctx->a, b = ctx->b, c = ctx->c, d = ctx->d;
    for (int i = 0; i < 64; i++) {
        uint32_t f; int g;
        if (i < 16) { f = (b & c) | (~b & d); g = i; }
        else if (i < 32) { f = (d & b) | (~d & c); g = (5*i + 1) % 16; }
        else if (i < 48) { f = b ^ c ^ d; g = (3*i + 5) % 16; }
        else { f = c ^ (b | ~d); g = (7*i) % 16; }
        uint32_t temp = d; d = c; c = b;
        b = b + md5_left_rotate(a + f + MD5_K[i] + m[g], MD5_S[i]);
        a = temp;
    }
    ctx->a += a; ctx->b += b; ctx->c += c; ctx->d += d;
}

static void md5_init(md5_ctx_t *ctx) {
    ctx->a = 0x67452301; ctx->b = 0xefcdab89; ctx->c = 0x98badcfe; ctx->d = 0x10325476;
    ctx->len = 0; ctx->buf_len = 0;
}
static void md5_update(md5_ctx_t *ctx, const uint8_t *data, size_t len) {
    ctx->len += len;
    while (len > 0) {
        size_t take = 64 - ctx->buf_len;
        if (take > len) take = len;
        memcpy(ctx->buf + ctx->buf_len, data, take);
        ctx->buf_len += (unsigned)take;
        data += take; len -= take;
        if (ctx->buf_len == 64) { md5_process_block(ctx, ctx->buf); ctx->buf_len = 0; }
    }
}
static void md5_final(md5_ctx_t *ctx, uint8_t out[16]) {
    /* Standard RFC 1321 padding: a single 0x80 byte, then zero bytes
     * until the block is exactly 56 bytes full (appending one extra full
     * 64-byte block first if there isn't room), then the original
     * bit-length as a little-endian 64-bit trailer. Padding bytes are
     * never passed through md5_update (they are not message bytes, and
     * must not perturb ctx->len). */
    uint64_t bit_len = ctx->len * 8;
    ctx->buf[ctx->buf_len++] = 0x80;
    if (ctx->buf_len > 56) {
        while (ctx->buf_len < 64) ctx->buf[ctx->buf_len++] = 0x00;
        md5_process_block(ctx, ctx->buf);
        ctx->buf_len = 0;
    }
    while (ctx->buf_len < 56) ctx->buf[ctx->buf_len++] = 0x00;
    for (int i = 0; i < 8; i++) ctx->buf[56 + i] = (uint8_t)(bit_len >> (8 * i));
    md5_process_block(ctx, ctx->buf);
    uint32_t words[4] = { ctx->a, ctx->b, ctx->c, ctx->d };
    for (int i = 0; i < 4; i++) {
        out[i*4+0] = (uint8_t)(words[i]);
        out[i*4+1] = (uint8_t)(words[i] >> 8);
        out[i*4+2] = (uint8_t)(words[i] >> 16);
        out[i*4+3] = (uint8_t)(words[i] >> 24);
    }
}
static void md5_hex_upper(const uint8_t digest[16], char out[33]) {
    static const char hex[] = "0123456789ABCDEF";
    for (int i = 0; i < 16; i++) {
        out[i*2] = hex[(digest[i] >> 4) & 0xF];
        out[i*2+1] = hex[digest[i] & 0xF];
    }
    out[32] = 0;
}
static void md5_hex_of(const uint8_t *data, size_t len, char out[33]) {
    md5_ctx_t ctx; md5_init(&ctx);
    md5_update(&ctx, data, len);
    uint8_t digest[16]; md5_final(&ctx, digest);
    md5_hex_upper(digest, out);
}

/* RFC 6234 SHA-256, compact reference implementation (public domain algorithm;
 * written directly against the RFC's own pseudocode, not copied from any
 * existing codebase) -- verified below against the RFC's own test vectors before
 * this test trusts it on the real artifact. Needed for RC11's own "compare... by
 * SHA-256" and "stale map" hard- fail requirements: merged_image_map.json
 * records a whole-file SHA-256 this test must independently re-derive, not
 * merely re-read. */
typedef struct { uint32_t h[8]; uint64_t len; uint8_t buf[64]; unsigned buf_len; } sha256_ctx_t;

static const uint32_t SHA256_K[64] = {
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2,
};

static uint32_t sha256_rotr(uint32_t x, uint32_t c) { return (x >> c) | (x << (32 - c)); }

static void sha256_process_block(sha256_ctx_t *ctx, const uint8_t *block) {
    uint32_t w[64];
    for (int i = 0; i < 16; i++) {
        w[i] = ((uint32_t)block[i*4] << 24) | ((uint32_t)block[i*4+1] << 16) |
               ((uint32_t)block[i*4+2] << 8) | (uint32_t)block[i*4+3];
    }
    for (int i = 16; i < 64; i++) {
        uint32_t s0 = sha256_rotr(w[i-15], 7) ^ sha256_rotr(w[i-15], 18) ^ (w[i-15] >> 3);
        uint32_t s1 = sha256_rotr(w[i-2], 17) ^ sha256_rotr(w[i-2], 19) ^ (w[i-2] >> 10);
        w[i] = w[i-16] + s0 + w[i-7] + s1;
    }
    uint32_t a = ctx->h[0], b = ctx->h[1], c = ctx->h[2], d = ctx->h[3];
    uint32_t e = ctx->h[4], f = ctx->h[5], g = ctx->h[6], h = ctx->h[7];
    for (int i = 0; i < 64; i++) {
        uint32_t S1 = sha256_rotr(e, 6) ^ sha256_rotr(e, 11) ^ sha256_rotr(e, 25);
        uint32_t ch = (e & f) ^ (~e & g);
        uint32_t temp1 = h + S1 + ch + SHA256_K[i] + w[i];
        uint32_t S0 = sha256_rotr(a, 2) ^ sha256_rotr(a, 13) ^ sha256_rotr(a, 22);
        uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        uint32_t temp2 = S0 + maj;
        h = g; g = f; f = e; e = d + temp1;
        d = c; c = b; b = a; a = temp1 + temp2;
    }
    ctx->h[0] += a; ctx->h[1] += b; ctx->h[2] += c; ctx->h[3] += d;
    ctx->h[4] += e; ctx->h[5] += f; ctx->h[6] += g; ctx->h[7] += h;
}

static void sha256_init(sha256_ctx_t *ctx) {
    static const uint32_t iv[8] = {
        0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19,
    };
    memcpy(ctx->h, iv, sizeof(iv));
    ctx->len = 0; ctx->buf_len = 0;
}
static void sha256_update(sha256_ctx_t *ctx, const uint8_t *data, size_t len) {
    ctx->len += len;
    while (len > 0) {
        size_t take = 64 - ctx->buf_len;
        if (take > len) take = len;
        memcpy(ctx->buf + ctx->buf_len, data, take);
        ctx->buf_len += (unsigned)take;
        data += take; len -= take;
        if (ctx->buf_len == 64) { sha256_process_block(ctx, ctx->buf); ctx->buf_len = 0; }
    }
}
static void sha256_final(sha256_ctx_t *ctx, uint8_t out[32]) {
    uint64_t bit_len = ctx->len * 8;
    ctx->buf[ctx->buf_len++] = 0x80;
    if (ctx->buf_len > 56) {
        while (ctx->buf_len < 64) ctx->buf[ctx->buf_len++] = 0x00;
        sha256_process_block(ctx, ctx->buf);
        ctx->buf_len = 0;
    }
    while (ctx->buf_len < 56) ctx->buf[ctx->buf_len++] = 0x00;
    for (int i = 7; i >= 0; i--) ctx->buf[56 + (7 - i)] = (uint8_t)(bit_len >> (8 * i));
    sha256_process_block(ctx, ctx->buf);
    for (int i = 0; i < 8; i++) {
        out[i*4+0] = (uint8_t)(ctx->h[i] >> 24);
        out[i*4+1] = (uint8_t)(ctx->h[i] >> 16);
        out[i*4+2] = (uint8_t)(ctx->h[i] >> 8);
        out[i*4+3] = (uint8_t)(ctx->h[i]);
    }
}
static void sha256_hex_upper(const uint8_t digest[32], char out[65]) {
    static const char hex[] = "0123456789ABCDEF";
    for (int i = 0; i < 32; i++) {
        out[i*2] = hex[(digest[i] >> 4) & 0xF];
        out[i*2+1] = hex[digest[i] & 0xF];
    }
    out[64] = 0;
}
static void sha256_hex_of(const uint8_t *data, size_t len, char out[65]) {
    sha256_ctx_t ctx; sha256_init(&ctx);
    sha256_update(&ctx, data, len);
    uint8_t digest[32]; sha256_final(&ctx, digest);
    sha256_hex_upper(digest, out);
}

/* Finds `"field_name": "<hex_len hex chars>"` inside a JSON text buffer
 * (a hand-rolled scan, not a real JSON parser -- this project's own
 * merged_image_map.json is always written by tools/package_release.py in
 * a fixed, known shape, so a full parser would be needless weight here).
 * Returns 1 and fills `out` (hex_len chars + NUL) only if the field is
 * found AND is exactly hex_len valid uppercase-hex ASCII bytes; 0
 * otherwise (field absent, wrong length, or non-hex content) -- the
 * caller treats 0 as a hard failure, never a silent skip. */
static int find_json_hex_field(const uint8_t *buf, long size, const char *field_name, int hex_len, char *out) {
    char needle[80];
    snprintf(needle, sizeof(needle), "\"%s\": \"", field_name);
    size_t needle_len = strlen(needle);
    const char *pos = NULL;
    for (long i = 0; i + (long)needle_len < size; i++) {
        if (memcmp(buf + i, needle, needle_len) == 0) { pos = (const char *)buf + i + (long)needle_len; break; }
    }
    if (!pos) return 0;
    if (pos + hex_len > (const char *)buf + size) return 0;
    for (int i = 0; i < hex_len; i++) {
        uint8_t c = (uint8_t)pos[i];
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) return 0;
    }
    memcpy(out, pos, (size_t)hex_len);
    out[hex_len] = 0;
    return 1;
}

/* Same idea as find_json_hex_field, for a plain unquoted JSON integer
 * field (`"field_name": 12345`). Strengthens merged-image-map validation
 * against the application size: app_size_bytes is this test's own independent,
 * C-side cross-check of the same field tools/package_release.py's own
 * Python-side validate_map_against_bin already enforces -- an
 * independent reimplementation catching a bug the same code checking
 * itself cannot. Returns 1 and fills *out on success, 0 if the field is
 * absent or not a plain non-negative integer. */
static int find_json_int_field(const uint8_t *buf, long size, const char *field_name, long *out) {
    char needle[80];
    snprintf(needle, sizeof(needle), "\"%s\": ", field_name);
    size_t needle_len = strlen(needle);
    const char *pos = NULL;
    for (long i = 0; i + (long)needle_len < size; i++) {
        if (memcmp(buf + i, needle, needle_len) == 0) { pos = (const char *)buf + i + (long)needle_len; break; }
    }
    if (!pos) return 0;
    const char *end = (const char *)buf + size;
    if (pos >= end || !(*pos >= '0' && *pos <= '9')) return 0;
    long value = 0;
    const char *p = pos;
    while (p < end && *p >= '0' && *p <= '9') { value = value * 10 + (*p - '0'); p++; }
    *out = value;
    return 1;
}

static uint8_t *read_file(const char *path, long *size_out) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = (uint8_t *)malloc((size_t)(sz > 0 ? sz : 1));
    if (buf && sz > 0) { size_t n = fread(buf, 1, (size_t)sz, f); (void)n; }
    fclose(f);
    *size_out = sz;
    return buf;
}

static char *dup_str(const char *s) {
    size_t n = strlen(s) + 1;
    char *buf = (char *)malloc(n);
    if (buf) memcpy(buf, s, n);
    return buf;
}

/* Resolves the release directory robustly: env var override, then the
 * build-time-baked absolute project root, then the historical relative
 * path as a last resort (see this file's own top doc comment). Returns a
 * malloc'd string the caller frees. */
static char *resolve_release_dir(void) {
    const char *env = getenv("MTK_RELEASE_DIR");
    if (env && *env) return dup_str(env);
#ifdef MTK_PROJECT_ROOT
    {
        size_t n = strlen(MTK_PROJECT_ROOT) + 16;
        char *buf = (char *)malloc(n);
        snprintf(buf, n, "%s/release", MTK_PROJECT_ROOT);
        return buf;
    }
#else
    return dup_str("../../release");
#endif
}

int main(void) {
    /* Self-check: this test's own MD5 implementation must match the RFC 1321
     * test vectors before it is trusted on the real artifact -- a bug here would
     * silently make every comparison below meaningless. */
    {
        char hex[33];
        md5_hex_of((const uint8_t *)"", 0, hex);
        MTK_CHECK_EQ((int)strlen(hex), 32);
        MTK_CHECK(strcmp(hex, "D41D8CD98F00B204E9800998ECF8427E") == 0);
        md5_hex_of((const uint8_t *)"abc", 3, hex);
        MTK_CHECK(strcmp(hex, "900150983CD24FB0D6963F7D28E17F72") == 0);
    }
    /* Same self-check for the SHA-256 implementation, against the RFC 6234 /
     * FIPS 180-4 published test vectors. -- */
    {
        char hex[65];
        sha256_hex_of((const uint8_t *)"", 0, hex);
        MTK_CHECK_EQ((int)strlen(hex), 64);
        MTK_CHECK(strcmp(hex, "E3B0C44298FC1C149AFBF4C8996FB92427AE41E4649B934CA495991B7852B855") == 0);
        sha256_hex_of((const uint8_t *)"abc", 3, hex);
        MTK_CHECK(strcmp(hex, "BA7816BF8F01CFEA414140DE5DAE2223B00361A396177A9CB410FF61F20015AD") == 0);
    }

    char *release_dir = resolve_release_dir();
    char bin_path[1024], md5_path[1024], map_path_check[1024], partitions_path[1024], manifest_path[1024];
    snprintf(bin_path, sizeof(bin_path), "%s/MtkCore.bin", release_dir);
    snprintf(md5_path, sizeof(md5_path), "%s/MtkCore.md5", release_dir);
    snprintf(map_path_check, sizeof(map_path_check), "%s/merged_image_map.json", release_dir);
    snprintf(partitions_path, sizeof(partitions_path), "%s/partitions.csv", release_dir);
    snprintf(manifest_path, sizeof(manifest_path), "%s/PACKAGING_MANIFEST.md", release_dir);
    printf("test_release_artifact: resolved release directory: %s\n", release_dir);

    /* "In
     * the final release-artifact test configuration, missing MtkCore.bin,
     * MtkCore.md5, merged_image_map.json, partitions.csv, or PACKAGING_
     * MANIFEST.md must fail. Do not return success with a NOTE. If a
     * pre-packaging test mode is needed, make it an explicit separate
     * mode rather than silently skipping." This IS the final release-
     * artifact test configuration by default (MTK_RELEASE_PRE_PACKAGING
     * unset): every one of these five files must exist, or this test
     * hard-fails, listing exactly which are missing. The old skip-if-
     * missing behavior survives ONLY as an explicit, separate,
     * deliberately-opted-into mode (MTK_RELEASE_PRE_PACKAGING=1) for a
     * genuinely pre-packaging host-test run (before any idf.py build/
     * packaging has happened at all) -- never the silent default. */
    const char *pre_packaging_env = getenv("MTK_RELEASE_PRE_PACKAGING");
    int pre_packaging_mode = (pre_packaging_env && strcmp(pre_packaging_env, "1") == 0);

    const char *required_paths[5] = { bin_path, md5_path, map_path_check, partitions_path, manifest_path };
    const char *required_names[5] = { "MtkCore.bin", "MtkCore.md5", "merged_image_map.json", "partitions.csv", "PACKAGING_MANIFEST.md" };
    int any_missing = 0;
    for (int i = 0; i < 5; i++) {
        FILE *probe = fopen(required_paths[i], "rb");
        if (!probe) {
            any_missing = 1;
            if (pre_packaging_mode) {
                printf("NOTE (MTK_RELEASE_PRE_PACKAGING=1, explicit pre-packaging mode): '%s' is not "
                       "present.\n", required_paths[i]);
            } else {
                fprintf(stderr, "test_release_artifact: '%s' is missing -- the final release-artifact "
                                "test configuration requires it (set MTK_RELEASE_PRE_PACKAGING=1 "
                                "explicitly for a deliberate pre-packaging run instead)\n", required_paths[i]);
                MTK_CHECK(0); /* recorded as a real failure, named explicitly above -- never a silent skip */
            }
        } else {
            fclose(probe);
        }
    }
    if (any_missing) {
        if (pre_packaging_mode) {
            printf("OK (pre-packaging mode: skipping deep validation)\n");
            free(release_dir);
            return 0;
        }
        fprintf(stderr, "%d check(s) FAILED\n", mtk_test_failures);
        free(release_dir);
        return 1;
    }
    (void)required_names;

    long bin_size = 0;
    uint8_t *bin = read_file(bin_path, &bin_size);
    MTK_CHECK(bin != NULL);
    if (!bin) {
        fprintf(stderr, "test_release_artifact: '%s' could not be read despite existing (permissions?)\n", bin_path);
        free(release_dir);
        return 1;
    }

    /* ESP-IDF bootloader/app image magic byte is 0xE9 at the image start. */
    MTK_CHECK(bin_size > 0x11000);
    MTK_CHECK_EQ(bin[0], 0xE9); /* bootloader image at offset 0x0 */
    MTK_CHECK_EQ(bin[0x10000], 0xE9); /* application image at offset 0x10000 */

    /* Partition table magic: 0xAA 0x50 at offset 0x8000 (esp_partition.h
     * ESP_PARTITION_MAGIC = 0x50AA, little-endian on the wire = AA 50). */
    MTK_CHECK_EQ(bin[0x8000], 0xAA);
    MTK_CHECK_EQ(bin[0x8001], 0x50);

    long md5_size = 0;
    uint8_t *md5file = read_file(md5_path, &md5_size);
    MTK_CHECK(md5file != NULL);
    if (md5file) {
        /* Factory sidecar syntax: exactly 32 uppercase hex bytes, no
         * trailing newline/whitespace. */
        MTK_CHECK_EQ(md5_size, 32);
        for (long i = 0; i < md5_size && i < 32; i++) {
            uint8_t c = md5file[i];
            int ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F');
            MTK_CHECK(ok);
        }

        /* The actual proof this test previously never performed: hash the
         * REAL delivered binary and compare it byte-for-byte against the
         * REAL sidecar text. */
        char computed_hex[33];
        md5_hex_of(bin, (size_t)bin_size, computed_hex);
        if (md5_size == 32) {
            char sidecar_hex[33];
            memcpy(sidecar_hex, md5file, 32);
            sidecar_hex[32] = 0;
            MTK_CHECK(strcmp(computed_hex, sidecar_hex) == 0);
            if (strcmp(computed_hex, sidecar_hex) != 0) {
                fprintf(stderr, "test_release_artifact: MD5 mismatch -- computed %s, sidecar says %s "
                                "(release/MtkCore.md5 does not match release/MtkCore.bin; a stale sidecar "
                                "left over from an earlier candidate, or a .bin regenerated without "
                                "regenerating its sidecar, must never pass this gate)\n",
                        computed_hex, sidecar_hex);
            }
        }
        free(md5file);
    }

    /* RC11 "release validation must hard-fail for:... missing map, stale map,
     * application bytes differing from the audited build": the whole-file MD5
     * check above only proves MtkCore.bin matches MtkCore.md5 -- both could be a
     * stale pair, still internally consistent with each other. A THIRD,
     * independently-generated artifact (merged_image_map.json, written by
     * tools/package_ release.py at packaging time) records the whole-file
     * SHA-256 and a separate MD5 of ONLY the application segment
     * (application_offset..EOF); both are re-verified here against the actual
     * current.bin bytes, so an attacker or mistake would have to tamper with the
     * map too, not just the.bin+.md5 pair, to go undetected. Previously a
     * missing map, or one missing the app_md5_uppercase_hex field, was only a
     * NOTE (a forward-looking gate that did not yet apply to an existing pre-fix
     * artifact) -- now that a real, current release/merged_image_map.json always
     * carries these fields (tools/package_release.py always writes them), their
     * absence is itself a hard failure: a missing map is exactly as
     * unbound/unverifiable as a missing sidecar. */
    {
        char map_path[1024];
        snprintf(map_path, sizeof(map_path), "%s/merged_image_map.json", release_dir);
        long map_size = 0;
        uint8_t *map = read_file(map_path, &map_size);
        MTK_CHECK(map != NULL);
        if (!map) {
            fprintf(stderr, "test_release_artifact: '%s' is missing -- the release pair's own byte binding "
                            "to a specific audited build cannot be verified without it\n", map_path);
        } else {
            char app_md5_map_hex[33], sha256_map_hex[65];
            int have_app_md5 = find_json_hex_field(map, map_size, "app_md5_uppercase_hex", 32, app_md5_map_hex);
            int have_sha256 = find_json_hex_field(map, map_size, "sha256_uppercase_hex", 64, sha256_map_hex);
            MTK_CHECK(have_app_md5);
            MTK_CHECK(have_sha256);
            if (!have_app_md5) fprintf(stderr, "test_release_artifact: '%s' has no valid app_md5_uppercase_hex field\n", map_path);
            if (!have_sha256) fprintf(stderr, "test_release_artifact: '%s' has no valid sha256_uppercase_hex field\n", map_path);

            if (have_app_md5 && bin_size > 0x10000) {
                char app_computed_hex[33];
                md5_hex_of(bin + 0x10000, (size_t)(bin_size - 0x10000), app_computed_hex);
                MTK_CHECK(strcmp(app_computed_hex, app_md5_map_hex) == 0);
                if (strcmp(app_computed_hex, app_md5_map_hex) != 0) {
                    fprintf(stderr, "test_release_artifact: app-segment MD5 mismatch -- computed %s, "
                                    "merged_image_map.json says %s (the .bin's own application segment "
                                    "no longer matches what the map recorded -- a stale release pair, "
                                    "even one whose whole-file MD5 is internally self-consistent, must "
                                    "never pass this gate)\n",
                            app_computed_hex, app_md5_map_hex);
                }
            }
            if (have_sha256) {
                char sha256_computed_hex[65];
                sha256_hex_of(bin, (size_t)bin_size, sha256_computed_hex);
                MTK_CHECK(strcmp(sha256_computed_hex, sha256_map_hex) == 0);
                if (strcmp(sha256_computed_hex, sha256_map_hex) != 0) {
                    fprintf(stderr, "test_release_artifact: whole-file SHA-256 mismatch -- computed %s, "
                                    "merged_image_map.json says %s (stale map)\n",
                            sha256_computed_hex, sha256_map_hex);
                }
            }

            /* "strengthen merged-image-map validation... require
             * and verify... application size": independent C-side
             * cross-check of app_size_bytes against the real merged
             * binary's own application segment length. */
            long app_size_map = 0;
            int have_app_size = find_json_int_field(map, map_size, "app_size_bytes", &app_size_map);
            MTK_CHECK(have_app_size);
            if (!have_app_size) {
                fprintf(stderr, "test_release_artifact: '%s' has no valid app_size_bytes field\n", map_path);
            } else if (bin_size > 0x10000) {
                long real_app_size = bin_size - 0x10000;
                MTK_CHECK_EQ(app_size_map, real_app_size);
                if (app_size_map != real_app_size) {
                    fprintf(stderr, "test_release_artifact: app_size_bytes mismatch -- computed %ld, "
                                    "merged_image_map.json says %ld (stale map)\n",
                            real_app_size, app_size_map);
                }
            }
            free(map);
        }
    }

    free(bin);
    free(release_dir);

    if (mtk_test_failures) { fprintf(stderr, "%d check(s) FAILED\n", mtk_test_failures); return 1; }
    printf("OK\n");
    return 0;
}
