/* Minimal host-test assertion harness -- no external dependency (no
 * network access is available to fetch Unity/Catch2 in this environment).
 * A failed check prints file:line and the failing expression, then exits
 * nonzero so CTest reports the test as failed. */
#pragma once
#include <stdio.h>
#include <stdlib.h>

static int mtk_test_failures = 0;

#define MTK_CHECK(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "CHECK FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        mtk_test_failures++; \
    } \
} while (0)

#define MTK_CHECK_EQ(a, b) do { \
    long _a = (long)(a), _b = (long)(b); \
    if (_a != _b) { \
        fprintf(stderr, "CHECK_EQ FAILED %s:%d: %s (%ld) != %s (%ld)\n", __FILE__, __LINE__, #a, _a, #b, _b); \
        mtk_test_failures++; \
    } \
} while (0)

#define MTK_TEST_MAIN_BEGIN int main(void) {
#define MTK_TEST_MAIN_END \
    if (mtk_test_failures) { fprintf(stderr, "%d check(s) FAILED\n", mtk_test_failures); return 1; } \
    printf("OK\n"); return 0; \
    }
