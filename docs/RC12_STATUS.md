# RC12 engineering status

## Reviewed application identity

| Field | Value |
| --- | --- |
| Target | ESP32-C6 |
| SDK used for reviewed build | ESP-IDF v6.0.1 |
| Candidate identity | rc12 |
| Application size | 1,443,072 bytes |
| Application SHA-256 | `6c7a172619c2eaf66fbf8b60264c827f5d38afd33c13801487b9aa8b28209edb` |
| Reviewed build label | `build_rc12_astra_closure` |

This hash identifies the application image, not a complete merged flash image. A separately prepared merged package has its own size, hash, and checksum sidecar.

## Evidence reviewed

The focused final review confirmed the candidate identity, adapter configuration, build timestamps, saved build dry-run, schema comparisons, sanitizer results, and resource evidence. It found no remaining blocker in the reviewed correction and approved proceeding to packaging and hardware validation.

The reported host suite contained 70 tests, with full ASan/UBSan and TSan runs passing. The focused test was also repeated 100 times with each sanitizer configuration. Static resource evidence records 109,144 bytes of free DIRAM and nine selected stack chains passing the project's configured budget check. These figures are not measurements of worst-case runtime heap or hardware task-stack usage.

## Clean-source verification

On 2026-09-07, an explicit public source selection was copied into a fresh
temporary directory with no Git metadata, previous build directory, local
`sdkconfig`, release package, or evidence archive.

- Both source generators were byte-idempotent.
- The documented source-only host workflow passed 70/70 tests with the default
  ASan+UBSan configuration.
- A fresh ESP-IDF v6.0.1 ESP32-C6 build succeeded.
- The resulting application was byte-for-byte identical to the reviewed RC12
  application: 1,443,072 bytes, SHA-256
  `6c7a172619c2eaf66fbf8b60264c827f5d38afd33c13801487b9aa8b28209edb`.
- The resource gate passed with 109,144 bytes of free DIRAM.
- A post-build Ninja dry-run scheduled only the four expected ESP-IDF
  bootloader/check wrapper steps and no application compilation or relink.

This establishes source-set completeness and application reproducibility. It
does not validate a merged release package or hardware behavior.

## Approval limits

- Hardware regression for this exact candidate is not established by the engineering review.
- Earlier reports for RC11 or earlier RC12 binaries do not substitute for this candidate's evidence.
- Community compatibility is limited to implemented and verified interfaces; adapter inclusion alone does not establish compatibility with every community application.
- Publication hygiene must be rechecked after any later file or release change.
- Existing local staging directories are not automatically approved release packages.

Update this status only with evidence identifying the exact artifact tested. Record hardware results and merged-package identity separately.
