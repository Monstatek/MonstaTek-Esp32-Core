# RC12 engineering status

## Reviewed application identity

| Field | Value |
| --- | --- |
| Target | ESP32-C6 |
| SDK used for reviewed build | ESP-IDF v6.0.1 |
| Candidate identity | rc12 |
| Application size | 1,443,072 bytes |
| Application SHA-256 | `703c01b3a89b67b7d16961339151761ce84ba151e619e9849b9344c426862b96` |
| Local commit | `1035984572298d21b3038c32d71f6fd10232ed20` |

This hash identifies the application image, not a complete merged flash image. A separately prepared merged package has its own size, hash, and checksum sidecar.

Superseded on 2026-09-08: the prior reviewed build (`build_rc12_astra_closure`,
SHA-256 `6c7a172619c2eaf66fbf8b60264c827f5d38afd33c13801487b9aa8b28209edb`) was a
correct build of an earlier, incompletely-synced state of this repository. The
2026-09-08 review transferred accumulated RC12 hardening/blocker/closure-round
source changes that had not previously reached this repository and rewrote
prose throughout comments and documentation to describe engineering
rationale without naming development tooling. A follow-up correction pass
the same day found and rewrote a second, previously-missed instance of two
source comments that verbatim-matched community reference code, and added
`docs/THIRD_PARTY_NOTICES.md` disclosing that reference project and its
license status. See `docs/DECISION_LOG.md` for the itemized history. No
List A behavior, protocol value, identifier, or opcode changed. The size is
unchanged; the hash differs because the source differs.

## Evidence reviewed

The focused final review confirmed the candidate identity, adapter configuration, build timestamps, saved build dry-run, schema comparisons, sanitizer results, and resource evidence. It found no remaining blocker in the reviewed correction and approved proceeding to packaging and hardware validation.

The reported host suite contained 70 tests, with full ASan/UBSan and TSan runs passing. The focused Community/C3 deferred-completion test was independently repeated 100 times under each sanitizer configuration (200 total runs), zero failures. Static resource evidence records 109,144 bytes of free DIRAM and nine selected stack chains passing the project's configured budget check. These figures are not measurements of worst-case runtime heap or hardware task-stack usage.

## Clean-source verification

On 2026-09-08, a fresh clone of this repository's local commit
`1035984572298d21b3038c32d71f6fd10232ed20` (not a working-tree copy) was built
in isolation.

- Both source generators were byte-idempotent, verified independently inside
  the clone.
- The documented source-only host workflow passed 70/70 tests under both the
  default ASan+UBSan configuration and ThreadSanitizer, zero races.
- A fresh ESP-IDF v6.0.1 ESP32-C6 build succeeded and was independently
  repeated from a second, never-touched build directory; both produced the
  identical application: 1,443,072 bytes, SHA-256
  `703c01b3a89b67b7d16961339151761ce84ba151e619e9849b9344c426862b96`.
- The resource gate passed with 109,144 bytes of free DIRAM; the stack-usage
  gate passed 9/9 chains, measured from a separate disposable build directory
  so the reviewed build directory itself was never touched by the
  stack-usage tool.
- A post-build Ninja dry-run on the reviewed build directory scheduled only
  the four expected ESP-IDF bootloader/check wrapper steps and no application
  compilation or relink.

This establishes source-set completeness and application reproducibility. It
does not validate a merged release package or hardware behavior.

## Approval limits

- Hardware regression for this exact candidate is not established by the engineering review.
- Earlier reports for RC11 or earlier RC12 binaries do not substitute for this candidate's evidence.
- Community compatibility is limited to implemented and verified interfaces; adapter inclusion alone does not establish compatibility with every community application.
- Publication hygiene must be rechecked after any later file or release change.
- Existing local staging directories are not automatically approved release packages.

Update this status only with evidence identifying the exact artifact tested. Record hardware results and merged-package identity separately.
