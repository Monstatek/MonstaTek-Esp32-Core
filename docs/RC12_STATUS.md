# RC12 engineering status

## Reviewed application identity

| Field | Value |
| --- | --- |
| Target | ESP32-C6 |
| SDK used for reviewed build | ESP-IDF v6.0.1 |
| Candidate identity | rc12 |
| Application size | 1,443,072 bytes |
| Application SHA-256 | `00c11f8bb069f840c572df3bd5fed0b82a7a0d507657d6d93b89d2507642f790` |
| Reviewed source commit | `9dedbb12cdbd4f977693858e917f37c50ba14a48` |
| Merged-package size | 1,508,608 bytes |
| Merged-package MD5 | `814C2FF53B9B3C771658C07F651B890F` |
| Merged-package SHA-256 | `0455E4B77F6814B3EB56B3D19E0232515DCDAF43838D0E74CE6280CEF3DCD2B6` |

This application hash identifies the application image; the merged-package
fields identify the complete flash image (bootloader + partition table +
application) prepared by `tools/package_release.py`, which now records its
exact source commit and dirty/clean status honestly (see "Correction
history" below) rather than a fixed placeholder.

Application SHA-256 changes whenever the reviewed source commit changes,
even if compiled logic is unaffected: ESP-IDF embeds a Git-derived project
version string (the short commit hash) into the application image's own
app descriptor at a fixed offset, so rewriting reachable history changes
that embedded string, and therefore the image bytes and SHA-256, on its
own. Confirmed directly: `cmp -l` between the build made before and after
a history rewrite that changed only commit identity (no source content)
found exactly 71 differing bytes, concentrated at the embedded version
string and the two checksums derived from it (an embedded image digest and
the appended whole-image integrity hash) -- nowhere else in the 1,443,072-
byte image. Identical size, identical 109,144-byte free-DIRAM figure, and
identical stack-budget results across both builds confirm the compiled
logic itself did not change. Use the application SHA-256 together with the
exact reviewed source commit above to identify a specific build; the two
must always be read together, not the hash alone.

Superseded on 2026-09-08: an earlier reviewed build (SHA-256
`6c7a172619c2eaf66fbf8b60264c827f5d38afd33c13801487b9aa8b28209edb`) was a
correct build of an earlier, incompletely-synced state of this repository.

### Correction history (2026-09-08)

1. Transferred accumulated RC12 hardening/blocker/closure-round source
   changes that had not previously reached this repository, and rewrote
   prose throughout comments and documentation to describe engineering
   rationale without naming development tooling.
2. A follow-up review found and corrected three remaining items: a
   second, previously-missed instance of two source comments that
   verbatim-matched community reference code (`main/mtek_spi_runtime.c`);
   `docs/THIRD_PARTY_NOTICES.md` was added disclosing that reference
   project and its license status; and `tools/package_release.py`'s
   packaging manifest, which previously reported a fixed "no VCS"
   placeholder regardless of the real source tree, now detects and
   reports the actual Git commit and dirty/clean status, with tests.
3. A second follow-up review corrected a misdiagnosis from step 2's own
   report (an application-hash change had been incorrectly attributed to
   build-path sensitivity; see the embedded-version-string note above for
   the actual, confirmed cause) and hardened `tools/package_release.py`
   further: it now hard-fails on unusable/malformed Git metadata instead
   of silently reporting it as "no VCS"; it independently verifies, from
   two build-recorded metadata sources, that `--build-dir` was actually
   configured from `--project-root` before trusting either (rejecting an
   unrelated clean repository paired with someone else's build); and every
   source/build-root check now runs before any output-directory creation,
   so a rejected package leaves the requested output location exactly as
   it was found. All three additions have end-to-end tests exercising the
   real packaging entry point.
4. This record itself was updated to the resulting final source commit
   and re-verified artifact hashes below. This is a documentation-only
   change; see "Clean-source verification" for the exact one-file diff
   this update itself makes relative to the reviewed source commit.

See `docs/DECISION_LOG.md` for the itemized technical history. No List A
behavior, protocol value, identifier, or opcode changed at any point in
this correction history.

## Evidence reviewed

The focused final review confirmed the candidate identity, adapter configuration, build timestamps, saved build dry-run, schema comparisons, sanitizer results, and resource evidence. It found no remaining blocker in the reviewed correction and approved proceeding to packaging and hardware validation.

The reported host suite contained 70 tests, with full ASan/UBSan and TSan runs passing. The focused Community/C3 deferred-completion test was independently repeated 100 times under each sanitizer configuration (200 total runs), zero failures. Static resource evidence records 109,144 bytes of free DIRAM and nine selected stack chains passing the project's configured budget check. These figures are not measurements of worst-case runtime heap or hardware task-stack usage.

## Clean-source verification

On 2026-09-09, a fresh clone of this repository's local commit
`9dedbb12cdbd4f977693858e917f37c50ba14a48` (not a working-tree copy) was built
in isolation.

- Both source generators were byte-idempotent, verified independently inside
  the clone.
- The documented source-only host workflow passed 70/70 tests under both the
  default ASan+UBSan configuration and ThreadSanitizer, zero races. The
  focused Community/C3 deferred-completion test additionally passed 100/100
  adversarial repetitions under each configuration.
- A fresh ESP-IDF v6.0.1 ESP32-C6 build succeeded and was independently
  reconfirmed (same build directory, re-run after CMake regeneration); both
  produced the identical application: 1,443,072 bytes, SHA-256
  `00c11f8bb069f840c572df3bd5fed0b82a7a0d507657d6d93b89d2507642f790`.
- The resource gate passed with 109,144 bytes of free DIRAM; the stack-usage
  gate passed 9/9 chains, measured from a separate disposable build directory
  so the reviewed build directory itself was never touched by the
  stack-usage tool.
- A post-build Ninja dry-run on the reviewed build directory scheduled only
  the four expected ESP-IDF bootloader/check wrapper steps and no application
  compilation or relink.
- A complete merged updater package was generated and validated: the
  embedded application segment is byte-for-byte and SHA-256 identical to
  the audited application above at its documented offset (0x010000); the
  MD5 sidecar is exactly 32 uppercase hex bytes with no whitespace; the
  packaging manifest correctly recorded the exact source commit and a
  clean (non-dirty) working tree -- using the same packaging tool that
  built this package, now independently verifying that build/source-root
  binding rather than trusting it.

This record's own update is a documentation-only commit on top of
`9dedbb12cdbd4f977693858e917f37c50ba14a48` -- the exact source commit and
artifact identity above describe that commit, not this record's own later
commit, which touches no build-affecting file.

This establishes source-set completeness, application reproducibility for
this exact reviewed source commit (see the embedded-version-string note
above for why a *different* commit necessarily produces a different
application hash even with unchanged compiled logic), and a validated
updater package. It does not validate hardware behavior.

## Approval limits

- Hardware regression for this exact candidate is not established by the engineering review.
- Earlier reports for RC11 or earlier RC12 binaries do not substitute for this candidate's evidence.
- Community compatibility is limited to implemented and verified interfaces; adapter inclusion alone does not establish compatibility with every community application.
- Publication hygiene must be rechecked after any later file or release change.
- Existing local staging directories are not automatically approved release packages.

Update this status only with evidence identifying the exact artifact tested. Record hardware results and merged-package identity separately.
