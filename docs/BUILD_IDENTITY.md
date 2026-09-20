# Build identity and validation status

## Identifying a specific build

Do not identify a build from a version string alone. The authoritative record
for any given package is the generated `PACKAGING_MANIFEST.md` inside that
package, produced by `tools/package_release.py` and never hand-edited. It
records the exact source commit, the working tree's clean/dirty state at
packaging time, and the application and merged-image hashes for that build.

A tracked document cannot substitute for it. ESP-IDF embeds a Git-derived
version string in the application image's own descriptor, so the application
hash changes whenever the source commit changes even if no compiled logic
differs. A build's application SHA-256 and its exact source commit must
therefore always be read together, never the hash alone.

Two inputs to build identity are maintained by hand in `main/CMakeLists.txt`,
and both must be bumped in the change that finalizes a candidate:

- `MTK_RELEASE_CANDIDATE` -- wire-visible through `GET_VERSION`. A stale value
  misidentifies the binary to any peer that asks.
- `MTK_RELEASE_EPOCH_S` -- a fixed SOURCE_DATE_EPOCH-style release epoch, never
  derived from wall-clock build time, so every build of an identical source
  tree embeds an identical epoch.

## What engineering verification does and does not establish

The host suite, sanitizer runs, generator idempotence checks, resource-budget
and stack-budget gates establish source-level correctness and build
reproducibility for the exact commit they ran against. They do not establish
hardware behavior.

Specifically:

- Hardware regression for a given candidate is not established by host
  verification. Record hardware results separately, against the exact artifact
  identity the `PACKAGING_MANIFEST.md` names.
- Results for an earlier candidate do not carry forward to a later one.
- Community compatibility extends only to interfaces that are implemented and
  verified. Adapter inclusion alone does not establish compatibility with every
  community application.
- Publication hygiene must be re-checked after any later file or release
  change.
- A local staging directory is not an approved release package.
- An application-only binary is not an M1 updater package; see
  [release-package contract](RELEASE_PACKAGE.md).

## Resource and stack gates

`tools/check_resource_budget.py` reports free DIRAM against a documented
minimum, and `tools/check_stack_budget.py` measures real per-function frame
sizes (`-fstack-usage`) for a hand-traced set of task and callback call chains.
Both are static gates. Neither measures worst-case runtime heap use or actual
hardware task-stack consumption. `check_stack_budget.py` documents its own
scope limits: its chains are hand-traced and hard-coded, so a chain that
changes shape does not update itself, and a function in no chain is not
measured at all.
