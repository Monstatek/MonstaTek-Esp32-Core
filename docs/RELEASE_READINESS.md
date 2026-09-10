# Release readiness

Status: preparation in progress; hardware validation pending.

The organization repository began at `994e036547220943803c959e4e89319559db9ab3`.
Its source tree matches the previously verified source tree, but its commit
identity differs. Earlier build evidence does not identify an artifact built
from this repository's new history.

## Evidence requirements

- Freeze the source revision before producing the candidate. Record its complete
  commit ID and working-tree status in the package manifest.
- Record toolchain and configuration identity, build exit status, warnings,
  applicable test results, resource measurements, and reproducibility evidence.
- Verify the complete merged image and its sidecar against the release-package
  contract. Publish the partition CSV and segment map with the manifest.
- Keep hardware status `NOT_YET_HARDWARE_VALIDATED` until results are recorded
  for that exact image. Review source checks and hardware results separately.
- Prepare release notes with known limitations, compatibility scope, artifact
  identity, and recovery documentation. An empty result is not a passing result.
- Create the release tag on the source revision used for the approved artifact.
  Attach that exact artifact and its matching metadata. Do not rebuild after
  hardware validation and assume the replacement has inherited the result.

## Existing Desktop candidate

The previously supplied `rc12-pcap1` Desktop candidate was built from
`89ece90ec182b644e52e44f98b90e350b984b4ef` in the original repository.
Its merged-image SHA-256 is
`A29179F49972B11F958E0C50D8617F0FC6686963E0EC036F6C7C5F0247C48756`.
It must retain that identity. It is not an artifact of the new initial commit.

## Publication

Prepare the source and metadata before changing repository visibility. A public
source repository and a hardware-approved binary release are separate milestones.
Do not label an unvalidated candidate as a stable release.
