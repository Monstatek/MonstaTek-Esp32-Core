# M1 ESP32 release-package contract

The M1 updater consumes a complete merged ESP32-C6 flash image written at
offset `0x000000`; an application-only image is not sufficient.

Every candidate package must contain:

- `MtkCore.bin`, the full merged image.
- `MtkCore.md5`, generated from that exact binary after final packaging.
- `partitions.csv`.
- A merged-image map binding every segment to its exact source file and offset.
- Build identity, ESP-IDF version, binary size, MD5, SHA-256, flash offset,
  hardware-test status, and compatibility notes.

The established layout keeps the partition table at `0x8000` and application at
`0x10000`. Do not move, resize, remove, or rename partitions without a reviewed
migration plan. The package validator must reject missing or mismatched MD5
sidecars, unexpected filenames, incorrect segment mappings, invalid ESP image
headers/checksums, or a package whose embedded segments are not byte-bound to
the reviewed build inputs.

Validate on a computer first, then use the M1 SD-card updater before the Web
Manager path. SD-card recovery must remain available after Web Manager testing.
