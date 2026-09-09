# Third-party and community-reference notices

## Community/C3 reference firmware

To establish M1 hardware and wire-protocol interoperability facts for the
Community/C3 compatibility adapter (`components/mtek_transport_spi_compat`),
engineering consulted the community firmware project `bedge117/m1-esp32-brain`
at commit `e515182ad7e2c52689c0979a5e035cd2c28330c6`. This consultation is
disclosed here rather than presented as unconsulted independent derivation.

No substantial source implementation from that project was incorporated into
this tree. A provenance review identified two short source comments in
`main/mtek_spi_runtime.c` that matched that project's wording verbatim; both
were rewritten in independently chosen language describing the same
HANDSHAKE-timing fact (see `docs/DECISION_LOG.md`). No functional code
changed as a result.

No `LICENSE`, `COPYING`, or equivalent license file was present in the
inspected commit or its reachable history at the time of this review. This
document does not assign MIT, GPL, Apache, or any other license to that
project's code, and does not claim any license terms on its behalf. The
absence of a located license file is a fact about what was inspected, not a
statement that the project is unlicensed or license-free.

## ESP-IDF and bundled framework dependencies

This firmware builds against Espressif's ESP-IDF v6.0.1 (Apache License 2.0)
and the framework components it bundles (including FreeRTOS, NimBLE, and
mbedTLS), pulled from the toolchain rather than vendored into this
repository. Their license terms apply as published by their respective
projects and are not restated here.
