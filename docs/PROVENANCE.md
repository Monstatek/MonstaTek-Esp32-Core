# Engineering validation boundary

This file records validation scope only; licensing and attribution decisions are
handled separately from the engineering source set. See
`docs/THIRD_PARTY_NOTICES.md` for community-reference and dependency
attribution.

Portable behavior is covered by the host suite. Target builds verify ESP-IDF
integration, image generation, static resource use, and configured adapters.
Neither class of evidence replaces complete-device hardware regression.

For RC12, hardware-only gaps include worst-case runtime heap and task-stack
high-water measurements, sustained radio coexistence, and complete regression
of the shipped M1 UI/app flows. Record those results against the exact merged
image hash in the candidate release notes.

Comments that cite this document refer to this distinction between inspected or
host-tested behavior and behavior proven on physical hardware.
