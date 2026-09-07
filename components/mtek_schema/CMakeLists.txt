# AUTO-GENERATED sources under generated/ come from tools/gen_schema.py run
# against the accepted Task 002 contract's schemas.json. Regenerate with:
#   python3 tools/gen_schema.py --schema <path-to-schemas.json> --out components/mtek_schema
idf_component_register(
    SRCS
        "mtek_schema_codec.c"
        "mtek_opcode_overlay.c"
        "generated/mtek_schema_field_tables.c"
        "generated/mtek_opcode_registry.c"
        "generated/mtek_arbiter_tables.c"
    INCLUDE_DIRS "include"
)
