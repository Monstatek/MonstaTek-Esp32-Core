#!/usr/bin/env python3
"""Generates the mtek_schema component's C sources from the accepted Task 002
contract's schemas.json (machine-authoritative operation/event schema).

Usage: gen_schema.py --schema <path-to-schemas.json> --out <components/mtek_schema-dir>

This tool is a build-input generator, not a compiled part of the firmware.
It is run once by a maintainer against the accepted contract package and its
output (generated/*.c, include/*.h) is committed as ordinary generated
source -- the firmware build itself never reads schemas.json or needs
network/file access outside this tree.
"""
import argparse
import hashlib
import json
import os

PRIM_C_TYPE = {
    "u8": "uint8_t", "u16": "uint16_t", "u32": "uint32_t", "u64": "uint64_t",
    "i8": "int8_t", "i16": "int16_t", "i32": "int32_t", "i64": "int64_t",
    "bool": "uint8_t",
}
PRIM_FIELD_ENUM = {
    "u8": "MTK_F_U8", "u16": "MTK_F_U16", "u32": "MTK_F_U32", "u64": "MTK_F_U64",
    "i8": "MTK_F_I8", "i16": "MTK_F_I16", "i32": "MTK_F_I32", "i64": "MTK_F_I64",
    "bool": "MTK_F_BOOL",
}


class TypePool:
    """Deduplicates and names every generated C struct type."""

    def __init__(self):
        self.named = {}       # name -> {fields, kind}
        self.order = []
        self.anon_by_hash = {}  # content hash -> name
        self.anon_counter = 0

    def add_named(self, name, fields, kind):
        if name in self.named:
            return name
        self.named[name] = {"fields": fields, "kind": kind}
        self.order.append(name)
        return name

    def add_anon_struct(self, fields, hint):
        key = json.dumps(fields, sort_keys=True)
        h = hashlib.sha1(key.encode()).hexdigest()[:10]
        if h in self.anon_by_hash:
            return self.anon_by_hash[h]
        self.anon_counter += 1
        name = f"mtk_anon_{hint}_{self.anon_counter}_t"
        self.anon_by_hash[h] = name
        self.named[name] = {"fields": fields, "kind": "struct"}
        self.order.append(name)
        return name

    def find_anon_name(self, fields):
        key = json.dumps(fields, sort_keys=True)
        h = hashlib.sha1(key.encode()).hexdigest()[:10]
        return self.anon_by_hash[h]

    def direct_deps(self, name):
        """Named-type dependencies a struct's own field *declarations*
        require to already be fully defined (nested-by-value members)."""
        entry = self.named[name]
        if entry["kind"] == "bytes_typedef":
            return []
        deps = []
        for f in entry["fields"]:
            t = f["type"]
            if t == "ref":
                deps.append(ref_type_name(f["ref"]))
            elif t == "struct":
                deps.append(self.find_anon_name(f["fields"]))
            elif t == "array":
                elem = f["elem"]
                et = elem["type"]
                if et == "ref":
                    deps.append(ref_type_name(elem["ref"]))
                elif et == "struct":
                    deps.append(self.find_anon_name(elem["fields"]))
                elif et == "bytes":
                    deps.append(f"mtk_bytes{elem['max']}_t")
        return deps


def c_field_decl(pool, field, hint):
    """Returns (decl_string, extra_notes) for one struct member declaration."""
    t = field["type"]
    name = field["name"]
    if t in PRIM_C_TYPE:
        return f"    {PRIM_C_TYPE[t]} {name};"
    if t == "mac6":
        return f"    mtk_mac6_t {name};"
    if t == "ipv4":
        return f"    mtk_ipv4_t {name};"
    if t == "bytes":
        return f"    struct {{ uint16_t len; uint8_t data[{field['max']}]; }} {name};"
    if t == "utf8":
        return f"    struct {{ uint16_t len; uint8_t data[{field['max']}]; }} {name};"
    if t == "bytes_fixed":
        return f"    uint8_t {name}[{field['size']}];"
    if t == "ref":
        return f"    {ref_type_name(field['ref'])} {name};"
    if t == "struct":
        sub_name = pool.add_anon_struct(field["fields"], hint + "_" + name)
        return f"    {sub_name} {name};"
    if t == "array":
        elem = field["elem"]
        elem_c, elem_kind = elem_c_type(pool, elem, hint + "_" + name)
        return f"    struct {{ uint32_t count; {elem_c} items[{field['max']}]; }} {name};"
    raise ValueError(f"unhandled field type {t!r} in {hint}.{name}")


def elem_c_type(pool, elem, hint):
    t = elem["type"]
    if t in PRIM_C_TYPE:
        return PRIM_C_TYPE[t], "prim"
    if t == "mac6":
        return "mtk_mac6_t", "mac6"
    if t == "ipv4":
        return "mtk_ipv4_t", "ipv4"
    if t == "ref":
        return ref_type_name(elem["ref"]), "struct"
    if t == "struct":
        name = pool.add_anon_struct(elem["fields"], hint + "_elem")
        return name, "struct"
    if t == "bytes":
        # inline anonymous bytes-wrapper element type
        name = f"mtk_bytes{elem['max']}_t"
        if name not in pool.named:
            pool.named[name] = {"fields": None, "kind": "bytes_typedef", "max": elem["max"]}
            pool.order.append(name)
        return name, "bytes"
    raise ValueError(f"unhandled array element type {t!r} in {hint}")


def ref_type_name(ref):
    return f"mtk_{ref.lower()}_t"


def build_struct(pool, name, fields, kind="struct"):
    return pool.add_named(name, fields, kind)


def gen_struct_c_decl(name, entry):
    if entry["kind"] == "bytes_typedef":
        n = entry["max"]
        return f"typedef struct {{ uint16_t len; uint8_t data[{n}]; }} {name};"


# --- pass 2: descriptor table emission -------------------------------------

def len_prefix_code(lp):
    return {"u8": 1, "u16": 2, None: 0}.get(lp, 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--schema", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    with open(args.schema) as f:
        schema = json.load(f)

    pool = TypePool()

    # register named ref types first (Phase-1 data has no nesting among them)
    for tname, tfields in schema["types"].items():
        build_struct(pool, ref_type_name(tname), tfields, kind="struct")

    # collect every message struct (request/response/each event) per opcode
    messages = []  # list of dict(struct_name, fields, opcode_name, msg_kind)
    for op in schema["opcodes"]:
        opname = op["name"]
        if op.get("request_fields"):
            sn = f"mtk_{opname.lower()}_req_t"
            build_struct(pool, sn, op["request_fields"])
            messages.append((sn, "req", opname))
        if op.get("response_fields"):
            sn = f"mtk_{opname.lower()}_resp_t"
            build_struct(pool, sn, op["response_fields"])
            messages.append((sn, "resp", opname))
        for ev in op.get("events", []):
            evname = ev["name"]
            if ev.get("fields"):
                sn = f"mtk_{evname.lower()}_ev_t"
                build_struct(pool, sn, ev["fields"])
                messages.append((sn, "event:" + evname, opname))

    # Now render every named struct. Anonymous structs discovered while
    # rendering fields must be rendered *before* their containing struct
    # (C requires member types fully defined for struct-by-value members),
    # so we render in dependency order using an explicit worklist that
    # re-walks pool.order as it grows (add_anon_struct appends live).
    def field_decl(field, hint):
        return c_field_decl(pool, field, hint)

    rendered = {}
    idx = 0
    while idx < len(pool.order):
        name = pool.order[idx]
        idx += 1
        entry = pool.named[name]
        if entry["kind"] == "bytes_typedef":
            rendered[name] = gen_struct_c_decl(name, entry)
            continue
        lines = [f"typedef struct {{"]
        for f in entry["fields"]:
            lines.append(field_decl(f, name))
        lines.append(f"}} {name};")
        rendered[name] = "\n".join(lines)

    # Topologically sort so every nested-by-value member type is fully
    # defined before the struct that embeds it (anonymous nested structs
    # are discovered lazily while rendering their *parent*, so pool.order's
    # raw insertion order is parent-before-child and cannot be emitted
    # directly).
    emit_order = []
    visited = set()
    visiting = set()

    def visit(name):
        if name in visited:
            return
        if name in visiting:
            raise ValueError(f"cyclic struct dependency at {name}")
        visiting.add(name)
        for dep in pool.direct_deps(name):
            visit(dep)
        visiting.discard(name)
        visited.add(name)
        emit_order.append(name)

    for name in pool.order:
        visit(name)

    header_lines = []
    header_lines.append("/* AUTO-GENERATED by tools/gen_schema.py from the accepted contract's")
    header_lines.append(" * schemas.json. Do not hand-edit; regenerate instead. */")
    header_lines.append("#pragma once")
    header_lines.append("#include <stdint.h>")
    header_lines.append("")
    header_lines.append("typedef struct { uint8_t b[6]; } mtk_mac6_t;")
    header_lines.append("typedef struct { uint8_t b[4]; } mtk_ipv4_t;")
    header_lines.append("")
    for name in emit_order:
        header_lines.append(rendered[name])
        header_lines.append("")

    pool.order = emit_order

    os.makedirs(os.path.join(args.out, "include"), exist_ok=True)
    os.makedirs(os.path.join(args.out, "generated"), exist_ok=True)
    with open(os.path.join(args.out, "include", "mtek_schema_structs.h"), "w") as f:
        f.write("\n".join(header_lines))

    # ---- field descriptor tables (mtk_field_desc_t[] + mtk_struct_desc_t) ----
    desc_src = []
    desc_src.append("/* AUTO-GENERATED by tools/gen_schema.py. Do not hand-edit. */")
    desc_src.append('#include "mtek_schema_codec.h"')
    desc_src.append('#include "mtek_schema_structs.h"')
    desc_src.append("#include <stddef.h>")
    desc_src.append("")

    struct_desc_emitted = {}

    def emit_field_desc_array(struct_name, fields, arr_c_name):
        lines = [f"static const mtk_field_desc_t {arr_c_name}[] = {{"]
        for f in fields:
            t = f["type"]
            name = f["name"]
            offset = f"offsetof({struct_name}, {name})"
            if t in PRIM_FIELD_ENUM:
                lines.append(f'    {{ "{name}", {PRIM_FIELD_ENUM[t]}, {offset}, 0, 0, 0, NULL, MTK_F_U8 }},')
            elif t == "mac6":
                lines.append(f'    {{ "{name}", MTK_F_MAC6, {offset}, 0, 0, 0, NULL, MTK_F_U8 }},')
            elif t == "ipv4":
                lines.append(f'    {{ "{name}", MTK_F_IPV4, {offset}, 0, 0, 0, NULL, MTK_F_U8 }},')
            elif t == "bytes_fixed":
                lines.append(f'    {{ "{name}", MTK_F_BYTES_FIXED, {offset}, {f["size"]}, 0, 0, NULL, MTK_F_U8 }},')
            elif t in ("bytes", "utf8"):
                lp = len_prefix_code(f.get("len_prefix", "u8"))
                kind = "MTK_F_UTF8" if t == "utf8" else "MTK_F_BYTES"
                lines.append(f'    {{ "{name}", {kind}, {offset}, {f["max"]}, {lp}, 0, NULL, MTK_F_U8 }},')
            elif t == "ref":
                sub = ref_type_name(f["ref"])
                sub_desc = ensure_struct_desc(sub, pool.named[sub]["fields"])
                lines.append(f'    {{ "{name}", MTK_F_STRUCT, {offset}, sizeof({sub}), 0, 0, &{sub_desc}, MTK_F_U8 }},')
            elif t == "struct":
                sub = find_anon_name(f["fields"])
                sub_desc = ensure_struct_desc(sub, pool.named[sub]["fields"])
                lines.append(f'    {{ "{name}", MTK_F_STRUCT, {offset}, sizeof({sub}), 0, 0, &{sub_desc}, MTK_F_U8 }},')
            elif t == "array":
                elem = f["elem"]
                lp = len_prefix_code(f.get("len_prefix", "u8"))
                elem_desc_ptr, elem_size_expr, elem_kind = array_elem_meta(elem)
                lines.append(
                    f'    {{ "{name}", MTK_F_ARRAY, {offset}, {f["max"]}, {lp}, {elem_size_expr}, {elem_desc_ptr}, {elem_kind} }},'
                )
            else:
                raise ValueError(f"unhandled {t}")
        lines.append("};")
        return "\n".join(lines)

    def find_anon_name(fields):
        key = json.dumps(fields, sort_keys=True)
        h = hashlib.sha1(key.encode()).hexdigest()[:10]
        return pool.anon_by_hash[h]

    def array_elem_meta(elem):
        t = elem["type"]
        if t in PRIM_FIELD_ENUM:
            return "NULL", f"sizeof({PRIM_C_TYPE[t]})", PRIM_FIELD_ENUM[t]
        if t == "mac6":
            return "NULL", "sizeof(mtk_mac6_t)", "MTK_F_MAC6"
        if t == "ipv4":
            return "NULL", "sizeof(mtk_ipv4_t)", "MTK_F_IPV4"
        if t == "ref":
            sub = ref_type_name(elem["ref"])
            sub_desc = ensure_struct_desc(sub, pool.named[sub]["fields"])
            return f"&{sub_desc}", f"sizeof({sub})", "MTK_F_STRUCT"
        if t == "struct":
            sub = find_anon_name(elem["fields"])
            sub_desc = ensure_struct_desc(sub, pool.named[sub]["fields"])
            return f"&{sub_desc}", f"sizeof({sub})", "MTK_F_STRUCT"
        if t == "bytes":
            sub = f"mtk_bytes{elem['max']}_t"
            return "NULL", f"sizeof({sub})", "MTK_F_BYTES"
        raise ValueError(f"unhandled array elem {t}")

    def ensure_struct_desc(struct_name, fields):
        if struct_name in struct_desc_emitted:
            return struct_desc_emitted[struct_name]
        arr_name = f"{struct_name}_fields"
        desc_name = f"{struct_name}_desc"
        struct_desc_emitted[struct_name] = desc_name  # reserve to break recursion
        body = emit_field_desc_array(struct_name, fields, arr_name)
        desc_src.append(body)
        desc_src.append(
            f"const mtk_struct_desc_t {desc_name} = {{ {arr_name}, "
            f"sizeof({arr_name})/sizeof({arr_name}[0]), sizeof({struct_name}) }};"
        )
        desc_src.append("")
        return desc_name

    # emit descriptors for all named structs actually used as message bodies
    # or referenced types (recursively pulled in by ensure_struct_desc calls
    # triggered while emitting message field arrays below).
    header2 = ['#pragma once', '#include "mtek_schema_structs.h"', "", "typedef enum {",
               "    MTK_F_U8, MTK_F_U16, MTK_F_U32, MTK_F_U64, MTK_F_I8, MTK_F_I16, MTK_F_I32, MTK_F_I64,",
               "    MTK_F_BOOL, MTK_F_MAC6, MTK_F_IPV4, MTK_F_BYTES, MTK_F_BYTES_FIXED, MTK_F_UTF8,",
               "    MTK_F_ARRAY, MTK_F_STRUCT,",
               "} mtk_field_type_t;", "",
               "typedef struct mtk_struct_desc mtk_struct_desc_t;", "",
               "typedef struct mtk_field_desc {",
               "    const char *name;",
               "    mtk_field_type_t type;",
               "    uint16_t offset;",
               "    uint16_t max;          /* bytes/utf8 max, bytes_fixed size, or array max count */",
               "    uint8_t len_prefix;    /* 0=none(fixed), 1=u8, 2=u16 -- wire length-prefix width */",
               "    uint16_t elem_size;    /* ARRAY only: sizeof one C element */",
               "    const mtk_struct_desc_t *nested; /* STRUCT, or ARRAY-of-STRUCT: element field table */",
               "    mtk_field_type_t elem_type;      /* ARRAY only: element's own field type */",
               "} mtk_field_desc_t;", "",
               "struct mtk_struct_desc {",
               "    const mtk_field_desc_t *fields;",
               "    uint16_t field_count;",
               "    uint16_t struct_size;",
               "};", ""]

    # message struct descriptors, keyed for the opcode registry to reference
    msg_desc_names = {}
    for struct_name, msg_kind, opname in messages:
        fields = pool.named[struct_name]["fields"]
        desc_name = ensure_struct_desc(struct_name, fields)
        msg_desc_names[(opname, msg_kind)] = desc_name

    with open(os.path.join(args.out, "include", "mtek_schema_codec.h"), "w") as f:
        f.write("\n".join(header2))
    with open(os.path.join(args.out, "generated", "mtek_schema_field_tables.c"), "w") as f:
        f.write('#include "mtek_schema_message_descs.h"\n' + "\n".join(desc_src))

    # extern declarations for every struct_desc, so other translation units
    # (opcode registry) can reference them without redefining the tables.
    extern_lines = ["/* AUTO-GENERATED by tools/gen_schema.py. Do not hand-edit. */",
                     "#pragma once", '#include "mtek_schema_codec.h"']
    for desc_name in struct_desc_emitted.values():
        extern_lines.append(f"extern const mtk_struct_desc_t {desc_name};")
    with open(os.path.join(args.out, "include", "mtek_schema_message_descs.h"), "w") as f:
        f.write("\n".join(extern_lines))

    # ---- opcode registry ----
    gen_registry(schema, args.out, msg_desc_names)
    gen_arbiter(schema, args.out)
    gen_constants(schema, args.out)
    print(f"Generated {len(pool.order)} struct types, {len(messages)} message bodies, "
          f"{len(schema['opcodes'])} opcodes.")


def gen_constants(schema, out):
    lines = ["/* AUTO-GENERATED by tools/gen_schema.py. Do not hand-edit. */", "#pragma once", ""]
    lines.append("typedef enum {")
    for name, val in schema["status_codes"].items():
        lines.append(f"    MTK_STATUS_{name} = {val},")
    lines.append("} mtk_status_t;")
    lines.append("")
    lines.append("typedef enum {")
    for name, val in schema["operation_states"].items():
        if name.startswith("_"):
            continue
        lines.append(f"    MTK_OPSTATE_{name} = {val},")
    lines.append("} mtk_operation_state_t;")
    lines.append("")
    lines.append("typedef enum {")
    for name, val in schema["message_classes"].items():
        lines.append(f"    MTK_CLASS_{name} = {val},")
    lines.append("} mtk_message_class_t;")
    lines.append("")
    lines.append("typedef enum {")
    for name in schema["capability_states"]:
        if name.startswith("_") or name == "precedence_order":
            continue
        lines.append(f"    MTK_CAP_{name},")
    lines.append("} mtk_capability_state_t;")
    lines.append("")
    for cid, val in schema["core_budgets"].items():
        if isinstance(val, int):
            lines.append(f"#define MTK_BUDGET_{cid.upper()} {val}")
    lines.append("")
    lines.append("#define MTK_LAB_CONTROLS_ENABLED_DEFAULT 0 /* schemas.json lab_controls_policy */")
    with open(os.path.join(out, "include", "mtek_schema_constants.h"), "w") as f:
        f.write("\n".join(lines))


def gen_arbiter(schema, out):
    classes = schema["arbiter_classes"]
    owner = schema["arbiter_owner_by_class"]
    pairwise = schema["arbiter_pairwise_policy"]
    lines = ["/* AUTO-GENERATED by tools/gen_schema.py. Do not hand-edit. */", "#pragma once", ""]
    lines.append("typedef enum {")
    for c in classes:
        lines.append(f"    MTK_ARB_{c},")
    lines.append("    MTK_ARB_NONE,")
    lines.append("    MTK_ARB_CLASS_COUNT_RAW = MTK_ARB_NONE,")
    lines.append("} mtk_arbiter_class_t;")
    lines.append("")
    lines.append("typedef enum {")
    lines.append("    MTK_RADIO_OWNER_NONE = 0, MTK_RADIO_OWNER_WIFI, MTK_RADIO_OWNER_BLE,")
    lines.append("    MTK_RADIO_OWNER_ESPNOW, MTK_RADIO_OWNER_IEEE802154_RX, MTK_RADIO_OWNER_IEEE802154_TX,")
    lines.append("} mtk_radio_owner_t;")
    lines.append("")
    lines.append("typedef enum { MTK_POLICY_SERIALIZED, MTK_POLICY_CROSS_SUBSYSTEM_BUSY, MTK_POLICY_GUARDED,"
                  " MTK_POLICY_REJECTED, MTK_POLICY_DISABLED, MTK_POLICY_BUSY } mtk_arbiter_policy_t;")
    with open(os.path.join(out, "include", "mtek_arbiter_classes.h"), "w") as f:
        f.write("\n".join(lines))

    def owner_enum(o):
        return "MTK_RADIO_OWNER_" + o.replace("RADIO_OWNER_", "")

    def policy_enum(p):
        p = p.upper()
        if "CROSS_SUBSYSTEM" in p:
            return "MTK_POLICY_CROSS_SUBSYSTEM_BUSY"
        if "SERIALIZED" in p:
            return "MTK_POLICY_SERIALIZED"
        if "GUARDED" in p:
            return "MTK_POLICY_GUARDED"
        if "REJECTED" in p:
            return "MTK_POLICY_REJECTED"
        if "DISABLED" in p:
            return "MTK_POLICY_DISABLED"
        if "BUSY" in p:
            return "MTK_POLICY_BUSY"
        raise ValueError(p)

    src = ["/* AUTO-GENERATED by tools/gen_schema.py. Do not hand-edit. */",
           '#include "mtek_arbiter_classes.h"', "#include <stddef.h>", ""]
    src.append(f"const int mtk_arbiter_class_count = {len(classes)};")
    src.append("const mtk_radio_owner_t mtk_arbiter_owner[] = {")
    for c in classes:
        src.append(f"    [MTK_ARB_{c}] = {owner_enum(owner[c])},")
    src.append("};")
    src.append("")
    src.append("typedef struct { mtk_arbiter_class_t a, b; mtk_arbiter_policy_t policy; } mtk_pair_entry_t;")
    src.append("const mtk_pair_entry_t mtk_arbiter_pairwise[] = {")
    for pair, policy in pairwise.items():
        a, b = pair.split("|")
        src.append(f"    {{ MTK_ARB_{a}, MTK_ARB_{b}, {policy_enum(policy)} }},")
    src.append("};")
    src.append(f"const int mtk_arbiter_pairwise_count = {len(pairwise)};")
    src.append("")
    src.append("/* self-pairs: every active class conflicts with itself (BUSY); reserved classes are DISABLED */")
    reserved = {c for c in classes if owner[c] not in ("RADIO_OWNER_WIFI", "RADIO_OWNER_BLE")}
    src.append("const mtk_pair_entry_t mtk_arbiter_self_pairs[] = {")
    for c in classes:
        p = "MTK_POLICY_DISABLED" if c in reserved else "MTK_POLICY_BUSY"
        src.append(f"    {{ MTK_ARB_{c}, MTK_ARB_{c}, {p} }},")
    src.append("};")
    src.append(f"const int mtk_arbiter_self_pairs_count = {len(classes)};")
    with open(os.path.join(out, "generated", "mtek_arbiter_tables.c"), "w") as f:
        f.write("\n".join(src))


def gen_registry(schema, out, msg_desc_names):
    lines_h = ["/* AUTO-GENERATED by tools/gen_schema.py. Do not hand-edit. */", "#pragma once",
               '#include "mtek_schema_codec.h"', '#include "mtek_schema_constants.h"',
               '#include "mtek_arbiter_classes.h"', "#include <stdint.h>", ""]
    lines_h.append("typedef enum { MTK_LC_SYNCHRONOUS, MTK_LC_ACCEPTED_ASYNC } mtk_lifecycle_t;")
    lines_h.append("")
    lines_h.append("typedef struct {")
    lines_h.append("    const char *name;")
    lines_h.append("    uint16_t service_id;")
    lines_h.append("    uint16_t opcode;")
    lines_h.append("    mtk_lifecycle_t lifecycle;")
    lines_h.append("    mtk_arbiter_class_t resource_class;")
    lines_h.append("    uint8_t no_radio_lease;")
    lines_h.append("    mtk_capability_state_t cap_native;")
    lines_h.append("    mtk_capability_state_t cap_factory_uart;")
    lines_h.append("    mtk_capability_state_t cap_compat_c3;")
    lines_h.append("    uint32_t deadline_default_ms;")
    lines_h.append("    uint32_t deadline_min_ms;")
    lines_h.append("    uint32_t deadline_max_ms;")
    lines_h.append("    uint8_t cancellable;")
    lines_h.append("    uint8_t idempotent;")
    lines_h.append("    const mtk_struct_desc_t *req_desc;")
    lines_h.append("    const mtk_struct_desc_t *resp_desc;")
    lines_h.append("} mtk_opcode_entry_t;")
    lines_h.append("")
    lines_h.append(f"#define MTK_OPCODE_COUNT {len(schema['opcodes'])}")
    lines_h.append("extern const mtk_opcode_entry_t mtk_opcode_table[MTK_OPCODE_COUNT];")
    lines_h.append("const mtk_opcode_entry_t *mtk_opcode_find(uint16_t service_id, uint16_t opcode);")
    lines_h.append("")
    lines_h.append("typedef enum {")
    for op in schema["opcodes"]:
        lines_h.append(f"    MTK_OP_{op['name']},")
    lines_h.append("} mtk_op_index_t;")

    cap_map = {"SUPPORTED": "MTK_CAP_SUPPORTED", "DISABLED": "MTK_CAP_DISABLED",
               "BUSY": "MTK_CAP_BUSY", "UNAVAILABLE": "MTK_CAP_UNAVAILABLE",
               "UNSUPPORTED": "MTK_CAP_UNSUPPORTED"}

    src = ["/* AUTO-GENERATED by tools/gen_schema.py. Do not hand-edit. */",
           '#include "mtek_opcode_registry.h"', '#include "mtek_schema_message_descs.h"',
           '#include "mtek_opcode_overlay.h"',
           "#include <stddef.h>", "", ]
    src.append("const mtk_opcode_entry_t mtk_opcode_table[MTK_OPCODE_COUNT] = {")
    for op in schema["opcodes"]:
        lc = "MTK_LC_ACCEPTED_ASYNC" if op["lifecycle"] == "ACCEPTED_ASYNC" else "MTK_LC_SYNCHRONOUS"
        rc = op.get("resource_class")
        rc_c = f"MTK_ARB_{rc}" if rc else "MTK_ARB_NONE"
        nrl = 1 if op.get("no_radio_lease") else 0
        cap = op["capability_state"]
        meta = op.get("lifecycle_meta", {})
        dd = meta.get("deadline_default_ms") or 0
        dmin = meta.get("deadline_min_ms") or 0
        dmax = meta.get("deadline_max_ms") or 0
        canc = 1 if meta.get("cancellable") else 0
        idem = 1 if meta.get("idempotent") else 0
        req_desc = f"&{msg_desc_names[(op['name'], 'req')]}" if (op['name'], 'req') in msg_desc_names else "NULL"
        resp_desc = f"&{msg_desc_names[(op['name'], 'resp')]}" if (op['name'], 'resp') in msg_desc_names else "NULL"

        def cap_of(key):
            raw = cap[key].split()[0]
            return cap_map[raw]

        src.append(
            f'    {{ "{op["name"]}", {op["service"]}, {op["opcode"]}, {lc}, {rc_c}, {nrl}, '
            f'{cap_of("native")}, {cap_of("factory_uart")}, {cap_of("compat_c3")}, '
            f"{dd}, {dmin}, {dmax}, {canc}, {idem}, {req_desc}, {resp_desc} }},"
        )
    src.append("};")
    src.append("")
    src.append("const mtk_opcode_entry_t *mtk_opcode_find(uint16_t service_id, uint16_t opcode) {")
    src.append("    /* The test-only overlay is")
    src.append("     * consulted first. It is permanently empty in production (no")
    src.append("     * production code ever registers an overlay entry), so this is a")
    src.append("     * zero-iteration no-op there and the behavior is identical to the")
    src.append("     * table scan alone -- see mtek_opcode_overlay.h. */")
    src.append("    const mtk_opcode_entry_t *ov = mtk_opcode_overlay_find(service_id, opcode);")
    src.append("    if (ov) return ov;")
    src.append("    for (unsigned i = 0; i < MTK_OPCODE_COUNT; i++) {")
    src.append("        if (mtk_opcode_table[i].service_id == service_id && mtk_opcode_table[i].opcode == opcode)")
    src.append("            return &mtk_opcode_table[i];")
    src.append("    }")
    src.append("    return NULL;")
    src.append("}")

    with open(os.path.join(out, "include", "mtek_opcode_registry.h"), "w") as f:
        f.write("\n".join(lines_h))
    with open(os.path.join(out, "generated", "mtek_opcode_registry.c"), "w") as f:
        f.write("\n".join(src))


if __name__ == "__main__":
    main()
