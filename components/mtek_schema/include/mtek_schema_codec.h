#pragma once
#include "mtek_schema_structs.h"

typedef enum {
    MTK_F_U8, MTK_F_U16, MTK_F_U32, MTK_F_U64, MTK_F_I8, MTK_F_I16, MTK_F_I32, MTK_F_I64,
    MTK_F_BOOL, MTK_F_MAC6, MTK_F_IPV4, MTK_F_BYTES, MTK_F_BYTES_FIXED, MTK_F_UTF8,
    MTK_F_ARRAY, MTK_F_STRUCT,
} mtk_field_type_t;

typedef struct mtk_struct_desc mtk_struct_desc_t;

typedef struct mtk_field_desc {
    const char *name;
    mtk_field_type_t type;
    uint16_t offset;
    uint16_t max;          /* bytes/utf8 max, bytes_fixed size, or array max count */
    uint8_t len_prefix;    /* 0=none(fixed), 1=u8, 2=u16 -- wire length-prefix width */
    uint16_t elem_size;    /* ARRAY only: sizeof one C element */
    const mtk_struct_desc_t *nested; /* STRUCT, or ARRAY-of-STRUCT: element field table */
    mtk_field_type_t elem_type;      /* ARRAY only: element's own field type */
} mtk_field_desc_t;

struct mtk_struct_desc {
    const mtk_field_desc_t *fields;
    uint16_t field_count;
    uint16_t struct_size;
};
