idf_component_register(
    SRCS "mtek_system_logic.c"
    INCLUDE_DIRS "include"
    REQUIRES mtek_schema mtek_core mtek_arbiter mtek_router
)
