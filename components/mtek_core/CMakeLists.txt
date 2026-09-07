idf_component_register(
    SRCS "mtek_core.c" "mtek_async_queue.c"
    INCLUDE_DIRS "include"
    REQUIRES mtek_schema
)
