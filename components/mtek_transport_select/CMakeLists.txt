idf_component_register(
    SRCS "mtek_transport_select.c"
    INCLUDE_DIRS "include"
    REQUIRES mtek_transport_spi_native mtek_transport_spi_bedge
)
