idf_component_register(
    SRCS
        "opendroneid.c"
        "wifi.c"
    INCLUDE_DIRS
        "."
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    ODID_DISABLE_PRINTF=1
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=format
)
