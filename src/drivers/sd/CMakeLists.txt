idf_component_register(
    SRCS "sd.c"
         
    PRIV_REQUIRES driver

    INCLUDE_DIRS "."
)

add_compile_options(
    -Wtype-limits
)
