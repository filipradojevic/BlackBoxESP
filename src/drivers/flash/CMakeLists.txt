idf_component_register(
    SRCS
        "flash_common.c"
        "flash.c"
        "w25n01gv/w25n01gv.c"
        "gd5f2gq5ue/gd5f2gq5ue.c"

    INCLUDE_DIRS
        "."                  # flash/
        "w25n01gv"           # flash/w25n01gv
        "gd5f2gq5ue"         # flash/gd5f2gq5ue
        "${CMAKE_SOURCE_DIR}/config" #root/config

    PRIV_REQUIRES
        driver
)
