idf_component_register(
    SRCS 
        "lfs_flash.c"
        "lfs_sd.c"

    PRIV_REQUIRES 
        driver
        lfs
        sd
        flash

    INCLUDE_DIRS "."
)
