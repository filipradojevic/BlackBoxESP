idf_component_register(
    SRCS
        "udp_tl.c"

    INCLUDE_DIRS
        "." 

    REQUIRES
        tl
        udp    # { changed code } ensure udp headers are available
)