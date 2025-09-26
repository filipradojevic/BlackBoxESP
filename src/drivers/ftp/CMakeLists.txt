idf_component_register(
    SRCS 
        "ftp_handler.c"
        "ftp_helper.c"
         
    PRIV_REQUIRES 
        driver
        lfs 
        mav 
        

    INCLUDE_DIRS "."
)

