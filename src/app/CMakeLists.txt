idf_component_register(
    SRCS   
        "main.c"
        "task_log.c"
        "task_work.c"
    
    PRIV_REQUIRES  
        driver
        sd
        flash
        tl
        udp 
        udp_tl
        lfs
        ulog
        mav
        esp_timer
        ftp
        lfs_utilities
        eth
        fatfs

    INCLUDE_DIRS 
        "."
)
