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
        #udp 
        udp_tl
        lfs
        ulog
        mav
        esp_timer
        ftp
        lfs_utilities
        fatfs
        w5500

        # Ovo govori CMake-u da kompajleru i linkeru doda sve potrebne fajlove iz ovih komponenti.
    REQUIRES 
        # Zvanični Ethernet drajver
        esp_eth 
        # API za mrežne interfejse (IP, DHCP, itd.)
        esp_netif
        # Za SPI komunikaciju
        driver
        # Opšta sistemska podrška
        esp_event
        # Mrežni stek
        lwip

    INCLUDE_DIRS 
        "."
)
