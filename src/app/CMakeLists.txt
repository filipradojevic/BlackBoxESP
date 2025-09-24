idf_component_register(
    SRCS "main.c"
         "task_log.c"
         "task_work.c"
    
    PRIV_REQUIRES driver

    INCLUDE_DIRS "."
)
