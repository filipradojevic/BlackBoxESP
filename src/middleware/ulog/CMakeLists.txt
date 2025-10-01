set(ULOG_GEN_DIR "${CMAKE_CURRENT_LIST_DIR}/ulog-gen")
set(ULOG_BINARY_DIR "${CMAKE_BINARY_DIR}/ulog")
set(ULOG_XML "lisum")


# default messages folder -> project_root/messages
if (NOT DEFINED LSM_MESSAGES_DIR)
    set(LSM_MESSAGES_DIR "${PROJECT_DIR}/messages")
endif()

# full path to XML
set(ULOG_XML_PATH "${LSM_MESSAGES_DIR}/mavlink/${ULOG_XML}.xml")
if (NOT EXISTS "${ULOG_XML_PATH}")
    message(FATAL_ERROR "ulog-gen: XML file not found at: ${ULOG_XML_PATH}\n"
        "Place ${ULOG_XML}.xml in ${LSM_MESSAGES_DIR}/mavlink or set LSM_MESSAGES_DIR accordingly.")
endif()

message(STATUS "PROJECT_DIR = ${PROJECT_DIR}")
message(STATUS "Using Python: ${PYTHON_EXECUTABLE}")
message(STATUS "ULOG XML path: ${ULOG_XML_PATH}")

# ensure output dir exists
file(MAKE_DIRECTORY "${ULOG_BINARY_DIR}")

# find python
find_program(PYTHON_EXECUTABLE NAMES python3 python REQUIRED)

# run generator
execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" "${ULOG_GEN_DIR}/ulog_gen.py"
            -o "${ULOG_BINARY_DIR}"
            "${ULOG_XML_PATH}"
    RESULT_VARIABLE ULOG_GEN_RETVAL
    OUTPUT_VARIABLE ULOG_GEN_OUT
    ERROR_VARIABLE  ULOG_GEN_ERR
)

if (NOT ULOG_GEN_RETVAL EQUAL 0)
    message(FATAL_ERROR "ulog-gen failed (rc=${ULOG_GEN_RETVAL}):\n${ULOG_GEN_ERR}")
endif()

# collect generated sources
file(GLOB ULOG_GENERATED_SOURCES "${ULOG_BINARY_DIR}/*.c")
set(ULOG_SRCS ${ULOG_GENERATED_SOURCES} "${CMAKE_CURRENT_LIST_DIR}/ulog.c")

# register ESP-IDF component
idf_component_register(
    SRCS ${ULOG_SRCS}
    INCLUDE_DIRS
        "${ULOG_BINARY_DIR}"
        "."
    PRIV_REQUIRES
        lfs
)
