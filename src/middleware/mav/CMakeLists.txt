if (NOT DEFINED LSM_MESSAGES_DIR)
    set(LSM_MESSAGES_DIR "${PROJECT_DIR}/messages")
endif()

set(MAVLINK_DIR "${CMAKE_CURRENT_LIST_DIR}/mavlink")
# { changed code } generate mavlink outputs into component-specific build folder (writable)
set(MAVLINK_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/mavlink")
set(MAVLINK_DIALECT "lisum")

message(STATUS "PROJECT_DIR = ${PROJECT_DIR}")

set(MAV_DIALECT_XML "${LSM_MESSAGES_DIR}/mavlink/${MAVLINK_DIALECT}.xml")
if (NOT EXISTS "${MAV_DIALECT_XML}")
    message(FATAL_ERROR "mavgen: MAVLink dialect not found at: ${MAV_DIALECT_XML}\nPlace lisum.xml in ${LSM_MESSAGES_DIR}/mavlink or run idf.py with -DLSM_MESSAGES_DIR=/path/to/messages")
endif()

# find python
find_program(PYTHON_EXECUTABLE NAMES python3 python REQUIRED)

# osiguraj da postoji izlazni folder
file(MAKE_DIRECTORY "${MAVLINK_BINARY_DIR}/${MAVLINK_DIALECT}")

execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" "${MAVLINK_DIR}/pymavlink/tools/mavgen.py"
            --lang C --wire-protocol 2.0
            --output "${MAVLINK_BINARY_DIR}"
            "${MAV_DIALECT_XML}"
    RESULT_VARIABLE MAVGEN_RESULT
    OUTPUT_VARIABLE MAVGEN_OUT
    ERROR_VARIABLE MAVGEN_ERR
)

if (NOT MAVGEN_RESULT EQUAL 0)
    message(FATAL_ERROR "mavgen failed (rc=${MAVGEN_RESULT}):\n${MAVGEN_ERR}")
endif()

idf_component_register(
    SRCS
        "mav.c"
        "mavlink_wrapper.c"
        "${MAVLINK_BINARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h"
    
    INCLUDE_DIRS
        "."
        "${MAVLINK_BINARY_DIR}"
        "${MAVLINK_BINARY_DIR}/${MAVLINK_DIALECT}"

    PRIV_REQUIRES
        tl
)
