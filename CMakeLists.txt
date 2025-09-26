cmake_minimum_required(VERSION 3.16)

# --------------------------------------------------------------------------

# Extra components
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_SOURCE_DIR}/src/app" 
    "${CMAKE_SOURCE_DIR}/src/drivers" 
    "${CMAKE_SOURCE_DIR}/src/middleware"
    "${CMAKE_SOURCE_DIR}/src/middleware/tl"
    "${CMAKE_SOURCE_DIR}/src/middleware/tl/udp_tl"
)

add_compile_options(
    -Wno-dev
    -Wno-unused-function
    -Wno-type-limits 
    -Wno-address-of-packed-member
    -Wno-cast-align
    -Wno-shadow
    -Wno-stack-usage
    -Wno-conversion
    -Wno-aggressive-loop-optimizations
)

# Include ESP-IDF project.cmake
include($ENV{IDF_PATH}/tools/cmake/project.cmake)


# --- Build metadata for main.h (ESP32-S3 friendly) -------------------------
string(TIMESTAMP BUILD_UNIX_TIMESTAMP "%s" UTC)

find_program(GIT_EXECUTABLE NAMES git HINTS ENV PATH)
if (GIT_EXECUTABLE)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse --short=8 HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    execute_process(
        COMMAND "${GIT_EXECUTABLE}" describe --tags --abbrev=0
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_TAG
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
endif()

if (NOT GIT_HASH)
    set(GIT_HASH "unknown")
endif()
if (NOT GIT_TAG)
    set(GIT_TAG "v0.0.0")
endif()

# FreeRTOS version
set(OS_VERSION "v0.0.0")
if (EXISTS "${CMAKE_SOURCE_DIR}/src/middleware/FreeRTOS/manifest.yml")
    file(READ "${CMAKE_SOURCE_DIR}/src/middleware/FreeRTOS/manifest.yml" FREERTOS_MANIFEST)
    string(REGEX MATCH "version:[ \t]*\"([^\"]+)\"" _match "${FREERTOS_MANIFEST}")
    if (CMAKE_MATCH_1)
        set(OS_VERSION "${CMAKE_MATCH_1}")
    endif()
elseif(DEFINED ENV{IDF_VER})
    set(OS_VERSION "$ENV{IDF_VER}")
endif()

# LittleFS version
set(LFS_VERSION_BUILD "0")
if (EXISTS "${CMAKE_SOURCE_DIR}/src/middleware/lfs/littlefs/lfs.h")
    file(READ "${CMAKE_SOURCE_DIR}/src/middleware/lfs/littlefs/lfs.h" LFS_HEADER)
    string(REGEX MATCH "#define[ \t]+LFS_VERSION[ \t]+0x([0-9A-Fa-f]+)" _match "${LFS_HEADER}")
    if (CMAKE_MATCH_1)
        string(TOUPPER "${CMAKE_MATCH_1}" LFS_VERSION_BUILD)
    endif()
endif()

message(STATUS "Git Tag           : ${GIT_TAG}")
message(STATUS "Git Commit Hash   : ${GIT_HASH}")
message(STATUS "Build Timestamp   : ${BUILD_UNIX_TIMESTAMP}")
message(STATUS "FreeRTOS OS_VERSION : ${OS_VERSION}")
message(STATUS "LittleFS Version  : ${LFS_VERSION_BUILD}")

# Configure header with build metadata
set(GEN_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/app")

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/config/main.h.in"
    "${GEN_INCLUDE_DIR}/main.h"
    @ONLY
)

# Include generated headers globally
include_directories("${GEN_INCLUDE_DIR}")


project(BlackBoxESP)