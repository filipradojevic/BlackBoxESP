/**
 * @file    ftp_common.h
 * @brief   Common definitions, types, and enumerations used across the FTP subsystem.
 * @version 1.0.0
 * @date    2025-06-20
 * @author  BetaTehPro
 *
 * This header provides common constants, data structures, and enumerations 
 * for implementing the MAVLink FTP protocol subsystem. It defines session 
 * management structures, FTP payload layout, error codes, and command opcodes 
 * used throughout the FTP handler and related modules.
 *
 * The definitions here are designed to facilitate file transfer operations over 
 * MAVLink, including session control, file I/O operations, directory listing, 
 * and error reporting.
 */

#ifndef FTP_COMMON_H
#define FTP_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/* Middlewares */
#include "lfs.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FTP_MAX_SESSIONS         20
#define FTP_FILE_PATH_SIZE       110
#define FTP_FILE_NAME_SIZE       118
#define FTP_MAX_DATA_SIZE        239

#define CRC32_POLYNOM_REFLECTED  0xEDB88320U 
#define CRC32_INIT               0x0 

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

typedef struct {
    lfs_file_t file;                                /**< LFS file object for the session */
    bool in_use;                                    /**< Session usage flag */
    char filepath[FTP_FILE_NAME_SIZE];              /**< Filepath associated with session */
    uint8_t mode;                                   /**< File open mode */
} session_t;

/*******************************************************************************
 * Enumerations
 ******************************************************************************/

/**
 * @brief MAVLink FTP payload offset enumeration.
 */
typedef enum {
    SEQ_NUMBER           = 0,   /**< Sequence number (2 bytes) */
    SESSION              = 2,   /**< Session ID */
    OPCODE               = 3,   /**< Operation code */
    SIZE                 = 4,   /**< Payload size */
    REQUEST_OPCODE       = 5,   /**< Request opcode for ACK/NAK */
    BURST_COMPLETE       = 6,   /**< Burst complete flag */
    PADDING              = 7,   /**< Reserved padding byte */
    OFFSET               = 8,   /**< Offset for file operations (4 bytes) */
    DATA_START           = 12   /**< Start of data section */
} mavlink_ftp_payload_e;

/**
 * @brief Result of payload size validation.
 */
typedef enum {
    VALIDATE = 0,
    ERROR    = 1
} mavlink_ftp_size_validate_e;

/**
 * @brief Stataus of currently open directory
 */
typedef enum {
    CLOSED = 0,
    OPENED = 1
} dir_list_status_e;



/**
 * @brief NAK response error codes.
 */
typedef enum {
    NONE                 = 0,  /**< No error */
    FAIL                 = 1,  /**< Unknown failure */
    FAIL_ERRNO           = 2,  /**< Failure with errno in payload */
    INVALID_DATA_SIZE    = 3,  /**< Invalid payload size */
    INVALID_SESSION      = 4,  /**< Session is not open */
    NO_SESSION_AVAILABLE = 5,  /**< All sessions in use */
    END_OF_FILE          = 6,  /**< Offset past end of file */
    UNKNOWN_COMMAND      = 7,  /**< Unrecognized opcode */
    FILE_EXISTS          = 8,  /**< File or directory already exists */
    FILE_PROTECTED       = 9,  /**< File or directory is write-protected */
    FILE_NOT_FOUND       = 10  /**< File or directory not found */
} mavlink_ftp_nak_e;

/**
 * @brief FTP command opcodes.
 */
typedef enum {
    OP_NONE              = 0x00, /**< No operation */
    OP_TERMINATE_SESSION = 0x01, /**< Terminate session */
    OP_RESET_SESSIONS    = 0x02, /**< Reset all sessions */
    OP_LIST_DIRECTORY    = 0x03, /**< List contents of directory */
    OP_OPEN_FILE_RO      = 0x04, /**< Open file in read-only mode */
    OP_READ_FILE         = 0x05, /**< Read file */
    OP_CREATE_FILE       = 0x06, /**< Create file */
    OP_WRITE_FILE        = 0x07, /**< Write file */
    OP_REMOVE_FILE       = 0x08, /**< Remove file */
    OP_CREATE_DIRECTORY  = 0x09, /**< Create directory */
    OP_REMOVE_DIRECTORY  = 0x0A, /**< Remove directory */
    OP_OPEN_FILE_WO      = 0x0B, /**< Open file in write-only mode */
    OP_TRUNCATE_FILE     = 0x0C, /**< Truncate file to zero length */
    OP_RENAME            = 0x0D, /**< Rename file or directory */
    OP_CALC_FILE_CRC32   = 0x0E, /**< Calculate CRC32 of file */
    OP_BURST_READ_FILE   = 0x0F, /**< Burst-read file (multiple packets) */
    OP_RESET_SEQ_NUM     = 0X10, /**< Reset last seq number in system */
    OP_RESPONSE_ACK      = 0x80, /**< ACK response */
    OP_RESPONSE_NAK      = 0x81  /**< NAK response */
} ftp_opcode_e;


#ifdef __cplusplus
}
#endif

#endif /* FTP_COMMON_H */
