/**
 * @file    ftp_utilities.h
 * @brief   Utility functions for MAVLink FTP protocol handling with LittleFS and SD card/Flash.
 *
 * This header provides low-level helper functions to support MAVLink FTP operations,
 * including message payload preparation, path extraction and parsing, session management,
 * recursive file operations, and CRC32 checksum calculation.
 * These utilities are designed for use in embedded systems with LittleFS or SD-based storage.
 *
 * The functions here primarily aid in:
 * - Packing/unpacking FTP message fields,
 * - Managing FTP session states,
 * - Parsing file and directory paths from FTP payloads,
 * - Safely freeing resources,
 * - Handling filesystem operations and checksum calculations efficiently.
 *
 * Inline functions do not return errors; they operate on buffers or data passed by the caller.
 *
 * @version 1.0.0
 * @date    16.06.2025
 * @author  BetaTehPro
 */

#ifndef FTP_UTILITIES_H
#define FTP_UTILITIES_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string.h>
#include "ftp_common.h"

#include "lfs.h"
#include "mav.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/


/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/


/**
 * @brief Extracts a 32-bit unsigned integer from a 4-byte little-endian buffer.
 *
 * @param data Pointer to 4 bytes of little-endian encoded data.
 * @return uint32_t The reconstructed 32-bit unsigned integer.
 */

static inline __attribute__((always_inline)) uint32_t ftp_extract_u32(const uint8_t *data)
{
    uint32_t extract;
    
    extract = ((uint32_t)data[0] << 0 )|
              ((uint32_t)data[1] << 8 )|
              ((uint32_t)data[2] << 16)|
              ((uint32_t)data[3] << 24);

    return extract;
}


/**
 * @brief Inserts a 32-bit unsigned integer into a 4-byte buffer in little-endian format.
 *
 * @param dest Pointer to the destination buffer (minimum 4 bytes).
 * @param value The 32-bit unsigned integer to encode.
 */

static inline __attribute__((always_inline)) void ftp_insert_u32(uint8_t *dest, 
                                                                 uint32_t value)
{
    dest[0] = (uint8_t)(value >> 0 );
    dest[1] = (uint8_t)(value >> 8 );
    dest[2] = (uint8_t)(value >> 16);
    dest[3] = (uint8_t)(value >> 24);
}

/**
 * @brief Prepares the FTP protocol message payload with opcode, session ID, size, offset, and other header fields.
 *
 * @param data Pointer to the MAVLink FTP protocol message buffer.
 * @param opcode Main FTP operation code.
 * @param request_opcode Specific FTP request code.
 * @param session_id Session identifier.
 * @param size Size of the payload data.
 * @param data_start Index in payload where the data starts.
 * @param offset Offset position in the file/data.
 */

static inline __attribute__((always_inline)) void ftp_payload_prepare(mavlink_file_transfer_protocol_t *data,
                                                                      uint8_t opcode, 
                                                                      uint8_t request_opcode,
                                                                      uint8_t session_id, 
                                                                      uint8_t size,
                                                                      uint8_t data_start, 
                                                                      uint32_t offset)
{
    data->payload[SESSION]        = session_id;
    data->payload[OPCODE]         = opcode;
    data->payload[SIZE]           = size;
    data->payload[REQUEST_OPCODE] = request_opcode;

    data->payload[OFFSET + 0] = (uint8_t)((offset >>  0) & 0xFF); // LSB
    data->payload[OFFSET + 1] = (uint8_t)((offset >>  8) & 0xFF);
    data->payload[OFFSET + 2] = (uint8_t)((offset >> 16) & 0xFF);
    data->payload[OFFSET + 3] = (uint8_t)((offset >> 24) & 0xFF); // MSB

    if ((request_opcode != OP_READ_FILE        && 
         request_opcode != OP_LIST_DIRECTORY   && 
         request_opcode != OP_CALC_FILE_CRC32  &&
         request_opcode != OP_BURST_READ_FILE) || 
         (opcode == OP_RESPONSE_NAK))
    {
        data->payload[DATA_START] = data_start;
        /* Others zero */
        memset(&data->payload[DATA_START + 1], 0, 251 - (DATA_START + 1));
    }
}

/**
 * @brief Sets the sequence number in the FTP protocol message payload.
 *
 * @param data Pointer to the MAVLink FTP protocol message buffer.
 * @param current_seq_number Sequence number to set.
 */

static inline __attribute__((always_inline)) void ftp_set_seq(mavlink_file_transfer_protocol_t *data,
                                                              uint16_t current_seq_number)
{
    data->payload[SEQ_NUMBER + 0] = (uint8_t)((current_seq_number >> 0) & 0xFF);
    data->payload[SEQ_NUMBER + 1] = (uint8_t)((current_seq_number >> 8) & 0xFF);
}

/**
 * @brief Extracts the directory path string from an FTP protocol message payload.
 *
 * Copies up to dirpath_size-1 bytes from the FTP payload and null-terminates the output.
 *
 * @param ftp_msg Pointer to the FTP protocol message.
 * @param dirpath Buffer to receive the extracted directory path string.
 * @param dirpath_size Size of the dirpath buffer.
 */

static inline __attribute__((always_inline)) void ftp_extract_dirpath(const mavlink_file_transfer_protocol_t *ftp_msg, 
                                                                      char *dirpath, 
                                                                      uint16_t dirpath_size)
{
	/* Copy len of payload-a */
	const uint8_t *payload = ftp_msg->payload;

    /* Take the len of payload */
    size_t copy_len = payload[SIZE];
    if (copy_len >= dirpath_size) {
        copy_len = dirpath_size - 1;
    }

    /* Copy string from payload ftp */
    memcpy(dirpath, &payload[DATA_START], copy_len);
    dirpath[copy_len] = '\0';
}


/**
 * @brief Extracts the file path string from an FTP protocol message payload.
 *
 * Copies up to filepath_size-1 bytes from the FTP payload and null-terminates the output.
 *
 * @param ftp_msg Pointer to the FTP protocol message.
 * @param filepath Buffer to receive the extracted file path string.
 * @param filepath_size Size of the filepath buffer.
 */

static inline __attribute__((always_inline)) void ftp_extract_filepath(const mavlink_file_transfer_protocol_t *ftp_msg, 
                                                                       char *filepath, 
                                                                       uint16_t filepath_size)
{
	/* Copy len of payload-a */
	const uint8_t *payload = ftp_msg->payload;

    /* Take the len of payload */
    size_t copy_len = payload[SIZE];
    if (copy_len >= filepath_size) {
        copy_len = filepath_size - 1;
    }

    /* Copy string from payload ftp */
    memcpy(filepath, &payload[DATA_START], copy_len);
    filepath[copy_len] = '\0';
}

/**
 * @brief Checks if the filepath string is empty.
 *
 * @param filepath Pointer to the file path string.
 * @return true if filepath is empty (first character is '\0'), false otherwise.
 */

static inline __attribute__((always_inline)) bool ftp_filepath_is_empty(const uint8_t *filepath) 
{
    return filepath[0] == '\0';
}

/**
 * @brief Extracts the directory path from a full file path by truncating at the last slash '/'.
 *
 * If no slash is found, the directory path is set to an empty string.
 *
 * @param filepath Input full file path string.
 * @param filepath_size Size of the filepath buffer.
 * @param dirpath Output buffer for the directory path.
 * @param dirpath_size Size of the dirpath buffer.
 */

static inline __attribute__((always_inline)) void ftp_get_dirpath_from_filepath(char *filepath, 
                                                                                uint16_t filepath_size, 
                                                                                char *dirpath, 
                                                                                uint16_t dirpath_size)
{
    /* Copy string from filepath to dirpath */
    strncpy(dirpath, filepath, dirpath_size);
    dirpath[dirpath_size - 1] = '\0';

    /* Find last slash and put the end of char there */
    char *last_slash = strrchr(dirpath, '/');
    if (last_slash) {
        *last_slash = '\0';
    } else {
        dirpath[0] = '\0';
    }

}

/**
 * @brief Closes the file associated with a session and marks the session as free.
 *
 * @param lfs Pointer to the LittleFS instance.
 * @param sessions Array of session structures.
 * @param session_id Index of the session to free.
 */

static inline __attribute__((always_inline)) void ftp_free_session(lfs_t* lfs, 
                                                                   session_t* sessions, 
                                                                   int session_id)
{
    /* Goes through every session */
    if (session_id >= 0 && session_id < FTP_MAX_SESSIONS) {
        lfs_file_close(lfs, &sessions[session_id].file);
        sessions[session_id].in_use = false;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* FTP_UTILITIES_H */