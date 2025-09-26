/**
 * @file    ftp_handler.h
 * @brief   FTP protocol operation handlers for MAVLink file transfer service.
 *
 * This header defines the API for handling MAVLink FTP commands related to
 * file and directory management on LittleFS-based storage. It includes
 * function prototypes for processing FTP opcodes such as file open, read,
 * write, create, remove, rename, directory listing, session management,
 * CRC calculations, and burst reads.
 *
 * The handlers parse incoming FTP messages, perform filesystem operations,
 * manage FTP sessions, and prepare appropriate FTP responses.
 *
 * These functions enable reliable, efficient file transfers and remote
 * filesystem manipulation over the MAVLink protocol.
 *
 * @version 1.0.0
 * @date    20.06.2025
 * @author  BetaTehPro
 */

#ifndef FTP_HANDLER_H
#define FTP_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "ftp_common.h"
#include "mav.h"
#include "lfs.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define FTP_MAX_LIST_DIR_PAYLOAD_SIZE 150

/*******************************************************************************
 * Typedefs
 ******************************************************************************/
typedef enum {
    READ_ONLY      = 0x00, 
    WRITE_ONLY     = 0x10  
} ftp_filemode_e;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
/**
 * @brief Processes a MAVLink FTP command based on the opcode in the payload.
 *
 * This function decodes the opcode from the received MAVLink FTP message and
 * dispatches it to the appropriate `ftp_handle_op_*` handler function.
 * The `data` structure is modified in-place to contain the response that
 * will later be sent back over MAVLink.
 *
 * In case of a repeated sequence number, the logic may choose to resend
 * the last saved response. The function `ftp_save_last_response()` should be
 * called only after a valid response has been prepared.
 *
 * @param data Pointer to the MAVLink FTP message to be processed.
 * @param lfs Pointer to the LittleFS file system instance.
 * @param sessions Pointer to the array of FTP session structures.
 * @param file_cfg Pointer to the file configuration structure (used for read/write ops).
 * @param mav_gw_sky_handle Pointer to the MAVLink gateway communication handle.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */
bool ftp_process_opcode(mavlink_file_transfer_protocol_t *data, 
                        lfs_t *lfs,
                        session_t *sessions, 
                        struct lfs_file_config *file_cfg,
                        mav_t *mav_gw_sky_handle);

/**
 * @brief Handles the FTP OP_NONE (ping) operation.
 *
 * This function is called when an FTP message with the OP_NONE operation code is received,
 * which acts as a "ping" to check if the FTP service is responsive.
 * It responds by setting:
 * - OPCODE to OP_RESPONSE_ACK (acknowledgment)
 * - REQUEST_OPCODE to OP_NONE (to indicate this is a response to OP_NONE)
 *
 * @param data Pointer to the mavlink_file_transfer_protocol_t structure containing the FTP message.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */
void ftp_handle_op_none(mavlink_file_transfer_protocol_t* data);


/**
 * @brief Handles the FTP OP_TERMINATE_SESSION operation.
 *
 * This function processes a request to terminate an active file transfer session.
 * It reads the session ID from the payload and checks whether the session is in use.
 * 
 * - If the session is active, it frees the session resources using `ftp_free_session`
 *   and sends an ACK response.
 * - If the session is invalid or not active, it sends a NAK response with an error code.
 *
 * @param data     Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs      Pointer to the littlefs context.
 * @param sessions Pointer to the array of session structures.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */
void ftp_handle_op_terminate_session(mavlink_file_transfer_protocol_t* data, 
                                     lfs_t* lfs, 
                                     session_t* sessions);


/**
 * @brief Handles the FTP OP_RESET_SESSIONS operation.
 *
 * This function resets all active file transfer sessions. It iterates through all
 * available session slots and frees any session that is currently in use by calling
 * `ftp_free_session()`.
 *
 * After clearing the sessions, it prepares an ACK response to confirm the operation.
 *
 * @param data     Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs      Pointer to the littlefs context.
 * @param sessions Pointer to the array of session structures.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */
void ftp_handle_op_reset_session(mavlink_file_transfer_protocol_t* data, 
                                 lfs_t* lfs, 
                                 session_t* sessions);


/**
 * @brief Handles the FTP OP_LIST_DIRECTORY operation.
 *
 * This function is intended to list the contents of a directory on the file system.
 * It will parse the target path from the incoming FTP payload and attempt to open
 * the directory using the LittleFS API.
 *
 * For each valid entry in the directory (files and subdirectories), the function
 * will prepare and send directory entries back to the client over multiple FTP
 * response messages, depending on the number of entries and payload size limitations.
 *
 * The response will include:
 * - Entry name (file or directory)
 * - Entry type (file or folder)
 * - Optional metadata (size, permissions, etc.)
 *
 * If the directory cannot be opened or the path is invalid, a NAK response with
 * an appropriate error code will be returned.
 *
 * @param data     Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs      Pointer to the LittleFS context.
 * @param sessions Pointer to the array of session structures.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */
void ftp_handle_op_list_directory(mavlink_file_transfer_protocol_t* data, 
                                  lfs_t* lfs, 
                                  session_t* sessions);


/**
 * @brief Handles the FTP OP_OPEN_FILE_RO operation.
 *
 * This function is intended to open a file in read-only mode on the file system.
 * It will extract the file path from the incoming FTP payload and attempt to open
 * the file using the LittleFS API.
 *
 * If the file is successfully opened:
 * - A free session slot will be allocated.
 * - A session ID will be assigned to track the open file.
 * - An ACK response will be sent with the session ID in the payload.
 *
 * If the file does not exist, cannot be opened, or no session is available,
 * a NAK response with an appropriate error code will be returned.
 *
 * @param data     Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs      Pointer to the LittleFS context.
 * @param file_cfg Pointer to the file configuration structure (used for read/write ops).
 * @param sessions Pointer to the array of session structures.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */
void ftp_handle_op_open_file_ro(mavlink_file_transfer_protocol_t *data, 
                                lfs_t *lfs, 
                                struct lfs_file_config* file_cfg,
                                session_t* sessions);


/**
 * @brief Handles the FTP OP_READ_FILE operation.
 *
 * This function is intended to read a portion of data from a file that has
 * previously been opened in read-only mode.
 * 
 * It will:
 * - Extract the session ID and offset from the FTP payload.
 * - Validate the session and ensure it is active and associated with an open file.
 * - Read up to the maximum allowed number of bytes from the specified offset using LittleFS.
 * - Fill the FTP payload with the read data and send an ACK response.
 * 
 * If the session is invalid, closed, or if the read fails (e.g., offset out of bounds),
 * the function will send a NAK response with the appropriate error code.
 *
 * @param data     Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs      Pointer to the LittleFS context.
 * @param sessions Pointer to the array of session structures.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */
void ftp_handle_op_read_file(mavlink_file_transfer_protocol_t* data, 
                             lfs_t* lfs, 
                             session_t* sessions);


/**
 * @brief Handles the FTP OP_CREATE_FILE operation.
 *
 * This function creates a new file on the filesystem and allocates a session for writing to it.
 * The process includes path validation, checking for the existence of the parent directory,
 * and ensuring the file does not already exist.
 *
 * Function flow:
 * 1. Extracts the full file path and corresponding directory path from the FTP payload.
 * 2. Validates the length of the file and directory paths.
 * 3. Handles root-level creation by setting the directory path to ".".
 * 4. Verifies that the parent directory exists and is valid.
 * 5. Checks if a file with the same name already exists; if so, returns FILE_EXISTS.
 * 6. Attempts to allocate a new session for the file transfer.
 * 7. If the file doesn't exist, attempts to create it using LittleFS.
 *    - If successful, responds with ACK.
 *    - On failure, frees the session and responds with a NAK.
 *
 * Responds with:
 * - ACK on successful file creation and session allocation.
 * - NAK with appropriate error code on any failure (invalid path, directory not found,
 *   file already exists, no session available, or file creation error).
 *
 * @param data      Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs       Pointer to the LittleFS context.
 * @param file_cfg  Pointer to the LittleFS file configuration structure.
 * @param sessions  Pointer to the array of session structures.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */
void ftp_handle_op_create_file(mavlink_file_transfer_protocol_t* data, 
                               lfs_t* lfs, 
                               struct lfs_file_config* file_cfg, 
                               session_t* sessions);


/**
 * @brief Handles the FTP OP_WRITE_FILE operation.
 *
 * This function writes a chunk of data to a file that has previously been opened
 * in write mode via a valid FTP session. It performs validation of the session,
 * file state, and offset, then writes the requested data into the file using LittleFS.
 *
 * Expected payload format:
 * - SESSION       → ID of the session being used
 * - SIZE          → Number of bytes to write
 * - OFFSET        → File offset at which to start writing
 * - DATA_START+   → Actual data to be written
 *
 * Function flow:
 * 1. Extracts and validates the session ID.
 * 2. Verifies that the session is active and the file exists.
 * 3. Checks that the file path length is within limits.
 * 4. Seeks to the correct offset in the file.
 * 5. Writes the specified number of bytes to the file.
 * 6. Prepares an ACK response if successful, otherwise returns a NAK with an appropriate error code.
 *
 * Possible NAK reasons:
 * - INVALID_SESSION if the session ID is not valid.
 * - FILE_NOT_FOUND if the file no longer exists or is not a regular file.
 * - INVALID_DATA_SIZE if the file path exceeds the allowed size.
 * - END_OF_FILE if the seek operation fails (e.g. offset out of bounds).
 * - FAIL for any other write-related error.
 *
 * @param data     Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs      Pointer to the LittleFS context.
 * @param sessions Pointer to the array of session structures.
 * @param file_cfg Pointer to the file configuration structure (used for read/write ops).
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */
void ftp_handle_op_write_file(mavlink_file_transfer_protocol_t* data, 
                              lfs_t* lfs, 
                              session_t* sessions,
                              struct lfs_file_config *file_cfg);


/**
 * @brief Handles the FTP OP_REMOVE_FILE operation.
 *
 * This function deletes a regular file from the file system. It first verifies that
 * the target file exists and is a valid regular file. If any active FTP sessions
 * are associated with the file, they are closed before attempting deletion.
 *
 * Function flow:
 * 1. Extracts the full file path from the incoming payload.
 * 2. Checks whether the file exists using `lfs_stat`.
 * 3. Verifies that the target is a regular file (not a directory).
 * 4. Iterates through all sessions and frees any session associated with this file.
 * 5. Attempts to remove the file using `lfs_remove`.
 * 6. Sends an ACK if successful; otherwise, a NAK with an error code is sent.
 *
 * Possible NAK reasons:
 * - FILE_NOT_FOUND: File does not exist in the filesystem.
 * - FAIL: Target is not a regular file or deletion failed.
 *
 * @param data     Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs      Pointer to the LittleFS context.
 * @param sessions Pointer to the array of session structures.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */
void ftp_handle_op_remove_file(mavlink_file_transfer_protocol_t* data, 
                               lfs_t* lfs, 
                               session_t* sessions);


/**
 * @brief Handles the FTP OP_CREATE_DIRECTORY operation.
 *
 * This function attempts to create a new directory on the filesystem.
 * It first checks if the directory already exists, and if so, responds with an error.
 * If the directory does not exist, it attempts to create it using the LittleFS API.
 *
 * Function flow:
 * 1. Extracts the full directory path from the FTP payload.
 * 2. Checks if the directory already exists.
 *    - If it exists, sends a NAK response with FILE_EXISTS error.
 * 3. Attempts to create the directory.
 *    - On success, sends an ACK response.
 *    - On failure, sends a NAK response with FAIL error.
 *
 * @param data Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs  Pointer to the LittleFS context.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */
void ftp_handle_op_create_directory(mavlink_file_transfer_protocol_t* data, 
                                    lfs_t* lfs);


/**
 * @brief Handles the FTP OP_REMOVE_DIRECTORY operation.
 *
 * This function removes a directory and all its contents recursively from the filesystem.
 * It validates that the target path exists and is a directory.
 * Before removal, it frees any active sessions that are accessing files within the directory.
 *
 * Function flow:
 * 1. Extracts the directory path from the FTP payload.
 * 2. Checks if the directory exists.
 * 3. Confirms that the path is a directory.
 * 4. Frees sessions that are associated with files inside the directory.
 * 5. Recursively removes the directory and all its contents.
 * 6. Sends ACK on success or NAK with error on failure.
 *
 * Possible NAK reasons:
 * - FILE_NOT_FOUND if the directory does not exist.
 * - FAIL if the path is not a directory or removal fails.
 *
 * @param data     Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs      Pointer to the LittleFS context.
 * @param sessions Pointer to the array of session structures.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */

void ftp_handle_op_remove_directory(mavlink_file_transfer_protocol_t* data, 
                                    lfs_t* lfs, 
                                    session_t* sessions);


/**
 * @brief Handles the FTP OP_OPEN_FILE_WO operation.
 *
 * This function opens an existing file in write-only mode.
 * It performs validation on the provided file path and directory,
 * allocates a session for file transfer, and opens the file using LittleFS.
 *
 * Function flow:
 * 1. Extracts the full file path and directory path from the FTP payload.
 * 2. Validates the lengths of the paths.
 * 3. Verifies that the directory exists and is a valid directory.
 * 4. Checks if the target file exists.
 * 5. Allocates a file transfer session.
 * 6. Opens the file for write-only access if it exists and is a regular file.
 * 7. Sends an ACK response with the session ID on success.
 * 8. Sends a NAK response with appropriate error code on failure.
 *
 * @param data      Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs       Pointer to the LittleFS context.
 * @param file_cfg  Pointer to the LittleFS file configuration structure.
 * @param sessions  Pointer to the array of session structures.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */

void ftp_handle_op_open_file_wo(mavlink_file_transfer_protocol_t* data, 
                                lfs_t* lfs, 
                                struct lfs_file_config* file_cfg, 
                                session_t* sessions);


/**
 * @brief Handles the FTP OP_TRUNCATE_FILE operation.
 *
 * This function truncates (resizes) an existing file to a specified offset.
 * It validates the file path, checks that the file exists and is not a directory,
 * verifies the session, opens the file in write-only mode, and performs the truncation.
 *
 * Steps:
 * 1. Extract the file path from the FTP payload.
 * 2. Validate the length of the file path.
 * 3. Check if the file exists and is a regular file (not a directory).
 * 4. Extract the truncation offset from the payload.
 * 5. Verify that the session ID is valid and active.
 * 6. Open the file for writing.
 * 7. Truncate the file at the given offset.
 * 8. Prepare an ACK response if successful, or NAK with an error code if failed.
 *
 * @param data      Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs       Pointer to the LittleFS context.
 * @param file_cfg  Pointer to the LittleFS file configuration.
 * @param sessions  Pointer to the array of session structures.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */

void ftp_handle_op_truncate_file(mavlink_file_transfer_protocol_t* data, 
                                 lfs_t* lfs, 
                                 struct lfs_file_config* file_cfg, 
                                 session_t* sessions);



/**
 * @brief Handles the FTP OP_RENAME operation.
 *
 * This function renames a file or directory on the filesystem.
 * It extracts the old and new paths from the payload, validates them,
 * and attempts to rename the item using LittleFS. It handles errors
 * such as identical paths, invalid path lengths, file not found, or
 * other failures.
 *
 * Steps:
 * 1. Extract old and new file paths from the FTP payload.
 * 2. Check if the old and new paths are identical.
 * 3. Validate the lengths of both paths.
 * 4. Attempt to rename the file/directory in LittleFS.
 * 5. Prepare an ACK response if successful, or NAK with an appropriate error code if failed.
 *
 * @param data  Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs   Pointer to the LittleFS context.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */
void ftp_handle_op_rename(mavlink_file_transfer_protocol_t* data, 
                          lfs_t* lfs);


/**
 * @brief Handles the FTP OP_CALC_FILE_CRC32 operation.
 *
 * This function calculates the CRC32 checksum of a specified file.
 * It validates the file path length, performs the CRC32 calculation,
 * handles possible errors, and writes the resulting checksum into
 * the FTP payload before sending a response.
 *
 * Steps:
 * 1. Validate the length of the file path extracted from the FTP payload.
 * 2. Calculate the CRC32 checksum of the file using a helper function.
 * 3. Check for errors such as file not found or calculation failure.
 * 4. Write the calculated CRC32 checksum into the payload.
 * 5. Prepare an ACK response if successful, or NAK with an error code if failed.
 *
 * @param data      Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs       Pointer to the LittleFS context.
 * @param file_cfg  Pointer to the LittleFS file configuration.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */
void ftp_handle_op_calc_file_crc32(mavlink_file_transfer_protocol_t* data, 
                                   lfs_t* lfs, 
                                   struct lfs_file_config* file_cfg);

/**
 * @brief Handles the FTP OP_BURST_READ_FILE operation.
 *
 * This function performs a burst read of file data from an open read-only session
 * using LittleFS. It is optimized for high-speed data transmission by reading a 
 * large portion of the file in one go, within the constraints of the FTP payload size.
 *
 * The function expects that a file has already been opened in read-only mode 
 * via a valid session ID. It reads from the file starting at a given offset 
 * and sends the data in the response payload.
 *
 * Function flow:
 * 1. Extracts the session ID and offset from the FTP message payload.
 * 2. Validates the session ID and ensures the session is in use.
 * 3. Reads as many bytes as possible from the file (limited by payload size).
 * 4. Writes the read data into the outgoing FTP payload.
 * 5. If the end of file is reached during read, sets the BURST_COMPLETE flag.
 * 6. Sends an ACK response with the data and session information.
 * 7. If an error occurs, sends a NAK response with the appropriate error code.
 *
 * Payload format:
 * - SESSION       → ID of the active session.
 * - OFFSET        → File offset from which to start reading.
 * - DATA_START+   → Buffer where file data is copied into the payload.
 *
 * Possible NAK reasons:
 * - INVALID_SESSION if the session is invalid or not open.
 * - END_OF_FILE if the offset is beyond the end of file.
 * - FAIL if read fails for any other reason.
 *
 * @param data              Pointer to the incoming/outgoing MAVLink FTP message.
 * @param lfs               Pointer to the LittleFS context.
 * @param file_cfg          Pointer to the LittleFS file configuration structure.
 * @param sessions          Pointer to the array of session structures.
 * @param mav_gw_sky_handle Pointer to the MAVLink gateway/sky handler used to send burst responses.
 * @retval true            If operation was successfully handled.
 * @retval false           If an error occurred.
 */

void ftp_handle_op_burst_read_file(mavlink_file_transfer_protocol_t* data, 
                                   lfs_t* lfs, 
                                   struct lfs_file_config* file_cfg,
                                   session_t* sessions, 
                                   mav_t* mav_gw_sky_handle);

/**
 * @brief Handles the custom FTP OP_RESET_SEQ_NUM operation.
 *
 * This is a user-defined extension to the standard MAVLink FTP protocol,
 * used for testing purposes, synchronization after timeouts, or recovery
 * from out-of-sync sequence numbers.
 *
 * It resets both the current sequence number (used by the target device)
 * and the last known sequence number from the ground control station (GCS).
 *
 * @param data Pointer to the MAVLink FTP message structure (used for both input and response).
 * @param current_seq_number Pointer to the target's current sequence number counter.
 * @param last_seq_number_gcs Pointer to the last known sequence number received from the GCS.
 */

void ftp_handle_custom_reset_seq_num(mavlink_file_transfer_protocol_t* data,
                                     uint32_t* current_seq_number,
                                     uint16_t* last_seq_number_gcs);

#ifdef __cplusplus
}
#endif

#endif /* FTP_HANDLER_H */