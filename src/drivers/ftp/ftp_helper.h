/**
 * @file    ftp_helper.h
 * @brief   Helper functions and utilities for MAVLink FTP protocol handling with LittleFS and SD card.
 *
 * This header provides essential helper functions for managing FTP sessions,
 * preparing and parsing MAVLink FTP protocol messages, file and directory operations,
 * and CRC calculations on files stored in the LittleFS filesystem, typically
 * used with SD cards in embedded systems.
 *
 * It includes utilities for:
 * - Saving and retransmitting FTP responses,
 * - Preparing FTP message payloads,
 * - Extracting file and directory paths from messages,
 * - Managing FTP sessions,
 * - Recursive file/directory removal,
 * - Computing CRC32 checksums for files.
 *
 * Designed to support robust FTP communication over MAVLink in resource-constrained environments.
 *
 * @version 1.0.0
 * @date    16.06.2025
 * @author  BetaTehPro
 */

#ifndef FTP_HELPER_H
#define FTP_HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string.h>
#include "ftp_common.h"


/* Middlewares */
#include "lfs.h"
#include "mav.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


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
 * @brief Saves the last processed FTP response and its sequence number.
 *
 * This function extracts the sequence number from the provided MAVLink FTP
 * message and stores it along with a copy of the entire message. This saved
 * response can be reused later if a duplicate message with the same sequence
 * number is received, enabling retransmission of the last valid response.
 *
 * @param data Pointer to the MAVLink FTP message containing the response to save.
 * @param last_seq_number Pointer to the variable where the last sequence number will be stored.
 * @param last_response Pointer to the buffer where the last response message will be copied.
 */

void ftp_save_last_response(mavlink_file_transfer_protocol_t *data, mavlink_file_transfer_protocol_t *last_response);


/** 
* @brief This implement logic fot time out on blackbox side (only for op = burst read file)
*
* This function serves to resend the last packet containing the burst_complete = 1 flag, 
* so that the ground station can be certain that 
* the data transmission has been successfully received.
*
* @param queue_member It reads and copies the data in case a response from the drone has been received.
* @param mav_gw_sky_handle Handler to sending to GW Sky via Mavlink
* @param tx_msg Message to send
* @param time2try Resend value
* @param delay Delay beetween 2 messages [ms]
*/

void ftp_timeout(QueueSetHandle_t queue_member, mav_t *mav_gw_sky_handle, mavlink_message_t *tx_msg, uint8_t time2try, uint8_t delay);

/**
 * @brief Allocates a free session and optionally assigns a file path to it.
 *
 * This function scans through the session array to find a free session slot.
 * Once found, it marks the session as in use and copies the provided file path
 * into the session's filepath buffer, ensuring null-termination.
 *
 * @param sessions  Array of session_t structures representing active sessions.
 * @param filepath  Optional file path to assign to the allocated session; can be NULL.
 * @return int      Index of the allocated session on success, or -1 if no free session is available.
 */

int ftp_allocate_session(session_t* sessions, char *filepath);

/**
 * @brief Recursively removes a file or directory and all its contents from the filesystem.
 *
 * @param lfs  Pointer to the LittleFS instance.
 * @param path Path to the file or directory to be removed.
 * @return int 0 on success, or a negative error code on failure.
 */

int ftp_remove_recursive(lfs_t *lfs, char *path);

/**
 * @brief Calculates CRC32 checksum for a given file using LittleFS.
 *
 * @param lfs       Pointer to the LittleFS instance.
 * @param path      Path to the file for which CRC32 should be calculated.
 * @param cfg       Optional file configuration (can be NULL if not used).
 * @param err_code  Pointer to an integer to store the error code.
 *                  0 on success, negative value on error.
 * @return uint32_t Computed CRC32 value, or 0 if an error occurs.
 */

uint32_t ftp_calculate_crc32_for_file(lfs_t *lfs,
                                      const char *path,   
                                      const struct lfs_file_config *cfg, 
                                      int *err_code);

#ifdef __cplusplus
}
#endif

#endif /* FTP_HELPER_H */