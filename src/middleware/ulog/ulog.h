/**
 * @file    ulog.h
 * @brief   ULog library.
 * @version	1.0.0
 * @date    17.04.2025
 * @author  LisumLab
 */

#ifndef ULOG_H
#define ULOG_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdio.h>
#include <string.h>

#include "lfs.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define ULOG_INV 0xFFFFFFFFU

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*! @brief ULog States. */
typedef enum {
	ULOG_STATE_IDLE = 0,
	ULOG_STATE_HEADER_CREATED,
	ULOG_STATE_DEFINITIONS_CREATED,
	ULOG_STATE_WRITING_DATA
} ulog_state_e;

/*! @brief ULog Log Level. */
typedef enum ulog_log_level_e {
	ULOG_LOG_LEVEL_EMERG = '0',
	ULOG_LOG_LEVEL_ALERT = '1',
	ULOG_LOG_LEVEL_CRIT = '2',
	ULOG_LOG_LEVEL_ERR = '3',
	ULOG_LOG_LEVEL_WARNING = '4',
	ULOG_LOG_LEVEL_NOTICE = '5',
	ULOG_LOG_LEVEL_INFO = '6',
	ULOG_LOG_LEVEL_DEBUG = '7'
} ulog_log_level_e;

/*! @brief ULog Message ID. */
typedef uint16_t ulog_msg_id_t;

/*! @brief ULog. */
typedef struct ulog_s {
	uint64_t (*time_us)(void); //!< get device time [us]
	uint64_t sync_time;		   //!< time since the last of sync message

	lfs_t *lfs;		  //!< littlefs
	lfs_file_t *file; //!< littlefs file
	uint16_t msg_cnt; //!< number of subscribed messages

	ulog_state_e state; // state
} ulog_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

/**
 * @brief Start ULog.
 *
 * @param[in] log		ULog instance.
 * @param[in] lfs		LittleFS instance.
 * @param[in] file		LittleFS file.
 * @param[in] time_us	Device time function [us].
 * @return 0 - success, 1 - fail
 */
int32_t ulog_start(ulog_t *log, lfs_t *lfs, lfs_file_t *file,
				   uint64_t (*time_us)());

/**
 * @brief Write ULog information message.
 *
 * @param[in] log		ULog instance.
 * @param[in] key		Key string in form "type name" ("char[value_len] sys").
 * @param[in] key_len	Length of the key.
 * @param[in] value		Data correspoding to the key.
 * @param[in] value_len	Length of the value.
 * @return 0 - success, 1 - fail
 */
int32_t ulog_info(ulog_t *log, const char *key, uint8_t key_len,
				  const char *value, uint32_t value_len);

/**
 * @brief Write ULog parameter message.
 *
 * @param[in] log		ULog instance.
 * @param[in] key		Key string in form "type name" ("char[value_len] sys").
 * 						Data is restricted to int32 and float.
 * @param[in] key_len	Length of the key.
 * @param[in] value		Data correspoding to the key.
 * @return 0 - success, 1 - fail
 */
int32_t ulog_param(ulog_t *log, const char *key, uint8_t key_len,
				   const void *value);

/**
 * @brief Write ULog logged string message.
 *
 * @param[in] log			ULog instance.
 * @param[in] log_level		Logged string level, same as in the Linux kernel.
 * @param[in] message		Message.
 * @param[in] messagE_len	Message length.
 * @return 0 - success, 1 - fail
 */
int32_t ulog_log_string(ulog_t *log, ulog_log_level_e level,
						const char *message, uint16_t message_len);

/**
 * @brief Write synchronization message.
 *
 * @param[in] log	ULog instance.
 * @return 0 - success, 1 - fail
 */
int32_t ulog_sync(ulog_t *log);

/**
 * @brief Stop ULog.
 *
 * @param[in] log ULog instance.
 * @return None
 */
void ulog_stop(ulog_t *log);

#ifdef __cplusplus
}
#endif

#endif /* ULOG_H */