/**
 * @file    ulog.c
 * @brief   ULog library.
 * @version	1.0.0
 * @date    17.04.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "ulog.h"
#include "ulog_def.h"

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
 * Prototypes
 ******************************************************************************/

static int32_t ulog_header(ulog_t *log, uint64_t timestamp);

static int32_t ulog_flag_bit(ulog_t *log);

/*******************************************************************************
 * Code
 ******************************************************************************/

int32_t ulog_start(ulog_t *log, lfs_t *lfs, lfs_file_t *file,
				   uint64_t (*time_us)())
{
	log->lfs = lfs;
	log->file = file;
	log->msg_cnt = 0;
	log->time_us = time_us;

	/* write ulog header section */
	if (ulog_header(log, log->time_us()))
		return 1;

	if (ulog_flag_bit(log))
		return 1;

	log->state = ULOG_STATE_HEADER_CREATED;

	return 0;
}

int32_t ulog_info(ulog_t *log, const char *key, uint8_t key_len,
				  const char *value, uint32_t value_len)
{
	ulog_message_header_t header = {.msg_size =
										sizeof(uint8_t) + key_len + value_len,
									.msg_type = ULOG_DEF_MSG_TYPE_INFORMATION};

	/* check ulog state */
	if (log->state != ULOG_STATE_HEADER_CREATED &&
		log->state != ULOG_STATE_WRITING_DATA)
		return 1;

	/* write message header */
	if (lfs_file_write(log->lfs, log->file, &header, sizeof(header)) < 0)
		return 1;

	/* write key name length */
	if (lfs_file_write(log->lfs, log->file, &key_len, sizeof(key_len)) < 0)
		return 1;

	/* write key name */
	if (lfs_file_write(log->lfs, log->file, key, key_len) < 0)
		return 1;

	/* write key value */
	if (lfs_file_write(log->lfs, log->file, value, value_len) < 0)
		return 1;

	return 0;
}

int32_t ulog_param(ulog_t *log, const char *key, uint8_t key_len,
				   const void *value)
{
	ulog_message_header_t header = {.msg_size = sizeof(uint8_t) + key_len + 4,
									.msg_type = ULOG_DEF_MSG_TYPE_PARAMETER};

	/* check ulog state */
	if (log->state != ULOG_STATE_HEADER_CREATED &&
		log->state != ULOG_STATE_WRITING_DATA)
		return 1;

	/* write message header */
	if (lfs_file_write(log->lfs, log->file, &header, sizeof(header)) < 0)
		return 1;

	/* write key name length */
	if (lfs_file_write(log->lfs, log->file, &key_len, sizeof(key_len)) < 0)
		return 1;

	/* write key name */
	if (lfs_file_write(log->lfs, log->file, key, key_len) < 0)
		return 1;

	/* write key value */
	if (lfs_file_write(log->lfs, log->file, value, 4) < 0)
		return 1;

	return 0;
}

int32_t ulog_log_string(ulog_t *log, ulog_log_level_e level,
						const char *message, uint16_t message_len)
{
	uint8_t log_level = level;
	uint64_t timestamp = log->time_us();
	ulog_message_header_t header = {
		.msg_size = sizeof(uint8_t) + sizeof(uint64_t) + message_len,
		.msg_type = ULOG_DEF_MSG_TYPE_LOGGED_STRING};

	/* check state */
	if (log->state != ULOG_STATE_WRITING_DATA)
		return 1;

	/* write message header */
	if (lfs_file_write(log->lfs, log->file, &header, sizeof(header)) < 0)
		return 1;

	/* write log level */
	if (lfs_file_write(log->lfs, log->file, &log_level, sizeof(log_level)) < 0)
		return 1;

	/* write timestamp [us] */
	if (lfs_file_write(log->lfs, log->file, &timestamp, sizeof(timestamp)) < 0)
		return 1;

	/* write message */
	if (lfs_file_write(log->lfs, log->file, message, message_len) < 0)
		return 1;

	return 0;
}

int32_t ulog_sync(ulog_t *log)
{
	uint8_t sync_magic[] = {0x2FU, 0x73U, 0x13U, 0x20U,
							0x25U, 0x0CU, 0xBBU, 0x12U};
	ulog_message_header_t header = {.msg_size = sizeof(sync_magic),
									.msg_type = ULOG_DEF_MSG_TYPE_SYNCHRONIZATION};

	/* check state */
	if (log->state != ULOG_STATE_WRITING_DATA)
		return 1;

	/* write ulog header */
	if (lfs_file_write(log->lfs, log->file, &header, sizeof(header)) < 0)
		return 1;

	/* write sync magic */
	if (lfs_file_write(log->lfs, log->file, sync_magic, sizeof(sync_magic)) < 0)
		return 1;

	/* save sync time */
	log->sync_time = log->time_us();

	return 0;
}

void ulog_stop(ulog_t *log)
{
	log->state = ULOG_STATE_IDLE;
	log->msg_cnt = 0;
}

/****************************** static functions ******************************/

static int32_t ulog_header(ulog_t *log, uint64_t timestamp)
{
	uint8_t header[16] = {0};

	/* file magic */
	header[0] = 'U';
	header[1] = 'L';
	header[2] = 'o';
	header[3] = 'g';
	header[4] = 0x01U;
	header[5] = 0x12U;
	header[6] = 0x35U;
	/* version */
	header[7] = 0x01U;
	/* timestamp */
	memcpy(&header[8], &timestamp, sizeof(timestamp));

	if (lfs_file_write(log->lfs, log->file, header, sizeof(header)) < 0)
		return 1;

	return 0;
}

static int32_t ulog_flag_bit(ulog_t *log)
{
	ulog_message_header_t header = {.msg_size = sizeof(uint8_t) * 8 +
												sizeof(uint8_t) * 8 +
												sizeof(uint64_t) * 3,
									.msg_type = ULOG_DEF_MSG_TYPE_FLAG_BITS};
	uint8_t compat_flags[8] = {0};
	uint8_t incompat_flags[8] = {0};
	uint64_t append_offsets[3] = {0};

	/* write message header */
	if (lfs_file_write(log->lfs, log->file, &header, sizeof(header)) < 0)
		return 1;

	/* write compatible flags */
	if (lfs_file_write(log->lfs, log->file, compat_flags,
					   sizeof(compat_flags)) < 0)
		return 1;

	/* write incompatible flags */
	if (lfs_file_write(log->lfs, log->file, incompat_flags,
					   sizeof(incompat_flags)) < 0)
		return 1;

	/* write file offset */
	if (lfs_file_write(log->lfs, log->file, append_offsets,
					   sizeof(append_offsets)) < 0)
		return 1;

	return 0;
}

/********************************* End Of File ********************************/