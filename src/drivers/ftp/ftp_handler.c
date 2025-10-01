/**
 * @file    ftp_handler.c
 * @brief   MAVLink FTP Command Handler Module
 *
 * This module implements a lightweight and modular handler for processing
 * MAVLink-based FTP operations. It is designed to interact with a LittleFS
 * (lfs) filesystem and manage multiple file sessions in an embedded system
 * environment. Rather than acting as a traditional monolithic driver, this
 * component provides clear, self-contained functions that can be invoked
 * wherever needed in the application, ensuring flexibility and codebase
 * clarity.
 *
 * The module supports all primary FTP operations defined by the MAVLink
 * File Transfer Protocol, such as reading, writing, file creation, deletion,
 * and directory management. It also handles session tracking, response
 * formatting, and sequence number validation.
 *
 * @version	1.0.0
 * @date    20.06.2025
 * @author  BetaTehPro
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "ftp_handler.h"
#include "ftp_helper.h"
#include "ftp_utilities.h"
#include "task_log.h"

/*******************************************************************************6
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static mavlink_file_transfer_protocol_t last_response;
static uint32_t last_offset = 0x00;
static uint16_t current_seq_number_gcs = 0x00;
static uint16_t current_seq_number = 0x00;
static uint16_t last_seq_number_gcs = 0xFFFF;
static uint8_t dir_list_is_currently_open = CLOSED;
static lfs_dir_t dir_list;

/*******************************************************************************
 * Code
 ******************************************************************************/

bool ftp_process_opcode(mavlink_file_transfer_protocol_t* data, lfs_t* lfs,
						session_t* sessions, struct lfs_file_config* file_cfg,
						mav_t* mav_gw_sky_handle)
{

	/* Extract sequentional number */
	current_seq_number_gcs = ((data->payload[SEQ_NUMBER + 0] << 0) |
							  (data->payload[SEQ_NUMBER + 1] << 8));

	/* This is user created function to restart seq number if it's needed */
	if (data->payload[OPCODE] == OP_RESET_SEQ_NUM) {
		last_seq_number_gcs = 0;
		current_seq_number = 0;
		ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_RESET_SEQ_NUM, 0,
							VALIDATE, NONE, 0);
		return true;
	}

	/* Sequentional number protection - Timeot handle */
	if ((current_seq_number_gcs <= last_seq_number_gcs) &&
		(last_seq_number_gcs != 0xFFFF)) {
		memcpy(data, &last_response, sizeof(mavlink_file_transfer_protocol_t));
		return true;
	}

	/* Track opcode and execute the operation */
	switch (data->payload[OPCODE]) {
	case OP_NONE:
		ftp_handle_op_none(data);
		break;

	case OP_TERMINATE_SESSION:
		ftp_handle_op_terminate_session(data, lfs, sessions);
		break;

	case OP_RESET_SESSIONS:
		ftp_handle_op_reset_session(data, lfs, sessions);
		break;

	case OP_LIST_DIRECTORY:
		ftp_handle_op_list_directory(data, lfs, sessions);
		break;

	case OP_OPEN_FILE_RO:
		ftp_handle_op_open_file_ro(data, lfs, file_cfg, sessions);
		break;

	case OP_READ_FILE:
		ftp_handle_op_read_file(data, lfs, sessions);
		break;

	case OP_CREATE_FILE:
		ftp_handle_op_create_file(data, lfs, file_cfg, sessions);
		break;

	case OP_WRITE_FILE:
		ftp_handle_op_write_file(data, lfs, sessions, file_cfg);
		break;

	case OP_REMOVE_FILE:
		ftp_handle_op_remove_file(data, lfs, sessions);
		break;

	case OP_CREATE_DIRECTORY:
		ftp_handle_op_create_directory(data, lfs);
		break;

	case OP_REMOVE_DIRECTORY:
		ftp_handle_op_remove_directory(data, lfs, sessions);
		break;

	case OP_OPEN_FILE_WO:
		ftp_handle_op_open_file_wo(data, lfs, file_cfg, sessions);
		break;

	case OP_TRUNCATE_FILE:
		ftp_handle_op_truncate_file(data, lfs, file_cfg, sessions);
		break;

	case OP_RENAME:
		ftp_handle_op_rename(data, lfs);
		break;

	case OP_CALC_FILE_CRC32:
		ftp_handle_op_calc_file_crc32(data, lfs, file_cfg);
		break;

	case OP_BURST_READ_FILE:
		ftp_handle_op_burst_read_file(data, lfs, file_cfg, sessions,
									  mav_gw_sky_handle);
		break;

	default:
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_NONE, 0, ERROR,
							UNKNOWN_COMMAND, 0);
		break;
	}

	/* Remember last seq number */
	last_seq_number_gcs = current_seq_number_gcs;

	/* MAVLink requires the ground station to receive the next seq number */
	current_seq_number = current_seq_number_gcs + 1;
	data->payload[SEQ_NUMBER + 0] = (uint8_t)((current_seq_number >> 0) & 0xFF);
	data->payload[SEQ_NUMBER + 1] = (uint8_t)((current_seq_number >> 8) & 0xFF);

	/* Save last valid response once after processing */
	ftp_save_last_response(data, &last_response);

	return true;
}

void ftp_handle_op_none(mavlink_file_transfer_protocol_t* data)
{

	/* It's ping message */
	data->payload[OPCODE] = OP_RESPONSE_ACK;
	data->payload[REQUEST_OPCODE] = OP_NONE;

	return;
}

void ftp_handle_op_terminate_session(mavlink_file_transfer_protocol_t* data,
									 lfs_t* lfs, session_t* sessions)
{
	/* 1. Takes ID of a session */
	uint8_t session_id = data->payload[SESSION];

	/* 2. If something bad happend wiht list directory */
	if (dir_list_is_currently_open == OPENED) {
		int err = lfs_dir_close(lfs, &dir_list);
		if (err < LFS_ERR_OK) {
			ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_RESET_SESSIONS, 0,
								ERROR, FAIL, 0);
		}
		dir_list_is_currently_open = CLOSED;
	}

	/* 3. Protection for sessions */
	if (session_id > FTP_MAX_SESSIONS) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_TERMINATE_SESSION,
							session_id, ERROR, NO_SESSION_AVAILABLE, 0);
		return;
	}

	/* 4. Check if it's in use */
	if (sessions[session_id].in_use) {
		ftp_free_session(lfs, sessions, session_id);
		ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_TERMINATE_SESSION,
							session_id, VALIDATE, NONE, 0);
	} else {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_TERMINATE_SESSION,
							session_id, ERROR, INVALID_SESSION, 0);
	}

	return;
}

void ftp_handle_op_reset_session(mavlink_file_transfer_protocol_t* data,
								 lfs_t* lfs, session_t* sessions)
{
	/* 1. Iterate through all available FTP sessions */
	for (int session_id = 0; session_id < FTP_MAX_SESSIONS; session_id++) {
		if (sessions[session_id].in_use) {
			ftp_free_session(lfs, sessions, session_id);
		}
	}

	/* 2. if something bad happend wiht list directory*/
	if (dir_list_is_currently_open == OPENED) {
		int err = lfs_dir_close(lfs, &dir_list);
		if (err < LFS_ERR_OK) {
			ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_RESET_SESSIONS, 0,
								ERROR, FAIL, 0);
		}
		dir_list_is_currently_open = CLOSED;
	}

	ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_RESET_SESSIONS, 0, VALIDATE,
						NONE, 0);
	return;
}

void ftp_handle_op_list_directory(mavlink_file_transfer_protocol_t* data,
								  lfs_t* lfs, session_t* sessions)
{
	char dirpath[FTP_FILE_PATH_SIZE];
	struct lfs_info lfs_info;
	lfs_off_t offset;
	int err;
	bool payload_overflow = false;

	/* 1. Extract directory path and offset from payload*/
	ftp_extract_dirpath(data, dirpath, sizeof(dirpath));
	offset = ftp_extract_u32(&data->payload[OFFSET]);

	/* 2. Reset directory listing if new request or offset changed */
	if (((offset != last_offset) || (offset == 0)) &&
		(dir_list_is_currently_open == OPENED)) {
		lfs_dir_close(lfs, &dir_list);
		dir_list_is_currently_open = CLOSED;
		last_offset = 0;
	}

	/* 3. Validate directory path (reject empty or parent dir "..") */
	if (ftp_filepath_is_empty((uint8_t*)dirpath) ||
		strcmp(dirpath, "..") == 0) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_LIST_DIRECTORY, 0, ERROR,
							FILE_NOT_FOUND, 0);
		return;
	}

	/* 4. Validate directory path length */
	if (strlen(dirpath) >= FTP_FILE_PATH_SIZE) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_LIST_DIRECTORY, 0, ERROR,
							INVALID_DATA_SIZE, 0);
		return;
	}

	/* 5. Open directory if not already opened */
	if (dir_list_is_currently_open == CLOSED) {
		err = lfs_dir_open(lfs, &dir_list, dirpath);
		if (err < 0) {
			last_offset = 0;
			ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_LIST_DIRECTORY, 0,
								ERROR, FAIL, 0);

			return;
		}
		dir_list_is_currently_open = OPENED;
	}

	/* 6. Prepare pointer for writing entries to payload buffer */
	char* entry_ptr = (char*)&data->payload[DATA_START];
	int total_payload = 0;
	lfs_off_t current_offset = offset;

	// Loop through directory entries
	while (1) {
		err = lfs_dir_read(lfs, &dir_list, &lfs_info);

		// End of directory listing
		if (err == LFS_ERR_OK) {
			break;
		}

		// Handle directory read error
		else if (err < LFS_ERR_OK) {
			lfs_dir_close(lfs, &dir_list);
			dir_list_is_currently_open = CLOSED;
			last_offset = 0;
			ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_LIST_DIRECTORY, 0,
								ERROR, FAIL_ERRNO, current_offset);
			return;
		}

		// Format one directory entry:
		// - For directories: 'D' + name + "\0\0"
		// - For files: 'F' + name + '\t' + filesize + "\0\0"
		char one_entry[FTP_FILE_PATH_SIZE + 32];
		char* ptr = one_entry;
		char type = (lfs_info.type == LFS_TYPE_DIR) ? 'D' : 'F';

		*ptr++ = type;
		int name_len = strlen(lfs_info.name);
		memcpy(ptr, lfs_info.name, name_len);
		ptr += name_len;

		if (type == 'F') {
			*ptr++ = '\t';
			ptr += sprintf(ptr, "%lu", (unsigned long)lfs_info.size);
		}

		// Null-terminate with double zero as required by MAVLink FTP
		*ptr++ = '\0';
		*ptr++ = '\0';

		int entry_len = ptr - one_entry;

		// Check if adding this entry exceeds the max payload size
		if (total_payload + entry_len > FTP_MAX_LIST_DIR_PAYLOAD_SIZE) {
			// Mark overflow flag but still add the entry to payload
			payload_overflow = true;
		}

		// Copy the formatted entry into the payload buffer
		memcpy(entry_ptr, one_entry, entry_len);
		entry_ptr += entry_len;
		total_payload += entry_len;
		current_offset++;

		// If payload overflowed, stop processing further entries
		if (payload_overflow) {
			break;
		}
	}

	// If no entries were added, it means end of directory reached
	if (total_payload == 0) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_LIST_DIRECTORY, 0, ERROR,
							END_OF_FILE, offset);

		lfs_dir_close(lfs, &dir_list);
		dir_list_is_currently_open = CLOSED;
		last_offset = 0;
		return;
	}

	// Prepare and send ACK with the accumulated directory entries
	ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_LIST_DIRECTORY, 0,
						total_payload, 0, current_offset);

	// Update last offset to current for next call
	last_offset = current_offset;

	return;
}

void ftp_handle_op_open_file_ro(mavlink_file_transfer_protocol_t* data,
								lfs_t* lfs, struct lfs_file_config* file_cfg,
								session_t* sessions)
{
	char filepath[FTP_FILE_NAME_SIZE];
	char dirpath[FTP_FILE_PATH_SIZE];
	struct lfs_info lfs_info;
	lfs_soff_t file_size;
	int session_id = 0;
	int err;

	/* 1. Extract file path from payload */
	ftp_extract_filepath(data, filepath, sizeof(filepath));

	if (ftp_filepath_is_empty((uint8_t*)filepath)) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_RO, session_id,
							ERROR, INVALID_DATA_SIZE, 0);
		return;
	}

	ftp_get_dirpath_from_filepath(filepath, sizeof(filepath), dirpath,
								  sizeof(dirpath));

	/* 2. Check length of both paths */
	if (strlen(filepath) >= FTP_FILE_NAME_SIZE ||
		strlen(dirpath) >= FTP_FILE_PATH_SIZE) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_RO, 0, ERROR,
							INVALID_DATA_SIZE, 0);
		return;
	}

	/* 3. Handle root directory edge case */
	if (dirpath[0] == '\0') {
		dirpath[0] = '.';
		dirpath[1] = '\0';
	}

	/* 4. Verify directory exists and is valid */
	err = lfs_stat(lfs, dirpath, &lfs_info);
	if (err != LFS_ERR_OK || lfs_info.type != LFS_TYPE_DIR) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_RO, 0, ERROR,
							FILE_NOT_FOUND, 0);
		return;
	}

	/* 5. Verify file exists and is regular */
	err = lfs_stat(lfs, filepath, &lfs_info);
	if (err != LFS_ERR_OK || lfs_info.type != LFS_TYPE_REG) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_RO, 0, ERROR,
							FILE_NOT_FOUND, 0);
		return;
	}

	/* 6. Allocate session */
	session_id = ftp_allocate_session(sessions, filepath);
	if (session_id < 0) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_RO, 0, ERROR,
							NO_SESSION_AVAILABLE, 0);
		return;
	}

	/* Set the mode of the file */
	sessions[session_id].mode = READ_ONLY;

	/* 7. Attempt to open the file in read-only mode */
	err = lfs_file_opencfg(lfs, &sessions[session_id].file, filepath,
						   LFS_O_RDONLY, file_cfg);

	if (err == LFS_ERR_OK) {
		file_size = lfs_file_size(lfs, &sessions[session_id].file);
		if (file_size <= 0) {
			ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_RO, 0,
								ERROR, FAIL_ERRNO, 0);
			return;
		}

		// Respond with file size (inserted at DATA_START)
		ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_OPEN_FILE_RO, session_id,
							sizeof(uint32_t), NONE, 0);

		ftp_insert_u32(&data->payload[DATA_START], file_size);
	} else {
		ftp_free_session(lfs, sessions, session_id);
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_RO, 0, ERROR,
							FAIL, 0);
	}

	return;
}

void ftp_handle_op_read_file(mavlink_file_transfer_protocol_t* data, lfs_t* lfs,
							 session_t* sessions)
{
	char filepath[FTP_FILE_NAME_SIZE];
	struct lfs_info lfs_info;
	lfs_off_t offset;
	uint8_t session_id;
	uint8_t bytes_to_read;
	uint8_t* read_data;
	int read_result;
	int file_size;
	int seek_result;
	int err;

	/* 1. Validate session */
	session_id = data->payload[SESSION];
	if (!sessions[session_id].in_use) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_READ_FILE, session_id,
							ERROR, INVALID_SESSION, 0);
		return;
	}

	/* 2. Extract path from session */
	memcpy(filepath, sessions[session_id].filepath, FTP_FILE_NAME_SIZE);

	/* 3. Ensure the file is a regular file */
	err = lfs_stat(lfs, filepath, &lfs_info);
	if (err != LFS_ERR_OK || lfs_info.type != LFS_TYPE_REG) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_READ_FILE, session_id,
							ERROR, FILE_NOT_FOUND, 0);
		return;
	}

	/* 4. Get current file size */
	file_size = lfs_file_size(lfs, &sessions[session_id].file);
	if (file_size < 0) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_READ_FILE, session_id,
							ERROR, INVALID_DATA_SIZE, 0);
		return;
	}

	/* 5. Extract read offset from payload */
	offset = ftp_extract_u32(&data->payload[OFFSET]);

	seek_result =
		lfs_file_seek(lfs, &sessions[session_id].file, offset, LFS_SEEK_SET);
	if (seek_result < 0) {
		lfs_file_close(lfs, &sessions[session_id].file);
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_READ_FILE, session_id,
							ERROR, END_OF_FILE, 0);
		return;
	}

	/* 6. Verify read permissions */
	if (sessions[session_id].mode != READ_ONLY) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_READ_FILE, session_id,
							ERROR, FILE_PROTECTED, 0);
		return;
	}

	/* 7. Validate read size */
	bytes_to_read = data->payload[SIZE];
	if (bytes_to_read > FTP_MAX_DATA_SIZE) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_READ_FILE, session_id,
							ERROR, INVALID_DATA_SIZE, 0);
		return;
	}

	read_data = &data->payload[DATA_START];

	/* 8. Reading a file */
	read_result = lfs_file_read(lfs, &sessions[session_id].file, read_data,
								bytes_to_read);
	if (read_result < 0) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_READ_FILE, session_id,
							ERROR, FAIL, 0);
		return;
	}

	/* 9. Handle EOF explicitly */
	if ((offset == file_size) && (read_result == 0)) {
		/*end of file*/
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_READ_FILE, session_id,
							ERROR, END_OF_FILE, offset + read_result);
		return;
	}

	/* 10. Prepare success response with read data */
	ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_READ_FILE, session_id,
						read_result, 0, offset);
	return;
}

void ftp_handle_op_create_file(mavlink_file_transfer_protocol_t* data,
							   lfs_t* lfs, struct lfs_file_config* file_cfg,
							   session_t* sessions)
{

	char filepath[FTP_FILE_NAME_SIZE];
	char dirpath[FTP_FILE_PATH_SIZE];
	struct lfs_info lfs_info;
	int session_id = 0;
	int err;

	/* 1. Extract and validate file path from payload */
	ftp_extract_filepath(data, filepath, sizeof(filepath));

	if (ftp_filepath_is_empty((uint8_t*)filepath)) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_CREATE_FILE, session_id,
							ERROR, INVALID_DATA_SIZE, 0);
		return;
	}

	ftp_get_dirpath_from_filepath(filepath, sizeof(filepath), dirpath,
								  sizeof(dirpath));

	/* 2. Validate path lengths */
	if (strlen(filepath) >= FTP_FILE_NAME_SIZE ||
		strlen(dirpath) >= FTP_FILE_PATH_SIZE) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_CREATE_FILE, session_id,
							ERROR, INVALID_DATA_SIZE, 0);
		return;
	}

	/* 3. Handle empty directory path (root path) */
	if (dirpath[0] == '\0') {
		dirpath[0] = '.';
		dirpath[1] = '\0';
	}

	/* 4. Verify that the parent directory exists */
	err = lfs_stat(lfs, dirpath, &lfs_info);
	if (err != LFS_ERR_OK) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_CREATE_FILE, session_id,
							ERROR, FILE_NOT_FOUND, 0);
		return;
	}

	/* 5. Check if the file already exists */
	err = lfs_stat(lfs, filepath, &lfs_info);
	if (err == LFS_ERR_OK && lfs_info.type == LFS_TYPE_REG) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_CREATE_FILE, session_id,
							ERROR, FILE_EXISTS, 0);
		return;
	}

	/* 6. Allocate the session */
	session_id = ftp_allocate_session(sessions, filepath);
	if (session_id < 0) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_CREATE_FILE, session_id,
							ERROR, NO_SESSION_AVAILABLE, 0);
		return;
	}

	sessions[session_id].mode = WRITE_ONLY;

	/* 7.Attempt to create and open the file */
	if (err == LFS_ERR_NOENT) {
		err = lfs_file_opencfg(lfs, &sessions[session_id].file, filepath,
							   LFS_O_CREAT | LFS_O_WRONLY, file_cfg);

		if (err == LFS_ERR_OK) {
			/* File successfully created */
			ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_CREATE_FILE,
								session_id, VALIDATE, NONE, 0);
			return;
		} else {
			ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_CREATE_FILE,
								session_id, ERROR, FAIL, 0);
			return;
		}
	}

	return;
}

void ftp_handle_op_write_file(mavlink_file_transfer_protocol_t* data,
							  lfs_t* lfs, session_t* sessions,
							  struct lfs_file_config* file_cfg)
{

	uint8_t session_id = data->payload[SESSION];

	/* 1. Validate session */
	if (!sessions[session_id].in_use) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_WRITE_FILE, session_id,
							ERROR, INVALID_SESSION, 0);
		return;
	}

	/* 2. Verify session is in write mode */
	if (sessions[session_id].mode != WRITE_ONLY) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_WRITE_FILE, session_id,
							ERROR, FILE_PROTECTED, 0);
		return;
	}

	/* 3. Extract offset (4 bytes, little-endian) */
	lfs_off_t offset = ftp_extract_u32(&data->payload[OFFSET]);

	/* 4. Extract write size and data pointer */
	uint8_t bytes_to_write = data->payload[SIZE];
	uint8_t* write_data = &data->payload[DATA_START];

	/* 5. Perform the write operation */
	int write_result = lfs_file_write(lfs, &sessions[session_id].file,
									  write_data, bytes_to_write);
	if (write_result < 0) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_WRITE_FILE, session_id,
							ERROR, FAIL, 0);
		return;
	}

	/* 6. Send acknowledgment to confirm write success */
	ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_WRITE_FILE, session_id,
						VALIDATE, NONE, offset);
	return;
}

void ftp_handle_op_remove_file(mavlink_file_transfer_protocol_t* data,
							   lfs_t* lfs, session_t* sessions)
{
	char filepath[FTP_FILE_NAME_SIZE];
	char dirpath[FTP_FILE_PATH_SIZE];
	struct lfs_info lfs_info;
	int err;

	/* 1. Extract the full filepath from the payload */
	ftp_extract_filepath(data, filepath, sizeof(filepath));

	/* 2. Validate the filepath (protection against empty or invalid input) */
	if (ftp_filepath_is_empty((uint8_t*)filepath)) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_REMOVE_FILE, 0, ERROR,
							INVALID_DATA_SIZE, 0);
		return;
	}

	ftp_get_dirpath_from_filepath(filepath, sizeof(filepath), dirpath,
								  sizeof(dirpath));

	/* 3. If the file is in root directory, default dirpath to "./" */
	if (dirpath[0] == '\0') {
		dirpath[0] = '.';
		dirpath[1] = '\0';
	}

	/* 4. Validate if directory exists */
	err = lfs_stat(lfs, dirpath, &lfs_info);
	if (err < LFS_ERR_OK) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_REMOVE_FILE, 0, ERROR,
							FAIL_ERRNO, 0);
		return;
	}

	/* 5. Check if file exists */
	err = lfs_stat(lfs, filepath, &lfs_info);
	if (err < LFS_ERR_OK) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_REMOVE_FILE, 0, ERROR,
							FILE_NOT_FOUND, 0);
		return;
	}

	/* 6. Confirm that the target is a file or directory */
	if ((lfs_info.type != LFS_TYPE_REG) && (lfs_info.type != LFS_TYPE_DIR)) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_REMOVE_FILE, 0, ERROR,
							FILE_EXISTS, 0);
		return;
	}

	/* 7. Free sessions related to this file */
	for (int session_id = 0; session_id < FTP_MAX_SESSIONS; session_id++) {
		if (sessions[session_id].in_use &&
			strcmp(sessions[session_id].filepath, filepath) == 0) {
			ftp_free_session(lfs, sessions, session_id);
		}
	}

	/* 8. Perform the file/directory removal */
	if (lfs_info.type == LFS_TYPE_DIR) {
		ftp_remove_recursive(lfs, filepath);
	} else if (lfs_info.type == LFS_TYPE_REG) {
		lfs_remove(lfs, lfs_info.name);
	}

	/* 9. Send ACK or NAK based on the result of the remove operation */
	if (err == LFS_ERR_OK) {
		ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_REMOVE_FILE, 0, VALIDATE,
							NONE, 0);
		return;
	} else {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_REMOVE_FILE, 0, ERROR,
							FAIL, 0);
		return;
	}

	return;
}

void ftp_handle_op_create_directory(mavlink_file_transfer_protocol_t* data,
									lfs_t* lfs)
{
	char dirpath[FTP_FILE_PATH_SIZE];
	struct lfs_info lfs_info;
	int err;

	/* 1. Extract full file path and directory path from the payload */
	ftp_extract_filepath(data, dirpath, sizeof(dirpath));

	/* 2. Protection of and empty payload */
	if (ftp_filepath_is_empty((uint8_t*)dirpath) &&
		(strlen(dirpath) > FTP_FILE_PATH_SIZE)) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_CREATE_DIRECTORY, 0,
							ERROR, INVALID_DATA_SIZE, 0);
		return;
	}

	/* 3. Check if directory already exists */
	err = lfs_stat(lfs, dirpath, &lfs_info);
	if (err == LFS_ERR_OK && lfs_info.type == LFS_TYPE_DIR) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_CREATE_DIRECTORY, 0,
							ERROR, FILE_EXISTS, 0);
		return;
	}

	/* 4. Directory doesn't exist -> try to create it */
	err = lfs_mkdir(lfs, dirpath);
	if (err == LFS_ERR_OK) {
		ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_CREATE_DIRECTORY, 0,
							VALIDATE, NONE, 0);
		return;
	} else {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_CREATE_DIRECTORY, 0,
							ERROR, FAIL, 0);
		return;
	}

	return;
}

void ftp_handle_op_remove_directory(mavlink_file_transfer_protocol_t* data,
									lfs_t* lfs, session_t* sessions)
{
	char dirpath[FTP_FILE_PATH_SIZE];
	struct lfs_info lfs_info;
	lfs_dir_t dir;
	int err;
	int res;

	/* 1. Extract the directory path from the payload */
	ftp_extract_dirpath(data, dirpath, sizeof(dirpath));

	/* 2. Protection of and empty payload */
	if (ftp_filepath_is_empty((uint8_t*)dirpath)) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_REMOVE_DIRECTORY, 0,
							ERROR, INVALID_DATA_SIZE, 0);
		return;
	}

	/* 3. Check if the directory exists */
	err = lfs_stat(lfs, dirpath, &lfs_info);
	if (err < LFS_ERR_OK) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_REMOVE_DIRECTORY, 0,
							ERROR, FILE_NOT_FOUND, 0);
		return;
	}

	/* 4. Ensure that the path points to a directory */
	if (lfs_info.type != LFS_TYPE_DIR) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_REMOVE_DIRECTORY, 0,
							ERROR, FILE_EXISTS, 0);
		return;
	}

	/* 5. If trying to delete the root directory */
	if ((strcmp(dirpath, ".") == 0) || (strcmp(dirpath, "/") == 0)) {

		// Attempt to open the root directory
		err = lfs_dir_open(lfs, &dir, ".");
		if (err != LFS_ERR_OK) {
			ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_REMOVE_DIRECTORY, 0,
								ERROR, END_OF_FILE, 0);
			return;
		}

		while ((res = lfs_dir_read(lfs, &dir, &lfs_info)) > 0) {
			// Don't remove lfs elementary files
			if (strcmp(lfs_info.name, ".") == 0 ||
				strcmp(lfs_info.name, "..") == 0 ||
				strcmp(lfs_info.name, "boot_count") == 0) {
				continue;
			}

			// Remove regular files
			if (lfs_info.type == LFS_TYPE_REG) {
				err = lfs_remove(lfs, lfs_info.name);
				if (err != LFS_ERR_OK) {
					ftp_payload_prepare(data, OP_RESPONSE_NAK,
										OP_REMOVE_DIRECTORY, 0, ERROR,
										FAIL_ERRNO, 0);
				}
			}

			// Recursively remove subdirectories
			else if (lfs_info.type == LFS_TYPE_DIR) {
				ftp_remove_recursive(lfs, lfs_info.name);
			}
		}

		if (res < 0) {
			ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_REMOVE_DIRECTORY, 0,
								ERROR, FAIL, 0);

			lfs_dir_close(lfs, &dir);
			return;
		}

		// Close directory and confirm success
		lfs_dir_close(lfs, &dir);
		ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_REMOVE_DIRECTORY, 0,
							VALIDATE, NONE, 0);

		return;
	} else if (strcmp(dirpath, "..") == 0 ||
			   strcmp(dirpath, "boot_count") == 0) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_REMOVE_DIRECTORY, 0,
							ERROR, FAIL_ERRNO, 0);
		return;
	}

	/* 5. Free any sessions using files within this directory */
	for (int session_id = 0; session_id < FTP_MAX_SESSIONS; session_id++) {
		if (sessions[session_id].in_use) {
			if (strncmp(sessions[session_id].filepath, dirpath,
						sizeof(dirpath)) == 0 &&
				sessions[session_id].filepath[sizeof(dirpath)] == '/') {
				ftp_free_session(lfs, sessions, session_id);
			}
		}
	}

	/* 6. Try to recursively remove the directory */
	err = ftp_remove_recursive(lfs, dirpath);
	if (err == LFS_ERR_OK) {
		ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_REMOVE_DIRECTORY, 0,
							VALIDATE, NONE, 0);
	} else {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_REMOVE_DIRECTORY, 0,
							ERROR, FAIL, 0);
	}

	return;
}

void ftp_handle_op_open_file_wo(mavlink_file_transfer_protocol_t* data,
								lfs_t* lfs, struct lfs_file_config* file_cfg,
								session_t* sessions)
{
	char filepath[FTP_FILE_NAME_SIZE];
	char dirpath[FTP_FILE_PATH_SIZE];
	struct lfs_info lfs_info;
	int session_id = 0;
	int err;

	/* 1. Extract full file path and directory path from the payload */
	ftp_extract_filepath(data, filepath, sizeof(filepath));

	/* 2. Protection of and empty payload */
	if (ftp_filepath_is_empty((uint8_t*)filepath)) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_WO, session_id,
							ERROR, INVALID_DATA_SIZE, 0);
		return;
	}

	ftp_get_dirpath_from_filepath(filepath, sizeof(filepath), dirpath,
								  sizeof(dirpath));

	/* 3. Check length of both paths */
	if (strlen(filepath) >= FTP_FILE_NAME_SIZE ||
		strlen(dirpath) >= FTP_FILE_PATH_SIZE) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_WO, session_id,
							ERROR, INVALID_DATA_SIZE, 0);
		return;
	}

	/* 4. If the file is in root directory, default dirpath to "./" */
	if (dirpath[0] == '\0') {
		dirpath[0] = '.';
		dirpath[1] = '\0';
	}

	/* 5. Verify that the directory exists and is valid */
	err = lfs_stat(lfs, dirpath, &lfs_info);
	if (err != LFS_ERR_OK || lfs_info.type != LFS_TYPE_DIR) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_WO, session_id,
							ERROR, FILE_NOT_FOUND, 0);
		return;
	}

	/* 6. Check if the file exists */
	err = lfs_stat(lfs, filepath, &lfs_info);
	if (err == LFS_ERR_NOENT) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_WO, session_id,
							ERROR, FILE_NOT_FOUND, 0);
		return;
	}

	/* 7. Attempt to allocate a session for file transfer */
	session_id = ftp_allocate_session(sessions, filepath);
	if (session_id < 0) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_WO, session_id,
							ERROR, NO_SESSION_AVAILABLE, 0);
		return;
	}

	sessions[session_id].mode = WRITE_ONLY;

	/* 8. If path is valid and file exists, open it for write-only */
	if (err == LFS_ERR_OK && lfs_info.type == LFS_TYPE_REG) {
		err = lfs_file_opencfg(lfs, &sessions[session_id].file, filepath,
							   LFS_O_WRONLY, file_cfg);

		if (err == LFS_ERR_OK) {
			ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_OPEN_FILE_WO,
								session_id, VALIDATE, NONE, 0);
			return;
		} else {
			ftp_free_session(lfs, sessions, session_id);
			ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_WO,
								session_id, ERROR, FAIL, 0);
			return;
		}
	}

	/* 9. Path exists but is not a regular file */
	ftp_free_session(lfs, sessions, session_id);

	ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_OPEN_FILE_WO, session_id,
						ERROR, FAIL, 0);

	return;
}

void ftp_handle_op_truncate_file(mavlink_file_transfer_protocol_t* data,
								 lfs_t* lfs, struct lfs_file_config* file_cfg,
								 session_t* sessions)
{
	char filepath[FTP_FILE_NAME_SIZE];
	struct lfs_info lfs_info;
	lfs_off_t offset;
	int session_id = 0;
	int err;

	/* 1. Extract the session ID */
	session_id = data->payload[SESSION];

	/* 2. Validate the session */
	if (!sessions[session_id].in_use) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_TRUNCATE_FILE, session_id,
							ERROR, INVALID_SESSION, 0);
		return;
	}

	if (sessions[session_id].mode != WRITE_ONLY) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_TRUNCATE_FILE, session_id,
							ERROR, FILE_PROTECTED, 0);
		return;
	}

	/* 3. Retrieve the file path from the session */
	memcpy(filepath, sessions[session_id].filepath, FTP_FILE_NAME_SIZE);

	/* 4. Check if the file still exists and is a regular file */
	err = lfs_stat(lfs, filepath, &lfs_info);
	if (err != LFS_ERR_OK || lfs_info.type != LFS_TYPE_REG) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_TRUNCATE_FILE, session_id,
							ERROR, FILE_NOT_FOUND, 0);
		return;
	}

	/* 5. Extract 4-byte offset from the payload (little-endian format) */
	offset = ftp_extract_u32(&data->payload[OFFSET]);

	/* 7. Truncate the file at offset */
	err = lfs_file_truncate(lfs, &sessions[session_id].file, offset);
	if (err == LFS_ERR_OK) {
		ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_TRUNCATE_FILE, session_id,
							VALIDATE, NONE, 0);
		return;
	} else {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_TRUNCATE_FILE, session_id,
							ERROR, FAIL, 0);
		return;
	}

	return;
}

void ftp_handle_op_rename(mavlink_file_transfer_protocol_t* data, lfs_t* lfs)
{
	char* old_path = (char*)&data->payload[DATA_START];
	char* new_path = old_path + strlen(old_path) + 1;
	int err;

	/* 1. If old and new path are the same */
	if (strcmp(old_path, new_path) == 0) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_RENAME, 0, ERROR,
							FAIL_ERRNO, 0);
		return;
	}

	/* 2. Validate lengths */
	if (strlen(old_path) == 0 || strlen(new_path) == 0 ||
		strlen(old_path) >= FTP_FILE_PATH_SIZE ||
		strlen(new_path) >= FTP_FILE_PATH_SIZE) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_RENAME, 0, ERROR,
							INVALID_DATA_SIZE, 0);
		return;
	}

	/* 3. Try to rename */
	err = lfs_rename(lfs, old_path, new_path);

	/* 4. Handle error cases */
	if (err == LFS_ERR_NOENT) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_RENAME, 0, ERROR,
							FILE_NOT_FOUND, 0);
		return;
	} else if (err == LFS_ERR_OK) {
		ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_RENAME, 0, VALIDATE, NONE,
							0);
		return;
	} else {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_RENAME, 0, ERROR, FAIL,
							0);
		return;
	}
	return;
}

void ftp_handle_op_calc_file_crc32(mavlink_file_transfer_protocol_t* data,
								   lfs_t* lfs, struct lfs_file_config* file_cfg)
{
	const char* path = (const char*)&data->payload[DATA_START];
	uint32_t crc;
	int crc_err;

	/* 1. Validate the path */
	if (strlen(path) == 0 || strlen(path) >= LFS_NAME_MAX) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_CALC_FILE_CRC32, 0, ERROR,
							INVALID_DATA_SIZE, 0);
		return;
	}

	/* 2. Calculate Mavlink CRC32 */
	crc = ftp_calculate_crc32_for_file(lfs, path, file_cfg, &crc_err);

	/* 3. Validate crc */
	if (crc_err == LFS_ERR_NOENT) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_CALC_FILE_CRC32, 0, ERROR,
							FILE_NOT_FOUND, 0);
		return;
	} else if (crc_err < LFS_ERR_OK) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_CALC_FILE_CRC32, 0, ERROR,
							FAIL, 0);
		return;
	}

	ftp_insert_u32(&data->payload[DATA_START], crc);

	/* 4. Send ACK */
	ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_CALC_FILE_CRC32, 0, VALIDATE,
						NONE, 0);

	return;
}

void ftp_handle_op_burst_read_file(mavlink_file_transfer_protocol_t* data,
								   lfs_t* lfs, struct lfs_file_config* file_cfg,
								   session_t* sessions,
								   mav_t* mav_gw_sky_handle)
{
	char filepath[FTP_FILE_NAME_SIZE];
	struct lfs_info lfs_info;
	lfs_off_t offset;
	uint8_t session_id;
	uint8_t size;
	int seek_result;
	int err;

	/* 1. Validate forwarded session */
	session_id = data->payload[SESSION];
	if (!sessions[session_id].in_use) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_BURST_READ_FILE,
							session_id, ERROR, INVALID_SESSION, 0);
		return;
	}

	/* 2. Validate size */
	size = data->payload[SIZE];

	if (size > FTP_MAX_DATA_SIZE) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_BURST_READ_FILE,
							session_id, ERROR, INVALID_DATA_SIZE, 0);
		return;
	}

	/* 3. Validate if a file exists */
	memcpy(filepath, sessions[session_id].filepath, FTP_FILE_NAME_SIZE);
	err = lfs_stat(lfs, filepath, &lfs_info);
	if (err != LFS_ERR_OK || lfs_info.type != LFS_TYPE_REG) {
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_BURST_READ_FILE,
							session_id, ERROR, FILE_NOT_FOUND, 0);
		return;
	}

	/* 4. Extract 4-byte offset from the payload (little-endian format) */
	offset = ftp_extract_u32(&data->payload[OFFSET]);

	/* 5. Seek this file */
	seek_result =
		lfs_file_seek(lfs, &sessions[session_id].file, offset, LFS_SEEK_SET);
	if ((seek_result < 0) && (seek_result != LFS_ERR_INVAL)) {
		lfs_file_close(lfs, &sessions[session_id].file);
		ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_BURST_READ_FILE,
							session_id, ERROR, FAIL_ERRNO, 0);
		return;
	}

	/* 6. Allocate buffer*/
	uint32_t bytes_to_read = lfs_info.size - offset;
	uint8_t* read_data = &data->payload[DATA_START];
	uint32_t total_read = 0;
	uint32_t chunk_size = 0;

	/* 7. Preparing parameters for mavlink sending */
	static const uint8_t dev_mav_sysid = 0;
	static const uint8_t dev_mav_compid = MAV_COMP_ID_USER3;
	mavlink_message_t tx_msg;

	int read_result = 0;

	/* 8. Send loop */
	while (total_read <= bytes_to_read) {

		/* 7.1. Reading the chunk */
		chunk_size = (bytes_to_read - total_read > size)
						 ? size
						 : (bytes_to_read - total_read);

		read_result = lfs_file_read(lfs, &sessions[session_id].file,
									&read_data[0], chunk_size);

		/* 7.2. Something is wrong */
		if (read_result < 0) {
			ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_BURST_READ_FILE,
								session_id, ERROR, FAIL, 0);
			return;
		} else if (read_result == 0) {
			ftp_payload_prepare(data, OP_RESPONSE_NAK, OP_BURST_READ_FILE,
								session_id, ERROR, END_OF_FILE, total_read);
			data->payload[BURST_COMPLETE] = 1;
			current_seq_number--; // Last message don't go to mavsend and we
								  // increment it before
			/* 7.END -- LAST CHUNK  */
			break;
		} else if (read_result < size) {
			int start_index = DATA_START + read_result + 1;
			memset(&data->payload[start_index], 0, 251 - start_index);
		}

		/* 7.3. Send chunk via MavLink*/
		ftp_payload_prepare(data, OP_RESPONSE_ACK, OP_BURST_READ_FILE,
							session_id, read_result, 0, total_read);

		data->payload[SEQ_NUMBER + 0] =
			(uint8_t)((current_seq_number >> 0) & 0xFF);
		data->payload[SEQ_NUMBER + 1] =
			(uint8_t)((current_seq_number >> 8) & 0xFF);

		mavlink_msg_file_transfer_protocol_encode_chan(
			dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_0, &tx_msg, data);

		mav_send(mav_gw_sky_handle, &tx_msg);

		current_seq_number++;
		total_read += read_result;

		for (volatile int i = 0; i < 20000; i++) {
			__asm__ volatile("" ::: "memory");
		}
	}

	return;
}
