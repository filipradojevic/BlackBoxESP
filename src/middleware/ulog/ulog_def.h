/**
 * @file    ulog_def.h
 * @brief   ULog library types and definitions.
 * @version	1.0.0
 * @date    17.04.2025
 * @author  LisumLab
 */

#ifndef ULOG_DEF_H
#define ULOG_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* ULog message types */
#define ULOG_DEF_MSG_TYPE_FLAG_BITS 'B'
#define ULOG_DEF_MSG_TYPE_FORMAT_DEFINITION 'F'
#define ULOG_DEF_MSG_TYPE_INFORMATION 'I'
#define ULOG_DEF_MSG_TYPE_MULTI_INFORMATION 'M'
#define ULOG_DEF_MSG_TYPE_PARAMETER 'P'
#define ULOG_DEF_MSG_TYPE_DEFAULT_PARAMETER 'Q'
#define ULOG_DEF_MSG_TYPE_SUBSCRIPTION 'A'
#define ULOG_DEF_MSG_TYPE_UNSUBSCRIPTION 'R'
#define ULOG_DEF_MSG_TYPE_LOGGED_DATA 'D'
#define ULOG_DEF_MSG_TYPE_LOGGED_STRING 'L'
#define ULOG_DEF_MSG_TYPE_TAGGED_LOGGED_STRING 'C'
#define ULOG_DEF_MSG_TYPE_SYNCHRONIZATION 'S'
#define ULOG_DEF_MSG_TYPE_DROPOUT_MARK 'O'

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*! @brief ULog Message Header. */
typedef struct __attribute__((__packed__)) ulog_message_header_t {
	uint16_t msg_size;
	uint8_t msg_type;
} ulog_message_header_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ULOG_DEF_H */