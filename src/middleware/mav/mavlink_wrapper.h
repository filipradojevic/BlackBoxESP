/**
 * @file    mavlink_wrapper.h
 * @brief   MAVLink wrapper
 * @version	1.0.0
 * @date    05.02.2025
 * @author  LisumLab
 */

#ifndef MAVLINK_WRAPPER_H
#define MAVLINK_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifdef MAV_CONFIG
#include "mav_config.h"
#endif

/* m_mavlink_status array is defined externally */
#define MAVLINK_EXTERNAL_RX_STATUS
/* m_mavlink_buffer array is defined externally */
#define MAVLINK_EXTERNAL_RX_BUFFER

/* maximum number of communication buffer */
#ifndef MAVLINK_COMM_NUM_BUFFERS
#define MAVLINK_COMM_NUM_BUFFERS 4
#endif

/* maximum number of signing streams */
#ifndef MAVLINK_MAX_SIGNING_STREAMS
#define MAVLINK_MAX_SIGNING_STREAMS 4
#endif

#include "mavlink_types.h"

extern mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
extern mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];

extern mavlink_signing_streams_t m_mavlink_signing_streams;
extern mavlink_signing_t m_mavlink_signing[MAVLINK_COMM_NUM_BUFFERS];

#include "lisum/mavlink.h"

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_WRAPPER_H */