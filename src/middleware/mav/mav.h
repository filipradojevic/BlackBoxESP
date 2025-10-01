/**
 * @file    mav.h
 * @brief   MAVLink library.
 * @version	1.0.0
 * @date    05.02.2025
 * @author  LisumLab
 */

#ifndef MAV_H
#define MAV_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "../tl/tl_common.h"
#include "mavlink_wrapper.h"

#ifdef MAV_CONFIG
#include "mav_config.h"
#endif

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* size of internal array which will hold serialized message */
#ifndef MAV_BUFFER_SIZE
#define MAV_BUFFER_SIZE MAVLINK_MAX_PACKET_LEN
#endif

#ifndef MAV_MAX_LINK_CNT
#define MAV_MAX_LINK_CNT 4
#endif

#ifndef MAV_MAX_TRACK_CNT
#define MAV_MAX_TRACK_CNT 8
#endif

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/* forward declaration */
typedef struct mav_t mav_t;

/**
 * @brief   MAVLink receive callback.
 *
 * @param[in] mav   MAVLink handle.
 * @param[in] msg   Received MAVLink message.
 * @param[in] arg   User receive callback argument.
 * @return None.
 */
typedef void (*mav_receive_cb)(mav_t* mav, mavlink_message_t* msg, void* arg);

/*! @brief MAVLink link */
typedef struct mav_link_t {
	/* MAVLink message */
	mavlink_message_t msg;
	/* transport layer */
	tl_t* tl;
	/* MAVLink channel */
	uint8_t chan;

	/* recieve callback */
	mav_receive_cb recv_cb;
	void* recv_arg;

	/* MAVLink handle */
	mav_t* mav;
} mav_link_t;

/*! @brief MAVLink track */
typedef struct mav_track_t {
	/* track message ID */
	uint32_t msgid;
	/* track system ID */
	uint8_t sysid;
	/* track component ID */
	uint8_t compid;
} mav_track_t;

/*! @brief MAVLink handle */
typedef struct mav_t {
	/* internal buffer used for serialization */
	uint8_t buffer[MAV_BUFFER_SIZE];

	/* MAVLink message links */
	mav_link_t* link[MAV_MAX_LINK_CNT];
	uint8_t link_count;

	/* MAVLink message tracks */
	mav_track_t track[MAV_MAX_TRACK_CNT];
	uint32_t track_count;

	/* Device system ID */
	uint8_t sysid;
	/* Device component ID */
	uint8_t compid;
} mav_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

/**
 * @brief   Initialize MAVLink.
 *
 * @param[in] mav       MAVLink handle.
 * @param[in] sysid     ID of this system.
 * @param[in] compid    ID of this component (e.g. 200 for IMU).
 * @return None.
 */
void mav_init(mav_t* mav, uint8_t sysid, uint8_t compid);

/**
 * @brief   Initialize MAVLink link.
 *
 * @param[in] link      MAVLink link handle.
 * @param[in] chan      MAVLink Channel (e.g. MAVLINK_COMM_0).
 * @param[in] recv_cb   Receive callback.
 * @param[in] recv_arg  Receive callback argument.
 * @param[in] tl        Transport Layer.
 * @return None.
 */
void mav_link_init(mav_link_t* link, uint8_t chan, mav_receive_cb recv_cb,
				   void* recv_arg, tl_t* tl);

/**
 * @brief   Link MAVLink link to MAVLink handle.
 *
 * @param[in] mav   MAVLink handle.
 * @param[in] link  MAVLink link handle.
 * @return Link ID in case link was a success, or -1 in case link fails.
 */
uint32_t mav_link(mav_t* mav, mav_link_t* link);

/**
 * @brief   Get MAVLink handle primary channel.
 * @note    This represents MAVLink channel of the first link established
 *          successfully.
 *
 * @param[in] mav   MAVLink handle.
 * @return  Primary MAVLink channel if MAVLink handle contains valid link,
 *          or -1 in case there is none.
 */
uint8_t mav_get_chan(mav_t* mav);

/**
 * @brief   Setup MAVLink handle message signing.
 *
 * @param[in] mav   MAVLink handle.
 * @param[in] key   MAVLink secret key.
 * @param[in] cb    Callback for accepting unsigned messages.
 * @return None.
 */
void mav_sign(mav_t* mav, const uint8_t* key, mavlink_accept_unsigned_t cb);

/**
 * @brief   Track MAVLink message.
 *
 * @param[in] mav       MAVLink handle.
 * @param[in] msgid     MAVLink message ID of tracked message.
 * @param[in] sysid     MAVLink system ID of tracked message.
 * @param[in] compid    MAVLink component ID of tracked message.
 * @return Track Count if tracking was a success, or -1 in case track fails.
 */
uint8_t mav_track(mav_t* mav, uint32_t msgid, uint8_t sysid, uint8_t compid);

/**
 * @brief Send MAVLink message via all connected links.
 *
 * @param[in] mav   MAVLink handle.
 * @param[in] msg   MAVLink message.
 * @return Transmitted message size.
 */
uint16_t mav_send(mav_t* mav, const mavlink_message_t* msg);

#ifdef __cplusplus
}
#endif

#endif /* MAV_H */