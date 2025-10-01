/**
 * @file    mav.c
 * @brief   MAVLink library.
 * @version	1.0.0
 * @date    05.02.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <string.h>

#include "mav.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* Track not found */
#define MAV_TRACK_NOT_FOUND 0xFFFFFFFFU
/* Invalid channel */
#define MAV_CHAN_INVALID 0xFF

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* callback on receiving message */
static uint16_t mav_on_recv(void* arg, uint8_t* buffer, uint16_t size);

static uint32_t mav_track_find(mav_t* mav, uint32_t msgid, uint8_t sysid, uint8_t compid);

/*******************************************************************************
 * Code
 ******************************************************************************/

void mav_init(mav_t* mav, uint8_t sysid, uint8_t compid)
{
	mav->sysid = sysid;
	mav->compid = compid;
	mav->link_count = 0;
	mav->track_count = 0;
}

void mav_link_init(mav_link_t* link, uint8_t chan, mav_receive_cb recv_cb, void* recv_arg,
				   tl_t* tl)
{
	link->tl = tl;
	link->chan = chan;

	tl_on_recv_config(link->tl, mav_on_recv, link);

	link->recv_cb = recv_cb;
	link->recv_arg = recv_arg;

	link->mav = NULL;
}

uint32_t mav_link(mav_t* mav, mav_link_t* link)
{
	if (mav->link_count >= MAV_MAX_LINK_CNT || link == NULL)
		return -1;

	mav->link[mav->link_count] = link;
	link->mav = mav;

	return mav->link_count++;
}

uint8_t mav_get_chan(mav_t* mav)
{
	if (mav->link_count)
		return mav->link[0]->chan;
	else
		return MAV_CHAN_INVALID;
}

uint8_t mav_track(mav_t* mav, uint32_t msgid, uint8_t sysid, uint8_t compid)
{
	if (mav->track_count >= MAV_MAX_TRACK_CNT)
		return -1;

	mav_track_t* track;

	track = (mav_track_t*)&mav->track[mav->track_count];

	track->msgid = msgid;
	track->sysid = sysid;
	track->compid = compid;

	return mav->track_count++;
}

void mav_sign(mav_t* mav, const uint8_t* key, mavlink_accept_unsigned_t cb)
{
	mavlink_status_t* status;
	mavlink_signing_t* signing;
	uint8_t chan = mav_get_chan(mav);

	if (chan == MAV_CHAN_INVALID || chan > MAVLINK_COMM_NUM_BUFFERS)
		return;

	/* Signing link_id shall be the same as primary MAVLink channel. */
	signing = &m_mavlink_signing[chan];
	signing->link_id = chan;
	signing->flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
	signing->accept_unsigned_callback = cb;
	memcpy(signing->secret_key, key, sizeof(signing->secret_key));

	for (uint8_t i = 0; i < mav->link_count; i++) {
		status = mavlink_get_channel_status(mav->link[i]->chan);

		status->signing = signing;
		status->signing_streams = &m_mavlink_signing_streams;
	}
}

uint16_t mav_send(mav_t* mav, const mavlink_message_t* msg)
{
	uint16_t size;

	size = mavlink_msg_to_send_buffer(mav->buffer, msg);

	for (uint8_t i = 0; i < mav->link_count; i++) {
		mav_link_t* link;

		link = (mav_link_t*)mav->link[i];

		tl_send(link->tl, mav->buffer, size);
	}

	return size;
}

/****************************** static functions ******************************/

static uint16_t mav_on_recv(void* arg, uint8_t* buffer, uint16_t size)
{
	mavlink_status_t status_instance;			 // lokalna instanca statusa
	mavlink_status_t* status = &status_instance; // pokazivač ka instanci
	mav_link_t* link;
	mav_t* mav;

	link = (mav_link_t*)arg;
	mav = link->mav;

	if (mav == NULL)
		return size;

	for (uint32_t i = 0; i < size; i++) {
		/* parse messages */
		uint8_t ret = mavlink_parse_char(link->chan, buffer[i], &link->msg, status);
		if (!ret) {
			continue;
		}

		uint32_t track_id;

		track_id =
			mav_track_find(mav, link->msg.msgid, link->msg.sysid, link->msg.compid);

		if (track_id == MAV_TRACK_NOT_FOUND)
			continue;

		if (link->recv_cb != NULL) {
			link->recv_cb(mav, &link->msg, link->recv_arg);
		}
	}

	return size;
}

static uint32_t mav_track_find(mav_t* mav, uint32_t msgid, uint8_t sysid, uint8_t compid)
{
	for (uint32_t i = 0; i < mav->track_count; i++) {
		if (mav->track[i].msgid == msgid && mav->track[i].sysid == sysid &&
			mav->track[i].compid == compid)
			return i;
	}

	return MAV_TRACK_NOT_FOUND;
}

/********************************* End Of File ********************************/