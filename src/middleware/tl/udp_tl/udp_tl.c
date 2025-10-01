/**
 * @file    udp_tl.c
 * @brief   UDP Transport Layer.
 * @version	1.0.1
 * @date    07.02.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <string.h>

#include "udp_tl.h"

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

static int udp_tl_on_recv_wrap(udp_t *udp, uint8_t *data, uint16_t size,
							   void *tl);

/*******************************************************************************
 * Code
 ******************************************************************************/

void udp_tl_init(udp_tl_t *tl, udp_t *udp, uint8_t *ip, uint16_t port_local,
				 uint16_t port_remote)
{
	udp_tl_t *udp_tl;

	udp_tl = (udp_tl_t *)tl;

	/* set connection information */
	udp_tl->udp = udp;
	memcpy(udp_tl->ip, ip, 4);
	udp_tl->port_local = port_local;
	udp_tl->port_remote = port_remote;

	/* set send callback */
	udp_tl->tl.send = udp_tl_send;

	/* link receive callback to udp callbacks */
	udp_track(udp, ip, port_local, udp_tl_on_recv_wrap, tl);
}

uint16_t udp_tl_send(void *tl, const uint8_t *data, uint16_t size)
{
	udp_tl_t *udp_tl;

	udp_tl = (udp_tl_t *)tl;

	return udp_send(udp_tl->udp, udp_tl->ip, udp_tl->port_remote, data, size);
}

/****************************** static functions ******************************/

static int udp_tl_on_recv_wrap(udp_t *udp, uint8_t *data, uint16_t size,
							   void *tl)
{
	udp_tl_t *udp_tl;

	udp_tl = (udp_tl_t *)tl;

	if (udp_tl->tl.on_recv != NULL)
		udp_tl->tl.on_recv(udp_tl->tl.on_recv_arg, data, size);

	return 1;
}

/********************************* End Of File ********************************/