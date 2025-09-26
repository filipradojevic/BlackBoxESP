/**
 * @file    udp_tl.h
 * @brief   UDP Transport Layer.
 * @version	1.0.0
 * @date    24.01.2025
 * @author  LisumLab
 */

#ifndef UDP_TL_H
#define UDP_TL_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "../tl/tl_common.h"

#include "../udp/udp.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*! @brief UDP Transport Layer Interface. */
typedef struct udp_tl_t {
	tl_t tl; //!< Transport Layer

	udp_t* udp; //!< UDP Handler

	uint8_t ip[4];		  //!< Track IP
	uint16_t port_local;  //!< Track local port
	uint16_t port_remote; //!< Track remote port
} udp_tl_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

/**
 * @brief Initialize interface.
 *
 * @param[in] tl            UDP Transport layer.
 * @param[in] udp           UDP Device.
 * @param[in] ip            IP address remote device.
 * @param[in] port_local    Local port.
 * @param[in] port_remote   Remote port.
 * @return None
 */
void udp_tl_init(udp_tl_t* tl, udp_t* udp, uint8_t* ip, uint16_t port_local, uint16_t port_remote);

/**
 * @brief Send data from higher level protocols.
 *
 * @param[in] tl    UDP Transport layer.
 * @param[in] data  Transmit data.
 * @param[in] size  Transmit data size.
 * @return Data size on success and -1 on error.
 */
uint16_t udp_tl_send(void* tl, const uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* UDP_TL_H */