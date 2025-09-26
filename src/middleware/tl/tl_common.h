/**
 * @file    tl_common.h
 * @brief   Transport Layers common types and definitions.
 * @version	1.0.0
 * @date    24.01.2025
 * @author  LisumLab
 */

#ifndef TL_COMMON_H
#define TL_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*! @brief callback to be made by transport layer on receive */
typedef uint16_t (*tl_on_recv_t)(void *tl, uint8_t *data, uint16_t size);

/*! @brief send data to transport layer for processing */
typedef uint16_t (*tl_send_t)(void *tl, const uint8_t *data, uint16_t size);

/*! @brief Transport Layer Interface. */
typedef struct tl_t {
	/* callback to be made by transport layer protocol on receive */
	tl_on_recv_t on_recv;
	/* send data to transport layer for processing */
	tl_send_t send;
	/* argument to receive callback */
	void *on_recv_arg;

} tl_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

/**
 * @brief Configure Transport Layer On Receive Callback.
 *
 * @param[in] tl            Transport Layer.
 * @param[in] on_recv       Callback to be made by transport layer on receive.
 * @param[in] on_recv_arg   Argument to receive callback.
 * @return None
 */
void tl_on_recv_config(void *tl, tl_on_recv_t on_recv, void *on_recv_arg);

/**
 * @brief Send data via Transport Layer.
 *
 * @param[in] tl    Transport Layer.
 * @param[in] data  Data array.
 * @param[in] size  Data array size.
 * @return Number of sent bytes.
 */
uint16_t tl_send(void *tl, const uint8_t *data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* TL_COMMON_H */