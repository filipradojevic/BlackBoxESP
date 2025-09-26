/**
 * @file    uart_tl.h
 * @brief   UART Transport Layer.
 * @version	1.0.0
 * @date    24.01.2025
 * @author  LisumLab
 */

#ifndef UART_TL_H
#define UART_TL_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "../tl/tl_common.h"

#include "driver/uart.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#ifdef UART_TL_CONFIG
#include "uart_tl_config.h"
#endif

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*! @brief UART Transport Layer Interface for ESP32-S3 */
typedef struct uart_tl_t {
	tl_t tl; //!< Transport layer.

	uart_port_t port; //!< UART port number (UART_NUM_0, UART_NUM_1, ...)

	QueueHandle_t queue_rx;	 //!< Receive Queue.
	QueueHandle_t queue_tx;	 //!< Transmit Queue.
	SemaphoreHandle_t mutex; //!< Mutex for sending data via UART TL.

	uint8_t rx_byte; //!< Last received byte.

#ifdef UART_TL_DBG
	volatile uint32_t err_cnt; //!< Error Counter.
#endif

} uart_tl_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

/**
 * @brief Initialize UART Transport Layer for ESP32-S3.
 *
 * @param[in] tl             UART Transport Layer instance.
 * @param[in] port           UART port number (UART_NUM_0, UART_NUM_1, ...).
 * @param[in] rx_queue_size  Size of the RX queue.
 * @param[in] tx_queue_size  Size of the TX queue.
 * @return ESP_OK on success, ESP_FAIL on error.
 */
void uart_tl_init(uart_tl_t* tl, uart_port_t port, uint16_t rx_queue_size, uint16_t tx_queue_size);

/**
 * @brief Send data via UART Transport Layer.
 *
 * @param[in] tl    UART Transport Layer.
 * @param[in] data  Data array.
 * @param[in] size  Data array size.
 * @return Data size on success and -1 on error.
 */
uint16_t uart_tl_send(void* tl, const uint8_t* data, uint16_t size);

/**
 * @brief   Process UART Transport Layer data. This includes data transmit from
 *          transmit queue and notifying upper layer of data in receive queue.
 *
 * @param[in] tl    UART Transport Layer.
 * @return None.
 */
void uart_tl_process(uart_tl_t* tl);

#ifdef __cplusplus
}
#endif

#endif /* UART_TL_H */