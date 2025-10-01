/**
 * @file    uart_tl.c
 * @brief   UART Transport Layer.
 * @version	1.0.0
 * @date    24.01.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "uart_tl.h"

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

/* On recv function wrapper */
static int uart_tl_on_recv_wrap(void* tl, uint8_t* data, uint16_t size);

/* UART Callback (no longer a HAL callback, can use FreeRTOS task or event) */
static void uart_cb(void* arg)
{
	// On ESP32-S3, you can use a uart_event_t queue for RX/TX events
	// or read directly from the buffer
	(void)arg;
}

void uart_tl_init(uart_tl_t* tl, uart_port_t port, uint16_t rx_queue_size,
				  uint16_t tx_queue_size)
{
	uart_tl_t* uart_tl;

	uart_tl = (uart_tl_t*)tl;

	uart_tl->port = port;
	uart_tl->queue_rx = xQueueCreate(rx_queue_size, sizeof(uint8_t));
	uart_tl->queue_tx = xQueueCreate(tx_queue_size, sizeof(uint8_t));
	uart_tl->mutex = xSemaphoreCreateMutex();

	// Install the UART driver
	uart_driver_install(port, rx_queue_size, tx_queue_size, rx_queue_size,
						&uart_tl->queue_rx, 0);

	// Set default UART pins if needed
	// uart_set_pin(port, TX_PIN, RX_PIN, RTS_PIN, CTS_PIN);

	// Transport layer callbacks and send function
	uart_tl->tl.on_recv = NULL;
	uart_tl->tl.on_recv_arg = NULL;
	uart_tl->tl.send = uart_tl_send;

	// RX buffer initialization: ESP-IDF does not use HAL_UART_ReceiveISR
	// instead, you can use uart_read_bytes in a FreeRTOS task
}

uint16_t uart_tl_send(void* tl, const uint8_t* data, uint16_t size)
{
	uart_tl_t* uart_tl;

	uart_tl = (uart_tl_t*)tl;

	if (uxQueueSpacesAvailable(uart_tl->queue_tx) < size) {
#ifdef UART_TL_DBG
		uart_tl->err_cnt++;
#endif
		return -1;
	}

	xSemaphoreTake(uart_tl->mutex, portMAX_DELAY);

	for (uint16_t i = 0; i < size; i++) {
		xQueueSendToBack(uart_tl->queue_tx, (data + i), 0);
	}

	xSemaphoreGive(uart_tl->mutex);

	return size;
}

void uart_tl_process(uart_tl_t* tl)
{
	uart_tl_t* uart = (uart_tl_t*)tl;
	uint8_t byte;

	// Transmit
	if (pdPASS == xQueuePeek(uart->queue_tx, &byte, 0)) {
		if (uart_write_bytes(uart->port, (const char*)&byte, 1) > 0) {
			xQueueReceive(uart->queue_tx, &byte, 0); // Remove sent byte
		}
	}

	// Receive
	if (pdPASS == xQueueReceive(uart->queue_rx, &byte, 0)) {
		uart_tl_on_recv_wrap(uart, &byte, sizeof(byte));
	}
}

/****************************** static functions ******************************/

static int uart_tl_on_recv_wrap(void* tl, uint8_t* data, uint16_t size)
{
	uart_tl_t* uart_tl;

	uart_tl = (uart_tl_t*)tl;

	if (uart_tl->tl.on_recv == NULL)
		return 0;

	return uart_tl->tl.on_recv(uart_tl->tl.on_recv_arg, data, size);
}

void uart_cb(void* arg)
{
	uart_tl_t* uart = (uart_tl_t*)arg;
	uint8_t byte;
	BaseType_t taskWoken = pdFALSE;

	// Read one byte from UART
	int len = uart_read_bytes(uart->port, &byte, 1, 0);
	if (len > 0) {
		BaseType_t ret = xQueueSendToBackFromISR(uart->queue_rx, &byte, &taskWoken);
#ifdef UART_TL_DBG
		if (ret != pdPASS)
			uart->err_cnt++;
#endif
	}

	portYIELD_FROM_ISR(taskWoken);
}

/********************************* End Of File ********************************/