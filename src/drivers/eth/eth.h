#ifndef ETH_H
#define ETH_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_eth.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Ethernet HAL Physical Layer - used by higher level protocols. */
typedef struct eth_hal_phy_t {
	uint16_t (*recv)(void* phy, uint8_t* data, uint16_t size);
	uint16_t (*send)(void* phy, const uint8_t* data, uint16_t size);
} eth_hal_phy_t;

/*! @brief Initialize Ethernet HAL
 *
 * @param[out] eth_handle Optional out pointer receiving the esp_eth_handle_t created/used by HAL.
 */

/*! @brief Send a packet over Ethernet */
uint16_t HAL_ETH_Send(void* phy, const uint8_t* data, uint16_t size);

/*! @brief Receive a packet over Ethernet */
uint16_t HAL_ETH_Recv(void* phy, uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* ETH_H */
