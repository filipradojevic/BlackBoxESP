/**
 *	@file     flash_common.c
 *  @brief    Library used for common terms between different NAND flash devices
 *  @version  v1.1.
 *  @author   LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "flash_common.h"

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

/*******************************************************************************
 * Code
 ******************************************************************************/

esp_err_t FLASH_Transfer(flash_t* dev, uint8_t* tx, uint32_t txSize, uint8_t* rx,
						 uint32_t rxSize)
{
	esp_err_t status = ESP_FAIL;

	spi_transaction_t t = {.length = txSize * 8, .tx_buffer = tx, .rx_buffer = rx};

	gpio_set_level(dev->cs_gpio, 0);

	status = spi_device_transmit(dev->handle, &t);

	gpio_set_level(dev->cs_gpio, 1);

	return status;
}

esp_err_t FLASH_Transmit(flash_t* dev, uint8_t* tx, uint32_t txSize)
{
	esp_err_t status = ESP_FAIL;

	spi_transaction_t t = {.length = txSize * 8, .tx_buffer = tx, .rx_buffer = NULL};

	gpio_set_level(dev->cs_gpio, 0);

	status = spi_device_transmit(dev->handle, &t);

	if (status != ESP_OK) {

		gpio_set_level(dev->cs_gpio, 1);

		return ESP_FAIL;
	}

	gpio_set_level(dev->cs_gpio, 1);

	return ESP_OK;
}

/****************************** static functions ******************************/

/********************************* End Of File ********************************/