/**
 *	@file     flash.h
 *  @brief    Library used for communication with flash memory.
 *  @version  v1.1.
 *  @author   LisumLab
 */

#ifndef FLASH_H
#define FLASH_H

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
 * API
 ******************************************************************************/

/**
 * @brief Initialize Flash device on ESP32-S3.
 *
 * @param[in] dev      Flash device structure.
 * @param[in] spi      SPI device handle (from spi_bus_add_device).
 * @param[in] cs_gpio  GPIO number used as CS for this flash device.
 * @param[in] time_us  Pointer to function returning current time in microseconds.
 * @retval ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t FLASH_Init(flash_t* dev, spi_device_handle_t spi, gpio_num_t cs_gpio,
					 uint64_t (*time_us)(void));

/**
 *  @brief Check if Flash Device is busy.
 *
 *  @param[in] dev Flash device.
 *  @retval 0 - Flash Device isn't busy
 *          1 - Flash Device busy
 */
uint8_t FLASH_Busy(flash_t* dev);

/**
 *  @brief Setup DMA channel for Flash DMA transfers.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] chType   For what will DMA channel be used, receive or transmit.
 *  @param[in] dmaChannel  DMA Channel ID.
 *  @retval None.
 */
void FLASH_ConfigureDMA(flash_t* dev, flash_dma_ch_type_t chType, uint8_t dmaChannel);

/**
 *  @brief Setup DMA channel for Flash DMA transfers.
 *
 *  @param[in] dev          Flash device.
 *  @param[in] callback     Flash transfer callback.
 *  @param[in] userData     Flash transfer callback user data.
 *  @retval None.
 */
void FLASH_ConfigureCallback(flash_t* dev, flash_tran_callback_t callback,
							 void* userData);

/**
 *  @brief Erase single memory block of Flash device.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be erased.
 *  @param[in] exeType  Function execution type, wait for timeout period or not.
 *  @retval ESP_OK in case block erase is a success,
 *          ESP_FAIL in case block erase failed.
 */
esp_err_t FLASH_BlockErase(flash_t* dev, uint32_t blkAddr, uint8_t exeType);

/**
 *  @brief Erase memory area of Flash device.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] start    Start block of memory area
 *  @param[in] end      End block of memory area
 *  @retval ESP_OK in case area erase is a success,
 *          ESP_FAIL in case area erase failed.
 */
esp_err_t FLASH_AreaErase(flash_t* dev, uint32_t start, uint32_t end);

/**
 *  @brief Read page data into internal cache of Flash device.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Block address
 *  @param[in] pageAddr Page address
 *  @param[in] exeType  Function execution type, wait for timeout period or not.
 *  @retval ESP_OK in case cache load is a success,
 *          ESP_FAIL in case cache load failed.
 */
esp_err_t FLASH_CacheLoad(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
						  uint8_t exeType);

/**
 *  @brief Read data from internal cache of Flash device.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] column   Start column (starting element of internal cache).
 *  @param[in] recv     Buffer for receiving data.
 *  @param[in] rSize    Size of receive buffer.
 *  @param[in] tranType Transfer type, POLLING or DMA.
 *  @retval ESP_OK in case cache read is a success,
 *          ESP_FAIL in case cache read failed.
 */
esp_err_t FLASH_CacheRead(flash_t* dev, uint16_t column, uint8_t* recv, uint16_t rSize,
						  flash_tran_e tranType);

/**
 *  @brief Read single memory page of Flash device.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be read.
 *  @param[in] pageAddr Address of page to be read (in reference to blkAddr).
 *  @param[in] recv     Buffer for receiving data.
 *  @param[in] rSize    Size of receive buffer.
 *  @retval ESP_OK in case page read is a success,
 *          ESP_FAIL in case page read failed.
 */
esp_err_t FLASH_PageRead(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr, uint8_t* recv,
						 uint16_t rSize);

/**
 *  @brief Read memory page spare area of Flash device.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be read.
 *  @param[in] pageAddr Address of page to be read (in reference to blkAddr).
 *  @param[in] recv     Buffer for receiving data.
 *  @param[in] rSize    Size of receive buffer.
 *  @retval ESP_OK in case spare area read is a success,
 *          ESP_FAIL in case spare area read failed.
 */
esp_err_t FLASH_SpareAreaRead(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							  uint8_t* recv, uint16_t rSize);

/**
 *  @brief Write data into Flash device cache.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] column   Start column (starting element of internal cache).
 *  @param[in] data     Data buffer.
 *  @param[in] dataLen  Size of data buffer.
 *  @param[in] tranType Transfer type, POLLING or DMA.
 *  @retval ESP_OK in case cache write is a success,
 *          ESP_FAIL in case cache write failed.
 */
esp_err_t FLASH_CacheWrite(flash_t* dev, uint16_t column, uint8_t* data, uint16_t dataLen,
						   uint8_t tranType);

/**
 *  @brief Flush data from Flash device cache buffer into memory page.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be written into.
 *  @param[in] pageAddr Address of page to be written into (in ref to blkAddr).
 *  @param[in] exeType  Function execution type, wait for timeout period or not.
 *  @retval ESP_OK in case cache flush is a success,
 *          ESP_FAIL in case cache flush failed.
 */
esp_err_t FLASH_CacheFlush(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
						   uint8_t exeType);

/**
 *  @brief Write data into memory page of Flash device.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be written into.
 *  @param[in] pageAddr Address of page to be written into.
 *  @param[in] data     Data buffer.
 *  @param[in] dataLen  Size of data buffer.
 *  @retval ESP_OK in case page write is a success,
 *          ESP_FAIL in case page write failed.
 */
esp_err_t FLASH_PageWrite(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
						  uint8_t* data, uint16_t dataLen);

/***************************** utility functions ******************************/

/**
 *  @brief Reset Flash device.
 *
 *  @param[in] dev  Flash device.
 *  @retval ESP_OK in case device reset is a success,
 *          ESP_FAIL in case device reset failed.
 */
esp_err_t FLASH_Reset(flash_t* dev);

#endif /* FLASH_H */