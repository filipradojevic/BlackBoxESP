/**
 *	@file     gd5f2gq5ue.h
 *  @brief    GD5F2GQ5UE NAND flash driver.
 *  @version  v1.1.
 *  @author   LisumLab
 */

#ifndef GD5F2GQ5UE_H
#define GD5F2GQ5UE_H

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
 *  @brief Use JEDEC ID to identify GD5F2GQ5UE device.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] jedecID  Flash JEDEC ID.
 *  @param[in] devID    Flash Device ID
 *  @retval ESP_OK in case identification is a success,
 *          ESP_FAIL in case identification failed.
 */
esp_err_t GD5F2GQ5UE_Identify(flash_t* dev, uint8_t jedecID, uint16_t devID);

/**
 *  @brief Check if GD5F2GQ5UE is busy.
 *
 *  @param[in] dev Flash device.
 *  @retval ESP_OK - GD5F2GQ5UE isn't busy
 *          1 - GD5F2GQ5UE busy
 */
uint8_t GD5F2GQ5UE_Busy(flash_t* dev);

/**
 *  @brief Configure GD5F2GQ5UE.
 *
 *  @param[in] dev Flash device.
 *  @retval ESP_OK in case configuration is a success,
 *          ESP_FAIL in case configuration failed.
 */
esp_err_t GD5F2GQ5UE_Configure(flash_t* dev);

/**
 *  @brief Erase single memory block of GD5F2GQ5UE.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be erased.
 *  @param[in] exeType  Function execution type, wait for timeout period or not.
 *  @retval ESP_OK in case block erase is a success,
 *          ESP_FAIL in case block erase failed.
 */
esp_err_t GD5F2GQ5UE_BlockErase(flash_t* dev, uint32_t blkAddr,
								flash_exe_e exeType);

/**
 *  @brief Erase memory area of GD5F2GQ5UE.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] start    Start block of memory area
 *  @param[in] end      End block of memory area
 *  @retval ESP_OK in case area erase is a success,
 *          ESP_FAIL in case area erase failed.
 */
esp_err_t GD5F2GQ5UE_AreaErase(flash_t* dev, uint32_t start, uint32_t end);

/**
 *  @brief Read page data into internal cache of GD5F2GQ5UE.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Block address
 *  @param[in] pageAddr Page address
 *  @param[in] exeType  Function execution type, wait for timeout period or not.
 *  @retval ESP_OK in case cache load is a success,
 *          ESP_FAIL in case cache load failed.
 */
esp_err_t GD5F2GQ5UE_CacheLoad(flash_t* dev, uint32_t blkAddr,
							   uint32_t pageAddr, flash_exe_e exeType);

/**
 *  @brief Read data from internal cache of GD5F2GQ5UE.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] column   Start column (starting element of internal cache).
 *  @param[in] recv     Buffer for receiving data.
 *  @param[in] rSize    Size of receive buffer.
 *  @param[in] tranType Transfer type, POLLING or DMA.
 *  @retval ESP_OK in case cache read is a success,
 *          ESP_FAIL in case cache read failed.
 */
esp_err_t GD5F2GQ5UE_CacheRead(flash_t* dev, uint16_t column, uint8_t* recv,
							   uint16_t rSize, flash_tran_e tranType);

/**
 *  @brief Read single memory page of GD5F2GQ5UE.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be read.
 *  @param[in] pageAddr Address of page to be read (in reference to blkAddr).
 *  @param[in] recv     Buffer for receiving data.
 *  @param[in] rSize    Size of receive buffer.
 *  @retval ESP_OK in case page read is a success,
 *          ESP_FAIL in case page read failed.
 */
esp_err_t GD5F2GQ5UE_PageRead(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							  uint8_t* recv, uint16_t rSize);

/**
 *  @brief Read memory page spare area of GD5F2GQ5UE.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be read.
 *  @param[in] pageAddr Address of page to be read (in reference to blkAddr).
 *  @param[in] recv     Buffer for receiving data.
 *  @param[in] rSize    Size of receive buffer.
 *  @retval ESP_OK in case spare area read is a success,
 *          ESP_FAIL in case spare area read failed.
 */
esp_err_t GD5F2GQ5UE_SpareAreaRead(flash_t* dev, uint32_t blkAddr,
								   uint32_t pageAddr, uint8_t* recv,
								   uint16_t rSize);

/**
 *  @brief Write data into GD5F2GQ5UE cache.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] column   Start column (starting element of internal cache).
 *  @param[in] data     Data buffer.
 *  @param[in] dataLen  Size of data buffer.
 *  @param[in] tranType Transfer type, POLLING or DMA.
 *  @retval ESP_OK in case cache write is a success,
 *          ESP_FAIL in case cache write failed.
 */
esp_err_t GD5F2GQ5UE_CacheWrite(flash_t* dev, uint16_t column, uint8_t* data,
								uint16_t dataLen, uint8_t tranType);

/**
 *  @brief Flush data from GD5F2GQ5UE cache buffer into memory page.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be written into.
 *  @param[in] pageAddr Address of page to be written into (in ref to blkAddr).
 *  @param[in] exeType  Function execution type, wait for timeout period or not.
 *  @retval ESP_OK in case cache flush is a success,
 *          ESP_FAIL in case cache flush failed.
 */
esp_err_t GD5F2GQ5UE_CacheFlush(flash_t* dev, uint32_t blkAddr,
								uint32_t pageAddr, flash_exe_e exeType);

/**
 *  @brief Write data into memory page of GD5F2GQ5UE.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be written into.
 *  @param[in] pageAddr Address of page to be written into.
 *  @param[in] data     Data buffer.
 *  @param[in] dataLen  Size of data buffer.
 *  @retval ESP_OK in case page write is a success,
 *          ESP_FAIL in case page write failed.
 */
esp_err_t GD5F2GQ5UE_PageWrite(flash_t* dev, uint32_t blkAddr,
							   uint32_t pageAddr, uint8_t* data,
							   uint16_t dataLen);
/**
 *  @brief Reset GD5F2GQ5UE.
 *
 *  @param[in] dev  Flash device.
 *  @retval ESP_OK in case device reset is a success,
 *          ESP_FAIL in case device reset failed.
 */
esp_err_t GD5F2GQ5UE_Reset(flash_t* dev);

#endif /* GD5F2GQ5UE_H */