/**
 *	@file     w25n01gv.c
 *  @brief    W25N01GV NAND flash driver.
 *  @version  v1.1.
 *  @author   LisumLab
 */

#ifndef W25N01GV_H
#define W25N01GV_H

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
 *  @brief Use JEDEC ID to identify W25N01GV device.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] jedecID  Flash JEDEC ID.
 *  @param[in] devID    Flash Device ID
 *  @retval ESP_OK in case identification is a success,
 *          ESP_FAIL in case identification failed.
 */
esp_err_t W25N01GV_Identify(flash_t* dev, uint8_t jedecID, uint16_t devID);

/**
 *  @brief Check if W25N01GV is busy.
 *
 *  @param[in] dev Flash device.
 *  @retval 0 - W25N01GV isn't busy
 *          1 - W25N01GV busy
 */
uint8_t W25N01GV_Busy(flash_t* dev);

/**
 *  @brief Configure W25N01GV.
 *
 *  @param[in] dev Flash device.
 *  @retval ESP_OK in case configuration is a success,
 *          ESP_FAIL in case configuration failed.
 */
esp_err_t W25N01GV_Configure(flash_t* dev);

/**
 *  @brief Erase single memory block of W25N01GV.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be erased.
 *  @param[in] exeType  Function execution type, wait for timeout period or not.
 *  @retval ESP_OK in case block erase is a success,
 *          ESP_FAIL in case block erase failed.
 */
esp_err_t W25N01GV_BlockErase(flash_t* dev, uint32_t blkAddr,
							  flash_exe_e exeType);

/**
 *  @brief Erase memory area of W25N01GV.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] start    Start block of memory area
 *  @param[in] end      End block of memory area
 *  @retval ESP_OK in case area erase is a success,
 *          ESP_FAIL in case area erase failed.
 */
esp_err_t W25N01GV_AreaErase(flash_t* dev, uint32_t start, uint32_t end);

/**
 *  @brief Read page data into internal cache of W25N01GV.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Block address
 *  @param[in] pageAddr Page address
 *  @param[in] exeType  Function execution type, wait for timeout period or not.
 *  @retval ESP_OK in case cache load is a success,
 *          ESP_FAIL in case cache load failed.
 */
esp_err_t W25N01GV_CacheLoad(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							 flash_exe_e exeType);

/**
 *  @brief Read data from internal cache of W25N01GV.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] column   Start column (starting element of internal cache).
 *  @param[in] recv     Buffer for receiving data.
 *  @param[in] rSize    Size of receive buffer.
 *  @param[in] tranType Transfer type, POLLING or DMA.
 *  @retval ESP_OK in case cache read is a success,
 *          ESP_FAIL in case cache read failed.
 */
esp_err_t W25N01GV_CacheRead(flash_t* dev, uint16_t column, uint8_t* recv,
							 uint16_t rSize, flash_tran_e tranType);

/**
 *  @brief Read single memory page of W25N01GV.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be read.
 *  @param[in] pageAddr Address of page to be read (in reference to blkAddr).
 *  @param[in] recv     Buffer for receiving data.
 *  @param[in] rSize    Size of receive buffer.
 *  @retval ESP_OK in case page read is a success,
 *          ESP_FAIL in case page read failed.
 */
esp_err_t W25N01GV_PageRead(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							uint8_t* recv, uint16_t rSize);

/**
 *  @brief Read memory page spare area of W25N01GV.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be read.
 *  @param[in] pageAddr Address of page to be read (in reference to blkAddr).
 *  @param[in] recv     Buffer for receiving data.
 *  @param[in] rSize    Size of receive buffer.
 *  @retval ESP_OK in case spare area read is a success,
 *          ESP_FAIL in case spare area read failed.
 */
esp_err_t W25N01GV_SpareAreaRead(flash_t* dev, uint32_t blkAddr,
								 uint32_t pageAddr, uint8_t* recv,
								 uint16_t rSize);

/**
 *  @brief Write data into W25N01GV cache.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] column   Start column (starting element of internal cache).
 *  @param[in] data     Data buffer.
 *  @param[in] dataLen  Size of data buffer.
 *  @param[in] tranType Transfer type, POLLING or DMA.
 *  @retval ESP_OK in case cache write is a success,
 *          ESP_FAIL in case cache write failed.
 */
esp_err_t W25N01GV_CacheWrite(flash_t* dev, uint16_t column, uint8_t* data,
							  uint16_t dataLen, uint8_t tranType);

/**
 *  @brief Flush data from W25N01GV cache buffer into memory page.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be written into.
 *  @param[in] pageAddr Address of page to be written into (in ref to blkAddr).
 *  @param[in] exeType  Function execution type, wait for timeout period or not.
 *  @retval ESP_OK in case cache flush is a success,
 *          ESP_FAIL in case cache flush failed.
 */
esp_err_t W25N01GV_CacheFlush(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							  flash_exe_e exeType);

/**
 *  @brief Write data into memory page of W25N01GV.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] blkAddr  Address of block to be written into.
 *  @param[in] pageAddr Address of page to be written into.
 *  @param[in] data     Data buffer.
 *  @param[in] dataLen  Size of data buffer.
 *  @retval ESP_OK in case page write is a success,
 *          ESP_FAIL in case page write failed.
 */
esp_err_t W25N01GV_PageWrite(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							 uint8_t* data, uint16_t dataLen);
/**
 *  @brief Reset W25N01GV.
 *
 *  @param[in] dev  Flash device.
 *  @retval ESP_OK in case device reset is a success,
 *          ESP_FAIL in case device reset failed.
 */
esp_err_t W25N01GV_Reset(flash_t* dev);

#endif /* W25N01GV_H */