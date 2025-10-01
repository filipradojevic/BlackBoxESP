/**
 *	@file     flash_common.h
 *  @brief    Library used for common terms between different NAND flash devices
 *  @version  v1.1.
 *  @author   LisumLab
 */

#ifndef FLASH_COMMON_H
#define FLASH_COMMON_H

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* Flash Read JEDEC ID Command */
#define FLASH_GET_JEDEC_ID_CMD 0x9FU

/* Address of block reserved for Bad Block Management Look Up Table. */
#define FLASH_BBM_LUT_BLOCK_ADDR 0x00U

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*! @brief Flash device type, NOR or NAND. */
typedef enum { FLASH_DEV_TYPE_NOR = 0, FALSH_DEV_TYPE_NAND = 1 } flash_type_e;

/*! @brief Flash transfer type, POLLING or DMA. */
typedef enum { FLASH_POLLING = 0, FLASH_DMA = 1 } flash_tran_e;

/*! @brief Flash operation execution type, wait for tiemout period or not. */
typedef enum { FLASH_WAIT_EXECUTION = 0, FLASH_SET_TIMEOUT = 1 } flash_exe_e;

/*! @brief Flash DMA Channel Type, Receive or Transmit. */
typedef enum { FLASH_DMA_CH_TYPE_RX = 0, FLASH_DMA_CH_TYPE_TX = 1 } flash_dma_ch_type_t;

/*! @brief Flash geometry. */
typedef struct flash_geometry_t {
	uint16_t blkCnt;		//!< block count per device.
	uint16_t pageCnt;		//!< page count per block.
	uint32_t blkSize;		//!< block size in bytes [B].
	uint16_t pageSize;		//!< page size in bytes [B].
	uint16_t spareAreaSize; //!< spare area size.
	uint32_t devSize;		//!< total device size in bytes [B].

} flash_geometry_t;

/*! @brief Flash information. */
typedef struct flash_info_t {
	uint8_t jedecID;
	uint16_t devID;

} flash_info_t;

/*! @brief Flash bad block management look up table entry. */
typedef struct flash_bbmlut_t {
	uint16_t pba;
	uint16_t lba;

} flash_bbm_t;

/*! @brief Flash transfer callback. */
typedef void (*flash_tran_callback_t)(esp_err_t status, void* userData);

typedef struct flash_vtable flash_vtable;

/*! @brief Flash device. */
typedef struct flash_t {
	uint64_t (*time_us)(void); //!< time_us function pointer.

	spi_device_handle_t handle; //!< ESP32-S3 SPI device handle
	gpio_num_t cs_gpio;			//!< GPIO pin za CS

	/* NOTE: WIP */
	// flash_bbm_t bbm_lut[40]; /*<- max number of bbm lut entries is 40 */
	// uint8_t bbm_cnt;
	flash_info_t devInfo;	   //!< Flash device information.
	flash_geometry_t geometry; //!< Flash geometry.

	uint8_t dmaChannelTX; //!< DMA Channel used for transmitting data (optional)
	uint8_t dmaChannelRX; //!< DMA Channel used for receiving data (optional)
	flash_vtable* vtable; //!< Flash vtable.

	flash_tran_callback_t callback; //!< Flash callback
	void* userData;					//!< Flash callback user data
} flash_t;

/*! @brief Flash vtable. */
typedef struct flash_vtable {
	uint8_t (*busy)(flash_t* dev);
	esp_err_t (*configure)(flash_t* dev);

	/* erase functions */
	esp_err_t (*blockErase)(flash_t* dev, uint32_t blk_addr, flash_exe_e exe_type);
	esp_err_t (*areaErase)(flash_t* dev, uint32_t start, uint32_t end);

	/* read functions */
	esp_err_t (*cacheLoad)(flash_t* dev, uint32_t blk_addr, uint32_t page_addr,
						   flash_exe_e exe_type);

	esp_err_t (*cacheRead)(flash_t* dev, uint16_t column, uint8_t* recv, uint16_t rSize,
						   flash_tran_e tranType);

	esp_err_t (*pageRead)(flash_t* dev, uint32_t blk_addr, uint32_t page_addr,
						  uint8_t* recv, uint16_t r_size);

	esp_err_t (*spareAreaRead)(flash_t* dev, uint32_t blk_addr, uint32_t page_addr,
							   uint8_t* recv, uint16_t r_size);

	/* write functions */
	esp_err_t (*cacheWrite)(flash_t* dev, uint16_t column, uint8_t* data,
							uint16_t data_len, uint8_t tran_type);

	esp_err_t (*cacheFlush)(flash_t* dev, uint32_t blk_addr, uint32_t page_addr,
							flash_exe_e exe_type);

	esp_err_t (*pageWrite)(flash_t* dev, uint32_t blk_addr, uint32_t page_addr,
						   uint8_t* data, uint16_t data_len);

	/* utility functions */
	esp_err_t (*reset)(flash_t* dev);

} flash_vtable;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

/**
 *  @brief Flash SSP Transfer.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] tx       Transmit data.
 *  @param[in] txSize   Transmit data size.
 *  @param[in] rx       Receive data.
 *  @param[in] rxSize   Receive data size.
 *  @retval ESP_OK in case transfer is a success,
 *          ESP_FAIL in case transfer failed.
 *          ESP_FAIL in case transfer timeout occurred.
 */
esp_err_t FLASH_Transfer(flash_t* dev, uint8_t* tx, uint32_t txSize, uint8_t* rx,
						 uint32_t rxSize);

/**
 *  @brief Flash SSP Transmit.
 *
 *  @param[in] dev      Flash device.
 *  @param[in] tx       Transmit data.
 *  @param[in] txSize   Transmit data size.
 *  @retval ESP_OK in case transfer is a success,
 *          ESP_FAIL in case transfer failed.
 *          ESP_FAIL in case transfer timeout occurred.
 */
esp_err_t FLASH_Transmit(flash_t* dev, uint8_t* tx, uint32_t txSize);

#endif /* FLASH_COMMON_H */