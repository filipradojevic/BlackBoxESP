/**
 *	@file     gd5f2gq5ue.c
 *  @brief    GD5F2GQ5UE NAND flash driver.
 *  @version  v1.1.
 *  @author   LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>

#include "gd5f2gq5ue.h"
#include "gd5f2gq5ue_def.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* GD5F2GQ5UE vtable */
flash_vtable gd5f2gq5ue_vtable;

static volatile uint8_t dummy = 0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* read value of flash register. */
static esp_err_t GD5F2GQ5UE_GetFeature(flash_t* dev, uint8_t regAddr, uint8_t* regVal);

/* write value into flash register. */
static esp_err_t GD5F2GQ5UE_SetFeature(flash_t* dev, uint8_t regAddr, uint8_t regVal);

/* enable write commands. */
static esp_err_t GD5F2GQ5UE_WriteEnable(flash_t* dev);

/* disable write commands. */
static esp_err_t GD5F2GQ5UE_WriteDisable(flash_t* dev);

/* check ecc status after reading a page. */
static uint8_t GD5F2GQ5UE_CheckECC(flash_t* dev);

/*******************************************************************************
 * Code
 ******************************************************************************/

esp_err_t GD5F2GQ5UE_Identify(flash_t* dev, uint8_t jedecID, uint16_t devID)
{
	if (jedecID != GD5F2GQ5UE_JEDEC_ID)
		return ESP_FAIL;

	if (((devID >> 8) & 0xFF) != GD5F2GQ5UE_DEVICE_ID)
		return ESP_FAIL;

	gd5f2gq5ue_vtable.busy = GD5F2GQ5UE_Busy;
	gd5f2gq5ue_vtable.configure = GD5F2GQ5UE_Configure;
	gd5f2gq5ue_vtable.blockErase = GD5F2GQ5UE_BlockErase;
	gd5f2gq5ue_vtable.areaErase = GD5F2GQ5UE_AreaErase;
	gd5f2gq5ue_vtable.cacheLoad = GD5F2GQ5UE_CacheLoad;
	gd5f2gq5ue_vtable.cacheRead = GD5F2GQ5UE_CacheRead;
	gd5f2gq5ue_vtable.pageRead = GD5F2GQ5UE_PageRead;
	gd5f2gq5ue_vtable.spareAreaRead = GD5F2GQ5UE_SpareAreaRead;
	gd5f2gq5ue_vtable.cacheWrite = GD5F2GQ5UE_CacheWrite;
	gd5f2gq5ue_vtable.cacheFlush = GD5F2GQ5UE_CacheFlush;
	gd5f2gq5ue_vtable.pageWrite = GD5F2GQ5UE_PageWrite;
	gd5f2gq5ue_vtable.reset = GD5F2GQ5UE_Reset;

	dev->vtable = &gd5f2gq5ue_vtable;

	dev->geometry.blkCnt = GD5F2GQ5UE_BLOCKS_PER_DEV;
	dev->geometry.pageCnt = GD5F2GQ5UE_PAGES_PER_BLOCK;
	dev->geometry.blkSize = GD5F2GQ5UE_PAGE_SIZE * GD5F2GQ5UE_PAGES_PER_BLOCK;
	dev->geometry.pageSize = GD5F2GQ5UE_PAGE_SIZE;
	dev->geometry.devSize = dev->geometry.blkSize * GD5F2GQ5UE_BLOCKS_PER_DEV;
	dev->geometry.spareAreaSize = GD5F2GQ5UE_SPARE_AREA_SIZE;
	dev->devInfo.jedecID = jedecID;
	dev->devInfo.devID = devID;

	return 0;
}

uint8_t GD5F2GQ5UE_Busy(flash_t* dev)
{
	esp_err_t status;
	uint8_t resp;

	status = GD5F2GQ5UE_GetFeature(dev, GD5F2GQ5UE_STATUS_1_REG_ADDR, &resp);

	if (status != ESP_OK)
		return ESP_FAIL;

	return (resp & GD5F2GQ5UE_STATUS_1_OIP);
}

esp_err_t GD5F2GQ5UE_Configure(flash_t* dev)
{
	esp_err_t status = 1;

	status = GD5F2GQ5UE_SetFeature(dev, GD5F2GQ5UE_PROTECTION_REG_ADDR,
								   GD5F2GQ5UE_PROTECTION_NONE);
	if (status != ESP_OK)
		return ESP_FAIL;

	status = GD5F2GQ5UE_SetFeature(dev, GD5F2GQ5UE_FEATURE_1_REG_ADDR,
								   GD5F2GQ5UE_FEATURE_1_ECC_EN);
	if (status != ESP_OK)
		return ESP_FAIL;

	return ESP_OK;
}

esp_err_t GD5F2GQ5UE_BlockErase(flash_t* dev, uint32_t blkAddr, flash_exe_e exeType)
{
	esp_err_t status = ESP_FAIL;
	uint64_t cmdTime;
	uint32_t pa;
	uint8_t cmd[4] = {0};

	if (blkAddr > (GD5F2GQ5UE_BLOCKS_PER_DEV - 1))
		return ESP_FAIL;

	if (GD5F2GQ5UE_Busy(dev))
		return ESP_FAIL;

	status = GD5F2GQ5UE_WriteEnable(dev);
	if (status != ESP_OK)
		return ESP_FAIL;

	pa = (blkAddr << 6);

	cmd[0] = GD5F2GQ5UE_BLOCK_ERASE_CMD;
	cmd[1] = (pa >> 16) & 0xFFU;
	cmd[2] = (pa >> 8) & 0xFFU;
	cmd[3] = pa & 0xFFU;

	status = FLASH_Transmit(dev, cmd, sizeof(cmd));
	if (status != ESP_OK)
		return ESP_FAIL;

	cmdTime = dev->time_us();

	/* Check timeout period */
	if (exeType == FLASH_WAIT_EXECUTION) {
		while (GD5F2GQ5UE_Busy(dev)) {
			if (dev->time_us() - cmdTime >= GD5F2GQ5UE_BLOCK_ERASE_TIMEOUT_US) {
				dummy++;
				return ESP_FAIL;
			}
		}
	}

	return ESP_OK;
}

esp_err_t GD5F2GQ5UE_AreaErase(flash_t* dev, uint32_t start, uint32_t end)
{
	esp_err_t status = ESP_FAIL;

	if (start > end) {
		return ESP_FAIL;
	}

	for (uint32_t i = start; i <= end; i++) {
		status = GD5F2GQ5UE_BlockErase(dev, i, FLASH_WAIT_EXECUTION);
		if (status != ESP_OK)
			return ESP_FAIL;
	}

	return ESP_OK;
}

esp_err_t GD5F2GQ5UE_CacheLoad(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							   flash_exe_e exeType)
{
	esp_err_t status = ESP_FAIL;
	uint64_t cmdTime;
	uint32_t pa;
	uint8_t cmd[4] = {0};

	cmdTime = dev->time_us();

	while (GD5F2GQ5UE_Busy(dev)) {
		if (dev->time_us() - cmdTime >= GD5F2GQ5UE_PAGE_READ_TIMEOUT_US) {
			dummy++;
			return ESP_FAIL;
		}
	}

	if (pageAddr > (GD5F2GQ5UE_PAGES_PER_BLOCK - 1) ||
		blkAddr > (GD5F2GQ5UE_BLOCKS_PER_DEV - 1)) {
		dummy++;
		return ESP_FAIL;
	}

	pa = (blkAddr << 6) | pageAddr;

	/* load page into internal buffer of flash device */
	cmd[0] = GD5F2GQ5UE_PAGE_READ_TO_CACHE_CMD;
	cmd[1] = (pa >> 16) & 0xFFU;
	cmd[2] = (pa >> 8) & 0xFFU;
	cmd[3] = pa & 0xFFU;

	status = FLASH_Transmit(dev, cmd, sizeof(cmd));
	if (status != ESP_OK) {
		dummy++;
		return ESP_FAIL;
	}

	cmdTime = dev->time_us();

	/* Check timeout period */
	if (exeType == FLASH_WAIT_EXECUTION) {
		while (GD5F2GQ5UE_Busy(dev)) {
			if (dev->time_us() - cmdTime >= GD5F2GQ5UE_PAGE_READ_TIMEOUT_US) {
				dummy++;
				return ESP_FAIL;
			}
		}
	}

	return ESP_OK;
}

esp_err_t GD5F2GQ5UE_CacheRead(flash_t* dev, uint16_t column, uint8_t* recv,
							   uint16_t rSize, flash_tran_e tranType)
{
	esp_err_t status = ESP_FAIL;
	uint8_t cmd[4] = {0};

	if (GD5F2GQ5UE_Busy(dev))
		return ESP_FAIL;

	cmd[0] = GD5F2GQ5UE_READ_DATA_CMD;
	cmd[1] = (column >> 8) & 0xFFU;
	cmd[2] = column & 0xFFU;

	if (tranType == FLASH_POLLING) {
		status = FLASH_Transfer(dev, cmd, sizeof(cmd), recv, rSize);
	} else {

		// CS LOW
		gpio_set_level(dev->cs_gpio, 0);

		// Send cmd
		spi_transaction_t t_cmd = {
			.length = sizeof(cmd) * 8,
			.tx_buffer = cmd,
			.rx_buffer = NULL,
		};

		status = spi_device_transmit(dev->handle, &t_cmd);

		if (status == ESP_OK) {

			// Read data
			spi_transaction_t t_dma = {
				.length = rSize * 8,
				.tx_buffer = NULL,
				.rx_buffer = recv,
			};

			status = spi_device_transmit(dev->handle, &t_dma);
		}

		// CS HIGH
		gpio_set_level(dev->cs_gpio, 1);
	}

	/** @todo: add bad block managment */
	if (!GD5F2GQ5UE_CheckECC(dev)) {
		return ESP_OK;
	}

	return status;
}

esp_err_t GD5F2GQ5UE_PageRead(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							  uint8_t* recv, uint16_t rSize)
{
	esp_err_t status;

	status = GD5F2GQ5UE_CacheLoad(dev, blkAddr, pageAddr, FLASH_WAIT_EXECUTION);
	if (status != ESP_OK)
		return ESP_FAIL;

	status = GD5F2GQ5UE_CacheRead(dev, 0x00, recv, rSize, FLASH_POLLING);
	if (status != ESP_OK)
		return ESP_FAIL;

	return ESP_OK;
}

esp_err_t GD5F2GQ5UE_SpareAreaRead(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
								   uint8_t* recv, uint16_t rSize)
{
	esp_err_t status;

	status = GD5F2GQ5UE_CacheLoad(dev, blkAddr, pageAddr, FLASH_WAIT_EXECUTION);
	if (status != ESP_OK)
		return ESP_FAIL;

	status = GD5F2GQ5UE_CacheRead(dev, GD5F2GQ5UE_PAGE_SIZE, recv, rSize, FLASH_POLLING);
	if (status != ESP_OK)
		return ESP_FAIL;

	return ESP_OK;
}

esp_err_t GD5F2GQ5UE_CacheWrite(flash_t* dev, uint16_t column, uint8_t* data,
								uint16_t dataLen, uint8_t tranType)
{
	esp_err_t status = ESP_FAIL;
	uint8_t cmd[3] = {0};
	uint64_t cmdTime;

	if (((column + dataLen) > GD5F2GQ5UE_PAGE_SIZE)) {
		dummy++;
		return ESP_FAIL;
	}

	cmdTime = dev->time_us();

	while (GD5F2GQ5UE_Busy(dev)) {
		// Dodatna provera timeouta
		if (dev->time_us() - cmdTime >= GD5F2GQ5UE_BLOCK_ERASE_TIMEOUT_US) {
			dummy++;
			return ESP_FAIL;
		}
	}

	status = GD5F2GQ5UE_WriteEnable(dev);
	if (status != ESP_OK) {
		dummy++;

		return ESP_FAIL;
	}

	cmd[0] = GD5F2GQ5UE_PROGRAM_LOAD_CMD;
	cmd[1] = (column >> 8) & 0xFFU;
	cmd[2] = column & 0xFFU;

	// CS LOW
	gpio_set_level(dev->cs_gpio, 0);

	spi_transaction_t t_cmd = {
		.length = sizeof(cmd) * 8, // length in bits
		.tx_buffer = cmd,
		.rx_buffer = NULL,
	};

	status = spi_device_transmit(dev->handle, &t_cmd);

	if (status == ESP_OK) {

		if (tranType == FLASH_POLLING) {

			spi_transaction_t t_data = {
				.length = dataLen * 8,
				.tx_buffer = data,
				.rx_buffer = NULL,
			};

			status = spi_device_transmit(dev->handle, &t_data);
		}

		if (tranType == FLASH_DMA) {

			spi_transaction_t t_dma = {
				.length = dataLen * 8,
				.tx_buffer = data,
				.rx_buffer = NULL,
			};

			status = spi_device_transmit(dev->handle, &t_dma);
		}
	}

	// CS HIGH
	gpio_set_level(dev->cs_gpio, 1);

	return status;
}

esp_err_t GD5F2GQ5UE_CacheFlush(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
								flash_exe_e exeType)
{
	esp_err_t status = ESP_FAIL;
	uint64_t cmdTime;
	uint32_t pa;
	uint8_t cmd[4];

	if (GD5F2GQ5UE_Busy(dev)) {
		dummy++;
		return ESP_FAIL;
	}

	/** @todo: implement bad block managment */

	if (pageAddr > (GD5F2GQ5UE_PAGES_PER_BLOCK - 1) ||
		blkAddr > (GD5F2GQ5UE_BLOCKS_PER_DEV - 1)) {
		dummy++;
		return ESP_FAIL;
	}

	pa = (blkAddr << 6) | pageAddr;

	cmd[0] = GD5F2GQ5UE_PROGRAM_EXECUTE_CMD;
	cmd[1] = (pa >> 16) & 0xFFU;
	cmd[2] = (pa >> 8) & 0xFFU;
	cmd[3] = pa & 0xFFU;

	status = FLASH_Transmit(dev, cmd, sizeof(cmd));
	if (status != ESP_OK) {
		dummy++;
		return ESP_FAIL;
	}

	cmdTime = dev->time_us();

	if (exeType == FLASH_WAIT_EXECUTION) {
		// Čekajte u petlji
		while (GD5F2GQ5UE_Busy(dev)) {
			// Dodatna provera timeouta
			if (dev->time_us() - cmdTime >= GD5F2GQ5UE_PAGE_PROGRAM_TIMEOUT_US) {
				dummy++;
				return ESP_FAIL;
			}
		}
	}

	return ESP_OK;
}

esp_err_t GD5F2GQ5UE_PageWrite(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							   uint8_t* data, uint16_t dataLen)
{
	esp_err_t status = ESP_FAIL;

	status = GD5F2GQ5UE_CacheWrite(dev, 0, data, dataLen, FLASH_DMA);
	if (status != ESP_OK)
		return ESP_FAIL;

	status = GD5F2GQ5UE_CacheFlush(dev, blkAddr, pageAddr, FLASH_WAIT_EXECUTION);
	if (status != ESP_OK)
		return ESP_FAIL;

	return ESP_OK;
}

esp_err_t GD5F2GQ5UE_Reset(flash_t* dev)
{
	esp_err_t status = ESP_FAIL;
	uint64_t cmdTime;
	uint8_t cmd = GD5F2GQ5UE_DEV_RESET_CMD;

	if (GD5F2GQ5UE_Busy(dev))
		return ESP_FAIL;

	status = FLASH_Transmit(dev, &cmd, sizeof(cmd));
	if (status != ESP_OK)
		return ESP_FAIL;

	cmdTime = dev->time_us();

	/* Check timeout period */
	while (GD5F2GQ5UE_Busy(dev)) {
		if (dev->time_us() - cmdTime >= GD5F2GQ5UE_DEV_RESET_TIMEOUT_US)
			return ESP_FAIL;
	}

	/* configure flash device again */
	status = GD5F2GQ5UE_Configure(dev);
	if (status != ESP_OK)
		return ESP_FAIL;

	return ESP_OK;
}

/****************************** static functions ******************************/

/* read value of flash register. */
static esp_err_t GD5F2GQ5UE_GetFeature(flash_t* dev, uint8_t regAddr, uint8_t* regVal)
{
	esp_err_t status;
	uint8_t cmd[2] = {GD5F2GQ5UE_GET_FEATURE_CMD, regAddr};

	status = FLASH_Transfer(dev, cmd, sizeof(cmd), regVal, 1);

	return status;
}

/* write value into flash register. */
static esp_err_t GD5F2GQ5UE_SetFeature(flash_t* dev, uint8_t regAddr, uint8_t regVal)
{
	esp_err_t status = ESP_FAIL;
	uint8_t tmp;
	uint8_t cmd[3] = {0};

	if (GD5F2GQ5UE_Busy(dev))
		return ESP_FAIL;

	cmd[0] = GD5F2GQ5UE_SET_FEATURE_CMD;
	cmd[1] = regAddr;
	cmd[2] = regVal;

	status = FLASH_Transmit(dev, cmd, sizeof(cmd));
	if (status != ESP_OK)
		return ESP_FAIL;

	/* Check if the write was successful */
	status = GD5F2GQ5UE_GetFeature(dev, regAddr, &tmp);
	if (tmp != regVal || status != ESP_OK)
		return ESP_FAIL;

	return ESP_OK;
}

/* enable write commands. */
static esp_err_t GD5F2GQ5UE_WriteEnable(flash_t* dev)
{
	esp_err_t status = ESP_FAIL;
	uint8_t cmd = GD5F2GQ5UE_WRITE_ENABLE_CMD;

	status = FLASH_Transmit(dev, &cmd, sizeof(cmd));
	if (status != ESP_OK)
		return ESP_FAIL;

	uint8_t ret, retry = 0;
	do {
		status = GD5F2GQ5UE_GetFeature(dev, GD5F2GQ5UE_STATUS_1_REG_ADDR, &ret);
		if (++retry > GD5F2GQ5UE_WRITE_EN_RETRY)
			return ESP_FAIL;

	} while (!(ret & GD5F2GQ5UE_STATUS_1_WEL) || status != ESP_OK);

	return ESP_OK;
}

/* disable write commands. */
static esp_err_t GD5F2GQ5UE_WriteDisable(flash_t* dev)
{
	esp_err_t status;
	uint8_t cmd = GD5F2GQ5UE_WRITE_DISABLE_CMD;

	status = FLASH_Transmit(dev, &cmd, sizeof(cmd));
	if (status != ESP_OK)
		return ESP_FAIL;

	uint8_t ret, retry = 0;
	do {
		status = GD5F2GQ5UE_GetFeature(dev, GD5F2GQ5UE_STATUS_1_REG_ADDR, &ret);
		if (++retry > GD5F2GQ5UE_WRITE_EN_RETRY)
			return ESP_FAIL;

	} while ((ret & GD5F2GQ5UE_STATUS_1_WEL) || status != ESP_OK);

	return ESP_OK;
}

/* check ecc status after reading a page. */
static uint8_t GD5F2GQ5UE_CheckECC(flash_t* dev)
{
	uint8_t resp;
	uint8_t ecc;

	GD5F2GQ5UE_GetFeature(dev, GD5F2GQ5UE_STATUS_1_REG_ADDR, &resp);
	ecc = (resp & GD5F2GQ5UE_STATUS_1_ECCS1) | (resp & GD5F2GQ5UE_STATUS_1_ECCS0);
	ecc >>= 4;

	if (ecc == 0)
		return 1;
	else
		return 0;
}

/********************************* End Of File ********************************/