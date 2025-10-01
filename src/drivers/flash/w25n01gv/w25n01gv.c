/**
 *	@file     w25n01gv.c
 *  @brief    W25N01GV NAND flash driver.
 *  @version  v1.1.
 *  @author   LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>

#include "w25n01gv.h"
#include "w25n01gv_def.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* W25N01GV vtable */
flash_vtable w25n01gv_vtable;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* read value of flash register. */
static uint8_t W25N01GV_GetFeature(flash_t* dev, uint8_t regAddr);

/* write value into flash register. */
static esp_err_t W25N01GV_SetFeature(flash_t* dev, uint8_t regAddr, uint8_t regVal);

/* enable write commands. */
static esp_err_t W25N01GV_WriteEnable(flash_t* dev);

/* disable write commands. */
static esp_err_t W25N01GV_WriteDisable(flash_t* dev);

/* check ecc status after reading a page. */
static uint8_t W25N01GV_CheckECC(flash_t* dev);

/*******************************************************************************
 * Code
 ******************************************************************************/

esp_err_t W25N01GV_Identify(flash_t* dev, uint8_t jedecID, uint16_t devID)
{

	if (jedecID != W25N01GV_JEDEC_ID || devID != W25N01GV_DEVICE_ID)
		return ESP_FAIL;

	w25n01gv_vtable.busy = W25N01GV_Busy;
	w25n01gv_vtable.configure = W25N01GV_Configure;
	w25n01gv_vtable.blockErase = W25N01GV_BlockErase;
	w25n01gv_vtable.areaErase = W25N01GV_AreaErase;
	w25n01gv_vtable.cacheLoad = W25N01GV_CacheLoad;
	w25n01gv_vtable.cacheRead = W25N01GV_CacheRead;
	w25n01gv_vtable.pageRead = W25N01GV_PageRead;
	w25n01gv_vtable.spareAreaRead = W25N01GV_SpareAreaRead;
	w25n01gv_vtable.cacheWrite = W25N01GV_CacheWrite;
	w25n01gv_vtable.cacheFlush = W25N01GV_CacheFlush;
	w25n01gv_vtable.pageWrite = W25N01GV_PageWrite;
	w25n01gv_vtable.reset = W25N01GV_Reset;

	dev->vtable = &w25n01gv_vtable;

	dev->geometry.blkCnt = W25N01GV_BLOCKS_PER_DEV;
	dev->geometry.pageCnt = W25N01GV_PAGES_PER_BLOCK;
	dev->geometry.blkSize = W25N01GV_PAGE_SIZE * W25N01GV_PAGES_PER_BLOCK;
	dev->geometry.pageSize = W25N01GV_PAGE_SIZE;
	dev->geometry.devSize = dev->geometry.blkSize * W25N01GV_BLOCKS_PER_DEV;
	dev->geometry.spareAreaSize = W25N01GV_SPARE_AREA_SIZE;
	dev->devInfo.jedecID = jedecID;
	dev->devInfo.devID = devID;

	return ESP_OK;
}

uint8_t W25N01GV_Busy(flash_t* dev)
{
	esp_err_t resp;

	resp = W25N01GV_GetFeature(dev, W25N01GV_STATUS_REG_ADDR);

	return (resp & W25N01GV_STATUS_REGISTER_DEV_BUSY_MASK);
}

esp_err_t W25N01GV_Configure(flash_t* dev)
{
	esp_err_t status = ESP_FAIL;

	status =
		W25N01GV_SetFeature(dev, W25N01GV_PROTECTION_REG_ADDR, W25N01GV_PROTECTION_NONE);
	if (status != ESP_OK)
		return ESP_FAIL;

	status = W25N01GV_SetFeature(dev, W25N01GV_CONFIG_REG_ADDR,
								 W25N01GV_CONFIG_ENABLE_ECC |
									 W25N01GV_CONFIG_BUFFER_READ_MODE);
	if (status != ESP_OK)
		return ESP_FAIL;

	return ESP_OK;
}

esp_err_t W25N01GV_BlockErase(flash_t* dev, uint32_t blkAddr, flash_exe_e exeType)
{
	esp_err_t status = ESP_FAIL;
	uint64_t cmdTime;
	uint32_t pa;
	uint8_t cmd[4] = {0};

	if (blkAddr > (W25N01GV_BLOCKS_PER_DEV - 1))
		return ESP_FAIL;

	if (W25N01GV_Busy(dev))
		return ESP_FAIL;

	status = W25N01GV_WriteEnable(dev);
	if (status != ESP_OK)
		return ESP_FAIL;

	pa = (blkAddr << 6);

	cmd[0] = W25N01GV_BLOCK_ERASE_CMD;
	cmd[1] = (pa >> 16) & 0xFFU;
	cmd[2] = (pa >> 8) & 0xFFU;
	cmd[3] = pa & 0xFFU;

	status = FLASH_Transmit(dev, cmd, sizeof(cmd));
	if (status != ESP_OK)
		return ESP_FAIL;

	cmdTime = dev->time_us();

	/* Check timeout period */
	if (exeType == FLASH_WAIT_EXECUTION) {
		while (W25N01GV_Busy(dev)) {
			if (dev->time_us() - cmdTime >= W25N01GV_BLOCK_ERASE_TIMEOUT_US)
				return ESP_FAIL;
		}
	}

	return ESP_OK;
}

esp_err_t W25N01GV_AreaErase(flash_t* dev, uint32_t start, uint32_t end)
{
	esp_err_t status = ESP_FAIL;

	if (start > end)
		return ESP_FAIL;

	for (uint32_t i = start; i <= end; i++) {
		status = W25N01GV_BlockErase(dev, i, FLASH_WAIT_EXECUTION);
		if (status != ESP_OK)
			return ESP_FAIL;
	}

	return ESP_OK;
}

esp_err_t W25N01GV_CacheLoad(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							 flash_exe_e exeType)
{
	esp_err_t status = ESP_FAIL;
	uint64_t cmdTime;
	uint32_t pa;
	uint8_t cmd[4] = {0};

	if (W25N01GV_Busy(dev))
		return ESP_FAIL;

	if (pageAddr > (W25N01GV_PAGES_PER_BLOCK - 1) ||
		blkAddr > (W25N01GV_BLOCKS_PER_DEV - 1))
		return ESP_FAIL;

	pa = (blkAddr << 6) | pageAddr;

	/* load page into internal buffer of flash device */
	cmd[0] = W25N01GV_PAGE_DATA_READ_CMD;
	cmd[1] = (pa >> 16) & 0xFFU;
	cmd[2] = (pa >> 8) & 0xFFU;
	cmd[3] = pa & 0xFFU;

	status = FLASH_Transmit(dev, cmd, sizeof(cmd));
	if (status != ESP_OK)
		return ESP_FAIL;

	cmdTime = dev->time_us();

	/* Check timeout period */
	if (exeType == FLASH_WAIT_EXECUTION) {
		while (W25N01GV_Busy(dev)) {
			if (dev->time_us() - cmdTime >= W25N01GV_PAGE_READ_TIMEOUT_US)
				return ESP_FAIL;
		}
	}

	return ESP_OK;
}

esp_err_t W25N01GV_CacheRead(flash_t* dev, uint16_t column, uint8_t* recv, uint16_t rSize,
							 flash_tran_e tranType)
{
	esp_err_t status = ESP_FAIL;
	uint8_t cmd[4] = {0};

	if (W25N01GV_Busy(dev))
		return ESP_FAIL;

	cmd[0] = W25N01GV_READ_DATA_CMD;
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
	}

	/** @todo: add bad block managment */
	if (!W25N01GV_CheckECC(dev)) {
		return ESP_OK;
	}

	return status;
}

esp_err_t W25N01GV_PageRead(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							uint8_t* recv, uint16_t rSize)
{
	esp_err_t status;

	status = W25N01GV_CacheLoad(dev, blkAddr, pageAddr, FLASH_WAIT_EXECUTION);
	if (status != ESP_OK)
		return ESP_FAIL;

	status = W25N01GV_CacheRead(dev, 0x00, recv, rSize, FLASH_POLLING);
	if (status != ESP_OK)
		return ESP_FAIL;

	return ESP_OK;
}

esp_err_t W25N01GV_SpareAreaRead(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
								 uint8_t* recv, uint16_t rSize)
{
	esp_err_t status;

	status = W25N01GV_CacheLoad(dev, blkAddr, pageAddr, FLASH_WAIT_EXECUTION);
	if (status != ESP_OK)
		return ESP_FAIL;

	status = W25N01GV_CacheRead(dev, W25N01GV_PAGE_SIZE, recv, rSize, FLASH_POLLING);
	if (status != ESP_OK)
		return ESP_FAIL;

	return ESP_OK;
}

esp_err_t W25N01GV_CacheWrite(flash_t* dev, uint16_t column, uint8_t* data,
							  uint16_t dataLen, uint8_t tranType)
{
	esp_err_t status = ESP_FAIL;
	uint8_t cmd[3] = {0};

	if ((column + dataLen) > W25N01GV_PAGE_SIZE)
		return ESP_FAIL;

	if (W25N01GV_Busy(dev))
		return ESP_FAIL;

	status = W25N01GV_WriteEnable(dev);
	if (status != ESP_OK)
		return ESP_FAIL;

	cmd[0] = W25N01GV_PROGRAM_LOAD_CMD;
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

esp_err_t W25N01GV_CacheFlush(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							  flash_exe_e exeType)
{
	esp_err_t status = ESP_FAIL;
	uint64_t cmdTime;
	uint32_t pa;
	uint8_t cmd[4];

	if (W25N01GV_Busy(dev))
		return ESP_FAIL;

	/** @todo: implement bad block managment */

	if (pageAddr > (W25N01GV_PAGES_PER_BLOCK - 1) ||
		blkAddr > (W25N01GV_BLOCKS_PER_DEV - 1))
		return 0;

	pa = (blkAddr << 6) | pageAddr;

	cmd[0] = W25N01GV_PROGRAM_EXECUTE_CMD;
	cmd[1] = (pa >> 16) & 0xFFU;
	cmd[2] = (pa >> 8) & 0xFFU;
	cmd[3] = pa & 0xFFU;

	status = FLASH_Transmit(dev, cmd, sizeof(cmd));
	if (status != ESP_OK)
		return ESP_FAIL;

	cmdTime = dev->time_us();

	/* Check timeout period */
	if (exeType == FLASH_WAIT_EXECUTION) {
		while (W25N01GV_Busy(dev)) {
			if (dev->time_us() - cmdTime >= W25N01GV_PAGE_PROGRAM_TIMEOUT_US)
				return ESP_FAIL;
		}
	}

	return ESP_OK;
}

esp_err_t W25N01GV_PageWrite(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							 uint8_t* data, uint16_t dataLen)
{
	esp_err_t status = ESP_FAIL;

	status = W25N01GV_CacheWrite(dev, 0, data, dataLen, FLASH_POLLING);
	if (status != ESP_OK)
		return ESP_FAIL;

	status = W25N01GV_CacheFlush(dev, blkAddr, pageAddr, FLASH_WAIT_EXECUTION);
	if (status != ESP_OK)
		return ESP_FAIL;

	return ESP_OK;
}

esp_err_t W25N01GV_Reset(flash_t* dev)
{
	esp_err_t status = ESP_FAIL;
	uint64_t cmdTime;
	uint8_t cmd = W25N01GV_DEV_RESET_CMD;

	if (W25N01GV_Busy(dev))
		return ESP_FAIL;

	status = FLASH_Transmit(dev, &cmd, sizeof(cmd));
	if (status != ESP_OK)
		return ESP_FAIL;

	cmdTime = dev->time_us();

	/* Check timeout period */
	while (W25N01GV_Busy(dev)) {
		if (dev->time_us() - cmdTime >= W25N01GV_DEV_RESET_TIMEOUT_US)
			return ESP_FAIL;
	}

	/* configure flash device again */
	status = W25N01GV_Configure(dev);
	if (status != ESP_OK)
		return ESP_FAIL;

	return ESP_OK;
}

/****************************** static functions ******************************/

/* read value of status register. */
static uint8_t W25N01GV_GetFeature(flash_t* dev, uint8_t regAddr)
{
	uint8_t resp;
	uint8_t cmd[2] = {W25N01GV_GET_FEATURE_CMD, regAddr};

	FLASH_Transfer(dev, cmd, sizeof(cmd), &resp, sizeof(resp));

	return resp;
}

/* write value into status register. */
static esp_err_t W25N01GV_SetFeature(flash_t* dev, uint8_t regAddr, uint8_t regVal)
{
	esp_err_t status = ESP_FAIL;
	uint8_t tmp;
	uint8_t cmd[3] = {0};

	if (W25N01GV_Busy(dev))
		return ESP_FAIL;

	cmd[0] = W25N01GV_SET_FEATURE_CMD;
	cmd[1] = regAddr;
	cmd[2] = regVal;

	status = FLASH_Transmit(dev, cmd, sizeof(cmd));
	if (status != ESP_OK)
		return ESP_FAIL;

	/* Check if the write was successful. */
	tmp = W25N01GV_GetFeature(dev, regAddr);
	if (tmp != regVal)
		return ESP_FAIL;

	return ESP_OK;
}

/* enable write commands. */
static esp_err_t W25N01GV_WriteEnable(flash_t* dev)
{
	esp_err_t status = ESP_FAIL;
	uint8_t cmd = W25N01GV_WRITE_ENABLE_CMD;

	status = FLASH_Transmit(dev, &cmd, sizeof(cmd));
	if (status != ESP_OK)
		return ESP_FAIL;

	uint8_t ret, retry = 0;
	do {
		ret = W25N01GV_GetFeature(dev, W25N01GV_STATUS_REG_ADDR);
		if (++retry > W25N01GV_WRITE_EN_RETRY)
			return ESP_FAIL;

	} while (!(ret & W25N01GV_STATUS_REGISTER_WRITE_EN_MASK));

	return ESP_OK;
}

/* disable write commands. */
static esp_err_t W25N01GV_WriteDisable(flash_t* dev)
{
	esp_err_t status;
	uint8_t cmd = W25N01GV_WRITE_DISABLE_CMD;

	status = FLASH_Transmit(dev, &cmd, sizeof(cmd));
	if (status != ESP_OK)
		return ESP_FAIL;

	uint8_t ret, retry = 0;
	do {
		ret = W25N01GV_GetFeature(dev, W25N01GV_STATUS_REG_ADDR);
		if (++retry > W25N01GV_WRITE_EN_RETRY)
			return ESP_FAIL;

	} while (ret & W25N01GV_STATUS_REGISTER_WRITE_EN_MASK);

	return ESP_OK;
}

/* check ecc status after reading a page. */
static uint8_t W25N01GV_CheckECC(flash_t* dev)
{
	uint8_t resp;
	uint8_t ecc;

	resp = W25N01GV_GetFeature(dev, W25N01GV_STATUS_REG_ADDR);
	ecc = (resp & W25N01GV_STATUS_REGISTER_ECC0_MASK);
	ecc |= (resp & W25N01GV_STATUS_REGISTER_ECC1_MASK);
	ecc >>= 4;

	if (ecc == W25N01GV_ECC_STATUS_SUCCESS1 || ecc == W25N01GV_ECC_STATUS_SUCCESS2)
		return 1;
	else
		return 0;
}

/********************************* End Of File ********************************/