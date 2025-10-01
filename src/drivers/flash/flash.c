/**
 *	@file     flash.c
 *  @brief    Library used for communication with flash memory.
 *  @version  v1.1.
 *  @author   LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "flash.h"
#include "flashconf.h"

#ifdef FLASH_USE_W25N01GV
#include "w25n01gv/w25n01gv.h"
#endif

#ifdef FLASH_USE_GD5F2GQ5UE
#include "gd5f2gq5ue/gd5f2gq5ue.h"
#endif

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

/* Flash SSP DMA Transfer Callback */
void prvFLASH_SPI_CALLBACK(spi_transaction_t* trans);

/*******************************************************************************
 * Code
 ******************************************************************************/

esp_err_t FLASH_Init(flash_t* dev, spi_device_handle_t handle,
					 gpio_num_t cs_gpio, uint64_t (*time_us)(void))
{
	esp_err_t status;
	uint8_t dev_identified = 0;
	uint8_t jedec_id;
	uint16_t dev_id;
	uint8_t resp[3];
	uint8_t cmd[2] = {FLASH_GET_JEDEC_ID_CMD, 0x00U};

	dev->handle = handle;
	dev->cs_gpio = cs_gpio;

	// CS pin HIGH po defaultu
	gpio_set_level(dev->cs_gpio, 1);

	status = FLASH_Transfer(dev, cmd, sizeof(cmd), resp, sizeof(resp));
	if (status != ESP_OK)
		return ESP_FAIL;

	jedec_id = resp[0];
	dev_id = (uint16_t)((resp[1] << 8) | resp[2]);

#ifdef FLASH_USE_W25N01GV
	if (!dev_identified) {
		status = W25N01GV_Identify(dev, jedec_id, dev_id);
		if (status == ESP_OK)
			dev_identified = 1;
	}
#endif

#ifdef FLASH_USE_GD5F2GQ5UE
	if (!dev_identified) {
		status = GD5F2GQ5UE_Identify(dev, jedec_id, dev_id);
		if (status == ESP_OK)
			dev_identified = 1;
	}
#endif

	if (!dev_identified)
		return ESP_FAIL;

	dev->time_us = time_us;

	FLASH_Reset(dev);

	return ESP_OK;
}

uint8_t FLASH_Busy(flash_t* dev)
{
	return dev->vtable->busy(dev);
}

void FLASH_ConfigureDMA(flash_t* dev, flash_dma_ch_type_t chType,
						uint8_t dmaChannel)
{
	if (chType == FLASH_DMA_CH_TYPE_RX)
		dev->dmaChannelRX = dmaChannel;
	else if (chType == FLASH_DMA_CH_TYPE_TX)
		dev->dmaChannelTX = dmaChannel;
}

void FLASH_ConfigureCallback(flash_t* dev, flash_tran_callback_t callback,
							 void* userData)
{
	dev->callback = callback;
	dev->userData = userData;
}

esp_err_t FLASH_BlockErase(flash_t* dev, uint32_t blkAddr, uint8_t exeType)
{
	return dev->vtable->blockErase(dev, blkAddr, exeType);
}

esp_err_t FLASH_AreaErase(flash_t* dev, uint32_t start, uint32_t end)
{
	return dev->vtable->areaErase(dev, start, end);
}

esp_err_t FLASH_CacheLoad(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
						  uint8_t exeType)
{
	return dev->vtable->cacheLoad(dev, blkAddr, pageAddr, exeType);
}

esp_err_t FLASH_CacheRead(flash_t* dev, uint16_t column, uint8_t* recv,
						  uint16_t rSize, flash_tran_e tranType)
{
	return dev->vtable->cacheRead(dev, column, recv, rSize, tranType);
}

esp_err_t FLASH_PageRead(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
						 uint8_t* recv, uint16_t rSize)
{
	return dev->vtable->pageRead(dev, blkAddr, pageAddr, recv, rSize);
}

esp_err_t FLASH_SpareAreaRead(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
							  uint8_t* recv, uint16_t rSize)
{
	return dev->vtable->spareAreaRead(dev, blkAddr, pageAddr, recv, rSize);
}

esp_err_t FLASH_CacheWrite(flash_t* dev, uint16_t column, uint8_t* data,
						   uint16_t dataLen, uint8_t tranType)
{
	return dev->vtable->cacheWrite(dev, column, data, dataLen, tranType);
}

esp_err_t FLASH_CacheFlush(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
						   uint8_t exeType)
{
	return dev->vtable->cacheFlush(dev, blkAddr, pageAddr, exeType);
}

esp_err_t FLASH_PageWrite(flash_t* dev, uint32_t blkAddr, uint32_t pageAddr,
						  uint8_t* data, uint16_t dataLen)
{
	return dev->vtable->pageWrite(dev, blkAddr, pageAddr, data, dataLen);
}

esp_err_t FLASH_Reset(flash_t* dev)
{
	return dev->vtable->reset(dev);
}

/****************************** static functions ******************************/

/* Flash SSP DMA Transfer Callback */
void prvFLASH_SPI_CALLBACK(spi_transaction_t* trans)
{
	flash_t* dev = (flash_t*)trans->user;

	// CS pin vraćamo u HIGH nakon završetka transfera
	gpio_set_level(dev->cs_gpio, 1);

	// Poziv tvoje definisane callback funkcije, ako postoji
	if (dev->callback) {
		// esp_err_t status = (trans->flags & SPI_TRANS_DONE) ? ESP_OK : ESP_FAIL;
		esp_err_t status = ESP_OK;
		dev->callback(status, dev->userData);
	}
}

/********************************* End Of File ********************************/