/**
 * @file    sd.c
 * @brief   SDHC/SDXC driver
 * @version	1.0.0
 * @date    26.02.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "sd.h"
#include "sd_def.h"

#include <string.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* no resp on line */
#define SD_INVALID 0xFF

/* command token size [bytes] */
#define SD_CMD_SIZE_TOKEN_SIZE 6
/* token indicating start of data */
#define SD_START_DATA_TOKEN 0xFE
/* token indicating data was received correctly */
#define SD_DATA_OK_TOKEN 0x05
/* dont care token */
#define SD_DONT_CARE 0xFF

/* check R1 flags */
#define SD_R1_CHECK_RES_FIELDS(x) ((x & 0x80) == 0x00)
#define SD_R1_CHECK_IN_IDLE_STATE(x) ((x >> 0) & 0x01)
#define SD_R1_CHECK_ERASE_RESET(x) ((x >> 1) & 0x01)
#define SD_R1_CHECK_ILLEGAL_COMMAND(x) ((x >> 2) & 0x01)
#define SD_R1_CHECK_COM_CRC_ERROR(x) ((x >> 3) & 0x01)
#define SD_R1_CHECK_ERASE_SEQUENCE_ERROR(x) ((x >> 4) & 0x01)
#define SD_R1_CHECK_ADDRESS_ERROR(x) ((x >> 5) & 0x01)
#define SD_R1_CHECK_PARAMETER_ERROR(x) ((x >> 6) & 0x01)

/* data resp token */
#define SD_DATA_RESP_TOKEN_MASK(x) ((x) & 0b00011111)
#define SD_DATA_RESP_TOKEN_DATA_ACCEPTED 0b00000101
#define SD_DATA_RESP_TOKEN_DATA_REJECTED_DUE_CRC_ERROR 0b00001011
#define SD_DATA_RESP_TOKEN_DATA_REJECTED_DUE_WRITE_ERROR 0b00001101

/* card specific data size in bytes */
#define SD_CSD_SIZE 16

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

typedef struct __attribute__((__packed__)) sd_format_r1_t {
	uint8_t val;
} sd_format_r1_t;

typedef struct __attribute__((__packed__)) sd_format_r3_t {
	sd_format_r1_t r1;

	uint8_t val[4];
} sd_format_r3_t;

typedef struct __attribute__((__packed__)) sd_format_r7_t {
	sd_format_r1_t r1;

	uint8_t val[4];

} sd_format_r7_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* assert SD card */
static void sd_assert(sd_t* sd);

/* deassert SD card */
static void sd_deassert(sd_t* sd);

/* send data to SD card */
static uint32_t sd_write(sd_t* sd, uint8_t* tx, uint32_t size);

/* receive data from SD card */
static uint32_t sd_read(sd_t* sd, uint8_t* rx, uint32_t size);

/* send command */
static uint32_t sd_send_cmd(sd_t* sd, uint8_t cmd, uint32_t arg, uint8_t crc7);

/* get R1 response */
static uint32_t sd_r1_get(sd_t* sd, sd_format_r1_t* r1);

/* get R3 response */
static uint32_t sd_r3_get(sd_t* sd, sd_format_r3_t* r3);

/* get R7 response */
static uint32_t sd_r7_get(sd_t* sd, sd_format_r7_t* r7);

/* get data token */
static uint32_t sd_data_token_get(sd_t* sd, uint8_t* token, uint8_t retry_max);

/* send CMD8 and receive R1 */
static uint32_t sd_cmd0(sd_t* sd, sd_format_r1_t* r1);

/* send CMD8 and receive R7 response */
static uint32_t sd_cmd8(sd_t* sd, uint32_t arg, sd_format_r7_t* r7);

/* send CMD9 and receive R1 response */
static uint32_t sd_cmd9(sd_t* sd, sd_format_r1_t* r1, uint8_t* csd);

/* send CMD58 and receive R3 response */
static uint32_t sd_cmd58(sd_t* sd, sd_format_r3_t* r3);

/* send ACMD41 and receive R1 response */
static uint32_t sd_acmd41(sd_t* sd, sd_format_r1_t* r1);

/*******************************************************************************
 * Code
 ******************************************************************************/

uint32_t sd_init(sd_t* sd, spi_device_handle_t handle)
{
	uint8_t retry = 0;

	sd_format_r1_t r1;
	sd_format_r3_t r3;
	sd_format_r7_t r7;

	sd->initialized = 0;
	sd->handle = handle;

	/* go to idle state and enter SPI mode */
	retry = 0;
	do {
		if (sd_cmd0(sd, &r1))
			continue;

		if (SD_R1_CHECK_IN_IDLE_STATE(r1.val))
			break;

	} while (retry++ < SD_IDLE_RETRY_CNT);

	if (retry >= SD_IDLE_RETRY_CNT)
		return 1;

	/* send interface condition */
	retry = 0;
	do {
		/* if command is illegal card is SD_CARD_TYPE_V1_STANDARD_CAPACITY */
		if (sd_cmd8(sd, 0x000001AA, &r7))
			continue;

		uint8_t check_pattern = r7.val[3] & 0xFF;
		uint8_t voltage_accepted = r7.val[2] & 0x0F;

		if (check_pattern == 0xAA && voltage_accepted == 0x01)
			break;

	} while (retry++ < SD_IDLE_RETRY_CNT);

	if (retry >= SD_IDLE_RETRY_CNT)
		return 1;

	/* read OCR */
	retry = 0;
	do {
		if (sd_cmd58(sd, &r3))
			continue;

		sd->ocr = (r3.val[0] << 24) | (r3.val[1] << 16) | (r3.val[2] << 8) | r3.val[3];
		break;

	} while (retry++ < SD_IDLE_RETRY_CNT);

	if (retry >= SD_IDLE_RETRY_CNT)
		return 1;

	/* send operation condition */
	retry = 0;
	do {
		if (sd_acmd41(sd, &r1))
			continue;

		if (SD_R1_CHECK_IN_IDLE_STATE(r1.val) == 0)
			break;

	} while (retry++ < 10000);

	if (retry >= SD_IDLE_RETRY_CNT)
		return 1;

	/* get OCR again */
	retry = 0;
	do {
		if (sd_cmd58(sd, &r3))
			continue;

		sd->ocr = (r3.val[0] << 24) | (r3.val[1] << 16) | (r3.val[2] << 8) | r3.val[3];
		break;

	} while (retry++ < SD_IDLE_RETRY_CNT);

	if (retry >= SD_IDLE_RETRY_CNT)
		return 1;

	if (sd->ocr & 0x40000000)
		sd->info.type = SD_CARD_TYPE_V2_HIGH_OR_EXTENDED_CAPACITY;
	else
		sd->info.type = SD_CARD_TYPE_V2_STANDARD_CAPACITY;

	/* read CSD */
	uint8_t csd[SD_CSD_SIZE];

	retry = 0;
	do {
		if (sd_cmd9(sd, &r1, csd))
			continue;

		break;
	} while (retry++ < SD_IDLE_RETRY_CNT);

	if (retry >= SD_IDLE_RETRY_CNT)
		return 1;

	uint16_t read_bl_len;
	uint32_t c_size;

	read_bl_len = csd[5] & 0x0F;
	c_size = ((csd[7] & 0x3F) << 16) | (csd[8] << 8) | csd[9];

	sd->info.blk_len = 1 << read_bl_len;
	sd->info.blk_cnt = (c_size + 1) << 10;

	sd->initialized = 1;

	return 0;
}

uint32_t sd_blk_write(sd_t* sd, uint32_t addr, uint8_t* tx)
{
	sd_format_r1_t r1;
	uint8_t token = SD_START_DATA_TOKEN;

	if (!sd->initialized || sd_busy(sd))
		return 1;

	sd_assert(sd);

	/* send address */
	if (sd_send_cmd(sd, SD_DEF_CMD24, addr, SD_DONT_CARE)) {
		sd_deassert(sd);
		return 1;
	}

	/* poll for R1 response */
	if (sd_r1_get(sd, &r1)) {
		sd_deassert(sd);
		return 1;
	}

	/* send data token */
	if (sd_write(sd, &token, sizeof(token))) {
		sd_deassert(sd);
		return 1;
	}

	/* send data */
	if (sd_write(sd, tx, sd->info.blk_len)) {
		sd_deassert(sd);
		return 1;
	}

	/* send dummy CRC */
	uint8_t dummy = 0xFF;
	sd_write(sd, &dummy, sizeof(dummy));
	sd_write(sd, &dummy, sizeof(dummy));

	/* poll for token */
	do {
		if (sd_read(sd, &token, sizeof(token)))
			continue;

		if (token != SD_INVALID)
			break;

	} while (1);

	sd_deassert(sd);

	if ((token & 0x1F) != SD_DATA_OK_TOKEN)
		return 1;
	else
		return 0;
}

uint32_t sd_blk_read(sd_t* sd, uint32_t addr, uint8_t* rx)
{
	sd_format_r1_t r1;
	uint32_t ret = 1;
	uint8_t token;

	if (!sd->initialized || sd_busy(sd))
		return 1;

	sd_assert(sd);

	/* send address */
	if (sd_send_cmd(sd, SD_DEF_CMD17, addr, SD_DONT_CARE)) {
		sd_deassert(sd);
		return 1;
	}

	/* poll for R1 response */
	if (sd_r1_get(sd, &r1)) {
		sd_deassert(sd);
		return 1;
	}

	/* poll for token */
	do {
		if (sd_read(sd, &token, sizeof(token)))
			continue;

		if (token != SD_INVALID)
			break;
	} while (1);

	if (token == SD_START_DATA_TOKEN) {
		/* read data */
		if (sd_read(sd, rx, sd->info.blk_len)) {
			ret = 1;
		} else {
			/* dummy crc reads */
			uint16_t crc;
			sd_read(sd, (uint8_t*)&crc, sizeof(crc));
			ret = 0;
		}
	} else {
		ret = 1;
	}

	sd_deassert(sd);

	return ret;
}

uint32_t sd_busy(sd_t* sd)
{
	uint32_t ret = 0;
	uint8_t val = 0x00;

	sd_assert(sd);
	ret = sd_read(sd, &val, sizeof(val));
	sd_deassert(sd);

	if (ret || val == 0x00)
		return 1;
	else
		return 0;
}

/****************************** static functions ******************************/

static void sd_assert(sd_t* sd)
{
	gpio_set_level(sd->cs_gpio, 0);
}

static void sd_deassert(sd_t* sd)
{
	gpio_set_level(sd->cs_gpio, 1);
}

static uint32_t sd_write(sd_t* sd, uint8_t* tx, uint32_t size)
{
	esp_err_t ret = spi_device_transmit(sd->handle, &(spi_transaction_t){
														.length = size * 8,
														.tx_buffer = tx,
													});

	return (ret != ESP_OK) ? 1 : 0;
}

static uint32_t sd_read(sd_t* sd, uint8_t* rx, uint32_t size)
{
	esp_err_t ret = spi_device_transmit(sd->handle, &(spi_transaction_t){
														.length = size * 8,
														.rx_buffer = rx,
													});

	return (ret != ESP_OK) ? 1 : 0;
}

static uint32_t sd_send_cmd(sd_t* sd, uint8_t cmd, uint32_t arg, uint8_t crc7)
{
	uint32_t ret = 1;
	uint8_t token[SD_CMD_SIZE_TOKEN_SIZE] = {0};
	uint8_t dummy = 0xFF;

	/* some cards may require 8 clock cycles in order to properly receive cmd */
	ret = sd_write(sd, &dummy, sizeof(dummy));
	if (ret)
		return 1;

	token[0] = 0x40 | cmd;
	token[1] = (arg >> 24) & 0xFF;
	token[2] = (arg >> 16) & 0xFF;
	token[3] = (arg >> 8) & 0xFF;
	token[4] = arg & 0xFF;
	token[5] = (crc7 << 1) | 0x01;

	ret = sd_write(sd, token, sizeof(token));
	if (ret)
		return 1;

	return 0;
}

static uint32_t sd_r1_get(sd_t* sd, sd_format_r1_t* r1)
{
	uint8_t retry = 0;

	do {
		if (sd_read(sd, &r1->val, sizeof(r1->val)))
			continue;

		if (r1->val != 0xFF)
			break;

	} while (retry++ < 8);

	if (retry >= 8)
		return 1;

	if (!SD_R1_CHECK_RES_FIELDS(r1->val))
		return 1;

	if (SD_R1_CHECK_COM_CRC_ERROR(r1->val) || SD_R1_CHECK_ILLEGAL_COMMAND(r1->val))
		return 1;

	return 0;
}

static uint32_t sd_r3_get(sd_t* sd, sd_format_r3_t* r3)
{
	/* get R1 first */
	if (sd_r1_get(sd, &r3->r1))
		return 1;

	/* get R3 */
	if (sd_read(sd, r3->val, sizeof(r3->val)))
		return 1;

	return 0;
}

static uint32_t sd_r7_get(sd_t* sd, sd_format_r7_t* r7)
{
	/* get R1 first */
	if (sd_r1_get(sd, &r7->r1))
		return 1;

	if (sd_read(sd, r7->val, sizeof(r7->val)))
		return 1;

	return 0;
}

static uint32_t sd_data_token_get(sd_t* sd, uint8_t* token, uint8_t retry_max)
{
	uint8_t retry = 0;
	uint8_t rx;

	do {
		if (sd_read(sd, &rx, sizeof(rx)))
			continue;

		if (rx != SD_INVALID)
			break;

	} while (retry++ < retry_max);

	if (retry >= retry_max)
		return 1;

	*token = rx;

	return 0;
}

static uint32_t sd_send_app(sd_t* sd)
{
	sd_format_r1_t r1;

	sd_assert(sd);

	if (sd_send_cmd(sd, SD_DEF_CMD55, 0x00000000, SD_DONT_CARE)) {
		sd_deassert(sd);
		return 1;
	}

	if (sd_r1_get(sd, &r1)) {
		sd_deassert(sd);
		return 1;
	}

	sd_deassert(sd);

	return 0;
}

static uint32_t sd_cmd0(sd_t* sd, sd_format_r1_t* r1)
{
	sd_assert(sd);

	if (sd_send_cmd(sd, SD_DEF_CMD0, 0, SD_DEF_CMD0_CRC)) {
		sd_deassert(sd);
		return 1;
	}

	/* Get R1 response */
	if (sd_r1_get(sd, r1)) {
		sd_deassert(sd);
		return 1;
	}
	sd_deassert(sd);

	return 0;
}

static uint32_t sd_cmd8(sd_t* sd, uint32_t arg, sd_format_r7_t* r7)
{
	sd_assert(sd);
	if (sd_send_cmd(sd, SD_DEF_CMD8, arg, SD_DEF_CMD8_CRC)) {
		sd_deassert(sd);
		return 1;
	}

	/* get R7 response */
	if (sd_r7_get(sd, r7)) {
		sd_deassert(sd);
		return 1;
	}
	sd_deassert(sd);

	return 0;
}

static uint32_t sd_cmd9(sd_t* sd, sd_format_r1_t* r1, uint8_t* csd)
{
	uint8_t token;
	uint16_t crc;

	sd_assert(sd);
	if (sd_send_cmd(sd, SD_DEF_CMD9, 0x00000000, SD_DONT_CARE)) {
		sd_deassert(sd);
		return 1;
	}

	/* get R1 response */
	if (sd_r1_get(sd, r1)) {
		sd_deassert(sd);
		return 1;
	}

	/* get token */
	if (sd_data_token_get(sd, &token, 16)) {
		sd_deassert(sd);
		return 1;
	}

	if (token != SD_START_DATA_TOKEN) {
		sd_deassert(sd);
		return 1;
	}

	/* get csd */
	if (sd_read(sd, csd, SD_CSD_SIZE)) {
		sd_deassert(sd);
		return 1;
	}

	/* dummy crc reads */
	if (sd_read(sd, (uint8_t*)&crc, sizeof(crc))) {
		sd_deassert(sd);
		return 1;
	}

	sd_deassert(sd);

	return 0;
}

static uint32_t sd_cmd58(sd_t* sd, sd_format_r3_t* r3)
{
	sd_assert(sd);

	if (sd_send_cmd(sd, SD_DEF_CMD58, 0x00000000, SD_DONT_CARE)) {
		sd_deassert(sd);
		return 1;
	}

	if (sd_r3_get(sd, r3)) {
		sd_deassert(sd);
		return 1;
	}

	sd_deassert(sd);

	return 0;
}

static uint32_t sd_acmd41(sd_t* sd, sd_format_r1_t* r1)
{
	if (sd_send_app(sd))
		return 1;

	sd_assert(sd);

	if (sd_send_cmd(sd, SD_DEF_ACMD41, 0x40000000, SD_DONT_CARE)) {
		sd_deassert(sd);
		return 1;
	}

	if (sd_r1_get(sd, r1)) {
		sd_deassert(sd);
		return 1;
	}

	sd_deassert(sd);

	return 0;
}

/********************************* End Of File ********************************/