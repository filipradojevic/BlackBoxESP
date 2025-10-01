/**
 * @file    sd.h
 * @brief   SDHC/SDXC driver
 * @version	1.0.0
 * @date    26.02.2025
 * @author  LisumLab
 */

#ifndef SD_H
#define SD_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"

#ifdef SD_CONFIG
#include "sd_config.h"
#endif

/*******************************************************************************
 * Defines
 ******************************************************************************/

#ifndef SD_COM_TIMEOUT_MS
#define SD_COM_TIMEOUT_MS 1000
#endif

#ifndef SD_IDLE_RETRY_CNT
#define SD_IDLE_RETRY_CNT 10
#endif

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*! @brief Card Specific Data. */
typedef struct __attribute__((__packed__)) sd_csd_t {
	unsigned csd_structure : 2;
	unsigned res1 : 6;
	unsigned taac : 8;
	unsigned nsac : 8;
	unsigned tran_speed : 8;
	unsigned ccc : 12;
	unsigned read_bl_len : 4;
	unsigned read_bl_partial : 1;
	unsigned write_bl_misalign : 1;
	unsigned read_bl_misalign : 1;
	unsigned dsr_imp : 1;
	unsigned res2 : 6;
	unsigned c_size : 22;
	unsigned res3 : 1;
	unsigned erase_blk_len : 1;
	unsigned sector_size : 7;
	unsigned wp_grp_size : 7;
	unsigned wp_grp_enable : 1;
	unsigned res4 : 2;
	unsigned r2w_factor : 3;
	unsigned write_bl_len : 4;
	unsigned write_bl_partial : 1;
	unsigned res5 : 5;
	unsigned file_format_grp : 1;
	unsigned copy : 1;
	unsigned perm_write_protect : 1;
	unsigned tmp_write_protect : 1;
	unsigned file_format : 2;
	unsigned wp_upc : 1;
	unsigned res6 : 1;
	unsigned crc : 7;
	unsigned unused : 1;
} sd_csd_t;

/*! @brief Card Type. */
typedef enum sd_card_type_e {
	SD_CARD_TYPE_V1_STANDARD_CAPACITY = 0,
	SD_CARD_TYPE_V2_STANDARD_CAPACITY = 1,
	SD_CARD_TYPE_V2_HIGH_OR_EXTENDED_CAPACITY = 2
} sd_card_type_e;

/*! @brief SD Card Information. */
typedef struct sd_info_t {
	sd_card_type_e type; //!< card type
	uint16_t blk_len;	 //!< block length in bytes
	uint32_t blk_cnt;	 //!< block count
} sd_info_t;

typedef struct sd_t {
	spi_device_handle_t handle; //!< ESP-IDF SPI device handle
	gpio_num_t cs_gpio;			//!< GPIO pin za CS
	sd_info_t info;				//!< sd card information
	uint32_t ocr;				//!< sd card ocr
	uint8_t initialized;		//!< sd card initialization status
} sd_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

/**
 * @brief Initialize SD card.
 *
 * @param[in] sd        SD card instance.
 * @param[in] handle    SPI device handle.
 * @return 0 - init success, 1 - init fail
 */
uint32_t sd_init(sd_t* sd, spi_device_handle_t handle, gpio_num_t cs);

/**
 * @brief Write block of data.
 *
 * @param[in] sd    SD card instance.
 * @param[in] addr  Block address.
 * @param[in] tx    Write buffer (must be size of blk_len).
 * @return 0 - write success, 1 - write fail
 */
uint32_t sd_blk_write(sd_t* sd, uint32_t addr, uint8_t* tx);

/**
 * @brief Read block of data.
 *
 * @param[in] sd    SD card instance.
 * @param[in] addr  Block address.
 * @param[in] rx    Read buffer (must be size of blk_len).
 * @return 0 - read success, 1 - read fail
 */
uint32_t sd_blk_read(sd_t* sd, uint32_t addr, uint8_t* rx);

/**
 * @brief Check if SD card is busy.
 *
 * @param[in] sd    SD card instance.
 * @return 0 - free, 1 - busy
 */
uint32_t sd_busy(sd_t* sd);

#ifdef __cplusplus
}
#endif

#endif /* SD_H */