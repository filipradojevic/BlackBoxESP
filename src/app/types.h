/**
 * @file    types.h
 * @brief   Project common defines and types.
 * @version 1.0.0
 * @date    24.04.2025
 * @author  BetaTehPro
 */

#ifndef TYPES_H
#define TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
// #include "lfs.h"
#include "esp_timer.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* General */
#define QUEUE_NUMBERS 17

/* Autopilot Version */

#define OS_CUSTOM_VERSION OS_CUSTOM_HASH /**< From src/middleware/FreeRTOS/manifest.yml */
#define BOARD_TYPE 20					 /**< Ethernet port marked by marker */
#define BOARD_REVISION 2				 /**< Hardware Version V0.2 */
#define VENDOR_ID 0x4254				 /**< 'B' 'T' - Company Identifier */
#define PRODUCT_ID 20					 /**< Model Number */

/* Component information basic */

#define VENDOR_NAME "BetaTehPro"			   /**< Company name */
#define MODEL_NAME "HW_BLACK_BOX"			   /**< Model name */
#define SOFTWARE_VERSION FLIGHT_CUSTOM_VERSION /**< Software version */
#define HARDWARE_VERSION "DEV-Board V0.2"	   /**< Hardware version */
#define SERIAL_NUMBER "S_N: 020"			   /**< Serial number */
#define PAYLOAD_LENGTH 8					   /**< Length of custom version strings */

/* Blinky Led */

#define BLINKY_PERIOD_MS 500	 /**< LED green blinky period [ms] */
#define LED_RGB_GPIO GPIO_NUM_21 /**< GPIO number for blinky LED */

/* Flash configuration */

#define FLASH_CLOCK_RATE_HZ 22000000 /**< Flash clock (22 MHz) */
#define FLASH_INIT_MAX_RETRY_CNT 32	 /**< Max retries during init */
#define FLASH_SPI_HOST SPI2_HOST	 /**< SPI host */
#define FLASH_MISO_PIN GPIO_NUM_12	 /**< FLASH_MISO pin */
#define FLASH_MOSI_PIN GPIO_NUM_11	 /**< FLASH_MOSI pin */
#define FLASH_SCLK_PIN GPIO_NUM_10	 /**< FLASH_SCLK pin */
#define FLASH_CS_PIN GPIO_NUM_9		 /**< FLASH_CS pin */

/* SD card configuration */

#define SD_CLOCK_RATE_HZ 100000	 /**< SD clock (100 kHz) */
#define SD_INIT_MAX_RETRY_CNT 32 /**< Max retries during init */
#define SD_SPI_HOST SPI2_HOST	 /**< SPI host */
#define SD_MISO_PIN GPIO_NUM_5	 /**< SD_MISO pin */
#define SD_MOSI_PIN GPIO_NUM_6	 /**< SD_MOSI pin */
#define SD_SCLK_PIN GPIO_NUM_7	 /**< SD_SCLK pin */
#define SD_CS_PIN GPIO_NUM_4	 /**< SD_CS pin */

/* Ethernet configuration */
#define W5500_ETH_MOSI GPIO_NUM_11
#define W5500_ETH_MISO GPIO_NUM_12
#define W5500_ETH_CLK GPIO_NUM_13
#define W5500_ETH_CS GPIO_NUM_14
#define W5500_ETH_INT GPIO_NUM_10
#define W5500_ETH_RST GPIO_NUM_9

/* PSRAM configuration */

/* Choose between FLASH & SD card for LittleFS storage */

#define FLASH_OR_SD_LITTLEFS 0 /**< 1 is for Flash, 0 is for SD Card */

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

static inline uint64_t HAL_GetTimeUS(void)
{
	return (uint64_t)esp_timer_get_time();
}

#ifdef __cplusplus
}
#endif

#endif /* TYPES_H */