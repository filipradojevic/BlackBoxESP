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
#define QUEUE_NUMBERS 17

/* ---------------------------------------------------------------------------
 * System & Board Information
 * ------------------------------------------------------------------------- */
#define OS_CUSTOM_VERSION                                                      \
	OS_CUSTOM_HASH		 /* From src/middleware/FreeRTOS/manifest.yml */
#define BOARD_TYPE 20	 /* Ethernet port marked by marker */
#define BOARD_REVISION 2 /* Hardware Version V0.2 */
#define VENDOR_ID 0x4254 /* 'B' 'T' - Company Identifier */
#define PRODUCT_ID 20	 /* Model Number */

/* ---------------------------------------------------------------------------
 * LED / Blinky Task
 * ------------------------------------------------------------------------- */
#define BLINKY_PERIOD_MS 500 /* LED green blinky period [ms] */

/* ---------------------------------------------------------------------------
 * SD Card Configuration (SPI Mode)
 * ------------------------------------------------------------------------- */
#define SD_INIT_MAX_RETRY_CNT 32		   /* Max retries during init */

/* ---------------------------------------------------------------------------
 * Telemetry UART Interface
 * ------------------------------------------------------------------------- */

/* -----------------------------------------------
 * Logger UART Interface
 * ------------------------------------------------------------------------- */


/* ---------------------------------------------------------------------------
 * Flash Memory (External SSP)
 * ------------------------------------------------------------------------- */
#define FLASH_CLOCK_RATE_HZ 22000000		  /* Flash clock (22 MHz) */
#define FLASH_INIT_MAX_RETRY_CNT 100		  /* Max retries during init */

#define FLASH_SPI_HOST SPI2_HOST
#define FLASH_MISO_PIN GPIO_NUM_12 // change to your board pin
#define FLASH_MOSI_PIN GPIO_NUM_11 // change to your board pin
#define FLASH_SCLK_PIN GPIO_NUM_10 // change to your board pin
#define FLASH_CS_PIN GPIO_NUM_9	   // change to your board pin

#define SD_SPI_HOST SPI3_HOST
#define SD_MISO_PIN GPIO_NUM_12 // change to your board pin
#define SD_MOSI_PIN GPIO_NUM_11 // change to your board pin
#define SD_SCLK_PIN GPIO_NUM_10 // change to your board pin
#define SD_CS_PIN GPIO_NUM_9	// change to your board pin

/* ---------------------------------------------------------------------------
 * Application Payload
 * ------------------------------------------------------------------------- */
#define PAYLOAD_LENGTH 8 /* Data length for ArduPilot version message */

/* ---------------------------------------------------------------------------
 * Choose between FLASH & SD card for LittleFS storage
 * ------------------------------------------------------------------------- */
#define FLASH_OR_SD_LITTLEFS 1 /* 1 is for Flash, 0 is for SD Card */

/* Component information basic variables */
#define VENDOR_NAME "BetaTehPro"
#define MODEL_NAME "HW_BLACK_BOX"
#define SOFTWARE_VERSION FLIGHT_CUSTOM_VERSION
#define HARDWARE_VERSION "DEV-Board V0.2"
#define SERIAL_NUMBER "S_N: 020"

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