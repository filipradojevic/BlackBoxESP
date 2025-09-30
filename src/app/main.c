/**
 * @file    main.c
 * @brief   Black Box project.
 * @details This project acquires data from GW_SKY via UDP and records this
 *          data on block device. Logged data is stored on external flash
 *          memory using the LittleFS file system, ensuring reliability and
 *          wear-leveling.
 *
 * @version 1.0.0
 * @date    24.04.2025
 * @author  BetaTehPro
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "sdkconfig.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Application */
#include "main.h"
#include "task_log.h"
#include "task_work.h"
#include "types.h"

/* ESP-IDF / HAL */
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_cpu.h"
#include "esp_err.h"
#include "esp_timer.h"

/* External hardware drivers */
#include "eth.h"
#include "flash.h"
#include "flash_common.h"
#include "sd.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOSConfig_arch.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

/* Utilities */

// LittleFS
#include "lfs.h"
#include "mav.h"

// UDP
#include "udp.h"
#include "udp_tl.h"

// ULog
#include "ulog.h"
#include "ulog_altitude.h"
#include "ulog_attitude.h"
#include "ulog_battery_status.h"
#include "ulog_lisum_gnss_recv_data.h"
#include "ulog_lisum_manual_ctrl_hornet.h"
#include "ulog_lisum_power_hornet_act_data.h"
#include "ulog_lisum_power_motor_scaled_data.h"
#include "ulog_lisum_sensor_airspeed_data.h"
#include "ulog_named_value_float.h"
#include "ulog_scaled_imu.h"
#include "ulog_scaled_pressure.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Device information */

// UDP
const uint8_t dev_mac[6] = {0x4C, 0x50, 0x43, 0x01, 0x00, 0x02};
const uint8_t dev_ip[4] = {192, 168, 1, 102};

// gateway sky UDP information
const uint8_t gw_sky_mac[6] = {0x4C, 0x50, 0x43, 0x01, 0x00, 0x00};
const uint8_t gw_sky_ip[4] = {192, 168, 1, 100};
const uint16_t gw_sky_port = 1002;
const uint16_t gw_sky_local_port = 1002;

// ins UDP information
const uint8_t ins_mac[6] = {0x4C, 0x50, 0x43, 0x01, 0x00, 0x03};
const uint8_t ins_ip[4] = {192, 168, 1, 103};
const uint16_t ins_port = 1003;
const uint16_t ins_local_port = 1003;

/* Transport layers */
eth_hal_phy_t eth_phy;
esp_eth_handle_t eth_handle;
udp_tl_t mav_gw_sky_tl;
udp_tl_t mav_ins_tl;

// /* UDP */
udp_t udp;

/* FreeRTOS */
volatile uint32_t ulIdleCycleCount = 0UL;

/* Timers */
TimerHandle_t timer_blinky;

/* Queues */
QueueHandle_t queue_ulog_batt_status1;
QueueHandle_t queue_ulog_batt_status2;
QueueHandle_t queue_ulog_motor_scaled;
QueueHandle_t queue_ulog_scaled_imu1;
QueueHandle_t queue_ulog_scaled_imu2;
QueueHandle_t queue_ulog_scaled_pressure;
QueueHandle_t queue_ulog_altitude;
QueueHandle_t queue_ulog_attitude1;
QueueHandle_t queue_ulog_attitude2;
QueueHandle_t queue_ulog_lisum_gnss_data1;
QueueHandle_t queue_ulog_lisum_gnss_data2;
QueueHandle_t queue_ulog_lisum_gnss_data3;
QueueHandle_t queue_ulog_lisum_airspeed_data;
QueueHandle_t queue_ulog_lisum_act_data;
QueueHandle_t queue_ulog_lisum_manual_ctrl;
QueueHandle_t queue_mav_ftp_command;
QueueHandle_t queue_ulog_sinusoid_test;

/* Semaphores */
SemaphoreHandle_t mutex_mav_ftp;
SemaphoreHandle_t semaphore_logging_ready;

/* Tasks Handles */
TaskHandle_t task_work_handle = NULL;
TaskHandle_t task_log_global;

#if FLASH_OR_SD_LITTLEFS

/* Flash */
flash_t flash_dev;

#else

/* sd card */
sd_t sd;

#endif

/* Temporary variables */
uint64_t boot_time_ms = 0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void timer_blinky_cb(TimerHandle_t xTimer);

/*******************************************************************************
 * Code
 ******************************************************************************/

void app_main(void)
{

	gpio_reset_pin(LED_RGB_GPIO);
	gpio_set_direction(LED_RGB_GPIO, GPIO_MODE_OUTPUT);

	gpio_set_level(LED_RGB_GPIO, 1);

	/*------------------------------- FreeRTOS -------------------------------*/

	/* Queues batt */
	queue_ulog_batt_status1 = xQueueCreate(15, sizeof(ulog_battery_status_t));
	if (queue_ulog_batt_status1 == NULL)
		printf("Failed to create queue_ulog_batt_status1\n");

	queue_ulog_batt_status2 = xQueueCreate(1, sizeof(ulog_battery_status_t));
	if (queue_ulog_batt_status2 == NULL)
		printf("Failed to create queue_ulog_batt_status2\n");

	/* Queues motor */
	queue_ulog_motor_scaled = xQueueCreate(20, sizeof(ulog_lisum_power_motor_scaled_data_t));
	if (queue_ulog_motor_scaled == NULL)
		printf("Failed to create queue_ulog_motor_scaled\n");

	/* Queues scaled */
	queue_ulog_scaled_imu1 = xQueueCreate(8, sizeof(ulog_scaled_imu_t));
	if (queue_ulog_scaled_imu1 == NULL)
		printf("Failed to create queue_ulog_scaled_imu1\n");

	queue_ulog_scaled_imu2 = xQueueCreate(8, sizeof(ulog_scaled_imu_t));
	if (queue_ulog_scaled_imu2 == NULL)
		printf("Failed to create queue_ulog_scaled_imu2\n");

	queue_ulog_scaled_pressure = xQueueCreate(10, sizeof(ulog_scaled_pressure_t));
	if (queue_ulog_scaled_pressure == NULL)
		printf("Failed to create queue_ulog_scaled_pressure\n");

	/* Queues altitudes */
	queue_ulog_altitude = xQueueCreate(12, sizeof(ulog_altitude_t));
	if (queue_ulog_altitude == NULL)
		printf("Failed to create queue_ulog_altitude\n");

	queue_ulog_attitude1 = xQueueCreate(1, sizeof(ulog_attitude_t));
	if (queue_ulog_attitude1 == NULL)
		printf("Failed to create queue_ulog_attitude1\n");

	queue_ulog_attitude2 = xQueueCreate(1, sizeof(ulog_attitude_t));
	if (queue_ulog_attitude2 == NULL)
		printf("Failed to create queue_ulog_attitude2\n");

	/* Queues lisum */
	queue_ulog_lisum_gnss_data1 = xQueueCreate(1, sizeof(ulog_lisum_gnss_recv_data_t));
	if (queue_ulog_lisum_gnss_data1 == NULL)
		printf("Failed to create queue_ulog_lisum_gnss_data1\n");

	queue_ulog_lisum_gnss_data2 = xQueueCreate(1, sizeof(ulog_lisum_gnss_recv_data_t));
	if (queue_ulog_lisum_gnss_data2 == NULL)
		printf("Failed to create queue_ulog_lisum_gnss_data2\n");

	queue_ulog_lisum_gnss_data3 = xQueueCreate(1, sizeof(ulog_lisum_gnss_recv_data_t));
	if (queue_ulog_lisum_gnss_data3 == NULL)
		printf("Failed to create queue_ulog_lisum_gnss_data3\n");

	queue_ulog_lisum_airspeed_data = xQueueCreate(1, sizeof(ulog_lisum_sensor_airspeed_data_t));
	if (queue_ulog_lisum_airspeed_data == NULL)
		printf("Failed to create queue_ulog_lisum_airspeed_data\n");

	queue_ulog_lisum_act_data = xQueueCreate(10, sizeof(ulog_lisum_power_hornet_act_data_t));
	if (queue_ulog_lisum_act_data == NULL)
		printf("Failed to create queue_ulog_lisum_act_data\n");

	queue_ulog_lisum_manual_ctrl = xQueueCreate(1, sizeof(ulog_lisum_manual_ctrl_hornet_t));
	if (queue_ulog_lisum_manual_ctrl == NULL)
		printf("Failed to create queue_ulog_lisum_manual_ctrl\n");

	/* Queues ftp */
	queue_mav_ftp_command = xQueueCreate(1, sizeof(mavlink_file_transfer_protocol_t));
	if (queue_mav_ftp_command == NULL)
		printf("Failed to create queue_mav_ftp_command\n");

	/* Queues test */
	queue_ulog_sinusoid_test = xQueueCreate(15, sizeof(ulog_named_value_float_t));
	if (queue_ulog_sinusoid_test == NULL)
		printf("Failed to create queue_ulog_sinusoid_test\n");

	printf("Successfully created all queues.\n");

	/* Semaphores */
	mutex_mav_ftp = xSemaphoreCreateMutex();
	if (mutex_mav_ftp == NULL)
		printf("Failed to create mutex_mav_ftp\n");

	semaphore_logging_ready = xSemaphoreCreateBinary();
	if (semaphore_logging_ready == NULL)

		printf("Failed to create semaphore_logging_ready\n");

	printf("Successfully created all semaphores.\n");

	// #if FLASH_OR_SD_LITTLEFS

	// 	spi_bus_config_t flash_buscfg = {
	// 		.miso_io_num = FLASH_MISO_PIN,
	// 		.mosi_io_num = FLASH_MOSI_PIN,
	// 		.sclk_io_num = FLASH_SCLK_PIN,
	// 		.quadwp_io_num = -1,
	// 		.quadhd_io_num = -1,
	// 		.max_transfer_sz = 4096,
	// 	};

	// 	ESP_ERROR_CHECK(spi_bus_initialize(FLASH_SPI_HOST, &flash_buscfg, SPI_DMA_CH_AUTO));

	// 	spi_device_interface_config_t flash_devcfg = {
	// 		.clock_speed_hz = FLASH_CLOCK_RATE_HZ,
	// 		.mode = 0,
	// 		.spics_io_num = FLASH_CS_PIN,
	// 		.queue_size = 1,
	// 	};

	// 	spi_device_handle_t flash_spi_handle = NULL;

	// 	ESP_ERROR_CHECK(spi_bus_add_device(FLASH_SPI_HOST, &flash_devcfg, &flash_spi_handle));

	// 	memset(&flash_dev, 0, sizeof(flash_dev));

	// 	uint32_t retry_flash = 0;
	// 	esp_err_t status = ESP_FAIL;

	// 	do {
	// 		status = FLASH_Init(&flash_dev, flash_spi_handle, FLASH_CS_PIN, HAL_GetTimeUS);

	// 		if (status == ESP_OK) {
	// 			printf("Successfully initialized flash device\n");
	// 			break;
	// 		}

	// 		vTaskDelay(pdMS_TO_TICKS(1));

	// 	} while (retry_flash++ < FLASH_INIT_MAX_RETRY_CNT);

	// 	if (status != ESP_OK) {
	// 		/* toggle indicator pin (map to ESP GPIO_NUM_x) */
	// 		gpio_set_level((gpio_num_t)26, !gpio_get_level((gpio_num_t)26));
	// 		printf("Failed to initialize flash device\n");
	// 	}

	// 	/* Configure DMA channels if driver supports it (keeps original calls) */
	// 	FLASH_ConfigureDMA(&flash_dev, FLASH_DMA_CH_TYPE_RX, 0);
	// 	FLASH_ConfigureDMA(&flash_dev, FLASH_DMA_CH_TYPE_TX, 1);

	// #else

	/* -------------------------- SPI for SD card (ESP-IDF) ------------------------ */

	spi_bus_config_t sd_buscfg = {
		.miso_io_num = SD_MISO_PIN,
		.mosi_io_num = SD_MOSI_PIN,
		.sclk_io_num = SD_SCLK_PIN,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 4096,
	};

	ESP_ERROR_CHECK(spi_bus_initialize(SD_SPI_HOST, &sd_buscfg, SPI_DMA_CH_AUTO));

	spi_device_interface_config_t sd_devcfg = {
		.clock_speed_hz = SD_CLOCK_RATE_HZ,
		.mode = 0,
		.spics_io_num = SD_CS_PIN,
		.queue_size = 1,
	};

	spi_device_handle_t sd_spi_handle = NULL;

	ESP_ERROR_CHECK(spi_bus_add_device(SD_SPI_HOST, &sd_devcfg, &sd_spi_handle));

	memset(&sd, 0, sizeof(sd));

	uint32_t retry_sd = 0;
	esp_err_t status = ESP_FAIL;

	do {
		int32_t ret = sd_init(&sd, (void*)sd_spi_handle);

		if (ret == ESP_OK) {
			printf("Successfully initialized SD card\n");
			break;
		}

		vTaskDelay(pdMS_TO_TICKS(1));

	} while (retry_sd++ < SD_INIT_MAX_RETRY_CNT);

	if (retry_sd >= SD_INIT_MAX_RETRY_CNT || !sd.initialized) {
		printf("Failed to initialize SD card\n");
		gpio_set_level((gpio_num_t)25, 1);
	}

	// #endif

	HAL_ETH_Init(&eth_phy, dev_mac, eth_handle);

	/*--------------------------------- UDP ----------------------------------*/
	/* initialize UDP */
	udp_init(&udp, dev_mac, dev_ip, (udp_phy_t*)&eth_phy);

	/* add client MAC & IP to static ARP table */
	udp_arptab_add(&udp, gw_sky_mac, gw_sky_ip);
	udp_arptab_add(&udp, ins_mac, ins_ip);

	/*-------------------------------- UDP TL --------------------------------*/
	udp_tl_init(&mav_gw_sky_tl, &udp, (uint8_t*)gw_sky_ip, gw_sky_local_port, gw_sky_port);

	udp_tl_init(&mav_ins_tl, &udp, (uint8_t*)ins_ip, ins_local_port, ins_port);

	printf("Successfully initialized Ethernet and UDP\n");

	/*------------------------------- FreeRTOS -------------------------------*/
	BaseType_t result;

	result = xTaskCreate(task_work, "work", 4096, NULL, 1, &task_work_handle);
	if (result != pdPASS) {
		for (;;)
			;
	}

	printf("Successfully created work task\n");

	result = xTaskCreate(task_log, "log", 4096, NULL, 1, &task_log_global);
	if (result != pdPASS) {
		for (;;)
			;
	}

	printf("Successfully created log task\n");

	/* loop */
	while (1) {
		printf("Should never get here!\n");
		for (;;)
			;
	}
}
