/**
 * @file    task_lob.c
 * @brief   Task log - log data
 * @version 1.0.0
 * @date    10.04.2025
 * @author  BetaTehPro
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

/* Application */
#include "ftp_handler.h"
#include "ftp_helper.h"
#include "main.h"
#include "task_log.h"
#include "task_work.h"

/* Peripherals */
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"

/* External hardware drivers */
#include "flash.h"
#include "gd5f2gq5ue.h"
#include "sd.h"

/* Lib */
#include "mav.h"

/* Middleware */

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// LittleFS
#include "lfs.h"
#include "lfs_def.h"

// ULog
#include "ulog.h"
#include "ulog_altitude.h"
#include "ulog_attitude.h"
#include "ulog_battery_status.h"
#include "ulog_def.h"
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

#define MAIN_BATTERY_STATUS_ID 0 /* Main battery status multi-ID */
#define STBY_BATTERY_STATUS_ID 1 /* Standby battery status multi-ID */

#define INS_SCALED_IMU_ID 0		 /* Scaled IMU data multi-ID */
#define INS_COTS_SCALED_IMU_ID 1 /* COTS scaled IMU data multi-ID */

#define INS_ATTITUDE_ID 0	   /* Attitude multi-ID */
#define INS_COTS_ATTITUDE_ID 1 /* COTS attitude multi-ID */

#define INS_GNSS_DATA_ID 0		   /* GNSS data multi-ID */
#define INS_COTS_HR_GNSS_DATA_ID 1 /* COTS high-rate GNSS data multi-ID */
#define INS_COTS_LR_GNSS_DATA_ID 2 /* COTS low-rate GNSS data multi-ID */

#define LISUM_POWER_MOTOR_SCALED_DATA_ID 0 /* Power motor scaled data ID */

#define NAMED_VALUE_FLOAT_ID 1 /* Named value (float) ID */

#define ULOG_INFO_COUNT 1	/* Number of ULog information entries */
#define ULOG_PARAM_COUNT 11 /* Number of ULog parameter entries */

#define PAGE_SIZE_FLASH 2048U /* Flash page flash size in bytes */
#define PAGE_SIZE_SD 512U	  /* Flash page flash size in bytes */
#define LOOKAHEAD_SIZE 256U	  /* LittleFS lookahead buffer size in bytes */

#define TASK_PRIO_LOG 3 /* Task priority */

#define FILE_SYNC_THRESHOLD 100 * 2048 /* Threshold for file sync in bytes */

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/* ULog information entry */
typedef struct ulog_info_entry_t {
	char* key;
	char* val;
} ulog_info_entry_t;

/* ULog parameter entry */
typedef struct ulog_param_entry_t {
	char* key;
	int32_t val;
} ulog_param_entry_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* UDP Mavlink */
extern mav_t mav_gw_sky_handle;
extern mav_link_t mav_link_gw_sky_udp;

/* FreeRTOS */

// Queues
extern QueueHandle_t queue_ulog_batt_status1;
extern QueueHandle_t queue_ulog_batt_status2;
extern QueueHandle_t queue_ulog_motor_scaled;
extern QueueHandle_t queue_ulog_scaled_imu1;
extern QueueHandle_t queue_ulog_scaled_imu2;
extern QueueHandle_t queue_ulog_scaled_pressure;
extern QueueHandle_t queue_ulog_altitude;
extern QueueHandle_t queue_ulog_attitude1;
extern QueueHandle_t queue_ulog_attitude2;
extern QueueHandle_t queue_ulog_lisum_gnss_data1;
extern QueueHandle_t queue_ulog_lisum_gnss_data2;
extern QueueHandle_t queue_ulog_lisum_gnss_data3;
extern QueueHandle_t queue_ulog_lisum_airspeed_data;
extern QueueHandle_t queue_ulog_lisum_act_data;
extern QueueHandle_t queue_ulog_lisum_manual_ctrl;
extern QueueHandle_t queue_mav_ftp_command;
extern QueueHandle_t queue_ulog_sinusoid_test;

// Semaphores
extern SemaphoreHandle_t mutex_mav_ftp;
extern SemaphoreHandle_t semaphore_logging_ready;

// Tasks Handles
extern TaskHandle_t task_work_handle;
extern TaskHandle_t task_log_global;

/* Ulog Info */
static const ulog_info_entry_t ulog_infos[ULOG_INFO_COUNT] = {
	{.key = "char[11] sys_name", .val = "Hornet X-01"}};

static const ulog_param_entry_t ulog_params[ULOG_PARAM_COUNT] = {
	{.key = "int32_t NAMED_VALUE_FLOAT_ID", .val = NAMED_VALUE_FLOAT_ID},
	{.key = "int32_t MAIN_BATTERY_STATUS_ID", .val = MAIN_BATTERY_STATUS_ID},
	{.key = "int32_t STBY_BATTERY_STATUS_ID", .val = STBY_BATTERY_STATUS_ID},
	{.key = "int32_t INS_SCALED_IMU_ID", .val = INS_SCALED_IMU_ID},
	{.key = "int32_t INS_COTS_SCALED_IMU_ID", .val = INS_COTS_SCALED_IMU_ID},
	{.key = "int32_t INS_ATTITUDE_ID", .val = INS_ATTITUDE_ID},
	{.key = "int32_t LISUM_POWER_MOTOR_SCALED_DATA_ID",
	 .val = LISUM_POWER_MOTOR_SCALED_DATA_ID},
	{.key = "int32_t INS_COTS_ATTITUDE_ID", .val = INS_COTS_ATTITUDE_ID},
	{.key = "int32_t INS_GNSS_DATA_ID", .val = INS_GNSS_DATA_ID},
	{.key = "int32_t INS_COTS_HR_GNSS_DATA_ID",
	 .val = INS_COTS_HR_GNSS_DATA_ID},
	{.key = "int32_t INS_COTS_LOW_RATE_GNSS_DATA_ID",
	 .val = INS_COTS_LR_GNSS_DATA_ID}};

#if FLASH_OR_SD_LITTLEFS

/* Flash */
extern flash_t flash_dev;
// extern ssp_hal_slave_id_t flash_dev_slave;

/* Flash buffers and LittleFS structures */
static uint8_t read_buffer[PAGE_SIZE_FLASH]
	__attribute__((section("AHBSRAM0")));
static uint8_t prog_buffer[PAGE_SIZE_FLASH]
	__attribute__((section("AHBSRAM0")));
static uint8_t file_buffer[PAGE_SIZE_FLASH]
	__attribute__((section("AHBSRAM0")));
static uint8_t lookahead_buffer[LOOKAHEAD_SIZE]
	__attribute__((section("AHBSRAM0")));

#else

/* SD */
extern sd_t sd;

/* Flash buffers and LittleFS structures */
static uint8_t read_buffer[PAGE_SIZE_SD] __attribute__((section("AHBSRAM0")));
static uint8_t prog_buffer[PAGE_SIZE_SD] __attribute__((section("AHBSRAM0")));
static uint8_t lookahead_buffer[LOOKAHEAD_SIZE]
	__attribute__((section("AHBSRAM0")));
static uint8_t file_buffer[PAGE_SIZE_SD] __attribute__((section("AHBSRAM0")));

#endif

lfs_t lfs;
lfs_file_t lfs_file;
struct lfs_file_config file_cfg;
struct lfs_config lfs_cfg;
static uint32_t err_cnt = 0x00;

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_log(void* arg)
{
	/***************************************************************************
	 * Task Control & Timing
	 **************************************************************************/
	TickType_t sync_time = xTaskGetTickCount();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
	TickType_t log_t0;
#pragma GCC diagnostic pop
	int err = 0;
	BaseType_t data_available;

	/***************************************************************************
	 * Runtime Counters & Identifiers
	 **************************************************************************/
	uint32_t boot_count = 0;
	uint16_t batt_status1_id = 0;
	uint16_t batt_status2_id = 0;
	uint16_t motor_scaled_id = 0;
	uint16_t scaled_imu1_id = 0;
	uint16_t scaled_imu2_id = 0;
	uint16_t scaled_pressure_id = 0;
	uint16_t altitude_id = 0;
	uint16_t attitude1_id = 0;
	uint16_t attitude2_id = 0;
	uint16_t lisum_gnss_data1_id = 0;
	uint16_t lisum_gnss_data2_id = 0;
	uint16_t lisum_gnss_data3_id = 0;
	uint16_t lisum_airspeed_data_id = 0;
	uint16_t lisum_act_data_id = 0;
	uint16_t lisum_manual_ctrl_id = 0;
	uint16_t named_value_float_id = 0;

	/***************************************************************************
	 * ULog & MAVLink Data Structures
	 **************************************************************************/
	ulog_t ulog;
	ulog_battery_status_t ulog_batt_status1_data;
	ulog_battery_status_t ulog_batt_status2_data;
	ulog_lisum_power_motor_scaled_data_t ulog_lisum_motor_scaled_data;
	ulog_scaled_imu_t ulog_scaled_imu1_data;
	ulog_scaled_imu_t ulog_scaled_imu2_data;
	ulog_scaled_pressure_t ulog_scaled_pressure_data;
	ulog_altitude_t ulog_altitude_data;
	ulog_attitude_t ulog_attitude1_data;
	ulog_attitude_t ulog_attitude2_data;
	ulog_lisum_gnss_recv_data_t ulog_lisum_gnss_data1;
	ulog_lisum_gnss_recv_data_t ulog_lisum_gnss_data2;
	ulog_lisum_gnss_recv_data_t ulog_lisum_gnss_data3;
	ulog_lisum_sensor_airspeed_data_t ulog_lisum_airspeed_data;
	ulog_lisum_power_hornet_act_data_t ulog_lisum_act_data;
	ulog_lisum_manual_ctrl_hornet_t ulog_lisum_manual_ctrl_data;
	ulog_named_value_float_t ulog_named_data;

	uint16_t written_bytes = 0;

	/*------------------------------- LittleFS -------------------------------*/
#if FLASH_OR_SD_LITTLEFS

	flash_lfs_init_config(&lfs_cfg, &file_cfg, &flash_dev, read_buffer,
						  PAGE_SIZE_FLASH, prog_buffer, PAGE_SIZE_FLASH,
						  lookahead_buffer, LOOKAHEAD_SIZE, file_buffer,
						  PAGE_SIZE_FLASH);

#else

	sd_lfs_init_config(&lfs_cfg, &file_cfg, &sd, read_buffer, PAGE_SIZE_SD,
					   prog_buffer, PAGE_SIZE_SD, lookahead_buffer,
					   LOOKAHEAD_SIZE, file_buffer, PAGE_SIZE_SD);

#endif

	/* mount the filesystem */
	err = lfs_mount(&lfs, &lfs_cfg);

	/* reformat if mount fails, this should only happen on the first boot */
	if (err < 0) {
		err = lfs_format(&lfs, &lfs_cfg);
		if (err < 0) {
			err_cnt++;
		}

		err = lfs_mount(&lfs, &lfs_cfg);
		if (err < 0) {
			err_cnt++;
		}
	}

	/* read current boot count, used to create new log file */
	err = lfs_file_opencfg(&lfs, &lfs_file, "boot_count",
						   LFS_O_RDWR | LFS_O_CREAT, &file_cfg);
	if (err < 0) {
		err_cnt++;
	}

	err = lfs_file_read(&lfs, &lfs_file, &boot_count, sizeof(boot_count));
	if (err < 0) {
		err_cnt++;
	}

	/* update boot count */
	boot_count += 1;
	err = lfs_file_rewind(&lfs, &lfs_file);
	if (err < 0) {
		err_cnt++;
	}

	err = lfs_file_write(&lfs, &lfs_file, &boot_count, sizeof(boot_count));
	if (err < 0) {
		err_cnt++;
	}

	/* storage is not updated until file is closed or synced */
	err = lfs_file_close(&lfs, &lfs_file);
	if (err < 0) {
		err_cnt++;
	}

	/* create new file */
	char file_name[LFS_NAME_MAX];

	snprintf(file_name, sizeof(file_name), "log%u.ulg",
			 (unsigned int)boot_count);

	err = lfs_file_opencfg(&lfs, &lfs_file, file_name,
						   LFS_O_WRONLY | LFS_O_CREAT, &file_cfg);

	/* start ulog */
	ulog_start(&ulog, &lfs, &lfs_file, HAL_GetTimeUS);

	/* write informations */
	for (uint32_t i = 0; i < ULOG_INFO_COUNT; i++) {
		ulog_info_entry_t* info;

		info = (ulog_info_entry_t*)&ulog_infos[i];

		ulog_info(&ulog, info->key, strlen(info->key), info->val,
				  strlen(info->val));
	}

	/* write parameters */
	for (uint32_t i = 0; i < ULOG_PARAM_COUNT; i++) {
		ulog_param_entry_t* param;

		param = (ulog_param_entry_t*)&ulog_params[i];

		ulog_param(&ulog, param->key, strlen(param->key), &param->val);
	}

	// /* ulog formate messages */
	ulog_format_battery_status(&ulog);

	ulog_format_lisum_power_motor_scaled_data(&ulog);

	ulog_format_scaled_imu(&ulog);

	ulog_format_scaled_pressure(&ulog);

	ulog_format_altitude(&ulog);

	ulog_format_attitude(&ulog);

	ulog_format_lisum_gnss_recv_data(&ulog);

	ulog_format_lisum_sensor_airspeed_data(&ulog);

	ulog_format_lisum_power_hornet_act_data(&ulog);

	ulog_format_lisum_manual_ctrl_hornet(&ulog);

	ulog_format_named_value_float(&ulog);

	/* ulog subscription messages */
	ulog_subscribe_battery_status(&ulog, MAIN_BATTERY_STATUS_ID,
								  &batt_status1_id);

	ulog_subscribe_battery_status(&ulog, STBY_BATTERY_STATUS_ID,
								  &batt_status2_id);

	ulog_subscribe_lisum_power_motor_scaled_data(&ulog, 0x00, &motor_scaled_id);

	ulog_subscribe_scaled_imu(&ulog, INS_SCALED_IMU_ID, &scaled_imu1_id);

	ulog_subscribe_scaled_imu(&ulog, INS_COTS_SCALED_IMU_ID, &scaled_imu2_id);

	ulog_subscribe_scaled_pressure(&ulog, 0x00, &scaled_pressure_id);

	ulog_subscribe_altitude(&ulog, 0x00, &altitude_id);

	ulog_subscribe_attitude(&ulog, INS_ATTITUDE_ID, &attitude1_id);

	ulog_subscribe_attitude(&ulog, INS_COTS_ATTITUDE_ID, &attitude2_id);

	ulog_subscribe_lisum_gnss_recv_data(&ulog, INS_GNSS_DATA_ID,
										&lisum_gnss_data1_id);

	ulog_subscribe_lisum_gnss_recv_data(&ulog, INS_COTS_HR_GNSS_DATA_ID,
										&lisum_gnss_data2_id);

	ulog_subscribe_lisum_gnss_recv_data(&ulog, INS_COTS_LR_GNSS_DATA_ID,
										&lisum_gnss_data3_id);

	ulog_subscribe_lisum_sensor_airspeed_data(&ulog, 0x00,
											  &lisum_airspeed_data_id);

	ulog_subscribe_lisum_power_hornet_act_data(&ulog, 0x00, &lisum_act_data_id);

	ulog_subscribe_lisum_manual_ctrl_hornet(&ulog, 0x00, &lisum_manual_ctrl_id);

	ulog_subscribe_named_value_float(&ulog, 0x00, &named_value_float_id);

	// err = lfs_file_close(&lfs, &lfs_file);
	// if (err < 0)
	// 	err_cnt++;

	log_t0 = xTaskGetTickCount();

	/* Initialization is done successfully */
	gpio_set_level((gpio_num_t)26, !gpio_get_level((gpio_num_t)26));

	xTaskNotifyGive(task_work_handle);

	for (;;) {

		xSemaphoreTake(semaphore_logging_ready, portMAX_DELAY);

		do {
			data_available = pdFALSE;

			/* Battery status 1 */
			if (xQueueReceive(queue_ulog_batt_status1, &ulog_batt_status1_data,
							  0) == pdPASS) {
				written_bytes += sizeof(ulog_batt_status1_data) + 3;
				ulog_write_battery_status(&ulog, batt_status1_id,
										  &ulog_batt_status1_data);
				data_available = pdTRUE;
			}

			/* Battery status 1 */
			if (xQueueReceive(queue_ulog_batt_status2, &ulog_batt_status2_data,
							  0) == pdPASS) {
				written_bytes += sizeof(ulog_batt_status2_data) + 3;
				ulog_write_battery_status(&ulog, batt_status2_id,
										  &ulog_batt_status2_data);
				data_available = pdTRUE;
			}

			/* Lisum power motor scaled */
			if (xQueueReceive(queue_ulog_motor_scaled,
							  &ulog_lisum_motor_scaled_data, 0) == pdPASS) {
				written_bytes += sizeof(ulog_lisum_motor_scaled_data) + 3;
				ulog_write_lisum_power_motor_scaled_data(
					&ulog, motor_scaled_id, &ulog_lisum_motor_scaled_data);
				data_available = pdTRUE;
			}

			/* Scaled Imu 1 */
			if (xQueueReceive(queue_ulog_scaled_imu1, &ulog_scaled_imu1_data,
							  0) == pdPASS) {
				written_bytes += sizeof(ulog_scaled_imu1_data) + 3;
				ulog_write_scaled_imu(&ulog, scaled_imu1_id,
									  &ulog_scaled_imu1_data);
				data_available = pdTRUE;
			}

			/* Scaled Imu 2 */
			if (xQueueReceive(queue_ulog_scaled_imu2, &ulog_scaled_imu2_data,
							  0) == pdPASS) {
				written_bytes += sizeof(ulog_scaled_imu2_data) + 3;
				ulog_write_scaled_imu(&ulog, scaled_imu2_id,
									  &ulog_scaled_imu2_data);
				data_available = pdTRUE;
			}

			/* Scaled pressure */
			if (xQueueReceive(queue_ulog_scaled_pressure,
							  &ulog_scaled_pressure_data, 0) == pdPASS) {
				written_bytes += sizeof(ulog_scaled_pressure_data) + 3;
				ulog_write_scaled_pressure(&ulog, scaled_pressure_id,
										   &ulog_scaled_pressure_data);
				data_available = pdTRUE;
			}

			/* Altitude */
			if (xQueueReceive(queue_ulog_altitude, &ulog_altitude_data, 0) ==
				pdPASS) {
				written_bytes += sizeof(ulog_altitude_data) + 3;
				ulog_write_altitude(&ulog, altitude_id, &ulog_altitude_data);
				data_available = pdTRUE;
			}

			/* Attitude 1 */
			if (xQueueReceive(queue_ulog_attitude1, &ulog_attitude1_data, 0) ==
				pdPASS) {
				written_bytes += sizeof(ulog_attitude1_data) + 3;
				ulog_write_attitude(&ulog, attitude1_id, &ulog_attitude1_data);
				data_available = pdTRUE;
			}

			/* Attitude 2 */
			if (xQueueReceive(queue_ulog_attitude2, &ulog_attitude2_data, 0) ==
				pdPASS) {
				written_bytes += sizeof(ulog_attitude2_data) + 3;
				ulog_write_attitude(&ulog, attitude2_id, &ulog_attitude2_data);
				data_available = pdTRUE;
			}

			/* Lisum gnss 1 */
			if (xQueueReceive(queue_ulog_lisum_gnss_data1,
							  &ulog_lisum_gnss_data1, 0) == pdPASS) {
				written_bytes += sizeof(ulog_lisum_gnss_data1) + 3;
				ulog_write_lisum_gnss_recv_data(&ulog, lisum_gnss_data1_id,
												&ulog_lisum_gnss_data1);
				data_available = pdTRUE;
			}

			/* Lisum gnss 2 */
			if (xQueueReceive(queue_ulog_lisum_gnss_data2,
							  &ulog_lisum_gnss_data2, 0) == pdPASS) {
				written_bytes += sizeof(ulog_lisum_gnss_data2) + 3;
				ulog_write_lisum_gnss_recv_data(&ulog, lisum_gnss_data2_id,
												&ulog_lisum_gnss_data2);
				data_available = pdTRUE;
			}

			/* Lisum gnss 3 */
			if (xQueueReceive(queue_ulog_lisum_gnss_data3,
							  &ulog_lisum_gnss_data3, 0) == pdPASS) {
				written_bytes += sizeof(ulog_lisum_gnss_data3) + 3;
				ulog_write_lisum_gnss_recv_data(&ulog, lisum_gnss_data3_id,
												&ulog_lisum_gnss_data3);
				data_available = pdTRUE;
			}

			/* Lisum sensor airspeed */
			if (xQueueReceive(queue_ulog_lisum_airspeed_data,
							  &ulog_lisum_airspeed_data, 0) == pdPASS) {
				written_bytes += sizeof(ulog_lisum_airspeed_data) + 3;
				ulog_write_lisum_sensor_airspeed_data(
					&ulog, lisum_airspeed_data_id, &ulog_lisum_airspeed_data);
				data_available = pdTRUE;
			}

			/* Lisum power hornet actuator */
			if (xQueueReceive(queue_ulog_lisum_act_data, &ulog_lisum_act_data,
							  0) == pdPASS) {
				written_bytes += sizeof(ulog_lisum_act_data) + 3;
				ulog_write_lisum_power_hornet_act_data(&ulog, lisum_act_data_id,
													   &ulog_lisum_act_data);
				data_available = pdTRUE;
			}

			/* Lisum manual ctrl hornet */
			if (xQueueReceive(queue_ulog_lisum_manual_ctrl,
							  &ulog_lisum_manual_ctrl_data, 0) == pdPASS) {
				written_bytes += sizeof(ulog_lisum_manual_ctrl_data) + 3;
				ulog_write_lisum_manual_ctrl_hornet(
					&ulog, lisum_manual_ctrl_id, &ulog_lisum_manual_ctrl_data);
				data_available = pdTRUE;
			}

			/* Named value float */
			if (xQueueReceive(queue_ulog_sinusoid_test, &ulog_named_data, 0) ==
				pdPASS) {
				written_bytes += sizeof(ulog_named_data) + 3;
				ulog_write_named_value_float(&ulog, named_value_float_id,
											 &ulog_named_data);
				data_available = pdTRUE;
			}

		} while (data_available);

		/* write ulog sync */
		if (xTaskGetTickCount() - sync_time >= LOG_SYNC_PERIOD_MS) {
			sync_time = xTaskGetTickCount();

			// lfs_file_sync(&lfs, &lfs_file);

			ulog_sync(&ulog);

			gpio_set_level((gpio_num_t)26, !gpio_get_level((gpio_num_t)26));
		}
	}
}
