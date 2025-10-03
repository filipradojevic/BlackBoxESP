// /**
//  * @file    task_lob.c
//  * @brief   Task log - log data
//  * @version 1.0.0
//  * @date    10.04.2025
//  * @author  BetaTehPro
//  */

// /*******************************************************************************
//  * Includes
//  ******************************************************************************/

// #include <stdint.h>

// /* Application */
// #include "ftp_handler.h"
// #include "ftp_helper.h"
// #include "main.h"
// #include "task_log.h"
// #include "task_work.h"

// /* Peripherals */
// #include "driver/gpio.h"
// #include "driver/spi_master.h"
// #include "esp_timer.h"
// #include "esp_task_wdt.h"

// /* External hardware drivers */
// #include "flash.h"
// #include "gd5f2gq5ue.h"
// #include "sd.h"

// /* Lib */
// #include "mav.h"

// /* Middleware */

// // FreeRTOS
// #include "freertos/FreeRTOS.h"
// #include "freertos/queue.h"
// #include "freertos/semphr.h"

// // LittleFS
// #include "lfs.h"
// #include "lfs_def.h"

// // ULog
// #include "ulog.h"
// #include "ulog_altitude.h"
// #include "ulog_attitude.h"
// #include "ulog_battery_status.h"
// #include "ulog_def.h"
// #include "ulog_lisum_gnss_recv_data.h"
// #include "ulog_lisum_manual_ctrl_hornet.h"
// #include "ulog_lisum_power_hornet_act_data.h"
// #include "ulog_lisum_power_motor_scaled_data.h"
// #include "ulog_lisum_sensor_airspeed_data.h"
// #include "ulog_named_value_float.h"
// #include "ulog_scaled_imu.h"
// #include "ulog_scaled_pressure.h"

// /*******************************************************************************
//  * Defines
//  ******************************************************************************/

// // #define MAIN_BATTERY_STATUS_ID 0 /* Main battery status multi-ID */
// // #define STBY_BATTERY_STATUS_ID 1 /* Standby battery status multi-ID */

// // #define INS_SCALED_IMU_ID 0		 /* Scaled IMU data multi-ID */
// // #define INS_COTS_SCALED_IMU_ID 1 /* COTS scaled IMU data multi-ID */

// // #define INS_ATTITUDE_ID 0	   /* Attitude multi-ID */
// // #define INS_COTS_ATTITUDE_ID 1 /* COTS attitude multi-ID */

// // #define INS_GNSS_DATA_ID 0		   /* GNSS data multi-ID */
// // #define INS_COTS_HR_GNSS_DATA_ID 1 /* COTS high-rate GNSS data multi-ID */
// // #define INS_COTS_LR_GNSS_DATA_ID 2 /* COTS low-rate GNSS data multi-ID */

// // #define LISUM_POWER_MOTOR_SCALED_DATA_ID 0 /* Power motor scaled data ID */

// // #define NAMED_VALUE_FLOAT_ID 1 /* Named value (float) ID */

// // #define ULOG_INFO_COUNT 1	/* Number of ULog information entries */
// // #define ULOG_PARAM_COUNT 11 /* Number of ULog parameter entries */

// // #define PAGE_SIZE_FLASH 2048U /* Flash page flash size in bytes */
// // #define PAGE_SIZE_SD 512U	  /* Flash page flash size in bytes */
// // #define LOOKAHEAD_SIZE 256U	  /* LittleFS lookahead buffer size in bytes */

// // #define TASK_PRIO_LOG 3 /* Task priority */

// // #define FILE_SYNC_THRESHOLD 100 * 2048 /* Threshold for file sync in bytes */

// /*******************************************************************************
//  * Typedefs
//  ******************************************************************************/

// // /* ULog information entry */
// // typedef struct ulog_info_entry_t {
// // 	char* key;
// // 	char* val;
// // } ulog_info_entry_t;

// // /* ULog parameter entry */
// // typedef struct ulog_param_entry_t {
// // 	char* key;
// // 	int32_t val;
// // } ulog_param_entry_t;

// /*******************************************************************************
//  * Variables
//  ******************************************************************************/

// /* UDP Mavlink */
// extern mav_t mav_gw_sky_handle;
// extern mav_link_t mav_link_gw_sky_udp;

// /* FreeRTOS */

// // Queues
// extern QueueHandle_t queue_ulog_batt_status1;
// extern QueueHandle_t queue_ulog_batt_status2;
// extern QueueHandle_t queue_ulog_motor_scaled;
// extern QueueHandle_t queue_ulog_scaled_imu1;
// extern QueueHandle_t queue_ulog_scaled_imu2;
// extern QueueHandle_t queue_ulog_scaled_pressure;
// extern QueueHandle_t queue_ulog_altitude;
// extern QueueHandle_t queue_ulog_attitude1;
// extern QueueHandle_t queue_ulog_attitude2;
// extern QueueHandle_t queue_ulog_lisum_gnss_data1;
// extern QueueHandle_t queue_ulog_lisum_gnss_data2;
// extern QueueHandle_t queue_ulog_lisum_gnss_data3;
// extern QueueHandle_t queue_ulog_lisum_airspeed_data;
// extern QueueHandle_t queue_ulog_lisum_act_data;
// extern QueueHandle_t queue_ulog_lisum_manual_ctrl;
// extern QueueHandle_t queue_mav_ftp_command;
// extern QueueHandle_t queue_ulog_sinusoid_test;

// // Semaphores
// extern SemaphoreHandle_t mutex_mav_ftp;
// extern SemaphoreHandle_t semaphore_logging_ready;

// // Tasks Handles
// extern TaskHandle_t task_work_handle;
// extern TaskHandle_t task_log_global;

// extern ulog_t ulog;
// extern uint32_t boot_count;
// extern uint16_t batt_status1_id;
// extern uint16_t batt_status2_id;
// extern uint16_t motor_scaled_id;
// extern uint16_t scaled_imu1_id;
// extern uint16_t scaled_imu2_id;
// extern uint16_t scaled_pressure_id;
// extern uint16_t altitude_id;
// extern uint16_t attitude1_id;
// extern uint16_t attitude2_id;
// extern uint16_t lisum_gnss_data1_id;
// extern uint16_t lisum_gnss_data2_id;
// extern uint16_t lisum_gnss_data3_id;
// extern uint16_t lisum_airspeed_data_id;
// extern uint16_t lisum_act_data_id;
// extern uint16_t lisum_manual_ctrl_id;
// extern uint16_t named_value_float_id;


// /*******************************************************************************
//  * Code
//  ******************************************************************************/

// void task_log(void* arg)
// {
// 	/***************************************************************************
// 	 * Task Control & Timing
// 	 **************************************************************************/
// 	TickType_t sync_time = xTaskGetTickCount();
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
// 	TickType_t log_t0;
// #pragma GCC diagnostic pop
// 	int err = 0;
// 	BaseType_t data_available;

// 	/***************************************************************************
// 	 * ULog & MAVLink Data Structures
// 	 **************************************************************************/
// 	ulog_battery_status_t ulog_batt_status1_data;
// 	ulog_battery_status_t ulog_batt_status2_data;
// 	ulog_lisum_power_motor_scaled_data_t ulog_lisum_motor_scaled_data;
// 	ulog_scaled_imu_t ulog_scaled_imu1_data;
// 	ulog_scaled_imu_t ulog_scaled_imu2_data;
// 	ulog_scaled_pressure_t ulog_scaled_pressure_data;
// 	ulog_altitude_t ulog_altitude_data;
// 	ulog_attitude_t ulog_attitude1_data;
// 	ulog_attitude_t ulog_attitude2_data;
// 	ulog_lisum_gnss_recv_data_t ulog_lisum_gnss_data1;
// 	ulog_lisum_gnss_recv_data_t ulog_lisum_gnss_data2;
// 	ulog_lisum_gnss_recv_data_t ulog_lisum_gnss_data3;
// 	ulog_lisum_sensor_airspeed_data_t ulog_lisum_airspeed_data;
// 	ulog_lisum_power_hornet_act_data_t ulog_lisum_act_data;
// 	ulog_lisum_manual_ctrl_hornet_t ulog_lisum_manual_ctrl_data;
// 	ulog_named_value_float_t ulog_named_data;


// 	log_t0 = xTaskGetTickCount();

// 	/* Initialization is done successfully */
// 	gpio_set_level((gpio_num_t)26, !gpio_get_level((gpio_num_t)26));

// 	vTaskPrioritySet(task_log_global, 1);
	
// 	printf("Initialized task_log and ready for operating!\n");

// 	for (;;) {

// 		xSemaphoreTake(semaphore_logging_ready, portMAX_DELAY);

// 		do {
// 			data_available = pdFALSE;

// 			/* Battery status 1 */
// 			if (xQueueReceive(queue_ulog_batt_status1, &ulog_batt_status1_data,
// 							  0) == pdPASS) {
// 				ulog_write_battery_status(&ulog, batt_status1_id,
// 										  &ulog_batt_status1_data);
// 				data_available = pdTRUE;
// 			}

// 			/* Battery status 1 */
// 			if (xQueueReceive(queue_ulog_batt_status2, &ulog_batt_status2_data,
// 							  0) == pdPASS) {
// 				ulog_write_battery_status(&ulog, batt_status2_id,
// 										  &ulog_batt_status2_data);
// 				data_available = pdTRUE;
// 			}

// 			/* Lisum power motor scaled */
// 			if (xQueueReceive(queue_ulog_motor_scaled,
// 							  &ulog_lisum_motor_scaled_data, 0) == pdPASS) {
// 				ulog_write_lisum_power_motor_scaled_data(
// 					&ulog, motor_scaled_id, &ulog_lisum_motor_scaled_data);
// 				data_available = pdTRUE;
// 			}

// 			/* Scaled Imu 1 */
// 			if (xQueueReceive(queue_ulog_scaled_imu1, &ulog_scaled_imu1_data,
// 							  0) == pdPASS) {
// 				ulog_write_scaled_imu(&ulog, scaled_imu1_id,
// 									  &ulog_scaled_imu1_data);
// 				data_available = pdTRUE;
// 			}

// 			/* Scaled Imu 2 */
// 			if (xQueueReceive(queue_ulog_scaled_imu2, &ulog_scaled_imu2_data,
// 							  0) == pdPASS) {
// 				ulog_write_scaled_imu(&ulog, scaled_imu2_id,
// 									  &ulog_scaled_imu2_data);
// 				data_available = pdTRUE;
// 			}

// 			/* Scaled pressure */
// 			if (xQueueReceive(queue_ulog_scaled_pressure,
// 							  &ulog_scaled_pressure_data, 0) == pdPASS) {
// 				ulog_write_scaled_pressure(&ulog, scaled_pressure_id,
// 										   &ulog_scaled_pressure_data);
// 				data_available = pdTRUE;
// 			}

// 			/* Altitude */
// 			if (xQueueReceive(queue_ulog_altitude, &ulog_altitude_data, 0) ==
// 				pdPASS) {
// 				ulog_write_altitude(&ulog, altitude_id, &ulog_altitude_data);
// 				data_available = pdTRUE;
// 			}

// 			/* Attitude 1 */
// 			if (xQueueReceive(queue_ulog_attitude1, &ulog_attitude1_data, 0) ==
// 				pdPASS) {
// 				ulog_write_attitude(&ulog, attitude1_id, &ulog_attitude1_data);
// 				data_available = pdTRUE;
// 			}

// 			/* Attitude 2 */
// 			if (xQueueReceive(queue_ulog_attitude2, &ulog_attitude2_data, 0) ==
// 				pdPASS) {
// 				ulog_write_attitude(&ulog, attitude2_id, &ulog_attitude2_data);
// 				data_available = pdTRUE;
// 			}

// 			/* Lisum gnss 1 */
// 			if (xQueueReceive(queue_ulog_lisum_gnss_data1,
// 							  &ulog_lisum_gnss_data1, 0) == pdPASS) {
// 				ulog_write_lisum_gnss_recv_data(&ulog, lisum_gnss_data1_id,
// 												&ulog_lisum_gnss_data1);
// 				data_available = pdTRUE;
// 			}

// 			/* Lisum gnss 2 */
// 			if (xQueueReceive(queue_ulog_lisum_gnss_data2,
// 							  &ulog_lisum_gnss_data2, 0) == pdPASS) {
// 				ulog_write_lisum_gnss_recv_data(&ulog, lisum_gnss_data2_id,
// 												&ulog_lisum_gnss_data2);
// 				data_available = pdTRUE;
// 			}

// 			/* Lisum gnss 3 */
// 			if (xQueueReceive(queue_ulog_lisum_gnss_data3,
// 							  &ulog_lisum_gnss_data3, 0) == pdPASS) {
// 				ulog_write_lisum_gnss_recv_data(&ulog, lisum_gnss_data3_id,
// 												&ulog_lisum_gnss_data3);
// 				data_available = pdTRUE;
// 			}

// 			/* Lisum sensor airspeed */
// 			if (xQueueReceive(queue_ulog_lisum_airspeed_data,
// 							  &ulog_lisum_airspeed_data, 0) == pdPASS) {
// 				ulog_write_lisum_sensor_airspeed_data(
// 					&ulog, lisum_airspeed_data_id, &ulog_lisum_airspeed_data);
// 				data_available = pdTRUE;
// 			}

// 			/* Lisum power hornet actuator */
// 			if (xQueueReceive(queue_ulog_lisum_act_data, &ulog_lisum_act_data,
// 							  0) == pdPASS) {
// 				ulog_write_lisum_power_hornet_act_data(&ulog, lisum_act_data_id,
// 													   &ulog_lisum_act_data);
// 				data_available = pdTRUE;
// 			}

// 			/* Lisum manual ctrl hornet */
// 			if (xQueueReceive(queue_ulog_lisum_manual_ctrl,
// 							  &ulog_lisum_manual_ctrl_data, 0) == pdPASS) {
// 				ulog_write_lisum_manual_ctrl_hornet(
// 					&ulog, lisum_manual_ctrl_id, &ulog_lisum_manual_ctrl_data);
// 				data_available = pdTRUE;
// 			}

// 			/* Named value float */
// 			if (xQueueReceive(queue_ulog_sinusoid_test, &ulog_named_data, 0) ==
// 				pdPASS) {
// 				ulog_write_named_value_float(&ulog, named_value_float_id,
// 											 &ulog_named_data);
// 				data_available = pdTRUE;
// 			}

// 		} while (data_available);

// 		/* write ulog sync */
// 		if (xTaskGetTickCount() - sync_time >= LOG_SYNC_PERIOD_MS) {
// 			sync_time = xTaskGetTickCount();

// 			// lfs_file_sync(&lfs, &lfs_file);

// 			ulog_sync(&ulog);

// 			gpio_set_level((gpio_num_t)26, !gpio_get_level((gpio_num_t)26));
// 		}
// 	}
// }
