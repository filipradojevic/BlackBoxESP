// /**
//  * @file    task_work.h
//  * @brief   Task work - process background tasks
//  * @version 1.0.0
//  * @date    24.04.2025
//  * @author  BetaTehPro
//  */

// #ifndef TASK_WORK_H
// #define TASK_WORK_H

// #ifdef __cplusplus
// extern "C" {
// #endif

// /*******************************************************************************
//  * Includes
//  ******************************************************************************/

// #include "main.h"
// #include "types.h"
// #include <stdint.h>

// #include "lfs.h"
// #include "mav.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/queue.h"

// /*******************************************************************************
//  * Defines
//  ******************************************************************************/

// /*******************************************************************************
//  * Typedefs
//  ******************************************************************************/

// /*******************************************************************************
//  * Variables
//  ******************************************************************************/
// extern const char vendor_name[];
// extern const char model_name[];
// extern const char software_version[];
// extern const char hardware_version[];
// extern const char serial_number[];

// /* Extern temporary variables */
// extern uint64_t boot_time_ms;

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
// extern QueueHandle_t queue_ulog_sinusoid_test;
// extern QueueHandle_t queue_mav_ftp_command;

// /*******************************************************************************
//  * API
//  ******************************************************************************/

// /**
//  * @brief Parses a version string in the format "vX.Y.Z" to a 32-bit integer.
//  *
//  * @param version_str Pointer to the version string.
//  * @return Encoded version as 0x76MMmmpp ('v', major, minor, patch).
//  */
// static inline uint32_t parse_version_string(const char* version_str)
// {
// 	int major = 0, minor = 0, patch = 0;

// 	if (version_str[0] == 'v' || version_str[0] == 'V') {
// 		version_str++; // skip 'v'
// 	}

// 	sscanf(version_str, "%d.%d.%d", &major, &minor, &patch);

// 	return (((uint32_t)'v' & 0xFF) << 24) | ((major & 0xFF) << 16) |
// 		   ((minor & 0xFF) << 8) | ((patch & 0xFF) << 0);
// }

// /**
//  * @brief Creates and fills a mavlink_component_information_basic_t struct.
//  *
//  * @return Filled mavlink_component_information_basic_t structure.
//  */
// static inline mavlink_component_information_basic_t make_comp_info_basic(void)
// {
// 	mavlink_component_information_basic_t data;
// 	memset(&data, 0, sizeof(data));

// 	strncpy(data.vendor_name, vendor_name, sizeof(data.vendor_name));
// 	strncpy(data.model_name, model_name, sizeof(data.model_name));
// 	strncpy(data.software_version, software_version,
// 			sizeof(data.software_version));
// 	strncpy(data.hardware_version, hardware_version,
// 			sizeof(data.hardware_version));
// 	strncpy(data.serial_number, serial_number, sizeof(data.serial_number));

// 	data.capabilities =
// 		MAV_PROTOCOL_CAPABILITY_MAVLINK2 | MAV_PROTOCOL_CAPABILITY_FTP;
// 	data.time_manufacture_s = BUILD_UNIX_TIMESTAMP;
// 	data.time_boot_ms = boot_time_ms;

// 	return data;
// }

// /**
//  * @brief Creates and fills a mavlink_autopilot_version_t struct.
//  *
//  * @return Filled mavlink_autopilot_version_t structure.
//  */
// static inline mavlink_autopilot_version_t make_autopilot_version(void)
// {
// 	mavlink_autopilot_version_t data;
// 	memset(&data, 0, sizeof(data));

// 	data.capabilities =
// 		MAV_PROTOCOL_CAPABILITY_MAVLINK2 | MAV_PROTOCOL_CAPABILITY_FTP;
// 	data.uid = 0x0102030405060708ULL;
// 	data.board_version = (BOARD_TYPE << 16) | BOARD_REVISION;
// 	data.vendor_id = VENDOR_ID;
// 	data.product_id = PRODUCT_ID;
// 	data.flight_sw_version = parse_version_string(FLIGHT_CUSTOM_VERSION);
// 	data.os_sw_version = parse_version_string(OS_CUSTOM_VERSION);
// 	data.middleware_sw_version = (((uint32_t)'v' & 0xFF) << 24) |
// 								 ((LFS_VERSION_MAJOR & 0xFF) << 16) |
// 								 ((LFS_VERSION_MINOR & 0xFF) << 8) | (0 & 0xFF);

// 	for (int i = 0; i < PAYLOAD_LENGTH; i++) {
// 		data.flight_custom_version[i] = FLIGHT_CUSTOM_HASH[i];
// 		data.middleware_custom_version[i] = MIDDLEWARE_CUSTOM_HASH[i];
// 		data.os_custom_version[i] = OS_CUSTOM_HASH[i];
// 	}

// 	memset(data.uid2, 0, sizeof(data.uid2));
// 	strncpy((char*)data.uid2, serial_number, sizeof(data.uid2));

// 	return data;
// }

// static inline BaseType_t check_queues(void)
// {
// 	QueueHandle_t queues[] = {
// 		queue_ulog_lisum_manual_ctrl, queue_ulog_batt_status1,
// 		queue_ulog_batt_status2,	  queue_ulog_motor_scaled,
// 		queue_ulog_scaled_imu1,		  queue_ulog_scaled_imu2,
// 		queue_ulog_scaled_pressure,	  queue_ulog_altitude,
// 		queue_ulog_attitude1,		  queue_ulog_attitude2,
// 		queue_ulog_lisum_gnss_data1,  queue_ulog_lisum_gnss_data2,
// 		queue_ulog_lisum_gnss_data3,  queue_ulog_lisum_airspeed_data,
// 		queue_ulog_lisum_act_data,	  queue_ulog_sinusoid_test,
// 		queue_mav_ftp_command};

// 	for (size_t i = 0; i < sizeof(queues) / sizeof(queues[0]); i++) {
// 		if (queues[i] != NULL && uxQueueMessagesWaiting(queues[i]) > 0) {
// 			return pdTRUE;
// 		}
// 	}
// 	return pdFALSE;
// }

// /*******************************************************************************
//  * Prototypes
//  ******************************************************************************/

// void task_work(void* arg);

// #ifdef __cplusplus
// }
// #endif

// #endif /* TASK_WORK_H */