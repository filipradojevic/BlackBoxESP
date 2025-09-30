/**
 * @file    task_work.c
 * @brief   Task work - process background tasks
 * @version 1.0.0
 * @date    24.04.2025
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
#include "types.h"

/* Peripherals */
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"

/* External hardware drivers */
#include "flash.h"
#include "gd5f2gq5ue.h"

/* Lib */
#include "mav.h"

/* Middleware */

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// LittleFS
#include "lfs.h"
#include "lfs_def.h"

// UDP
#include "udp.h"
#include "udp_tl.h"

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

#define ARP_GRAT_PERIOD_MS 10000 /* gratuitous ARP period [ms] */

#define MAV_GW_SKY_TRACK_COUNT 19 /* MAVLink gateway sky track count */
#define MAV_INS_TRACK_COUNT 9	  /* MAVLink Ins track count */

#define MAV_PROTOCOL_CAPABILITY_MAVLINK2 ((uint64_t)1 << 0)
#define MAV_PROTOCOL_CAPABILITY_FTP ((uint64_t)1 << 10)

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* UDP Mavlink */
extern udp_t udp;

extern udp_tl_t mav_gw_sky_tl;
extern udp_tl_t mav_ins_tl;

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
extern QueueHandle_t queue_ulog_sinusoid_test;
extern QueueHandle_t queue_mav_ftp_command;

// Semaphores
extern SemaphoreHandle_t semaphore_logging_ready;

// Tasks Handles
extern TaskHandle_t task_work_handle;

// MAVLink information
static const uint8_t dev_mav_sysid = 0;
static const uint8_t dev_mav_compid = MAV_COMP_ID_USER3;

static const uint8_t pwr_man_mav_sysid = 0;
static const uint8_t pwr_man_mav_compid = MAV_COMP_ID_USER2;

static const uint8_t ins_mav_sysid = 0;
static const uint8_t ins_mav_compid = MAV_COMP_ID_USER4;

static const uint8_t act_master_mav_sysid = 0;
static const uint8_t act_master_mav_compid = MAV_COMP_ID_USER5;

static const uint8_t ins_cots_mav_sysid = 0;
static const uint8_t ins_cots_mav_compid = MAV_COMP_ID_USER14;

static const uint8_t heli_mav_sysid = 0;
static const uint8_t heli_mav_compid = MAV_COMP_ID_USER52;

static const uint8_t batt1_mav_sysid = 0;
static const uint8_t batt1_mav_compid = MAV_COMP_ID_BATTERY;

static const uint8_t batt2_mav_sysid = 0;
static const uint8_t batt2_mav_compid = MAV_COMP_ID_BATTERY2;

static const uint8_t mp_mav_sysid = 255;
static const uint8_t mp_mav_compid = MAV_COMP_ID_MISSIONPLANNER;

static const mav_track_t mav_gw_sky_tracks[MAV_GW_SKY_TRACK_COUNT] = {

	{.msgid = MAVLINK_MSG_ID_BATTERY_STATUS,
	 .sysid = batt1_mav_sysid,
	 .compid = batt1_mav_compid},

	{.msgid = MAVLINK_MSG_ID_BATTERY_STATUS,
	 .sysid = batt2_mav_sysid,
	 .compid = batt2_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_POWER_MOTOR_SCALED_DATA,
	 .sysid = pwr_man_mav_sysid,
	 .compid = pwr_man_mav_compid},

	{.msgid = MAVLINK_MSG_ID_SCALED_IMU,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_SCALED_PRESSURE,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_ALTITUDE,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_ATTITUDE,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_GNSS_RECV_DATA,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_SENSOR_AIRSPEED_DATA,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_POWER_HORNET_ACT_DATA,
	 .sysid = act_master_mav_sysid,
	 .compid = act_master_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_MANUAL_CTRL_HORNET,
	 .sysid = act_master_mav_sysid,
	 .compid = act_master_mav_compid},

	{.msgid = MAVLINK_MSG_ID_SCALED_IMU,
	 .sysid = ins_cots_mav_sysid,
	 .compid = ins_cots_mav_compid},

	{.msgid = MAVLINK_MSG_ID_ATTITUDE,
	 .sysid = ins_cots_mav_sysid,
	 .compid = ins_cots_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_GNSS_RECV_DATA,
	 .sysid = ins_cots_mav_sysid,
	 .compid = ins_cots_mav_compid},

	{.msgid = MAVLINK_MSG_ID_COMMAND_LONG,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_COMMAND_LONG,
	 .sysid = heli_mav_sysid,
	 .compid = heli_mav_compid},

	{.msgid = MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL,
	 .sysid = heli_mav_sysid,
	 .compid = heli_mav_compid},

	{.msgid = MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid}};

static const mav_track_t mav_ins_tracks[MAV_INS_TRACK_COUNT] = {

	{.msgid = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_SCALED_PRESSURE,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_ALTITUDE,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_POWER_MOTOR_SCALED_DATA,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_POWER_HORNET_ACT_DATA,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_BATTERY_STATUS,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_BATTERY_STATUS,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_SCALED_IMU,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_SCALED_IMU,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid}};

mav_t mav_gw_sky_handle;
mav_t mav_ins_handle;
mav_link_t mav_link_ins_udp;
mav_link_t mav_link_gw_sky_udp;

/* MAVLink Component Information Basic defines */
const char vendor_name[] = VENDOR_NAME;
const char model_name[] = MODEL_NAME;
const char software_version[] = SOFTWARE_VERSION;
const char hardware_version[] = HARDWARE_VERSION;
const char serial_number[] = SERIAL_NUMBER;

/* Extern temporary variables */
extern uint64_t boot_time_ms;

extern lfs_t lfs;
extern lfs_file_t lfs_file;
extern struct lfs_file_config file_cfg;

session_t sessions[FTP_MAX_SESSIONS];
int session_id;

uint8_t file_sync = 1;

BaseType_t is_empty;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void mav_gw_sky_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg);

static void mav_ins_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg);

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_work(void* arg)
{

	/* initialize MAVLink handle */
	mav_init(&mav_gw_sky_handle, dev_mav_sysid, dev_mav_compid);

	/* initialize MAVLink links */
	mav_link_init(&mav_link_gw_sky_udp, MAVLINK_COMM_2, mav_gw_sky_recv_cb,
				  NULL, (tl_t*)&mav_gw_sky_tl);

	mav_link_init(&mav_link_ins_udp, MAVLINK_COMM_0, mav_ins_recv_cb, NULL,
				  (tl_t*)&mav_ins_tl);

	/* connect MAVLink links to MAVLink handles */
	mav_link(&mav_gw_sky_handle, &mav_link_gw_sky_udp);
	mav_link(&mav_ins_handle, &mav_link_ins_udp);

	/* track gateway sky messages */
	for (uint8_t i = 0; i < MAV_GW_SKY_TRACK_COUNT; i++) {

		mav_track_t* track;

		track = (mav_track_t*)&mav_gw_sky_tracks[i];

		mav_track(&mav_gw_sky_handle, track->msgid, track->sysid,
				  track->compid);
	}

	for (uint8_t i = 0; i < MAV_INS_TRACK_COUNT; i++) {

		mav_track_t* track;

		track = (mav_track_t*)&mav_ins_tracks[i];

		mav_track(&mav_ins_handle, track->msgid, track->sysid, track->compid);
	}

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	TickType_t arp_grat_time = xTaskGetTickCount();

	const TickType_t HALF_SLICE_TICKS = 1; // minimum 1 tick ~357 µs

	TickType_t last_tick = xTaskGetTickCount();

	for (;;) {

		udp_process(&udp);

		TickType_t now = xTaskGetTickCount();

		/* data arrived, pass the CPU time to other task */
		if ((now - last_tick) >= HALF_SLICE_TICKS) {

			is_empty = check_queues();

			if (is_empty == pdTRUE) {

				last_tick = now;

				xSemaphoreGive(semaphore_logging_ready);

				taskYIELD();
			}
		}

		/* periodic ARP unchanged */
		if ((now - arp_grat_time) >= pdMS_TO_TICKS(ARP_GRAT_PERIOD_MS)) {

			arp_grat_time = now;

			udp_arp_grat(&udp);
		}
	}
}

static void mav_gw_sky_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg)
{

	switch (msg->msgid) {

	case MAVLINK_MSG_ID_HEARTBEAT: {

		gpio_set_level((gpio_num_t)26, !gpio_get_level((gpio_num_t)26));

		break;
	}
	case MAVLINK_MSG_ID_BATTERY_STATUS: {

		mavlink_battery_status_t data;

		mavlink_msg_battery_status_decode(msg, &data);

		ulog_battery_status_t log_data = {
			.timestamp = HAL_GetTimeUS(),
			.id = data.id,
			.battery_function = data.battery_function,
			.type = data.type,
			.temperature = data.temperature,
			.voltages[0] = data.voltages[0],
			.voltages[1] = data.voltages[1],
			.voltages[2] = data.voltages[2],
			.voltages[3] = data.voltages[3],
			.voltages[4] = data.voltages[4],
			.voltages[5] = data.voltages[5],
			.voltages[6] = data.voltages[6],
			.voltages[7] = data.voltages[7],
			.voltages[8] = data.voltages[8],
			.voltages[9] = data.voltages[9],
			.current_battery = data.current_battery,
			.current_consumed = data.current_consumed,
			.energy_consumed = data.energy_consumed,
			.battery_remaining = data.battery_remaining,
			.time_remaining = data.time_remaining,
			.charge_state = data.charge_state,
			.voltages_ext[0] = data.voltages_ext[0],
			.voltages_ext[1] = data.voltages_ext[1],
			.voltages_ext[2] = data.voltages_ext[2],
			.voltages_ext[3] = data.voltages_ext[3],
			.mode = data.mode,
			.fault_bitmask = data.fault_bitmask};

		if (msg->compid == batt1_mav_compid) {

			xQueueSendToBack(queue_ulog_batt_status1, &log_data, (TickType_t)0);

		} else if (msg->compid == batt2_mav_compid) {

			xQueueSendToBack(queue_ulog_batt_status2, &log_data, (TickType_t)0);
		}

		break;
	}
	case MAVLINK_MSG_ID_LISUM_POWER_MOTOR_SCALED_DATA: {

		mavlink_lisum_power_motor_scaled_data_t data;

		mavlink_msg_lisum_power_motor_scaled_data_decode(msg, &data);

		ulog_lisum_power_motor_scaled_data_t log_data = {
			.timestamp = HAL_GetTimeUS(),
			.time = data.time,
			.w_gg = data.w_gg,
			.w_gg_N = data.w_gg_N,
			.w_ft = data.w_ft,
			.w_ft_N = data.w_ft_N,
			.fuel_flow = data.fuel_flow,
			.oil_flow_arm = data.oil_flow_arm,
			.oil_flow_reducer = data.oil_flow_reducer,
			.battery_volt = data.battery_volt,
			.temp_front_bearing = data.temp_front_bearing,
			.temp_rear_bearing = data.temp_rear_bearing,
			.temp_elastic_bearing = data.temp_elastic_bearing,
			.temp_rigid_bearing = data.temp_rigid_bearing,
			.temp_input_oil = data.temp_input_oil,
			.temp_arm_oil = data.temp_arm_oil,
			.temp_reducer_oil = data.temp_reducer_oil,
			.temp_exhaust_fume = data.temp_exhaust_fume,
			.press_arm_oil = data.press_arm_oil,
			.fuel_level = data.fuel_level,
			.oil_level_bearing = data.oil_level_bearing,
			.oil_level_reducer = data.oil_level_reducer,
			.PWM_fuel_pump = data.PWM_fuel_pump,
			.current_fuel_pump = data.current_fuel_pump,
			.PWM_oil_pump = data.PWM_oil_pump,
			.current_oil_pump = data.current_oil_pump,
			.PWM_oil_suction_pump_reducer = data.PWM_oil_suction_pump_reducer,
			.PWM_oil_suction_pump_arm = data.PWM_oil_suction_pump_arm,
			.PWM_oil_pressure_pump = data.PWM_oil_pressure_pump,
			.activate = data.activate,
			.turn_on = data.turn_on,
			.valve_state = data.valve_state,
			.pressure_state = data.pressure_state,
			.warning = data.warning,
			.error = data.error,
			.EvnC[0] = data.EvnC[0],
			.EvnC[1] = data.EvnC[1],
			.EvnC[2] = data.EvnC[2],
			.EvnC[3] = data.EvnC[3],
			.EvnC[4] = data.EvnC[4],
			.EvnC[5] = data.EvnC[5],
			.EvnC[6] = data.EvnC[6],
			.EvnC[7] = data.EvnC[7],
			.res[0] = data.res[0],
			.res[1] = data.res[1],
			.res[2] = data.res[2],
			.res[3] = data.res[3],
			.res[4] = data.res[4],
			.res[5] = data.res[5],
			.res[6] = data.res[6],
			.res[7] = data.res[7]};

		xQueueSendToBack(queue_ulog_motor_scaled, &log_data, (TickType_t)0);

		break;
	}
	case MAVLINK_MSG_ID_SCALED_IMU: {
		mavlink_scaled_imu_t data;

		mavlink_msg_scaled_imu_decode(msg, &data);

		ulog_scaled_imu_t log_data = {.timestamp = HAL_GetTimeUS(),
									  .time_boot_ms = data.time_boot_ms,
									  .xacc = data.xacc,
									  .yacc = data.yacc,
									  .zacc = data.zacc,
									  .xgyro = data.xgyro,
									  .ygyro = data.ygyro,
									  .zgyro = data.zgyro,
									  .xmag = data.xmag,
									  .ymag = data.ymag,
									  .zmag = data.zmag,
									  .temperature = data.temperature};

		if (msg->compid == ins_mav_compid) {

			xQueueSendToBack(queue_ulog_scaled_imu1, &log_data, (TickType_t)0);

		} else if (msg->compid == ins_cots_mav_compid) {

			xQueueSendToBack(queue_ulog_scaled_imu2, &log_data, (TickType_t)0);
		}

		break;
	}
	case MAVLINK_MSG_ID_SCALED_PRESSURE: {

		mavlink_scaled_pressure_t data;

		mavlink_msg_scaled_pressure_decode(msg, &data);

		ulog_scaled_pressure_t log_data = {.timestamp = HAL_GetTimeUS(),
										   .time_boot_ms = data.time_boot_ms,
										   .press_abs = data.press_abs,
										   .press_diff = data.press_diff,
										   .temperature = data.temperature,
										   .temperature_press_diff =
											   data.temperature_press_diff};

		xQueueSendToBack(queue_ulog_scaled_pressure, &log_data, (TickType_t)0);

		break;
	}
	case MAVLINK_MSG_ID_ALTITUDE: {

		mavlink_altitude_t data;

		mavlink_msg_altitude_decode(msg, &data);

		ulog_altitude_t log_data = {.timestamp = HAL_GetTimeUS(),
									.time_usec = data.time_usec,
									.altitude_monotonic =
										data.altitude_monotonic,
									.altitude_amsl = data.altitude_amsl,
									.altitude_local = data.altitude_local,
									.altitude_relative = data.altitude_relative,
									.altitude_terrain = data.altitude_terrain,
									.bottom_clearance = data.bottom_clearance};

		xQueueSendToBack(queue_ulog_altitude, &log_data, (TickType_t)0);

		break;
	}
	case MAVLINK_MSG_ID_ATTITUDE: {

		mavlink_attitude_t data;

		mavlink_msg_attitude_decode(msg, &data);

		ulog_attitude_t log_data = {.timestamp = HAL_GetTimeUS(),
									.time_boot_ms = data.time_boot_ms,
									.roll = data.roll,
									.pitch = data.pitch,
									.yaw = data.yaw,
									.rollspeed = data.rollspeed,
									.pitchspeed = data.pitchspeed,
									.yawspeed = data.yawspeed};

		if (msg->compid == ins_mav_compid) {

			xQueueSendToBack(queue_ulog_attitude1, &log_data, (TickType_t)0);

		} else if (msg->compid == ins_cots_mav_compid) {

			xQueueSendToBack(queue_ulog_attitude2, &log_data, (TickType_t)0);
		}

		break;
	}
	case MAVLINK_MSG_ID_LISUM_GNSS_RECV_DATA: {

		mavlink_lisum_gnss_recv_data_t data;

		mavlink_msg_lisum_gnss_recv_data_decode(msg, &data);

		ulog_lisum_gnss_recv_data_t log_data = {.timestamp = HAL_GetTimeUS(),
												.dev_id = data.dev_id,
												.lon = data.lon,
												.lat = data.lat,
												.alt = data.alt,
												.head = data.head,
												.head_rtk = data.head_rtk,
												.vel_ne = data.vel_ne,
												.vel_d = data.vel_d,
												.fix_type = data.fix_type,
												.carr_sol = data.carr_sol,
												.rtk_head_valid =
													data.rtk_head_valid,
												.sv_num = data.sv_num};

		if (msg->compid == ins_mav_compid) {

			xQueueSendToBack(queue_ulog_lisum_gnss_data1, &log_data,
							 (TickType_t)0);

		} else if (msg->compid == ins_cots_mav_compid) {

			if (data.dev_id == 1) {

				xQueueSendToBack(queue_ulog_lisum_gnss_data2, &log_data,
								 (TickType_t)0);

			}

			else if (data.dev_id == 2) {

				xQueueSendToBack(queue_ulog_lisum_gnss_data3, &log_data,
								 (TickType_t)0);
			}
		}

		break;
	}
	case MAVLINK_MSG_ID_LISUM_SENSOR_AIRSPEED_DATA: {

		mavlink_lisum_sensor_airspeed_data_t data;

		mavlink_msg_lisum_sensor_airspeed_data_decode(msg, &data);

		ulog_lisum_sensor_airspeed_data_t log_data = {
			.timestamp = HAL_GetTimeUS(),
			.id = data.id,
			.airspeed = data.airspeed,
			.temperature = data.temperature,
			.raw_press = data.raw_press,
			.flags = data.flags};

		xQueueSendToBack(queue_ulog_lisum_airspeed_data, &log_data,
						 (TickType_t)0);

		break;
	}
	case MAVLINK_MSG_ID_LISUM_POWER_HORNET_ACT_DATA: {

		mavlink_lisum_power_hornet_act_data_t data;

		mavlink_msg_lisum_power_hornet_act_data_decode(msg, &data);

		ulog_lisum_power_hornet_act_data_t log_data = {
			.timestamp = HAL_GetTimeUS(),
			.pos_act1 = data.pos_act1,
			.abs_pos_act1 = data.abs_pos_act1,
			.vel_act1 = data.vel_act1,
			.curr_act1 = data.curr_act1,
			.sw_act1 = data.sw_act1,
			.abs_enc_sw_act1 = data.abs_enc_sw_act1,
			.pos_act2 = data.pos_act2,
			.abs_pos_act2 = data.abs_pos_act2,
			.vel_act2 = data.vel_act2,
			.curr_act2 = data.curr_act2,
			.sw_act2 = data.sw_act2,
			.abs_enc_sw_act2 = data.abs_enc_sw_act2,
			.pos_act3 = data.pos_act3,
			.abs_pos_act3 = data.abs_pos_act3,
			.vel_act3 = data.vel_act3,
			.curr_act3 = data.curr_act3,
			.sw_act3 = data.sw_act3,
			.abs_enc_sw_act3 = data.abs_enc_sw_act3,
			.pos_act4 = data.pos_act4,
			.abs_pos_act4 = data.abs_pos_act4,
			.vel_act4 = data.vel_act4,
			.curr_act4 = data.curr_act4,
			.sw_act4 = data.sw_act4,
			.abs_enc_sw_act4 = data.abs_enc_sw_act4};

		xQueueSendToBack(queue_ulog_lisum_act_data, &log_data, (TickType_t)0);

		break;
	}
	case MAVLINK_MSG_ID_LISUM_MANUAL_CTRL_HORNET: {

		mavlink_lisum_manual_ctrl_hornet_t data;

		mavlink_msg_lisum_manual_ctrl_hornet_decode(msg, &data);

		ulog_lisum_manual_ctrl_hornet_t log_data = {
			.timestamp = HAL_GetTimeUS(),
			.mode = data.mode,
			.pos_sp_act1 = data.pos_sp_act1,
			.pos_sp_act2 = data.pos_sp_act2,
			.pos_sp_act3 = data.pos_sp_act3,
			.pos_sp_act4 = data.pos_sp_act4};

		xQueueSendToBack(queue_ulog_lisum_manual_ctrl, &log_data,
						 (TickType_t)0);

		break;
	}
	case MAVLINK_MSG_ID_COMMAND_LONG: {

		mavlink_command_long_t data;

		mavlink_msg_command_long_decode(msg, &data);

		if (data.command == MAV_CMD_REQUEST_MESSAGE) {

			if (data.param1 == MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC) {

				mavlink_message_t tx_msg;

				mavlink_component_information_basic_t reply_info;

				reply_info = make_comp_info_basic();

				mavlink_msg_component_information_basic_encode_chan(
					dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_2, &tx_msg,
					&reply_info);

				mav_send(&mav_gw_sky_handle, &tx_msg);

				break;
			}

			else if (data.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
				mavlink_message_t tx_msg;

				mavlink_autopilot_version_t reply_info;

				reply_info = make_autopilot_version();

				mavlink_msg_autopilot_version_encode_chan(
					dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_2, &tx_msg,
					&reply_info);

				mav_send(&mav_gw_sky_handle, &tx_msg);

				break;
			}
		}

		break;
	}
	case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL: {

		mavlink_file_transfer_protocol_t data;

		mavlink_message_t tx_msg;

		mavlink_msg_file_transfer_protocol_decode(msg, &data);

		if (file_sync != 0) {

			lfs_file_close(&lfs, &lfs_file);

			file_sync = 0;
		}

		if (ftp_process_opcode(&data, &lfs, sessions, &file_cfg,
							   &mav_gw_sky_handle) != false) {

			mavlink_msg_file_transfer_protocol_encode_chan(
				dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_2, &tx_msg, &data);

			if (data.payload[BURST_COMPLETE] != 0U) {

				ftp_timeout(queue_mav_ftp_command, &mav_gw_sky_handle, &tx_msg,
							FTP_TRIES, FTP_DELAY);

			} else {

				mav_send(&mav_gw_sky_handle, &tx_msg);
			}
		}

		break;
	}

	default:
		break;
	}
}

static void mav_ins_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg)
{
	switch (msg->msgid) {

	case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: {

		mavlink_named_value_float_t data;

		mavlink_msg_named_value_float_decode(msg, &data);

		ulog_named_value_float_t log_data = {.time_boot_ms = data.time_boot_ms,
											 .timestamp = HAL_GetTimeUS(),
											 .value = data.value};

		xQueueSendToBack(queue_ulog_sinusoid_test, &log_data, (TickType_t)0);

		break;
	}
	case MAVLINK_MSG_ID_SCALED_PRESSURE: {

		mavlink_scaled_pressure_t data;

		mavlink_msg_scaled_pressure_decode(msg, &data);

		ulog_scaled_pressure_t log_data = {.time_boot_ms = data.time_boot_ms,
										   .timestamp = HAL_GetTimeUS(),
										   .press_abs = data.press_abs,
										   .press_diff = data.press_diff,
										   .temperature = data.temperature,
										   .temperature_press_diff =
											   data.temperature_press_diff};

		xQueueSendToBack(queue_ulog_scaled_pressure, &log_data, (TickType_t)0);

		break;
	}

	case MAVLINK_MSG_ID_ALTITUDE: {

		mavlink_altitude_t data;

		mavlink_msg_altitude_decode(msg, &data);

		ulog_altitude_t log_data = {.timestamp = HAL_GetTimeUS(),
									.time_usec = data.time_usec,
									.altitude_monotonic =
										data.altitude_monotonic,
									.altitude_amsl = data.altitude_amsl,
									.altitude_local = data.altitude_local,
									.altitude_relative = data.altitude_relative,
									.altitude_terrain = data.altitude_terrain,
									.bottom_clearance = data.bottom_clearance};

		xQueueSendToBack(queue_ulog_altitude, &log_data, (TickType_t)0);

		break;
	}
	case MAVLINK_MSG_ID_LISUM_POWER_MOTOR_SCALED_DATA: {

		mavlink_lisum_power_motor_scaled_data_t data;

		mavlink_msg_lisum_power_motor_scaled_data_decode(msg, &data);

		ulog_lisum_power_motor_scaled_data_t log_data = {
			.timestamp = HAL_GetTimeUS(),
			.time = data.time,
			.w_gg = data.w_gg,
			.w_gg_N = data.w_gg_N,
			.w_ft = data.w_ft,
			.w_ft_N = data.w_ft_N,
			.fuel_flow = data.fuel_flow,
			.oil_flow_arm = data.oil_flow_arm,
			.oil_flow_reducer = data.oil_flow_reducer,
			.battery_volt = data.battery_volt,
			.temp_front_bearing = data.temp_front_bearing,
			.temp_rear_bearing = data.temp_rear_bearing,
			.temp_elastic_bearing = data.temp_elastic_bearing,
			.temp_rigid_bearing = data.temp_rigid_bearing,
			.temp_input_oil = data.temp_input_oil,
			.temp_arm_oil = data.temp_arm_oil,
			.temp_reducer_oil = data.temp_reducer_oil,
			.temp_exhaust_fume = data.temp_exhaust_fume,
			.press_arm_oil = data.press_arm_oil,
			.fuel_level = data.fuel_level,
			.oil_level_bearing = data.oil_level_bearing,
			.oil_level_reducer = data.oil_level_reducer,
			.PWM_fuel_pump = data.PWM_fuel_pump,
			.current_fuel_pump = data.current_fuel_pump,
			.PWM_oil_pump = data.PWM_oil_pump,
			.current_oil_pump = data.current_oil_pump,
			.PWM_oil_suction_pump_reducer = data.PWM_oil_suction_pump_reducer,
			.PWM_oil_suction_pump_arm = data.PWM_oil_suction_pump_arm,
			.PWM_oil_pressure_pump = data.PWM_oil_pressure_pump,
			.activate = data.activate,
			.turn_on = data.turn_on,
			.valve_state = data.valve_state,
			.pressure_state = data.pressure_state,
			.warning = data.warning,
			.error = data.error,
			.EvnC[0] = data.EvnC[0],
			.EvnC[1] = data.EvnC[1],
			.EvnC[2] = data.EvnC[2],
			.EvnC[3] = data.EvnC[3],
			.EvnC[4] = data.EvnC[4],
			.EvnC[5] = data.EvnC[5],
			.EvnC[6] = data.EvnC[6],
			.EvnC[7] = data.EvnC[7],
			.res[0] = data.res[0],
			.res[1] = data.res[1],
			.res[2] = data.res[2],
			.res[3] = data.res[3],
			.res[4] = data.res[4],
			.res[5] = data.res[5],
			.res[6] = data.res[6],
			.res[7] = data.res[7]};

		xQueueSendToBack(queue_ulog_motor_scaled, &log_data, (TickType_t)0);

		break;
	}

	case MAVLINK_MSG_ID_LISUM_POWER_HORNET_ACT_DATA: {

		mavlink_lisum_power_hornet_act_data_t data;

		mavlink_msg_lisum_power_hornet_act_data_decode(msg, &data);

		ulog_lisum_power_hornet_act_data_t log_data = {
			.timestamp = HAL_GetTimeUS(),
			.pos_act1 = data.pos_act1,
			.abs_pos_act1 = data.abs_pos_act1,
			.vel_act1 = data.vel_act1,
			.curr_act1 = data.curr_act1,
			.sw_act1 = data.sw_act1,
			.abs_enc_sw_act1 = data.abs_enc_sw_act1,
			.pos_act2 = data.pos_act2,
			.abs_pos_act2 = data.abs_pos_act2,
			.vel_act2 = data.vel_act2,
			.curr_act2 = data.curr_act2,
			.sw_act2 = data.sw_act2,
			.abs_enc_sw_act2 = data.abs_enc_sw_act2,
			.pos_act3 = data.pos_act3,
			.abs_pos_act3 = data.abs_pos_act3,
			.vel_act3 = data.vel_act3,
			.curr_act3 = data.curr_act3,
			.sw_act3 = data.sw_act3,
			.abs_enc_sw_act3 = data.abs_enc_sw_act3,
			.pos_act4 = data.pos_act4,
			.abs_pos_act4 = data.abs_pos_act4,
			.vel_act4 = data.vel_act4,
			.curr_act4 = data.curr_act4,
			.sw_act4 = data.sw_act4,
			.abs_enc_sw_act4 = data.abs_enc_sw_act4};

		xQueueSendToBack(queue_ulog_lisum_act_data, &log_data, (TickType_t)0);

		break;
	}

	case MAVLINK_MSG_ID_BATTERY_STATUS: {

		mavlink_battery_status_t data;

		mavlink_msg_battery_status_decode(msg, &data);

		ulog_battery_status_t log_data = {
			.timestamp = HAL_GetTimeUS(),
			.id = data.id,
			.battery_function = data.battery_function,
			.type = data.type,
			.temperature = data.temperature,
			.voltages[0] = data.voltages[0],
			.voltages[1] = data.voltages[1],
			.voltages[2] = data.voltages[2],
			.voltages[3] = data.voltages[3],
			.voltages[4] = data.voltages[4],
			.voltages[5] = data.voltages[5],
			.voltages[6] = data.voltages[6],
			.voltages[7] = data.voltages[7],
			.voltages[8] = data.voltages[8],
			.voltages[9] = data.voltages[9],
			.current_battery = data.current_battery,
			.current_consumed = data.current_consumed,
			.energy_consumed = data.energy_consumed,
			.battery_remaining = data.battery_remaining,
			.time_remaining = data.time_remaining,
			.charge_state = data.charge_state,
			.voltages_ext[0] = data.voltages_ext[0],
			.voltages_ext[1] = data.voltages_ext[1],
			.voltages_ext[2] = data.voltages_ext[2],
			.voltages_ext[3] = data.voltages_ext[3],
			.mode = data.mode,
			.fault_bitmask = data.fault_bitmask};

		xQueueSendToBack(queue_ulog_batt_status1, &log_data, (TickType_t)0);

		break;
	}
	case MAVLINK_MSG_ID_SCALED_IMU: {

		mavlink_scaled_imu_t data;

		mavlink_msg_scaled_imu_decode(msg, &data);

		ulog_scaled_imu_t log_data = {.timestamp = HAL_GetTimeUS(),
									  .time_boot_ms = data.time_boot_ms,
									  .xacc = data.xacc,
									  .yacc = data.yacc,
									  .zacc = data.zacc,
									  .xgyro = data.xgyro,
									  .ygyro = data.ygyro,
									  .zgyro = data.zgyro,
									  .xmag = data.xmag,
									  .ymag = data.ymag,
									  .zmag = data.zmag,
									  .temperature = data.temperature};

		if (msg->compid == mp_mav_compid) {

			xQueueSendToBack(queue_ulog_scaled_imu1, &log_data, (TickType_t)0);

		} else {

			xQueueSendToBack(queue_ulog_scaled_imu2, &log_data, (TickType_t)0);
		}

		break;
	}

	default:
		break;
	}
}
