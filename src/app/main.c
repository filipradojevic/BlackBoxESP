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
#include "driver/sdspi_host.h"
#include "esp_cpu.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sdmmc_cmd.h"
#include "esp_task_wdt.h"

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
#include "lfs_def.h"
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
TaskHandle_t task_work_handle;
TaskHandle_t task_log_global;

#if FLASH_OR_SD_LITTLEFS

/* Flash */
flash_t flash_dev;

#else

/* sd card */
// sd_t sd;
sdmmc_card_t sd;
sdspi_dev_handle_t sd_spi_handle;

#endif

/* Flash buffers and LittleFS structures */
static uint8_t read_buffer[PAGE_SIZE_SD];
static uint8_t prog_buffer[PAGE_SIZE_SD]; 
static uint8_t lookahead_buffer[LOOKAHEAD_SIZE];
static uint8_t file_buffer[PAGE_SIZE_SD];

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

lfs_t lfs;
lfs_file_t lfs_file;
struct lfs_file_config file_cfg;
struct lfs_config lfs_cfg;
static uint32_t err_cnt = 0x00;

ulog_t ulog;
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


/* Temporary variables */
uint64_t boot_time_ms = 0;

static spi_device_handle_t w5500_spi;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void timer_blinky_cb(TimerHandle_t xTimer);

// --------------------- SPI helpers ---------------------
static esp_err_t w5500_write(uint16_t addr, uint8_t ctrl, const uint8_t *buf, uint16_t len) {
    uint8_t header[3] = {(addr >> 8) & 0xFF, addr & 0xFF, ctrl};
    spi_transaction_t t[2] = {0};

    t[0].length = 8 * 3;
    t[0].tx_buffer = header;
    t[1].length = 8 * len;
    t[1].tx_buffer = buf;

    esp_err_t ret = spi_device_transmit(w5500_spi, &t[0]);
    if(ret != ESP_OK) return ret;
    return spi_device_transmit(w5500_spi, &t[1]);
}

static esp_err_t w5500_read(uint16_t addr, uint8_t ctrl, uint8_t *buf, uint16_t len) {
    uint8_t tx[len + 3];           // header + dummy
    tx[0] = (addr >> 8) & 0xFF;
    tx[1] = addr & 0xFF;
    tx[2] = ctrl;
    memset(tx + 3, 0, len);        // dummy bajtovi za čitanje

    spi_transaction_t t = {0};
    t.length = 8 * (len + 3);      // header + payload
    t.tx_buffer = tx;
    t.rx_buffer = buf;             // ovde dolazi RX

    return spi_device_transmit(w5500_spi, &t);
}


// --------------------- HAL send/recv ---------------------
static uint16_t w5500_send(void *phy, const uint8_t *data, uint16_t size) {
    // simplifikovano: write direktno u TX buffer 0
    // TX buffer base 0x4000
    w5500_write(0x4000, 0x10, data, size); // 0x10: write, block 0, common memory
    // trigger SEND
    uint8_t cmd = 0x20; // SEND command
    w5500_write(0x0001, 0x10, &cmd, 1);
    return size;
}

static uint16_t w5500_recv(void *phy, uint8_t *data, uint16_t size) {
    uint8_t rx_len[2];
    esp_err_t ret = w5500_read(0x0026, 0x11, rx_len, 2);
    if(ret != ESP_OK) return 0;  // SPI error

    uint16_t len = (rx_len[0] << 8) | rx_len[1];
    if(len == 0 || len > size) return 0;  // odmah vraća, bez daljih SPI transfera

    ret = w5500_read(0x6000, 0x11, data, len);  // čitaj samo ako ima podataka
    if(ret != ESP_OK) return 0;

    uint8_t cmd = 0x40; // RECV command
    w5500_write(0x0001, 0x10, &cmd, 1);

    return len;
}

// --------------------- HAL init ---------------------
void HAL_ETH_Init(eth_hal_phy_t *phy, const uint8_t *mac, void *handle) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = W5500_ETH_MOSI,
        .miso_io_num = W5500_ETH_MISO,
        .sclk_io_num = W5500_ETH_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
	spi_device_interface_config_t devcfg = {
		.command_bits = 16,
		.address_bits = 8,
		.clock_speed_hz = 36 * 1000 * 1000,
		.mode = 0,
		.spics_io_num = W5500_ETH_CS,
		.queue_size = 20,
		.flags = 0,  // <- bitno za W5500
	};

    ESP_ERROR_CHECK(spi_bus_initialize(ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(ETH_SPI_HOST, &devcfg, &w5500_spi));

    // reset
    gpio_set_direction(W5500_ETH_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(W5500_ETH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(W5500_ETH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

    // upiši MAC adresu
    w5500_write(W5500_SHAR, 0x10, mac, 6);

    // popuni eth_hal_phy_t
    phy->send = w5500_send;
    phy->recv = w5500_recv;

    printf("W5500 HAL initialized\n");
}

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

// Konfiguracija SPI bus-a
   /*  spi_bus_config_t sd_buscfg = {
        .miso_io_num = SD_MISO_PIN,
        .mosi_io_num = SD_MOSI_PIN,
        .sclk_io_num = SD_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(1, &sd_buscfg, SPI_DMA_CH_AUTO));

    // Konfiguracija SPI device-a (SD kartice)
    spi_device_interface_config_t sd_devcfg = {
        .clock_speed_hz = SD_CLOCK_RATE_HZ,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };

    spi_device_handle_t sd_spi_handle = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(1, &sd_devcfg, &sd_spi_handle));

    // Inicijalizacija SD kartice
    memset(&sd, 0, sizeof(sd));
    sd.handle = sd_spi_handle;
    sd.cs_gpio = SD_CS_PIN;

	uint32_t retry_sd = 0;
	esp_err_t status = ESP_FAIL;


	do {
		int32_t ret = sd_init(&sd, sd_spi_handle, SD_CS_PIN);

		if (ret == ESP_OK) {
			printf("Successfully initialized SD card\n");
			break;
		}

		vTaskDelay(pdMS_TO_TICKS(1));

	} while (retry_sd++ < SD_INIT_MAX_RETRY_CNT);

	if (retry_sd >= SD_INIT_MAX_RETRY_CNT || !sd.initialized) {
		printf("Failed to initialize SD card\n");
		// prvo konfiguracija pina 25
		gpio_reset_pin(25);
		gpio_set_direction(25, GPIO_MODE_OUTPUT);
		gpio_set_level(25, 1);
	} */

	// #endif

	HAL_ETH_Init(&eth_phy, dev_mac, eth_handle);

	// /*--------------------------------- UDP ----------------------------------
	/* initialize UDP */
	udp_init(&udp, dev_mac, dev_ip, (udp_phy_t*)&eth_phy);

	/* add client MAC & IP to static ARP table */
	udp_arptab_add(&udp, gw_sky_mac, gw_sky_ip);
	udp_arptab_add(&udp, ins_mac, ins_ip);

	// /*-------------------------------- UDP TL --------------------------------*/
	udp_tl_init(&mav_gw_sky_tl, &udp, (uint8_t*)gw_sky_ip, gw_sky_local_port, gw_sky_port);

	udp_tl_init(&mav_ins_tl, &udp, (uint8_t*)ins_ip, ins_local_port, ins_port);

	printf("Successfully initialized Ethernet and UDP\n");

	// Konfiguracija SPI bus-a
    esp_err_t ret;

    // 1. Inicijalizacija SPI magistrale
    spi_bus_config_t bus_cfg = {
        .miso_io_num = SD_MISO_PIN,
        .mosi_io_num = SD_MOSI_PIN,
        .sclk_io_num = SD_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        printf("SPI bus init failed: %s", esp_err_to_name(ret));
        return;
    }

    // 2. Konfigurišemo SDSPI host
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST; // koristimo SPI2 (HSPI na ESP32)

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_PIN;
    slot_config.host_id = host.slot;

    // 3. Mount device na bus
    ret = sdspi_host_init_device(&slot_config, &sd_spi_handle);
    if (ret != ESP_OK) {
        printf("Failed to init SD device: %s", esp_err_to_name(ret));
        return;
    }

    // 4. Pokušaj inicijalizacije kartice sa retry mehanizmom
    uint32_t retry = 0;
    while (retry++ < SD_INIT_MAX_RETRY_CNT) {
        ret = sdmmc_card_init(&host, &sd);
        if (ret == ESP_OK) {
            printf("SD card initialized successfully \n!");
            printf("Name: %s, Capacity: %llu MB \n",
                     sd.cid.name,
                     (uint64_t)sd.csd.capacity * sd.csd.sector_size / (1024*1024));
					 break;
		}
        vTaskDelay(pdMS_TO_TICKS(10));
    }
	
	if(ret != ESP_OK){
		printf("Failed to initialize SD card after %d retries \n", SD_INIT_MAX_RETRY_CNT);
	}

	
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
	int err = lfs_mount(&lfs, &lfs_cfg);

	printf("Mounted lfs!\n");

	/* reformat if mount fails, this should only happen on the first boot */
	if (err < 0) {
		err = lfs_format(&lfs, &lfs_cfg);
		if (err < 0) {
			err_cnt++;
		}
		printf("Formated lfs!\n");

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

	printf("Opened boot_count lfs!\n");

	err = lfs_file_read(&lfs, &lfs_file, &boot_count, sizeof(boot_count));
	if (err < 0) {
		err_cnt++;
	}

	printf("Readed boot_count lfs!\n");

	/* update boot count */
	boot_count += 1;
	err = lfs_file_rewind(&lfs, &lfs_file);
	if (err < 0) {
		err_cnt++;
	}

	printf("Rewinded boot_count lfs!\n");

	err = lfs_file_write(&lfs, &lfs_file, &boot_count, sizeof(boot_count));
	if (err < 0) {
		err_cnt++;
	}

	printf("Writted boot_count lfs!\n");


	/* storage is not updated until file is closed or synced */
	err = lfs_file_close(&lfs, &lfs_file);
	if (err < 0) {
		err_cnt++;
	}

	printf("Closed boot_count lfs!\n");

	/* create new file */
	char file_name[LFS_NAME_MAX];

	snprintf(file_name, sizeof(file_name), "log%u.ulg",
			 (unsigned int)boot_count);

	err = lfs_file_opencfg(&lfs, &lfs_file, file_name,
						   LFS_O_WRONLY | LFS_O_CREAT, &file_cfg);

	printf("Opened log1 lfs!\n");

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

	err = lfs_file_close(&lfs, &lfs_file);
	if (err < 0)
		err_cnt++;

	printf("Finished with ULOG!\n");



	/*------------------------------- FreeRTOS -------------------------------*/
	// esp_task_wdt_deinit(); // deaktivira TWDT
	
	BaseType_t result;
	
	result = xTaskCreate(task_work, "work", 4096, NULL, 1, &task_work_handle);
	if (result != pdPASS) {
		for (;;)
			;
	}

	printf("Successfully created work task\n");

	result = xTaskCreate(task_log, "log", 4096, NULL, 2, &task_log_global);
	if (result != pdPASS) {
		for (;;)
			;
	}

	printf("Successfully created log task\n");

}
