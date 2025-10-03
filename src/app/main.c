#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "w5500.h"
#include <string.h>
#include <stdio.h>
#include "esp_timer.h"

#define TAG "W5500_MAIN"

// SPI pinovi za W5500
#define W5500_MISO 12
#define W5500_MOSI 11
#define W5500_SCK  13
#define W5500_CS   14
#define W5500_RST  9
#define W5500_INT  10
#define MSG_LEN 500
#define UDP_PORT 1002

#define RX_BUFFER_SIZE 2048
static uint8_t rx_buffer[RX_BUFFER_SIZE];

static w5500_t w5500_dev;

static char msg_to_send[MSG_LEN];

// IP i port računara na koji šaljemo UDP pakete
uint8_t pc_ip[4] = {192, 168, 1, 101};
uint16_t pc_port = 1002;

    // 4. Postavljanje mrežnih parametara
uint8_t w5500_gateway[] = {192, 168, 1, 1};      // Gateway (ruter)
uint8_t w5500_subnet[]  = {255, 255, 255, 0};    // Subnet maska

void prepare_message() {
    const char *pattern = "Rustjebolji!"; // 14 bajtova
    size_t pattern_len = strlen(pattern);

    size_t pos = 0;
    while(pos + pattern_len < MSG_LEN) {
        memcpy(&msg_to_send[pos], pattern, pattern_len);
        pos += pattern_len;
    }
    // Završetak sa preostalim bajtovima
    if(pos < MSG_LEN) {
        memcpy(&msg_to_send[pos], pattern, MSG_LEN - pos);
    }
}

void app_main(void)
{
    esp_err_t ret;

    // 1. Inicijalizacija SPI bus-a
    ESP_LOGI(TAG, "Inicijalizacija SPI bus-a");
    spi_bus_config_t buscfg = {
        .miso_io_num = W5500_MISO,
        .mosi_io_num = W5500_MOSI,
        .sclk_io_num = W5500_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2048,
    };
    ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // 2. Dodavanje W5500 na SPI bus
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20 * 1000 * 1000, // 10 MHz
        .mode = 0,
        .spics_io_num = W5500_CS,
        .queue_size = 20,
    };
    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &w5500_dev.spi);
    ESP_ERROR_CHECK(ret);

    // 3. Inicijalizacija W5500 čipa
    uint8_t mac[6] = {0x02, 0x00, 0x00, 0x12, 0x34, 0x56};
    uint8_t ip[4]  = {192, 168, 1, 150};
    w5500_init(&w5500_dev, mac, ip,
               W5500_CS, W5500_RST,
               W5500_MISO, W5500_MOSI, W5500_SCK, W5500_INT);



    // 5. Konfiguriši PHY i mrežu
    w5500_set_phy_allcapable(&w5500_dev);
    w5500_set_network_config(&w5500_dev, w5500_gateway, w5500_subnet);
    w5500_set_phy_allcapable(&w5500_dev);

    // 6. Otvori UDP socket na zadatom portu
    ESP_LOGI(TAG, "Otvaranje UDP socket0...");
    w5500_udp_open(&w5500_dev, UDP_PORT);

    ESP_LOGI(TAG, "UDP echo loop start...");

    prepare_message();
    const uint16_t EXPECTED_LEN = MSG_LEN;

    // 7. Glavna petlja: šalji poruku svake sekunde

    while(1) {
        // ----------------------------------------------------
        // START: Merenje vremena
        // ----------------------------------------------------
        int64_t start_time = esp_timer_get_time();


        // ESP_LOGI(TAG, "Poslato: %s", msg_to_send); // Isključeno da ne remeti tajming
        uint16_t sent = w5500_sendto(&w5500_dev, (uint8_t*)msg_to_send, EXPECTED_LEN, pc_ip, pc_port);
        if (sent == 0) {
            ESP_LOGW(TAG, "Link pao, pokušavam reset...");
            W5500_ResetAndInit(&w5500_dev);
            while(!w5500_is_link_up(&w5500_dev)) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            continue; // preskoči trenutnu iteraciju
        }

        // čekaj recv
        int rx_len = 0;
        while(rx_len == 0) {
            rx_len = w5500_recv(&w5500_dev, rx_buffer, RX_BUFFER_SIZE);
        }

        
        // STOP: Merenje vremena
        int64_t end_time = esp_timer_get_time();
        int64_t latency_us = end_time - start_time;
        // ----------------------------------------------------

        // 3. Null-terminate i provera

        if (rx_len != EXPECTED_LEN) {
            ESP_LOGE(TAG, "GREŠKA DUŽINE: Očekivano %d, Primljeno %d", EXPECTED_LEN, rx_len);
        }
        
        if (rx_len < RX_BUFFER_SIZE) {
            rx_buffer[rx_len] = '\0';
            
            // Finalni ispis uključuje izmerenu latenciju
            ESP_LOGI(TAG, "ECHO: %s | Latencija: %lld us", rx_buffer, latency_us);
        } else {
            ESP_LOGE(TAG, "Greška: Primljeni podaci su preveliki za bafer!");
        }
    }

}