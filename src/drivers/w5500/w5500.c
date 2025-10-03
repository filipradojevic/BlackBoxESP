#include "w5500.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "W5500_SPI"

// ----------------- Helper functions -----------------
static void w5500_select(w5500_t *dev)   { gpio_set_level(dev->pin_cs, 0); }
static void w5500_deselect(w5500_t *dev) { gpio_set_level(dev->pin_cs, 1); }
extern uint8_t w5500_gateway[4]; // Gateway (npr. {192,168,1,1})
extern uint8_t w5500_subnet[4];  // Subnet maska (npr. {255,255,255,0})

// ----------------- Low-level SPI -----------------

// Burst write više bajtova
static esp_err_t w5500_write_buffer(w5500_t *dev, uint16_t addr, uint8_t block_ctrl, const uint8_t *data, size_t len)
{
    size_t total_len = 3 + len;
    uint8_t *tx_buf = alloca(total_len);
    if (!tx_buf) return ESP_ERR_NO_MEM;

    tx_buf[0] = (addr >> 8) & 0xFF;
    tx_buf[1] = addr & 0xFF;
    tx_buf[2] = W5500_WRITE_OPCODE | block_ctrl;
    memcpy(&tx_buf[3], data, len);

    w5500_select(dev);
    esp_err_t ret = spi_device_transmit(dev->spi, &(spi_transaction_t){
        .length = 8 * total_len,
        .tx_buffer = tx_buf,
        .rx_buffer = NULL
    });
    w5500_deselect(dev);
    return ret;
}

// Burst read više bajtova
static esp_err_t w5500_read_buffer(w5500_t *dev, uint16_t addr, uint8_t block_ctrl, uint8_t *buffer, size_t len)
{
    size_t total_len = 3 + len;
    uint8_t *tx_buf = alloca(total_len);
    uint8_t *rx_buf = alloca(total_len);
    if (!tx_buf || !rx_buf) return ESP_ERR_NO_MEM;

    memset(tx_buf, 0, total_len);
    tx_buf[0] = (addr >> 8) & 0xFF;
    tx_buf[1] = addr & 0xFF;
    tx_buf[2] = W5500_READ_OPCODE | block_ctrl;

    w5500_select(dev);
    esp_err_t ret = spi_device_transmit(dev->spi, &(spi_transaction_t){
        .length = 8 * total_len,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf
    });
    w5500_deselect(dev);

    if(ret == ESP_OK)
        memcpy(buffer, &rx_buf[3], len);

    return ret;
}

// Pisanje jednog registra (za kompatibilnost)
static esp_err_t w5500_write_reg(w5500_t *dev, uint16_t addr, uint8_t block_ctrl, uint8_t data)
{
    return w5500_write_buffer(dev, addr, block_ctrl, &data, 1);
}

// Čitanje jednog registra (za kompatibilnost)
static esp_err_t w5500_read_reg(w5500_t *dev, uint16_t addr, uint8_t block_ctrl, uint8_t *data)
{
    return w5500_read_buffer(dev, addr, block_ctrl, data, 1);
}

// ----------------- Initialization -----------------
void w5500_init(w5500_t *dev, uint8_t *mac, uint8_t *ip,
                int pin_cs, int pin_rst,
                int pin_miso, int pin_mosi, int pin_sclk, int pin_int)
{
    memcpy(dev->mac, mac, 6);
    memcpy(dev->ip, ip, 4);

    dev->pin_cs   = pin_cs;
    dev->pin_rst  = pin_rst;
    dev->pin_miso = pin_miso;
    dev->pin_mosi = pin_mosi;
    dev->pin_sclk = pin_sclk;
    dev->pin_int  = pin_int;

    gpio_set_direction(dev->pin_cs, GPIO_MODE_OUTPUT);
    w5500_deselect(dev);

    gpio_set_direction(dev->pin_rst, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->pin_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(dev->pin_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG,"W5500 init done");
}

// ----------------- PHY -----------------
uint8_t w5500_read_version(w5500_t *dev)
{
    uint8_t ver = 0;
    w5500_read_reg(dev, W5500_VERSIONR, 0, &ver);
    return ver;
}

// ----------------- Network config -----------------
static void w5500_write_ip_addr(w5500_t *dev, uint16_t addr, uint8_t *data)
{
    w5500_write_buffer(dev, addr, 0x00, data, 4);
}

void w5500_set_network_config(w5500_t *dev, uint8_t *gateway, uint8_t *subnet)
{
    w5500_write_ip_addr(dev, 0x0001, gateway);  // GAR
    w5500_write_ip_addr(dev, 0x0005, subnet);   // SUBR
    w5500_write_buffer(dev, 0x0009, 0x00, dev->mac, 6); // SHAR
    w5500_write_ip_addr(dev, 0x000F, dev->ip);  // SIPR

    // Postavi RX/TX buffer size 2KB
    w5500_write_reg(dev, 0x001A, 0x00, 0x02); // Sn_RXBUF_SIZE
    w5500_write_reg(dev, 0x001B, 0x00, 0x02); // Sn_TXBUF_SIZE
}

// ----------------- UDP socket -----------------
void w5500_udp_open(w5500_t *dev, uint16_t port)
{
    uint8_t sr;
    const uint8_t S0_BLOCK = 0x08;

    w5500_write_reg(dev, 0x0001, S0_BLOCK, 0x10); // CLOSE
    for(int i=0;i<50;i++){
        w5500_read_reg(dev, 0x0003, S0_BLOCK, &sr);
        if(sr==0x00) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    w5500_write_reg(dev, 0x0000, S0_BLOCK, 0x02); // Sn_MR = UDP
    w5500_write_reg(dev, 0x0004, S0_BLOCK, port >> 8);
    w5500_write_reg(dev, 0x0005, S0_BLOCK, port & 0xFF);
    w5500_write_reg(dev, 0x0001, S0_BLOCK, 0x01); // OPEN

    for(int i=0;i<200;i++){
        w5500_read_reg(dev, 0x0003, S0_BLOCK, &sr);
        if(sr==0x22) { ESP_LOGI(TAG,"UDP socket0 READY!"); break; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int w5500_is_link_up(w5500_t *dev)
{
    uint8_t phy_cfg;
    
    // Čitanje PHYCFGR registra (Adresa 0x0035, Common Block 0x00)
    w5500_read_reg(dev, W5500_PHYCFGR, 0x00, &phy_cfg); 
    
    // Provera da li je bit 0 (LINK) postavljen
    return (phy_cfg & W5500_LINK_STATUS_BIT) ? 1 : 0;
}

int W5500_ResetAndInit(w5500_t *dev) {
    // Hard reset W5500
    gpio_set_level(dev->pin_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(dev->pin_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(500));


    // Re-inicijalizacija W5500
    w5500_init(dev, dev->mac, dev->ip,
               dev->pin_cs, dev->pin_rst,
               dev->pin_miso, dev->pin_mosi, dev->pin_sclk, dev->pin_int);

    // 5. Konfiguriši PHY i mrežu
    w5500_set_phy_allcapable(dev);
    w5500_set_network_config(dev, w5500_gateway, w5500_subnet);
    w5500_set_phy_allcapable(dev);

    // 6. Otvori UDP socket na zadatom portu
    ESP_LOGI(TAG, "Otvaranje UDP socket0...");
    w5500_udp_open(dev, UDP_PORT);

    for(int i=0;i<50;i++){
        if(w5500_is_link_up(dev)) {
            ESP_LOGI(TAG,"PHY link aktivan pre slanja");
            break;
        }
        ESP_LOGW(TAG,"Čekanje PHY linka...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "W5500 re-init complete");
    return 1; // uspešno
}

// ----------------- UDP send/recv -----------------
// ----------------- UDP send/recv -----------------
uint16_t w5500_sendto(w5500_t *dev, const uint8_t *buffer, uint16_t len, uint8_t *dst_ip, uint16_t dst_port)
{
    const uint8_t S0_BLOCK = 0x08;
    uint16_t tx_ptr;
    uint8_t ptr_h, ptr_l;

    if (!w5500_is_link_up(dev)) {
        ESP_LOGW(TAG, "Link nije aktivan u sendto!");
        return 0;
    }

    if(len > 2048) len = 2048;

    // destinacija
    w5500_write_buffer(dev, 0x000C, S0_BLOCK, dst_ip, 4); // Sn_DIPR
    w5500_write_reg(dev, 0x0010, S0_BLOCK, dst_port >> 8);
    w5500_write_reg(dev, 0x0011, S0_BLOCK, dst_port & 0xFF);

    // Sn_TX_WR
    w5500_read_reg(dev, 0x0024, S0_BLOCK, &ptr_h);
    w5500_read_reg(dev, 0x0025, S0_BLOCK, &ptr_l);
    tx_ptr = (ptr_h << 8) | ptr_l;

    // BURST WRITE celog paketa
    w5500_write_buffer(dev, tx_ptr, 0x10, buffer, len);

    tx_ptr += len;
    w5500_write_reg(dev, 0x0024, S0_BLOCK, tx_ptr >> 8);
    w5500_write_reg(dev, 0x0025, S0_BLOCK, tx_ptr & 0xFF);

    // SEND
    w5500_write_reg(dev, 0x0001, S0_BLOCK, 0x20);

    uint8_t ir;
    do {
        w5500_read_reg(dev, 0x0002, S0_BLOCK, &ir);
    } while(!(ir & 0x10));
    w5500_write_reg(dev, 0x0002, S0_BLOCK, 0x10);

    return len;
}

uint16_t w5500_recv(w5500_t *dev, uint8_t *buffer, uint16_t max_len)
{
    const uint8_t S0_BLOCK = 0x08;
    const uint8_t S0_RX_BUFFER_CTRL = 0x18;
    uint16_t rx_size;
    uint8_t size_h, size_l;
    uint8_t ptr_h, ptr_l;
    uint16_t rx_ptr;
    
    // 1. Čitanje Sn_RX_RSR (Veličina podataka u baferu)
    w5500_read_reg(dev, 0x0026, S0_BLOCK, &size_h);
    w5500_read_reg(dev, 0x0027, S0_BLOCK, &size_l);
    rx_size = (size_h << 8) | size_l;
    
    if(rx_size == 0) return 0;
    // Opcionalno, proveriti da li je rx_size veća od maksimalne TCP/UDP veličine
    if(rx_size > 2048) rx_size = 2048; 

    // 2. Čitanje Sn_RX_RD (Pokazivač za čitanje)
    w5500_read_reg(dev, 0x0028, S0_BLOCK, &ptr_h);
    w5500_read_reg(dev, 0x0029, S0_BLOCK, &ptr_l);
    rx_ptr = (ptr_h << 8) | ptr_l;

    // --------------------------------------------------------
    // KRITIČNA PROMENA: Alociraj privremeni bafer na HEAP-u
    // --------------------------------------------------------
    uint8_t *temp_buffer = (uint8_t *)malloc(rx_size);
    if (temp_buffer == NULL) {
        ESP_LOGE("W5500_RECV", "Greska: malloc neuspesan.");
        return 0; 
    }

    // 3. Blok čitanje SVIH podataka (zaglavlje + payload) u HEAP bafer
    w5500_read_buffer(dev, rx_ptr, S0_RX_BUFFER_CTRL, temp_buffer, rx_size);

    // 4. Ažuriranje Sn_RX_RD (Povećanje pokazivača)
    rx_ptr += rx_size;
    w5500_write_reg(dev, 0x0028, S0_BLOCK, rx_ptr >> 8);
    w5500_write_reg(dev, 0x0029, S0_BLOCK, rx_ptr & 0xFF);

    // 5. Izdaj RECV komandu
    w5500_write_reg(dev, 0x0001, S0_BLOCK, 0x40); // RECV

    // 6. Ekstrakcija čistih podataka
    uint16_t data_len = rx_size > W5500_UDP_HEADER_LEN ? rx_size - W5500_UDP_HEADER_LEN : 0;
    if(data_len > max_len) data_len = max_len;
    
    // Kopiranje čistih podataka iz privremenog HEAP bafera u korisnički bafer
    memcpy(buffer, &temp_buffer[W5500_UDP_HEADER_LEN], data_len);

    // 7. OBAVEZNO OSLOBAĐANJE HEAP memorije
    free(temp_buffer);

    return data_len;
}


uint8_t w5500_read_phycfgr(w5500_t *dev) {
    uint8_t val = 0;
    w5500_read_reg(dev, W5500_PHYCFGR, 0, &val);
    return val;
}

void w5500_write_phycfgr(w5500_t *dev, uint8_t value) {
    w5500_write_reg(dev, W5500_PHYCFGR, 0, value);
}

void w5500_set_phy_allcapable(w5500_t *dev) {
    uint8_t phy = w5500_read_phycfgr(dev);

    // Enable all capabilities
    phy &= ~(0x78);
    phy |= (1 << 6) | (0x7 << 3);
    w5500_write_phycfgr(dev, phy);

    // Reset link
    phy &= ~(1 << 7);
    w5500_write_phycfgr(dev, phy);
    vTaskDelay(pdMS_TO_TICKS(50));
    phy |= (1 << 7);
    w5500_write_phycfgr(dev, phy);

    ESP_LOGI(TAG, "Čekanje PHY linka...");
    for(int i=0;i<100;i++){
        phy = w5500_read_phycfgr(dev);
        if(phy & 0x01) { 
            ESP_LOGI(TAG,"PHY link aktivan!");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
