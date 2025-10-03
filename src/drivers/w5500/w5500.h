#ifndef W5500_H
#define W5500_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define W5500_VERSIONR     0x0039
#define W5500_PHYCFGR      0x002E
#define W5500_WRITE_OPCODE 0x04
#define W5500_READ_OPCODE  0x00
#define W5500_S0_REG_BLOCK (0x01 << 3) // 0x08
#define W5500_S0_TX_BUFFER (0x02 << 3) // 0x10
#define W5500_S0_RX_BUFFER (0x03 << 3) // 0x18
#define W5500_UDP_HEADER_LEN 8
// Bit 0 u PHYCFGR registru označava status linka: 1 = Aktivan, 0 = Neaktivan
#define W5500_LINK_STATUS_BIT 0x01 

#define UDP_PORT 1002

typedef struct {
    spi_device_handle_t spi;
    uint8_t mac[6];
    uint8_t ip[4];

    int pin_cs;
    int pin_rst;
    int pin_miso;
    int pin_mosi;
    int pin_sclk;
    int pin_int;
} w5500_t;

// HAL interface
typedef struct {
    uint16_t (*recv)(void *phy, uint8_t *data, uint16_t size);
    uint16_t (*send)(void *phy, const uint8_t *data, uint16_t size);
} eth_hal_phy_t;

// Init
void w5500_init(w5500_t *dev,
                uint8_t *mac, uint8_t *ip,
                int pin_cs, int pin_rst,
                int pin_miso, int pin_mosi, int pin_sclk, int pin_int);

uint8_t w5500_read_version(w5500_t *dev);
uint8_t w5500_read_phycfgr(w5500_t *dev);
void w5500_write_phycfgr(w5500_t *dev, uint8_t value);
void w5500_set_phy_allcapable(w5500_t *dev);

uint16_t w5500_recv(w5500_t *dev, uint8_t *buffer, uint16_t max_len);
uint16_t w5500_send(w5500_t *dev, const uint8_t *buffer, uint16_t len);

uint16_t w5500_phy_recv(void *phy, uint8_t *data, uint16_t size);
uint16_t w5500_phy_send(void *phy, const uint8_t *data, uint16_t size);

// Otvaranje UDP socket0 na određenom portu
void w5500_udp_open(w5500_t *dev, uint16_t port);
uint16_t w5500_sendto(w5500_t *dev, const uint8_t *buffer, uint16_t len, uint8_t *dst_ip, uint16_t dst_port);
void w5500_set_network_config(w5500_t *dev, uint8_t *gateway, uint8_t *subnet);

int w5500_is_link_up(w5500_t *dev);
int W5500_ResetAndInit(w5500_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* W5500_H */
