#include "eth.h"

typedef struct {
	esp_eth_handle_t eth_handle;
	QueueHandle_t
		rx_queue; // queue of pointers/blocks pushed by driver's RX callback
} eth_phy_adapter_t;

typedef struct {
	uint8_t* data;
	size_t len;
} eth_rx_item_t;

static eth_phy_adapter_t adapter;

/* Called from driver RX path */
static esp_err_t eth_rx_cb(esp_eth_handle_t h, uint8_t* buf, uint32_t len,
						   void* priv)
{
	eth_phy_adapter_t* a = (eth_phy_adapter_t*)priv;

	uint8_t* copy = malloc(len);
	if (!copy)
		return ESP_FAIL;
	memcpy(copy, buf, len);

	eth_rx_item_t item = {.data = copy, .len = len};
	if (xQueueSend(a->rx_queue, &item, 0) != pdTRUE) {
		free(copy);
		return ESP_FAIL;
	}
	return ESP_OK;
}

/* HAL send: use stored eth_handle */
uint16_t HAL_ETH_Send(void* phy, const uint8_t* data, uint16_t size)
{
	eth_phy_adapter_t* a = (eth_phy_adapter_t*)phy;
	if (!a || !a->eth_handle)
		return 0;

	esp_err_t err = esp_eth_transmit(a->eth_handle, (void*)data, size);
	return (err == ESP_OK) ? size : 0;
}

/* HAL recv: read struct with len */
uint16_t HAL_ETH_Recv(void* phy, uint8_t* data, uint16_t size)
{
	eth_phy_adapter_t* a = (eth_phy_adapter_t*)phy;
	if (!a || !a->rx_queue)
		return 0;

	eth_rx_item_t item;
	if (xQueueReceive(a->rx_queue, &item, pdMS_TO_TICKS(1000)) == pdTRUE) {
		size_t copy_len = (size < item.len) ? size : item.len;
		memcpy(data, item.data, copy_len);
		free(item.data);
		return (uint16_t)copy_len;
	}
	return 0;
}

/* init: */
void HAL_ETH_Init(eth_hal_phy_t* eth, const uint8_t* mac,
				  esp_eth_handle_t eth_handle)
{
	static eth_phy_adapter_t adapter;

	adapter.eth_handle = eth_handle;
	adapter.rx_queue = xQueueCreate(8, sizeof(eth_rx_item_t));

	if (!adapter.rx_queue) {
		printf("ETH_HAL failed to create rx queue\n");
		return;
	}

	// registruj RX callback
	esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, (void*)mac);
	esp_eth_update_input_path(eth_handle, eth_rx_cb, &adapter);

	eth->recv = HAL_ETH_Recv;
	eth->send = HAL_ETH_Send;
	eth->priv = &adapter;

	printf("ETH_HAL initialized\n");
}
