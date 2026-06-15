// SPDX-License-Identifier: Apache-2.0
//
// See uart_bridge.h.

#include "uart_bridge.h"

#include <string.h>

#include "ble_nus.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "uart_bridge";

#define UART_PORT ((uart_port_t)CONFIG_BRIDGE_UART_PORT_NUM)

#if CONFIG_BRIDGE_UART_FLOW_CTRL
#define UART_FLOW_CTRL UART_HW_FLOWCTRL_CTS_RTS
#define UART_RTS_GPIO CONFIG_BRIDGE_UART_RTS_GPIO
#define UART_CTS_GPIO CONFIG_BRIDGE_UART_CTS_GPIO
#else
#define UART_FLOW_CTRL UART_HW_FLOWCTRL_DISABLE
#define UART_RTS_GPIO UART_PIN_NO_CHANGE
#define UART_CTS_GPIO UART_PIN_NO_CHANGE
#endif

// Bytes received over BLE, waiting to be clocked out of the UART TX line.
static StreamBufferHandle_t s_ble_to_uart;

void uart_bridge_on_ble_rx(const uint8_t *data, uint16_t len, void *arg) {
  // NimBLE host task context: enqueue without blocking. If the UART cannot keep
  // up (slow baud vs. fast BLE), the stream buffer fills and excess is dropped.
  size_t queued = xStreamBufferSend(s_ble_to_uart, data, len, 0);
  if (queued < len)
    ESP_LOGW(TAG, "BLE->UART overflow, dropped %u bytes", len - (unsigned)queued);
}

// Drain the stream buffer to the UART peripheral. uart_write_bytes copies into
// the driver's TX ring buffer and only blocks if that fills.
static void uart_tx_task(void *arg) {
  // Drain in larger gulps than one BLE payload so a burst of N x 244 B writes
  // is handed to the UART driver in a few calls rather than dozens.
  uint8_t buf[1024];
  for (;;) {
    size_t n = xStreamBufferReceive(s_ble_to_uart, buf, sizeof(buf), portMAX_DELAY);
    if (n)
      uart_write_bytes(UART_PORT, buf, n);
  }
}

// Pull bytes off the UART RX line and push them out over BLE notifications.
// When no peer is subscribed, data is discarded (an idle bridge behaves like an
// unplugged cable rather than buffering stale bytes for the next connection).
static void uart_rx_task(void *arg) {
  const int chunk = CONFIG_BRIDGE_UART_RX_CHUNK;
  uint8_t *buf = malloc(chunk);
  if (!buf) {
    ESP_LOGE(TAG, "no mem for rx chunk");
    vTaskDelete(NULL);
    return;
  }
  for (;;) {
    int n = uart_read_bytes(UART_PORT, buf, chunk,
                            pdMS_TO_TICKS(CONFIG_BRIDGE_UART_RX_TIMEOUT_MS));
    if (n <= 0)
      continue;
    if (!ble_nus_ready())
      continue;
    esp_err_t err = ble_nus_send(buf, n, 2000);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
      ESP_LOGW(TAG, "UART->BLE send failed: %s", esp_err_to_name(err));
  }
}

esp_err_t uart_bridge_init(void) {
  const uart_config_t cfg = {
      .baud_rate = CONFIG_BRIDGE_UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_FLOW_CTRL,
      .rx_flow_ctrl_thresh = 122,
      .source_clk = UART_SCLK_DEFAULT,
  };

  ESP_RETURN_ON_ERROR(uart_driver_install(UART_PORT,
                                          CONFIG_BRIDGE_UART_RX_BUF_SIZE,
                                          CONFIG_BRIDGE_UART_TX_BUF_SIZE, 0, NULL,
                                          0),
                      TAG, "driver_install");
  ESP_RETURN_ON_ERROR(uart_param_config(UART_PORT, &cfg), TAG, "param_config");
  ESP_RETURN_ON_ERROR(uart_set_pin(UART_PORT, CONFIG_BRIDGE_UART_TX_GPIO,
                                   CONFIG_BRIDGE_UART_RX_GPIO, UART_RTS_GPIO,
                                   UART_CTS_GPIO),
                      TAG, "set_pin");

  s_ble_to_uart = xStreamBufferCreate(CONFIG_BRIDGE_BLE_TO_UART_BUF_SIZE, 1);
  ESP_RETURN_ON_FALSE(s_ble_to_uart, ESP_ERR_NO_MEM, TAG, "stream buf alloc");

  xTaskCreate(uart_tx_task, "uart_tx", 3072, NULL, 10, NULL);
  xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 10, NULL);

  ESP_LOGI(TAG, "UART%d @ %d baud, TX=%d RX=%d flow_ctrl=%d", UART_PORT,
           CONFIG_BRIDGE_UART_BAUD_RATE, CONFIG_BRIDGE_UART_TX_GPIO,
           CONFIG_BRIDGE_UART_RX_GPIO, UART_FLOW_CTRL != UART_HW_FLOWCTRL_DISABLE);
  return ESP_OK;
}
