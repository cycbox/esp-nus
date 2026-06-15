// SPDX-License-Identifier: Apache-2.0
//
// High-throughput BLE 5 UART bridge for ESP32-C6.
//
// Wires the hardware-UART bridge to the Nordic UART Service: bytes arriving on
// the UART RX pin are notified to the connected central, and bytes written by
// the central are clocked out the UART TX pin. See README.md for wiring and
// throughput tuning.

#include "esp_log.h"
#include "sdkconfig.h"

#include "ble_nus.h"
#include "uart_bridge.h"

static const char *TAG = "main";

static void on_ble_event(ble_nus_event_t event, void *arg) {
  ESP_LOGI(TAG, "BLE %s",
           event == BLE_NUS_CONNECTED ? "connected" : "disconnected");
}

void app_main(void) {
  // UART first: this creates the BLE->UART stream buffer that the NUS receive
  // callback writes into.
  ESP_ERROR_CHECK(uart_bridge_init());

  const ble_nus_config_t nus = {
      .device_name = CONFIG_BRIDGE_BLE_NAME,
      .rx_cb = uart_bridge_on_ble_rx,
      .event_cb = on_ble_event,
      .cb_arg = NULL,
  };
  ESP_ERROR_CHECK(ble_nus_init(&nus));

  ESP_LOGI(TAG, "bridge up; advertising as \"%s\"", CONFIG_BRIDGE_BLE_NAME);
}
