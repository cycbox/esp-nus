// SPDX-License-Identifier: Apache-2.0
//
// High-throughput Nordic UART Service (NUS) peripheral for ESP32-C6 (NimBLE).
//
// This is the BLE half of the UART<->BLE bridge. It advertises the Nordic UART
// Service, and on connect negotiates the BLE 5 features that matter for
// throughput: the LE 2M PHY, Data Length Extension (DLE), a large ATT MTU and a
// short connection interval. Outbound data is delivered via flow-controlled
// notifications so a fast UART source can never outrun the controller.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Called from the NimBLE host task whenever the peer writes to the RX
// characteristic. `data`/`len` are only valid for the duration of the call;
// copy out anything you need to keep. Keep this callback short and non-blocking.
typedef void (*ble_nus_rx_cb_t)(const uint8_t *data, uint16_t len, void *arg);

// Connection state notifications, delivered from the NimBLE host task.
typedef enum {
  BLE_NUS_DISCONNECTED,
  BLE_NUS_CONNECTED,
} ble_nus_event_t;

typedef void (*ble_nus_event_cb_t)(ble_nus_event_t event, void *arg);

typedef struct {
  const char *device_name;     // advertised name (required)
  ble_nus_rx_cb_t rx_cb;       // BLE -> app, required for the bridge
  ble_nus_event_cb_t event_cb; // optional connect/disconnect hook
  void *cb_arg;                // passed back to both callbacks
} ble_nus_config_t;

// Bring up NimBLE and start advertising. Returns once the host task is running;
// the link is not yet connected at this point. Call once.
esp_err_t ble_nus_init(const ble_nus_config_t *config);

// Queue `len` bytes to the peer over the TX characteristic, fragmenting to the
// negotiated ATT MTU. Blocks (up to `timeout_ms`, 0 = forever) while the
// controller drains in-flight notifications. Returns:
//   ESP_OK                 - all bytes queued
//   ESP_ERR_INVALID_STATE  - no peer connected / not subscribed
//   ESP_ERR_TIMEOUT        - flow control credits not available in time
//   ESP_FAIL               - host stack error
esp_err_t ble_nus_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

// True once a peer is connected and subscribed to TX notifications.
bool ble_nus_ready(void);

// Largest application payload that fits one notification right now
// (negotiated ATT_MTU - 3). 0 if not connected.
uint16_t ble_nus_max_payload(void);

#ifdef __cplusplus
}
#endif
