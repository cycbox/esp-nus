// SPDX-License-Identifier: Apache-2.0
//
// Hardware-UART side of the bridge. Moves bytes both directions:
//   UART RX peripheral  --(task)-->  ble_nus_send()        (UART -> BLE)
//   ble_nus rx callback --(stream)-> UART TX peripheral    (BLE -> UART)
//
// The BLE->UART path is decoupled through a FreeRTOS stream buffer so the
// NimBLE host task never blocks on UART transmission.
#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Configure the UART peripheral and spawn the two bridge tasks. Call once,
// before ble_nus_init(), so the BLE->UART stream buffer exists by the time the
// RX callback can fire.
esp_err_t uart_bridge_init(void);

// ble_nus_rx_cb_t: feed bytes received over BLE toward the UART TX line.
// Runs in the NimBLE host task context, so it only enqueues and returns.
void uart_bridge_on_ble_rx(const uint8_t *data, uint16_t len, void *arg);

#ifdef __cplusplus
}
#endif
