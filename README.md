# esp-nus — High-throughput BLE 5 UART bridge (ESP32-C6)

A transparent UART-BLE cable replacement for the ESP32-C6, exposing the
Nordic UART Service (NUS) and tuned for throughput.

Bytes arriving on the hardware UART RX pin are streamed to the connected BLE
central as TX notifications; bytes the central writes to the RX characteristic
are clocked out the UART TX pin.

## NUS GATT layout

| Role | UUID | Properties |
|------|------|-----------|
| Service | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | primary |
| RX (central → device) | `6E400002-…` | Write, Write No Response |
| TX (device → central) | `6E400003-…` | Notify |

## What makes it "high throughput"

Negotiated on every connection (each best-effort; a central may decline any):

- **LE 2M PHY** — `ble_gap_set_prefered_le_phy()`, ~2× the raw 1M rate.
- **Data Length Extension** — `ble_gap_set_data_len()` requests 251-byte
  link-layer payloads so large notifications aren't fragmented on air.
- **Large ATT MTU (preferred up to 512)** — set as the preferred MTU; the
  central initiates the exchange. For best efficiency use an MTU of **247**:
  the resulting 244-byte payload (247 − 3 ATT) plus the 4-byte L2CAP header is
  exactly 251 bytes, so each notification/write rides in a **single link-layer
  PDU** with DLE — no on-air fragmentation.
- **7.5 ms connection interval** — `ble_gap_update_params()` for the minimum
  interval and zero latency.
- **Flow-controlled notifications** — a credit pool (`NUS_TX_CREDITS`, default
  16) keeps several notifications pipelined to the controller without ever
  exhausting the NimBLE mbuf pool. Credits are returned on
  `BLE_GAP_EVENT_NOTIFY_TX`, so a fast UART source applies natural backpressure
  instead of dropping data or spinning.

## Build, flash, monitor

```sh
. ~/esp/esp-idf/export.sh
idf.py set-target esp32c6
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## Configuration

`idf.py menuconfig` → **UART<->BLE Bridge**:

| Setting | Default | Notes |
|---------|---------|-------|
| Advertised BLE device name | `ESP-NUS` | |
| UART port number | `0` | console is moved to USB-Serial-JTAG, freeing UART0 |
| UART baud rate | `921600` | match expected BLE throughput |
| UART TX / RX GPIO | `16` / `17` | C6 UART0 default pads |
| HW (RTS/CTS) flow control | off | **recommended at high baud** to avoid RX overruns |
| RTS / CTS GPIO | `6` / `7` | when flow control enabled |
| Driver RX / TX buffers | `8192` each | absorb bursts while the other side drains |
| UART read chunk / timeout | `1024` B / `20` ms | latency vs. BLE write size |
| BLE→UART stream buffer | `16384` | hold a full inbound N×244 B batch |
