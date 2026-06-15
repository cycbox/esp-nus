// SPDX-License-Identifier: Apache-2.0
//
// Throughput knobs negotiated on every connection:
//   * LE 2M PHY        - ble_gap_set_prefered_le_phy()  (~2x raw rate vs 1M)
//   * DLE              - ble_gap_set_data_len()          (251-byte LL payloads)
//   * Large ATT MTU    - preferred MTU set at init, exchange kicked on connect
//   * 7.5 ms interval  - ble_gap_update_params()
//
// Outbound flow control: a counting semaphore hands out a fixed number of
// "credits". A credit is taken before each notification is queued and returned
// in BLE_GAP_EVENT_NOTIFY_TX once the host has passed it to the controller.
// This bounds the number of in-flight mbufs so a fast UART can't exhaust the
// msys pool, while still keeping the air interface saturated.

#include "ble_nus.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_npl.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_nus";

// Preferred ATT MTU. 247 is the sweet spot: payload = MTU - 3 = 244 B, which
// becomes a 247 B ATT PDU + 4 B L2CAP header = 251 B = exactly one DLE LL PDU
// (and one msys block). Each notification is therefore a single, full LL packet
// with no wasteful trailing fragment, which lets a central (Android in
// particular) pack whole notifications into each connection event. A larger MTU
// raises the per-notification ceiling but forces L2CAP fragmentation across
// multiple LL PDUs with a short, inefficient tail PDU.
#define NUS_PREFERRED_MTU 247

// Number of notifications allowed in flight at once. Kept below both the msys
// block budget (MSYS_1_BLOCK_COUNT=64) and the controller's ACL queue
// (ACL_FROM_LL_COUNT=30) so neither ble_hs_mbuf_from_flat() nor
// ble_gatts_notify_custom() fails under load. With a 244 B payload now fitting
// in one msys block / one ACL packet, each credit costs exactly one of each, so
// 24 stays clear of both limits while keeping the pipeline deep enough to span a
// 15 ms (Android-typical) connection interval.
#define NUS_TX_CREDITS 24

// How often to log measured TX throughput and the estimated number of LL PDUs
// per connection event while a transfer is running.
#define NUS_STATS_PERIOD_MS 1000

// Connection interval, in 1.25 ms units. 6 == 7.5 ms, the BLE minimum.
#define NUS_CONN_ITVL_MIN 6
#define NUS_CONN_ITVL_MAX 18
#define NUS_CONN_LATENCY 0
#define NUS_CONN_TIMEOUT 400 // 4 s, in 10 ms units

// Nordic UART Service UUIDs.
// Service 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
// RX char 6E400002-... (peer writes -> we receive)
// TX char 6E400003-... (we notify -> peer receives)
static const ble_uuid128_t nus_svc_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3,
                     0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t nus_chr_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3,
                     0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t nus_chr_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3,
                     0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static ble_nus_config_t s_cfg;
static uint8_t s_own_addr_type;

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_val_handle;       // value handle of the TX characteristic
static volatile bool s_tx_subscribed;  // peer enabled notifications on TX
static SemaphoreHandle_t s_tx_credits; // counting semaphore, NUS_TX_CREDITS

// Periodic throughput/packets-per-event stats. Counters are bumped by the sender
// (uart_rx_task) and sampled + reset by the callout on the NimBLE host task.
static struct ble_npl_callout s_stats_co;
static volatile uint32_t s_stat_notifs; // notifications queued since last sample
static volatile uint32_t s_stat_bytes;  // app bytes queued since last sample
static volatile uint32_t s_stat_pdus;   // LL PDUs implied by those notifications
static int64_t s_stat_last_us;          // timestamp of the last sample

// One-shot retry of our connection-parameter update. The central may run its
// own update concurrently, colliding with ours (HCI "LL transaction collision");
// we back off briefly and try once more rather than racing it.
static struct ble_npl_callout s_param_co;
static bool s_param_retried;

static void ble_nus_advertise(void);

// Human-readable PHY name for the 1M/2M/Coded enum used in PHY-update events.
static const char *phy_str(uint8_t phy) {
  switch (phy) {
  case BLE_GAP_LE_PHY_1M:
    return "1M";
  case BLE_GAP_LE_PHY_2M:
    return "2M";
  case BLE_GAP_LE_PHY_CODED:
    return "Coded";
  default:
    return "?";
  }
}

// Read back and log the parameters the controller actually settled on for this
// connection: interval (1.25 ms units), latency, and supervision timeout.
static void log_conn_params(uint16_t conn_handle) {
  struct ble_gap_conn_desc desc;
  int rc = ble_gap_conn_find(conn_handle, &desc);
  if (rc) {
    ESP_LOGW(TAG, "conn_find rc=%d", rc);
    return;
  }
  ESP_LOGI(TAG, "conn params: interval=%.2f ms latency=%d timeout=%d ms",
           desc.conn_itvl * 1.25, desc.conn_latency,
           desc.supervision_timeout * 10);
}

static int gatt_rx_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
  // Peer wrote to RX: hand every fragment of the mbuf chain to the app.
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
    return BLE_ATT_ERR_UNLIKELY;
  if (!s_cfg.rx_cb)
    return 0;

  for (struct os_mbuf *om = ctxt->om; om != NULL; om = SLIST_NEXT(om, om_next)) {
    if (om->om_len)
      s_cfg.rx_cb(om->om_data, om->om_len, s_cfg.cb_arg);
  }
  return 0;
}

static int gatt_tx_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
  // TX is notify-only; nothing to read or write directly.
  return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_svc_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = &nus_chr_rx_uuid.u,
                    .access_cb = gatt_rx_access,
                    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                },
                {
                    .uuid = &nus_chr_tx_uuid.u,
                    .access_cb = gatt_tx_access,
                    .flags = BLE_GATT_CHR_F_NOTIFY,
                    .val_handle = &s_tx_val_handle,
                },
                {0},
            },
    },
    {0},
};

// Drain or refill the credit pool to exactly NUS_TX_CREDITS. Used on connect to
// guarantee a clean pipeline and on disconnect to release any blocked sender.
static void tx_credits_reset(void) {
  if (!s_tx_credits)
    return;
  for (int i = 0; i < NUS_TX_CREDITS; i++)
    xSemaphoreGive(s_tx_credits); // caps out at NUS_TX_CREDITS, extra gives fail
}

// Request our preferred connection interval/latency/timeout. Best-effort: the
// central may reject it or run its own update that collides with ours (see the
// retry path in BLE_GAP_EVENT_CONN_UPDATE).
static void request_conn_params(uint16_t conn_handle) {
  struct ble_gap_upd_params params = {
      .itvl_min = NUS_CONN_ITVL_MIN,
      .itvl_max = NUS_CONN_ITVL_MAX,
      .latency = NUS_CONN_LATENCY,
      .supervision_timeout = NUS_CONN_TIMEOUT,
  };
  int rc = ble_gap_update_params(conn_handle, &params);
  if (rc)
    ESP_LOGW(TAG, "conn param update rc=%d", rc);
}

// Deferred one-shot retry after a collision with the central's own update.
static void param_retry_cb(struct ble_npl_event *ev) {
  if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE)
    return;
  ESP_LOGI(TAG, "retrying conn param update after collision");
  request_conn_params(s_conn_handle);
}

// Ask the peer for every throughput feature we can. Each is best-effort: a
// central is free to reject or ignore any of them, so failures are logged but
// not fatal.
static void negotiate_fast_params(uint16_t conn_handle) {
  int rc;

  rc = ble_gap_set_prefered_le_phy(conn_handle, BLE_GAP_LE_PHY_2M_MASK,
                                   BLE_GAP_LE_PHY_2M_MASK,
                                   BLE_GAP_LE_PHY_CODED_ANY);
  if (rc)
    ESP_LOGW(TAG, "set 2M PHY rc=%d", rc);

  rc = ble_gap_set_data_len(conn_handle, BLE_HCI_SET_DATALEN_TX_OCTETS_MAX,
                            BLE_HCI_SET_DATALEN_TX_TIME_MAX);
  if (rc)
    ESP_LOGW(TAG, "set data len rc=%d", rc);

  // MTU exchange is initiated by the central; our preferred MTU
  // (ble_att_set_preferred_mtu) bounds the negotiated result. See
  // BLE_GAP_EVENT_MTU for the settled value.

  request_conn_params(conn_handle);
}

// Sample the TX counters and log measured throughput plus an estimate of how
// many LL PDUs the link is pushing per connection event. The controller does
// not expose the real per-event packet count, so we derive it: PDUs sent over
// the sample window, divided by the number of connection events in that window
// (window / interval). With 244 B payloads each notification is one LL PDU.
static void stats_timer_cb(struct ble_npl_event *ev) {
  uint32_t notifs = s_stat_notifs;
  uint32_t bytes = s_stat_bytes;
  uint32_t pdus = s_stat_pdus;
  s_stat_notifs = 0;
  s_stat_bytes = 0;
  s_stat_pdus = 0;

  int64_t now = esp_timer_get_time();
  double dt = (now - s_stat_last_us) / 1e6;
  s_stat_last_us = now;

  if (notifs && dt > 0) {
    struct ble_gap_conn_desc desc;
    double itvl_ms = 0;
    if (ble_gap_conn_find(s_conn_handle, &desc) == 0)
      itvl_ms = desc.conn_itvl * 1.25;
    double events = (itvl_ms > 0) ? (dt * 1000.0 / itvl_ms) : 0;
    double pkts_per_event = (events > 0) ? (pdus / events) : 0;
    ESP_LOGI(TAG,
             "tx: %.1f kB/s, %.0f notif/s, ~%.1f LL PDU/conn-event "
             "(interval=%.2f ms)",
             bytes / dt / 1000.0, notifs / dt, pkts_per_event, itvl_ms);
  }

  if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE)
    ble_npl_callout_reset(&s_stats_co,
                          ble_npl_time_ms_to_ticks32(NUS_STATS_PERIOD_MS));
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    if (event->connect.status == 0) {
      s_conn_handle = event->connect.conn_handle;
      s_tx_subscribed = false;
      s_param_retried = false;
      tx_credits_reset();
      ESP_LOGI(TAG, "connected; handle=%d", s_conn_handle);
      log_conn_params(s_conn_handle);
      negotiate_fast_params(s_conn_handle);
      s_stat_notifs = 0;
      s_stat_bytes = 0;
      s_stat_pdus = 0;
      s_stat_last_us = esp_timer_get_time();
      ble_npl_callout_reset(&s_stats_co,
                            ble_npl_time_ms_to_ticks32(NUS_STATS_PERIOD_MS));
      if (s_cfg.event_cb)
        s_cfg.event_cb(BLE_NUS_CONNECTED, s_cfg.cb_arg);
    } else {
      ESP_LOGW(TAG, "connect failed; status=%d", event->connect.status);
      ble_nus_advertise();
    }
    return 0;

  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG, "disconnected; reason=%d", event->disconnect.reason);
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_tx_subscribed = false;
    ble_npl_callout_stop(&s_stats_co);
    ble_npl_callout_stop(&s_param_co);
    tx_credits_reset(); // unblock any sender parked on a credit
    if (s_cfg.event_cb)
      s_cfg.event_cb(BLE_NUS_DISCONNECTED, s_cfg.cb_arg);
    ble_nus_advertise();
    return 0;

  case BLE_GAP_EVENT_SUBSCRIBE:
    if (event->subscribe.attr_handle == s_tx_val_handle) {
      s_tx_subscribed = event->subscribe.cur_notify;
      ESP_LOGI(TAG, "TX notifications %s",
               s_tx_subscribed ? "enabled" : "disabled");
    }
    return 0;

  case BLE_GAP_EVENT_NOTIFY_TX:
    // One event per notification once the host hands it to the controller.
    // Return the credit so the sender can queue the next fragment.
    if (event->notify_tx.attr_handle == s_tx_val_handle)
      xSemaphoreGive(s_tx_credits);
    return 0;

  case BLE_GAP_EVENT_MTU:
    ESP_LOGI(TAG, "ATT MTU now %d (payload %d)", event->mtu.value,
             event->mtu.value - 3);
    return 0;

  case BLE_GAP_EVENT_DATA_LEN_CHG:
    // DLE settled: the LL PDU sizes/times the controllers agreed on.
    ESP_LOGI(TAG, "data len: TX=%dB/%dus RX=%dB/%dus",
             event->data_len_chg.max_tx_octets,
             event->data_len_chg.max_tx_time,
             event->data_len_chg.max_rx_octets,
             event->data_len_chg.max_rx_time);
    return 0;

  case BLE_GAP_EVENT_CONN_UPDATE:
    // Fires when a connection-parameter update completes (ours or the peer's).
    if (event->conn_update.status == 0) {
      log_conn_params(event->conn_update.conn_handle);
    } else {
      ESP_LOGW(TAG, "conn update failed; status=%d",
               event->conn_update.status);
      // A collision with the central's own update (HCI 0x23, "LL transaction
      // collision") is transient: back off briefly and retry once.
      if (event->conn_update.status ==
              BLE_HS_HCI_ERR(BLE_ERR_LMP_COLLISION) &&
          !s_param_retried && s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        s_param_retried = true;
        ble_npl_callout_reset(&s_param_co, ble_npl_time_ms_to_ticks32(500));
      }
    }
    return 0;

  case BLE_GAP_EVENT_CONN_UPDATE_REQ:
    return 0;

  case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
    if (event->phy_updated.status == 0)
      ESP_LOGI(TAG, "PHY now: TX=%s RX=%s",
               phy_str(event->phy_updated.tx_phy),
               phy_str(event->phy_updated.rx_phy));
    else
      ESP_LOGW(TAG, "PHY update failed; status=%d",
               event->phy_updated.status);
    return 0;

  case BLE_GAP_EVENT_ADV_COMPLETE:
    ble_nus_advertise();
    return 0;

  default:
    return 0;
  }
}

static void ble_nus_advertise(void) {
  // Advertising packet: flags + the 128-bit service UUID (fills most of the 31
  // bytes). The full device name goes in the scan response.
  struct ble_hs_adv_fields adv = {0};
  adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  adv.uuids128 = (ble_uuid128_t *)&nus_svc_uuid;
  adv.num_uuids128 = 1;
  adv.uuids128_is_complete = 1;

  int rc = ble_gap_adv_set_fields(&adv);
  if (rc) {
    ESP_LOGE(TAG, "adv_set_fields rc=%d", rc);
    return;
  }

  struct ble_hs_adv_fields rsp = {0};
  const char *name = ble_svc_gap_device_name();
  rsp.name = (uint8_t *)name;
  rsp.name_len = strlen(name);
  rsp.name_is_complete = 1;
  rc = ble_gap_adv_rsp_set_fields(&rsp);
  if (rc)
    ESP_LOGW(TAG, "adv_rsp_set_fields rc=%d (name too long?)", rc);

  struct ble_gap_adv_params adv_params = {
      .conn_mode = BLE_GAP_CONN_MODE_UND,
      .disc_mode = BLE_GAP_DISC_MODE_GEN,
  };
  rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                         gap_event_cb, NULL);
  if (rc)
    ESP_LOGE(TAG, "adv_start rc=%d", rc);
  else
    ESP_LOGI(TAG, "advertising as \"%s\"", name);
}

static void on_sync(void) {
  // Prefer the 2M PHY for any future connection before one is established.
  int rc = ble_gap_set_prefered_default_le_phy(BLE_GAP_LE_PHY_2M_MASK,
                                               BLE_GAP_LE_PHY_2M_MASK);
  if (rc)
    ESP_LOGW(TAG, "default 2M PHY rc=%d", rc);

  rc = ble_hs_util_ensure_addr(0);
  if (rc) {
    ESP_LOGE(TAG, "ensure_addr rc=%d", rc);
    return;
  }
  rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
  if (rc) {
    ESP_LOGE(TAG, "infer addr type rc=%d", rc);
    return;
  }
  ble_nus_advertise();
}

static void on_reset(int reason) {
  ESP_LOGW(TAG, "nimble host reset; reason=%d", reason);
}

static void host_task(void *param) {
  nimble_port_run(); // returns only on nimble_port_stop()
  nimble_port_freertos_deinit();
}

esp_err_t ble_nus_init(const ble_nus_config_t *config) {
  if (!config || !config->device_name)
    return ESP_ERR_INVALID_ARG;
  s_cfg = *config;

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  if (err != ESP_OK)
    return err;

  s_tx_credits = xSemaphoreCreateCounting(NUS_TX_CREDITS, NUS_TX_CREDITS);
  if (!s_tx_credits)
    return ESP_ERR_NO_MEM;

  err = nimble_port_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init rc=%d", err);
    return err;
  }

  ble_hs_cfg.sync_cb = on_sync;
  ble_hs_cfg.reset_cb = on_reset;

  ble_npl_callout_init(&s_stats_co, nimble_port_get_dflt_eventq(),
                       stats_timer_cb, NULL);
  ble_npl_callout_init(&s_param_co, nimble_port_get_dflt_eventq(),
                       param_retry_cb, NULL);

  // Cap the ATT MTU so each notification is exactly one LL PDU; the effective
  // value is the minimum of the two peers' preferences, settled during MTU
  // exchange (see BLE_GAP_EVENT_MTU).
  int rc = ble_att_set_preferred_mtu(NUS_PREFERRED_MTU);
  if (rc)
    ESP_LOGW(TAG, "set preferred MTU rc=%d", rc);

  ble_svc_gap_init();
  ble_svc_gatt_init();

  rc = ble_gatts_count_cfg(gatt_svcs);
  if (rc) {
    ESP_LOGE(TAG, "gatts_count_cfg rc=%d", rc);
    return ESP_FAIL;
  }
  rc = ble_gatts_add_svcs(gatt_svcs);
  if (rc) {
    ESP_LOGE(TAG, "gatts_add_svcs rc=%d", rc);
    return ESP_FAIL;
  }

  rc = ble_svc_gap_device_name_set(s_cfg.device_name);
  if (rc)
    ESP_LOGW(TAG, "device_name_set rc=%d", rc);

  nimble_port_freertos_init(host_task);
  return ESP_OK;
}

bool ble_nus_ready(void) {
  return s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_tx_subscribed;
}

uint16_t ble_nus_max_payload(void) {
  if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE)
    return 0;
  uint16_t mtu = ble_att_mtu(s_conn_handle);
  return mtu > 3 ? mtu - 3 : 0;
}

esp_err_t ble_nus_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms) {
  if (!ble_nus_ready())
    return ESP_ERR_INVALID_STATE;
  if (len == 0)
    return ESP_OK;

  const TickType_t ticks =
      timeout_ms ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY;

  uint16_t offset = 0;
  while (offset < len) {
    uint16_t chunk = ble_nus_max_payload();
    if (chunk == 0)
      return ESP_ERR_INVALID_STATE; // disconnected mid-send
    if (chunk > len - offset)
      chunk = len - offset;

    if (xSemaphoreTake(s_tx_credits, ticks) != pdTRUE)
      return ESP_ERR_TIMEOUT;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data + offset, chunk);
    if (!om) {
      // msys momentarily exhausted; hand the credit back and let the in-flight
      // notifications drain before retrying this fragment.
      xSemaphoreGive(s_tx_credits);
      vTaskDelay(1);
      continue;
    }

    // notify_custom consumes `om` (frees it) on both success and failure.
    int rc = ble_gatts_notify_custom(s_conn_handle, s_tx_val_handle, om);
    if (rc != 0) {
      xSemaphoreGive(s_tx_credits); // no NOTIFY_TX event will arrive
      if (rc == BLE_HS_ENOMEM) {
        vTaskDelay(1);
        continue;
      }
      if (rc == BLE_HS_ENOTCONN)
        return ESP_ERR_INVALID_STATE;
      ESP_LOGW(TAG, "notify rc=%d", rc);
      return ESP_FAIL;
    }
    // Account this notification for the periodic throughput stats. One LL PDU
    // per 247 B of L2CAP SDU (ATT PDU = chunk + 3, plus a 4 B L2CAP header).
    s_stat_notifs++;
    s_stat_bytes += chunk;
    s_stat_pdus += (chunk + 3 + 4 + 250) / 251;
    offset += chunk;
  }
  return ESP_OK;
}
