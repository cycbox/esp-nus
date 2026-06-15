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
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_nus";

// Number of notifications allowed in flight at once. Kept comfortably below the
// msys block budget (see sdkconfig.defaults) so ble_hs_mbuf_from_flat() does not
// fail under load. More credits -> deeper pipeline -> higher throughput, up to
// the point the controller's ACL buffers saturate. With a 244 B payload now
// fitting in one msys block, 16 credits stay well under the 64-block pool while
// keeping the controller's ACL queue (ACL_FROM_LL_COUNT=30) fed.
#define NUS_TX_CREDITS 16

// Connection interval, in 1.25 ms units. 6 == 7.5 ms, the BLE minimum.
#define NUS_CONN_ITVL_MIN 6
#define NUS_CONN_ITVL_MAX 6
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

static void ble_nus_advertise(void);

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

  // MTU exchange is initiated by the central; our large preferred MTU
  // (ble_att_set_preferred_mtu) bounds the negotiated result. See
  // BLE_GAP_EVENT_MTU for the settled value.

  struct ble_gap_upd_params params = {
      .itvl_min = NUS_CONN_ITVL_MIN,
      .itvl_max = NUS_CONN_ITVL_MAX,
      .latency = NUS_CONN_LATENCY,
      .supervision_timeout = NUS_CONN_TIMEOUT,
  };
  rc = ble_gap_update_params(conn_handle, &params);
  if (rc)
    ESP_LOGW(TAG, "conn param update rc=%d", rc);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    if (event->connect.status == 0) {
      s_conn_handle = event->connect.conn_handle;
      s_tx_subscribed = false;
      tx_credits_reset();
      ESP_LOGI(TAG, "connected; handle=%d", s_conn_handle);
      negotiate_fast_params(s_conn_handle);
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

  case BLE_GAP_EVENT_CONN_UPDATE:
  case BLE_GAP_EVENT_CONN_UPDATE_REQ:
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

  // Request the largest ATT MTU NimBLE supports; the effective value is the
  // minimum of the two peers' preferences, settled during MTU exchange.
  int rc = ble_att_set_preferred_mtu(BLE_ATT_MTU_MAX);
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
    offset += chunk;
  }
  return ESP_OK;
}
