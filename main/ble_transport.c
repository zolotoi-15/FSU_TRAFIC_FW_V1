/* ============================================================
   ble_transport.c — GATT-сервер для конфигурации и мониторинга.

   (См. ble_transport.h — описание сервиса и команд.)

   === FIX BLE-NAME-CLASH ===
       Функции fsu_ble_* (были ble_transport_*) — конфликт с
       nimble/transport.h (void ble_transport_init(void)).
       #include "esp_mac.h" для esp_read_mac/ESP_MAC_BT.

   === FIX BLE-RAW-TRANSPORT ===
       Write в CMD/CFG с первым байтом PKT_MAGIC (0x4C) —
       протокольный пакет (те же байты, что и по LoRa).
       Передаётся в transport_handle_raw_rx().
   ============================================================ */

#include "ble_transport.h"
#include "ble_config.h"
#include "transport.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"     /* === FIX esp_read_mac / ESP_MAC_BT === */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "BLE";

#define BLE_DEVICE_NAME_PREFIX "FSU_"
#define PKT_MAGIC 0x4C

static const ble_uuid128_t s_svc_uuid = BLE_UUID128_INIT(
    0xF0, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

#define UUID_CFG     0xFF01
#define UUID_CMD     0xFF02
#define UUID_STATUS  0xFF03
#define UUID_RESP    0xFF04

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_cfg_handle;
static uint16_t s_cmd_handle;
static uint16_t s_status_handle;
static uint16_t s_resp_handle;

static SemaphoreHandle_t s_ble_mtx = NULL;
static char s_cfg_buf[4096];

static int ble_gap_event(struct ble_gap_event *event, void *arg);

static int gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (uuid16 == UUID_CFG) {
            size_t clen = sizeof(s_cfg_buf);
            if (!config_export_json(s_cfg_buf, &clen)) {
                return BLE_ATT_ERR_UNLIKELY;
            }
            os_mbuf_append(ctxt->om, s_cfg_buf, clen);
            return 0;
        }
        return BLE_ATT_ERR_UNLIKELY;

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len >= sizeof(s_cfg_buf)) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        os_mbuf_copydata(ctxt->om, 0, len, s_cfg_buf);
        s_cfg_buf[len] = '\0';

        /* === FIX BLE-RAW-TRANSPORT === */
        if (len >= 4 && (uint8_t)s_cfg_buf[0] == PKT_MAGIC) {
            ESP_LOGI(TAG, "RAW packet via BLE: %u bytes", (unsigned)len);
            transport_handle_raw_rx((const uint8_t *)s_cfg_buf, len);
            return 0;
        }
        /* === END FIX BLE-RAW-TRANSPORT === */

        if (uuid16 == UUID_CMD || uuid16 == UUID_CFG) {
            ESP_LOGI(TAG, "CMD via BLE: %s", s_cfg_buf);
            ble_config_handle_cmd(s_cfg_buf, len);
        }
        return 0;
    }
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            { .uuid = BLE_UUID16_DECLARE(UUID_CFG),
              .access_cb = gatt_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
              .val_handle = &s_cfg_handle },
            { .uuid = BLE_UUID16_DECLARE(UUID_CMD),
              .access_cb = gatt_access,
              .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
              .val_handle = &s_cmd_handle },
            { .uuid = BLE_UUID16_DECLARE(UUID_STATUS),
              .access_cb = NULL,
              .flags = BLE_GATT_CHR_F_NOTIFY,
              .val_handle = &s_status_handle },
            { .uuid = BLE_UUID16_DECLARE(UUID_RESP),
              .access_cb = NULL,
              .flags = BLE_GATT_CHR_F_NOTIFY,
              .val_handle = &s_resp_handle },
            { 0 }
        }
    },
    { 0 }
};

static void ble_advertise(void)
{
    if (ble_gap_adv_active()) {
        ESP_LOGD(TAG, "advertising already active, skip");
        return;
    }
    
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids128 = &s_svc_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) { ESP_LOGE(TAG, "adv_set_fields rc=%d", rc); return; }

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event, NULL);
    if (rc != 0) ESP_LOGE(TAG, "adv_start rc=%d", rc);
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "BLE connected");
        } else {
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_advertise();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "BLE disconnected reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_advertise();
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_advertise();
        return 0;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU=%d", event->mtu.value);
        return 0;
    default:
        return 0;
    }
}

static void ble_on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    ble_advertise();
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset reason=%d", reason);
}

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* === FIX BLE-NAME-CLASH === */
esp_err_t fsu_ble_init(void)
{
    if (s_ble_mtx) return ESP_OK;
    
    ESP_LOGI(TAG, "fsu_ble_init: heap free=%u, largest=%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    
    s_ble_mtx = xSemaphoreCreateMutex();
    if (!s_ble_mtx) { 
        ESP_LOGE(TAG, "BLE mutex alloc failed"); 
        return ESP_ERR_NO_MEM; 
    }

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_ble_mtx); s_ble_mtx = NULL;
        return err;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg rc=%d (need more msys blocks?)", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs rc=%d", rc);
        return ESP_FAIL;
    }

    char name[32];
    if (g_config.role == DEVICE_ROLE_MASTER) {
        snprintf(name, sizeof(name), "%sROOT", BLE_DEVICE_NAME_PREFIX);
    } else {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_BT);
        snprintf(name, sizeof(name), "%sNODE_%02X%02X",
                 BLE_DEVICE_NAME_PREFIX, mac[4], mac[5]);
    }
    rc = ble_svc_gap_device_name_set(name);
    if (rc != 0) {
        ESP_LOGE(TAG, "device_name_set rc=%d", rc);
        return ESP_FAIL;
    }

    ble_att_set_preferred_mtu(512);
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE init OK, name='%s'", name);
    return ESP_OK;
}

bool fsu_ble_is_connected(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

esp_err_t fsu_ble_notify_status(const char *json, size_t len)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return ESP_ERR_INVALID_STATE;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(json, len);
    if (!om) return ESP_FAIL;
    int rc = ble_gatts_notify_custom(s_conn_handle, s_status_handle, om);
    return rc == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t fsu_ble_notify_response(const char *json, size_t len)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return ESP_ERR_INVALID_STATE;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(json, len);
    if (!om) return ESP_FAIL;
    int rc = ble_gatts_notify_custom(s_conn_handle, s_resp_handle, om);
    return rc == 0 ? ESP_OK : ESP_FAIL;
}

bool fsu_ble_send_raw(const uint8_t *data, size_t len)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return false;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) return false;
    int rc = ble_gatts_notify_custom(s_conn_handle, s_cmd_handle, om);
    return rc == 0;
}
/* === END FIX BLE-NAME-CLASH === */