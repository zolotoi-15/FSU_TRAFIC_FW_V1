#include "mesh_link.h"
#include "traffic_light.h"
#include "shift_register.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_mesh.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

static const char *TAG = "mesh_link";

/* fallback-значения (Kconfig-символы Mesh в v6.1 не существуют) */
#ifndef CONFIG_MESH_MAX_LAYER
#define CONFIG_MESH_MAX_LAYER              6
#endif
#ifndef CONFIG_MESH_ROUTER_SSID
#define CONFIG_MESH_ROUTER_SSID            "Keenetic-4250"
#endif
#ifndef CONFIG_MESH_ROUTER_PASSWD
#define CONFIG_MESH_ROUTER_PASSWD          "VTuVMHPi"
#endif
#ifndef CONFIG_MESH_AP_PASSWD
#define CONFIG_MESH_AP_PASSWD              "mesh_pass_123456"
#endif
#ifndef CONFIG_MESH_AP_AUTHMODE
#define CONFIG_MESH_AP_AUTHMODE            4
#endif
#ifndef CONFIG_MESH_AP_CONNECTIONS
#define CONFIG_MESH_AP_CONNECTIONS         3
#endif
#ifndef CONFIG_MESH_NON_MESH_AP_CONNECTIONS
#define CONFIG_MESH_NON_MESH_AP_CONNECTIONS 0
#endif

/* Mesh ID — как в примере */
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77 };

static mesh_role_t       s_role = MESH_ROLE_NODE;
static uint8_t           s_peer_mac[6] = {0};
static bool              s_peer_known = false;
static bool              s_peer_connected = false;
static volatile int8_t   s_last_rssi = 0;
static uint8_t           s_seq = 0;
static SemaphoreHandle_t s_send_mutex = NULL;

static mesh_status_cb_t  s_status_cb = NULL;

static char   *s_cfg_buf = NULL;
static size_t  s_cfg_received = 0;

static char    *s_json_buf = NULL;
static size_t   s_json_received = 0;
static uint8_t  s_json_from_mac[6] = {0};
static mesh_json_cb_t s_json_cb = NULL;

#define MESH_RX_SIZE  (sizeof(mesh_packet_t))
static uint8_t s_rx_buf[MESH_RX_SIZE];
#define MESH_BUF_MAX  32768

#define MESH_DIAG_RAW_RX   1
#define MESH_DIAG_SEND     1

/* ============================================================ */
/*                      Helpers                                  */
/* ============================================================ */
static void mac_to_str(const uint8_t *mac, char *buf, size_t len)
{
    if (!mac) { snprintf(buf, len, "??:??:??:??:??:??"); return; }
    snprintf(buf, len, MACSTR, MAC2STR(mac));
}

static bool mac_is_broadcast(const uint8_t *mac)
{
    for (int i = 0; i < 6; i++) if (mac[i] != 0xFF) return false;
    return true;
}

static uint8_t get_real_wifi_channel(void)
{
    uint8_t ch = 0;
    wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
    return (esp_wifi_get_channel(&ch, &sec) == ESP_OK) ? ch : 0;
}

static void update_last_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        s_last_rssi = ap.rssi;
    }
}

static void cfg_buf_free(void)
{
    if (s_cfg_buf) { free(s_cfg_buf); s_cfg_buf = NULL; }
    s_cfg_received = 0;
}
static void json_buf_free(void)
{
    if (s_json_buf) { free(s_json_buf); s_json_buf = NULL; }
    s_json_received = 0;
}

/* ============================================================ */
/*              Отправка одного mesh-пакета                      */
/* ============================================================ */
static esp_err_t mesh_send_packet(const uint8_t *dst_mac, uint8_t type,
                                  const void *payload, uint8_t len)
{
    if (!s_send_mutex) return ESP_ERR_INVALID_STATE;
    if (len > MESH_MAX_PAYLOAD) return ESP_ERR_INVALID_SIZE;
    if (xSemaphoreTake(s_send_mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    mesh_packet_t pkt = {0};
    pkt.magic = MESH_MAGIC;
    pkt.type  = type;
    pkt.seq   = s_seq++;
    pkt.flags = MESH_FLAG_NONE;
    pkt.payload_len = len;
    if (len > 0 && payload) memcpy(pkt.payload, payload, len);

    mesh_addr_t to = {0};
    if (dst_mac && !mac_is_broadcast(dst_mac)) {
        memcpy(to.addr, dst_mac, 6);
    } else {
        memset(to.addr, 0xFF, 6);
    }

    mesh_data_t md = {
        .data  = (uint8_t *)&pkt,
        .size  = sizeof(pkt),
        .proto = MESH_PROTO_BIN,
        .tos   = MESH_TOS_P2P,
    };

    esp_err_t err = esp_mesh_send(&to, &md, MESH_DATA_P2P, NULL, 0);

#if MESH_DIAG_SEND
    char ms[18]; mac_to_str(to.addr, ms, sizeof(ms));
    if (err == ESP_OK) ESP_LOGI(TAG, "send OK to %s (type=0x%02X len=%u)", ms, type, len);
    else               ESP_LOGW(TAG, "send FAILED to %s: %s", ms, esp_err_to_name(err));
#endif

    xSemaphoreGive(s_send_mutex);
    return err;
}

/* ============================================================ */
/*          Обработка пакета                                     */
/* ============================================================ */
static void handle_rx_packet(const mesh_addr_t *from,
                             const mesh_data_t *data)
{
    const mesh_packet_t *pkt = (const mesh_packet_t *)data->data;
    if (pkt->magic != MESH_MAGIC) {
        ESP_LOGW(TAG, "bad magic 0x%08X", (unsigned)pkt->magic);
        return;
    }

    update_last_rssi();

    if (!s_peer_known || memcmp(s_peer_mac, from->addr, 6) != 0) {
        memcpy(s_peer_mac, from->addr, 6);
        s_peer_known = true;
        s_peer_connected = true;
        char ms[18]; mac_to_str(s_peer_mac, ms, sizeof(ms));
        ESP_LOGI(TAG, "peer connected: %s rssi=%d", ms, s_last_rssi);
    }

    switch (pkt->type) {
    case MSG_HELLO:
        if (s_role == MESH_ROLE_ROOT) {
            uint8_t ack = 1;
            mesh_send_packet(s_peer_mac, MSG_HELLO_ACK, &ack, 1);
        }
        break;

    case MSG_HELLO_ACK:
        s_peer_connected = true;
        break;

    case MSG_STATUS:
        if (s_role == MESH_ROLE_ROOT && s_status_cb &&
            pkt->payload_len >= sizeof(mesh_status_t)) {
            s_status_cb((const mesh_status_t *)pkt->payload);
        }
        break;

    case MSG_CMD_TRAFFIC:
        if (s_role == MESH_ROLE_NODE &&
            pkt->payload_len >= sizeof(mesh_cmd_traffic_t)) {
            const mesh_cmd_traffic_t *c = (const mesh_cmd_traffic_t *)pkt->payload;
            switch (c->action) {
            case 0: traffic_light_start();       break;
            case 1: traffic_light_stop();        break;
            case 2: traffic_light_pause();       break;
            case 3: traffic_light_resume();      break;
            case 4: traffic_light_reset();       break;
            case 5: traffic_light_next_phase();  break;
            }
            if (c->mode >= 1 && c->mode <= 3)
                traffic_light_set_mode((traffic_mode_t)c->mode);
            uint8_t ack = 1;
            mesh_send_packet(s_peer_mac, MSG_ACK, &ack, 1);
        }
        break;

    case MSG_CMD_RELAY:
        if (s_role == MESH_ROLE_NODE &&
            pkt->payload_len >= sizeof(mesh_cmd_relay_t)) {
            const mesh_cmd_relay_t *c = (const mesh_cmd_relay_t *)pkt->payload;
            if (c->channel < SR_OUTPUT_COUNT) {
                if (c->state) sr_output_on(c->channel);
                else          sr_output_off(c->channel);
            }
            uint8_t ack = 1;
            mesh_send_packet(s_peer_mac, MSG_ACK, &ack, 1);
        }
        break;

    case MSG_CMD_CONFIG:
        if (s_role == MESH_ROLE_NODE &&
            pkt->payload_len >= sizeof(mesh_cmd_config_hdr_t)) {
            const mesh_cmd_config_hdr_t *h = (const mesh_cmd_config_hdr_t *)pkt->payload;

            if ((size_t)pkt->payload_len < sizeof(*h) + h->chunk_len) {
                ESP_LOGE(TAG, "config chunk hdr overflow");
                break;
            }
            const char *chunk = (const char *)(pkt->payload + sizeof(*h));

            if (h->chunk_idx == 0) cfg_buf_free();
            if (!s_cfg_buf) s_cfg_buf = malloc(MESH_BUF_MAX);

            if (!s_cfg_buf || h->chunk_len > MESH_BUF_MAX ||
                s_cfg_received + h->chunk_len > MESH_BUF_MAX) {
                ESP_LOGE(TAG, "config buffer overflow");
                cfg_buf_free();
                break;
            }

            memcpy(s_cfg_buf + s_cfg_received, chunk, h->chunk_len);
            s_cfg_received += h->chunk_len;

            uint8_t ack = h->chunk_idx & 0xFF;
            mesh_send_packet(s_peer_mac, MSG_ACK, &ack, 1);
            ESP_LOGI(TAG, "config chunk %u/%u (%u bytes)",
                     h->chunk_idx + 1, h->chunk_total, (unsigned)s_cfg_received);
        }
        break;

    case MSG_CMD_CONFIG_END:
        if (s_role == MESH_ROLE_NODE && s_cfg_buf && s_cfg_received > 0) {
            ESP_LOGI(TAG, "config complete (%u bytes), applying...",
                     (unsigned)s_cfg_received);
            traffic_light_stop();
            config_load_json(s_cfg_buf, s_cfg_received);
            traffic_light_set_mode((traffic_mode_t)g_config.traffic.mode);
            traffic_light_reset();
            if (g_config.traffic.enabled && g_config.traffic.auto_start)
                traffic_light_start();
            cfg_buf_free();
        }
        break;

    case MSG_JSON_REPORT:
        if (s_role == MESH_ROLE_ROOT &&
            pkt->payload_len >= sizeof(mesh_json_hdr_t)) {
            const mesh_json_hdr_t *h = (const mesh_json_hdr_t *)pkt->payload;

            if ((size_t)pkt->payload_len < sizeof(*h) + h->chunk_len) {
                ESP_LOGE(TAG, "json chunk hdr overflow");
                break;
            }
            const char *chunk = (const char *)(pkt->payload + sizeof(*h));

            if (h->chunk_idx == 0) {
                json_buf_free();
                memcpy(s_json_from_mac, from->addr, 6);
            }
            if (!s_json_buf) s_json_buf = malloc(MESH_BUF_MAX);

            if (!s_json_buf || h->chunk_len > MESH_BUF_MAX ||
                s_json_received + h->chunk_len > MESH_BUF_MAX) {
                ESP_LOGE(TAG, "json buffer overflow");
                json_buf_free();
                break;
            }

            memcpy(s_json_buf + s_json_received, chunk, h->chunk_len);
            s_json_received += h->chunk_len;
        }
        break;

    case MSG_JSON_REPORT_END:
        if (s_role == MESH_ROLE_ROOT && s_json_buf && s_json_received > 0) {
            char ms[18]; mac_to_str(s_json_from_mac, ms, sizeof(ms));
            ESP_LOGI(TAG, "node JSON complete from %s (%u bytes)",
                     ms, (unsigned)s_json_received);
            if (s_json_cb) s_json_cb(s_json_from_mac, s_json_buf, s_json_received);
            json_buf_free();
        }
        break;

    case MSG_ACK:
        break;

    default:
        ESP_LOGD(TAG, "unknown msg type 0x%02X", pkt->type);
        break;
    }
}

/* ============================================================ */
/*          Задача приёма                                       */
/* ============================================================ */
static void mesh_rx_task(void *arg)
{
    mesh_addr_t from;
    mesh_data_t data;
    int flag = 0;

    ESP_LOGI(TAG, "mesh_rx_task started");

    while (1) {
        data.data = s_rx_buf;
        data.size = MESH_RX_SIZE;

        esp_err_t err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        if (err != ESP_OK || data.size == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

#if MESH_DIAG_RAW_RX
        char ms[18]; mac_to_str(from.addr, ms, sizeof(ms));
        ESP_LOGI(TAG, "RAW RX: len=%d from %s flag=0x%x", data.size, ms, flag);
#endif

        if (data.size != sizeof(mesh_packet_t)) {
            ESP_LOGW(TAG, "bad size %d", data.size);
            continue;
        }

        handle_rx_packet(&from, &data);
    }
}

/* ============================================================ */
/*            Сканирование и выбор родителя (node)              */
/* ============================================================ */
static void node_pick_parent(int num)
{
    mesh_assoc_t     parent_assoc = { .layer = CONFIG_MESH_MAX_LAYER, .rssi = -120 };
    wifi_ap_record_t parent_rec   = {0};
    bool             parent_found = false;

    for (int i = 0; i < num; i++) {
        int ie_len = 0;
        mesh_assoc_t assoc;
        wifi_ap_record_t rec;
        esp_mesh_scan_get_ap_ie_len(&ie_len);
        esp_mesh_scan_get_ap_record(&rec, &assoc);

        char ms[18]; mac_to_str(rec.bssid, ms, sizeof(ms));

        if (ie_len == sizeof(assoc)) {
            ESP_LOGW(TAG, "<MESH>[%d]%s layer:%d/%d assoc:%d/%d rssi:%d bssid:%s",
                     i, rec.ssid, assoc.layer, assoc.layer_cap,
                     assoc.assoc, assoc.assoc_cap, rec.rssi, ms);

            if (assoc.mesh_type != MESH_IDLE && assoc.layer_cap &&
                assoc.assoc < assoc.assoc_cap && rec.rssi > -80) {
                if (!parent_found || assoc.layer < parent_assoc.layer) {
                    parent_found = true;
                    memcpy(&parent_rec,   &rec,   sizeof(rec));
                    memcpy(&parent_assoc,&assoc, sizeof(assoc));
                }
            }
        } else {
            ESP_LOGI(TAG, "[%d]%s bssid:%s ch:%u rssi:%d",
                     i, rec.ssid, ms, rec.primary, rec.rssi);
        }
    }
    esp_mesh_flush_scan_result();

    if (!parent_found) {
        ESP_LOGW(TAG, "no parent found, rescan…");
        wifi_scan_config_t sc = { .show_hidden = 1, .scan_type = WIFI_SCAN_TYPE_PASSIVE };
        esp_wifi_scan_stop();
        esp_wifi_scan_start(&sc, 0);
        return;
    }

    wifi_config_t parent = {0};
    parent.sta.channel = parent_rec.primary;
    memcpy(parent.sta.ssid, parent_rec.ssid, sizeof(parent_rec.ssid));
    parent.sta.bssid_set = 1;
    memcpy(parent.sta.bssid, parent_rec.bssid, 6);
    if (parent_rec.authmode != WIFI_AUTH_OPEN) {
        memcpy(parent.sta.password, CONFIG_MESH_AP_PASSWD,
               strlen(CONFIG_MESH_AP_PASSWD));
    }

    esp_err_t err = esp_mesh_set_parent(&parent,
        (mesh_addr_t *)&parent_assoc.mesh_id, MESH_NODE, parent_assoc.layer + 1);

    char ms[18]; mac_to_str(parent_rec.bssid, ms, sizeof(ms));
    ESP_LOGI(TAG, "set_parent → %s layer=%d err=%s",
             ms, parent_assoc.layer + 1, esp_err_to_name(err));
}

/* ============================================================ */
/*                    Mesh event handler                        */
/* ============================================================ */
static void mesh_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    switch (event_id) {
    case MESH_EVENT_STARTED: {
        mesh_addr_t id = {0};
        esp_mesh_get_id(&id);
        ESP_LOGI(TAG, "<STARTED> id=" MACSTR " layer=%d topology=%d ps=%d",
                 MAC2STR(id.addr), esp_mesh_get_layer(),
                 esp_mesh_get_topology(), esp_mesh_is_ps_enabled());

        if (s_role == MESH_ROLE_NODE) {
            esp_mesh_set_self_organized(0, 0);
            wifi_scan_config_t sc = { .show_hidden = 1, .scan_type = WIFI_SCAN_TYPE_PASSIVE };
            esp_wifi_scan_stop();
            esp_wifi_scan_start(&sc, 0);
        } else {
            esp_mesh_set_self_organized(0, 1);
            ESP_LOGI(TAG, "root: self-organized disabled, staying root");
        }
        break;
    }

    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *c = (mesh_event_connected_t *)event_data;
        memcpy(s_peer_mac, c->connected.bssid, 6);
        s_peer_known = true;
        s_peer_connected = true;
        update_last_rssi();
        char ms[18]; mac_to_str(s_peer_mac, ms, sizeof(ms));
        ESP_LOGI(TAG, "<PARENT_CONNECTED> parent=%s layer=%d rssi=%d duty=%d",
                 ms, c->self_layer, s_last_rssi, c->duty);
        break;
    }

    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *d = (mesh_event_disconnected_t *)event_data;
        ESP_LOGW(TAG, "<PARENT_DISCONNECTED> reason=%d", d->reason);
        s_peer_connected = false;
        s_peer_known = false;
        if (d->reason == WIFI_REASON_ASSOC_TOOMANY) {
            wifi_scan_config_t sc = { .show_hidden = 1, .scan_type = WIFI_SCAN_TYPE_PASSIVE };
            esp_wifi_scan_stop();
            esp_wifi_scan_start(&sc, 0);
        }
        break;
    }

    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *sd = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI(TAG, "<SCAN_DONE> number=%d", sd->number);
        if (s_role == MESH_ROLE_NODE) node_pick_parent(sd->number);
        break;
    }

    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *np = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGW(TAG, "<NO_PARENT_FOUND> scan_times=%d", np->scan_times);
        break;
    }

    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *cc = (mesh_event_child_connected_t *)event_data;
        char ms[18]; mac_to_str(cc->mac, ms, sizeof(ms));
        ESP_LOGI(TAG, "<CHILD_CONNECTED> aid=%d mac=%s", cc->aid, ms);
        /* NAT/DHCP уже включены в mesh_netif.c (mesh_netif_start_root_ap) */
        break;
    }

    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *cd = (mesh_event_child_disconnected_t *)event_data;
        char ms[18]; mac_to_str(cd->mac, ms, sizeof(ms));
        ESP_LOGW(TAG, "<CHILD_DISCONNECTED> aid=%d mac=%s", cd->aid, ms);
        break;
    }

    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *lc = (mesh_event_layer_change_t *)event_data;
        ESP_LOGI(TAG, "<LAYER_CHANGE> new_layer=%d", lc->new_layer);
        break;
    }

    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *tds = (mesh_event_toDS_state_t *)event_data;
        ESP_LOGI(TAG, "<TODS_STATE> %s",
                 *tds == MESH_TODS_REACHABLE ? "REACHABLE" : "UNREACHABLE");
        break;
    }

    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *rt = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGI(TAG, "<ROUTING_TABLE_ADD> add %d, new:%d",
                 rt->rt_size_change, rt->rt_size_new);
        break;
    }

    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *rt = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGI(TAG, "<ROUTING_TABLE_REMOVE> remove %d, new:%d",
                 rt->rt_size_change, rt->rt_size_new);
        break;
    }

    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *ns = (mesh_event_network_state_t *)event_data;
        ESP_LOGI(TAG, "<NETWORK_STATE> is_rootless=%d", ns->is_rootless);
        break;
    }

    default:
        ESP_LOGD(TAG, "event id=%" PRId32, event_id);
        break;
    }
}

/* ============================================================ */
/*                         Init                                 */
/* ============================================================ */
esp_err_t mesh_link_init(mesh_role_t role, uint8_t channel)
{
    s_role = role;
    if (!s_send_mutex) s_send_mutex = xSemaphoreCreateMutex();

    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID,
                                               &mesh_event_handler, NULL));

#ifdef CONFIG_MESH_TOPOLOGY
    ESP_ERROR_CHECK(esp_mesh_set_topology((esp_mesh_topology_t)CONFIG_MESH_TOPOLOGY));
#else
    ESP_ERROR_CHECK(esp_mesh_set_topology(MESH_TOPO_TREE));
#endif
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
   // ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(32));

#ifdef CONFIG_MESH_ENABLE_PS
    ESP_ERROR_CHECK(esp_mesh_enable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(60));
    ESP_ERROR_CHECK(esp_mesh_set_announce_interval(600, 3300));
#else
    ESP_ERROR_CHECK(esp_mesh_disable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));
#endif

    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    memcpy((uint8_t *)&cfg.mesh_id, MESH_ID, 6);
    cfg.channel = channel;
    cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
    memcpy(cfg.router.ssid, CONFIG_MESH_ROUTER_SSID, cfg.router.ssid_len);
    memcpy(cfg.router.password, CONFIG_MESH_ROUTER_PASSWD,
           strlen(CONFIG_MESH_ROUTER_PASSWD));

    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode((wifi_auth_mode_t)CONFIG_MESH_AP_AUTHMODE));
    cfg.mesh_ap.max_connection         = CONFIG_MESH_AP_CONNECTIONS;
    cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
    memcpy(cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD, strlen(CONFIG_MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

#if CONFIG_MESH_IE_CRYPTO_FUNCS
    ESP_ERROR_CHECK(esp_mesh_set_ie_crypto_funcs(&g_wifi_default_mesh_crypto_funcs));
    ESP_ERROR_CHECK(esp_mesh_set_ie_crypto_key(CONFIG_MESH_IE_CRYPTO_KEY,
                                               strlen(CONFIG_MESH_IE_CRYPTO_KEY)));
#else
    ESP_ERROR_CHECK(esp_mesh_set_ie_crypto_funcs(NULL));
#endif

ESP_LOGI(TAG, "Before esp_mesh_start: free internal=%u, largest internal=%u",
         heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
         heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
         
    ESP_ERROR_CHECK(esp_mesh_start());

    /* ВАЖНО: IP-слой создаём строго ПОСЛЕ esp_mesh_start(),
       иначе esp_mesh_route_init() повреждает кучу. */
 

    xTaskCreate(mesh_rx_task, "mesh_rx", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "mesh started role=%s channel=%u heap=%" PRIu32
                  " topology=%d ps=%d",
             role == MESH_ROLE_ROOT ? "ROOT" : "NODE",
             channel, esp_get_free_heap_size(),
             esp_mesh_get_topology(), esp_mesh_is_ps_enabled());
    return ESP_OK;
}

/* ============================================================ */
/*                     Root / Node API                          */
/* ============================================================ */
esp_err_t mesh_master_send_traffic(uint8_t action, uint8_t mode)
{
    if (s_role != MESH_ROLE_ROOT) return ESP_ERR_INVALID_STATE;
    mesh_cmd_traffic_t c = { .action = action, .mode = mode };
    return mesh_send_packet(s_peer_mac, MSG_CMD_TRAFFIC, &c, sizeof(c));
}

esp_err_t mesh_master_send_relay(uint8_t ch, bool state)
{
    if (s_role != MESH_ROLE_ROOT) return ESP_ERR_INVALID_STATE;
    mesh_cmd_relay_t c = { .channel = ch, .state = state ? 1 : 0 };
    return mesh_send_packet(s_peer_mac, MSG_CMD_RELAY, &c, sizeof(c));
}

esp_err_t mesh_master_send_config(const char *json, size_t len)
{
    if (s_role != MESH_ROLE_ROOT) return ESP_ERR_INVALID_STATE;
    if (len == 0 || len > MESH_BUF_MAX) return ESP_ERR_INVALID_SIZE;

    const size_t chunk_max = MESH_MAX_PAYLOAD - sizeof(mesh_cmd_config_hdr_t);
    uint16_t total = (uint16_t)((len + chunk_max - 1) / chunk_max);

    for (uint16_t i = 0; i < total; i++) {
        size_t off  = i * chunk_max;
        size_t clen = (len - off > chunk_max) ? chunk_max : (len - off);

        mesh_packet_t pkt = {0};
        pkt.magic = MESH_MAGIC;
        pkt.type  = MSG_CMD_CONFIG;
        pkt.seq   = s_seq++;
        pkt.payload_len = (uint8_t)(sizeof(mesh_cmd_config_hdr_t) + clen);

        mesh_cmd_config_hdr_t h = {
            .chunk_idx = i, .chunk_total = total, .chunk_len = (uint16_t)clen,
        };
        memcpy(pkt.payload, &h, sizeof(h));
        memcpy(pkt.payload + sizeof(h), json + off, clen);

        mesh_addr_t to;
        memcpy(to.addr, s_peer_mac, 6);
        mesh_data_t md = {
            .data  = (uint8_t *)&pkt, .size = sizeof(pkt),
            .proto = MESH_PROTO_BIN,  .tos  = MESH_TOS_P2P,
        };
        esp_err_t err = esp_mesh_send(&to, &md, MESH_DATA_P2P, NULL, 0);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    uint8_t end = 1;
    return mesh_send_packet(s_peer_mac, MSG_CMD_CONFIG_END, &end, 1);
}

esp_err_t mesh_slave_send_status(void)
{
    if (s_role != MESH_ROLE_NODE) return ESP_ERR_INVALID_STATE;
    if (!s_peer_known) return ESP_ERR_INVALID_STATE;

    mesh_status_t st = {
        .state       = (uint8_t)traffic_light_get_state(),
        .mode        = (uint8_t)traffic_light_get_mode(),
        .phase       = traffic_light_get_phase(),
        .fault       = (uint8_t)traffic_light_get_fault(),
        .fault_count = traffic_light_get_fault_count(),
        .sr_state    = sr_get_state(),
        .phase_ms    = traffic_light_get_phase_ms(),
        .uptime_ms   = (uint32_t)(esp_timer_get_time() / 1000),
        .rssi        = s_last_rssi,
    };
    return mesh_send_packet(s_peer_mac, MSG_STATUS, &st, sizeof(st));
}

esp_err_t mesh_node_send_json(const char *json, size_t len)
{
    if (s_role != MESH_ROLE_NODE) return ESP_ERR_INVALID_STATE;
    if (!s_peer_known) return ESP_ERR_INVALID_STATE;
    if (len == 0 || len > MESH_BUF_MAX) return ESP_ERR_INVALID_SIZE;

    const size_t chunk_max = MESH_MAX_PAYLOAD - sizeof(mesh_json_hdr_t);
    uint16_t total = (uint16_t)((len + chunk_max - 1) / chunk_max);

    for (uint16_t i = 0; i < total; i++) {
        size_t off  = i * chunk_max;
        size_t clen = (len - off > chunk_max) ? chunk_max : (len - off);

        mesh_packet_t pkt = {0};
        pkt.magic = MESH_MAGIC;
        pkt.type  = MSG_JSON_REPORT;
        pkt.seq   = s_seq++;
        pkt.payload_len = (uint8_t)(sizeof(mesh_json_hdr_t) + clen);

        mesh_json_hdr_t h = {
            .chunk_idx = i, .chunk_total = total, .chunk_len = (uint16_t)clen,
        };
        memcpy(pkt.payload, &h, sizeof(h));
        memcpy(pkt.payload + sizeof(h), json + off, clen);

        mesh_addr_t to;
        memcpy(to.addr, s_peer_mac, 6);
        mesh_data_t md = {
            .data  = (uint8_t *)&pkt, .size = sizeof(pkt),
            .proto = MESH_PROTO_BIN,  .tos  = MESH_TOS_P2P,
        };
        esp_err_t err = esp_mesh_send(&to, &md, MESH_DATA_P2P, NULL, 0);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    uint8_t end = 1;
    return mesh_send_packet(s_peer_mac, MSG_JSON_REPORT_END, &end, 1);
}

void mesh_set_json_cb(mesh_json_cb_t cb) { s_json_cb = cb; }

int8_t mesh_get_last_rssi(void)           { return s_last_rssi; }
const uint8_t *mesh_get_peer_mac(void)    { return s_peer_mac; }
bool mesh_peer_connected(void)            { return s_peer_connected; }
void mesh_set_status_cb(mesh_status_cb_t cb) { s_status_cb = cb; }
uint8_t mesh_get_current_channel(void)    { return get_real_wifi_channel(); }
uint8_t mesh_get_real_wifi_channel(void)  { return get_real_wifi_channel(); }

bool mesh_is_root_connected(void)
{
    if (s_role != MESH_ROLE_ROOT) return false;
    return s_peer_connected;
}

int mesh_get_layer(void) { return esp_mesh_get_layer(); }
int  mesh_get_topology(void)   { return (int)esp_mesh_get_topology(); }
bool mesh_is_ps_enabled(void)  { return esp_mesh_is_ps_enabled(); }

/* IP-адрес node (через NAT root) — берём у mesh_link_sta netif */
const char *mesh_get_ip(void)
{
    return "0.0.0.0";  /* IP-слой отключён, используем raw-протокол */
}