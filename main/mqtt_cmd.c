/* ============================================================
   mqtt_cmd.c — MQTT command handler + JSON config accumulator.

   === SIMPLIFIED-NO-CFG ===
       Убран transport_send_cfg — NODE больше не получает
       полную конфигурацию. После config:reload ROOT просто
       обновляет свой g_config; NODE получит новые маски через
       'c:' при следующей смене фазы (или force_phase).

   Остальные FIX сохранены:
     FIX ACCUM-DONE-HEURISTIC
     FIX ACCUM-OFFSET-SAFETY
     FIX ACCUM-BUFFER-SAFETY
     FIX ACCUM-USE-REAL-LEN
     FIX TRAFFIC-STOP-RESET-CFG
     FIX TRAFFIC-MODE-RESET-CFG

   === FIX LEAD-MS-RUNTIME ===
       Команды "sync_lead" (get) и "sync_lead:<ms>" (set) —
       меняют g_sync_lead_ms. Теперь это ТОЛЬКО fallback: при
       валидном UTC ROOT/NODE применяют маску по apply_utc_ms
       (см. root_apply_stage_sync_first / node_mask_scheduler_task).
       lead_ms остаётся запасным путём, если UTC недоступен.

   === FIX RT-CHANNELS ===
       config:reload: после config_load() + mqtt_apply_config_runtime()
       стартуем трафик через traffic_light_start_with_red() +
       root_apply_stage_sync_first(0, ...) — это первый c: с
       UTC-таймингом, чтобы ROOT и NODE включили красный
       ОДНОВРЕМЕННО. Раньше start_with_red() на ROOT вызывал
       apply_stage_dd(0) локально без c: — NODE получал c: только
       на следующем refresh от phase_sync_task (до 300 мс
       рассинхрона на первом включении).

       traffic:start при ALL_OFF — тоже через start_with_red() +
       root_apply_stage_sync_first(0, ...) для одновременного
       включения.
   ============================================================ */

#include "mqtt_internal.h"
#include "config_manager.h"
#include "traffic_light.h"
#include "wifi_ap.h"
#include "transport.h"
#include "rs485.h"
#include "can_bus.h"
#include "shift_register.h"
#include "sync_config.h"
#include "root_wd.h"        /* === FIX RT-CHANNELS: send_sync_to_node_async_utc === */
#include "time_sync.h"      /* === FIX RT-CHANNELS: time_sync_get_utc_ms === */
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "modem_ppp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "app_shared.h"


static const char *TAG = "mqtt_cmd";
#define NVS_NS_PENDING   "pending_cfg"
#define NVS_KEY_LEN      "len"
#define NVS_KEY_CHUNK    "chunk_%d"
#define PENDING_CHUNK_SZ 4096

extern void root_apply_new_config_from_mqtt(const char *json, int len);

/* ============================================================
   JSON accumulator
   ============================================================ */
static char *s_json_accum        = NULL;
static int   s_json_accum_len    = 0;
static int   s_json_accum_total  = 0;
static bool  s_json_accum_active = false;

void mqtt_json_accum_reset(void)
{
    if (s_json_accum) { free(s_json_accum); s_json_accum = NULL; }
    s_json_accum_len = 0;
    s_json_accum_total = 0;
    s_json_accum_active = false;
}

/* ============================================================
   mqtt_json_accum_feed
   === FIX ACCUM-DONE-HEURISTIC ===
   === FIX ACCUM-OFFSET-SAFETY ===
   === FIX ACCUM-BUFFER-SAFETY ===
   ============================================================ */
bool mqtt_json_accum_feed(const char *data, int data_len, int offset, int total)
{
    if (!data || data_len <= 0) return false;

    if (offset == 0) {
        mqtt_json_accum_reset();

        int t = (total > data_len && total <= MQTT_JSON_INPUT_SIZE)
                ? total : MQTT_JSON_INPUT_SIZE;
        s_json_accum = calloc(1, t + 1);
        if (!s_json_accum) {
            ESP_LOGE(TAG, "accum_feed: calloc(%d) failed", t + 1);
            return false;
        }
        s_json_accum_total  = t;
        s_json_accum_len    = 0;
        s_json_accum_active = true;
    }

    if (!s_json_accum_active) return false;

    if (offset > s_json_accum_len) {
        ESP_LOGW(TAG, "accum_feed: offset=%d > len=%d, reset",
                 offset, s_json_accum_len);
        mqtt_json_accum_reset();
        return false;
    }
    if (offset + data_len > s_json_accum_total) {
        ESP_LOGW(TAG, "accum_feed: overflow (offset=%d + len=%d > total=%d)",
                 offset, data_len, s_json_accum_total);
        mqtt_json_accum_reset();
        return false;
    }
    memcpy(s_json_accum + offset, data, data_len);
    s_json_accum_len = offset + data_len;

    bool done_by_total = (s_json_accum_total > 0 &&
                          s_json_accum_len >= s_json_accum_total);
    bool done_by_brace = false;
    if (s_json_accum_len > 0) {
        int end = s_json_accum_len - 1;
        while (end >= 0 && (s_json_accum[end] == ' ' ||
                            s_json_accum[end] == '\n' ||
                            s_json_accum[end] == '\r' ||
                            s_json_accum[end] == '\t')) {
            end--;
        }
        if (end >= 0 && s_json_accum[end] == '}') {
            int depth = 0;
            for (int i = 0; i <= end; i++) {
                if (s_json_accum[i] == '{') depth++;
                else if (s_json_accum[i] == '}') depth--;
            }
            if (depth == 0) done_by_brace = true;
        }
    }
    return done_by_total || done_by_brace;
}

static bool looks_like_config(const char *json, int len)
{
    if (!json || len < 2) return false;
    if (json[0] != '{') return false;
    if (strstr(json, "\"meta\"") || strstr(json, "\"traffic\"") ||
        strstr(json, "\"shift_register\"") || strstr(json, "\"mqtt\""))
        return true;
    return false;
}

static esp_err_t save_pending_cfg_to_nvs(const char *data, int data_len)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS_PENDING, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_u32(nvs, NVS_KEY_LEN, (uint32_t)data_len);
    if (err != ESP_OK) { nvs_close(nvs); return err; }

    char key[16];
    int n_chunks = 0;
    for (int off = 0; off < data_len; off += PENDING_CHUNK_SZ) {
        int chunk = (data_len - off > PENDING_CHUNK_SZ)
                    ? PENDING_CHUNK_SZ : (data_len - off);
        snprintf(key, sizeof(key), NVS_KEY_CHUNK, off / PENDING_CHUNK_SZ);
        err = nvs_set_blob(nvs, key, data + off, chunk);
        if (err != ESP_OK) { nvs_close(nvs); return err; }
        n_chunks++;
    }
    err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "save_pending_cfg OK (%d bytes, %d chunks)",
                 data_len, n_chunks);
    }
    return err;
}

/* ============================================================
   mqtt_json_accum_process
   === FIX ACCUM-USE-REAL-LEN ===
   ============================================================ */
void mqtt_json_accum_process(void)
{
    if (!s_json_accum || s_json_accum_len <= 0) {
        ESP_LOGW(TAG, "accum_process: no data (accum=%p len=%d)",
                 s_json_accum, s_json_accum_len);
        mqtt_json_accum_reset();
        return;
    }

    int real_len = s_json_accum_len;
    ESP_LOGI(TAG, "accum_process: %d bytes (buffer=%d), first='%c' last='%c'",
             real_len, s_json_accum_total,
             s_json_accum[0], s_json_accum[real_len - 1]);

    if (!looks_like_config(s_json_accum, real_len)) {
        ESP_LOGW(TAG, "accum_process: not a config (%d bytes), skip: '%.*s'",
                 real_len, real_len > 80 ? 80 : real_len, s_json_accum);
        mqtt_json_accum_reset();
        return;
    }

    esp_err_t err = save_pending_cfg_to_nvs(s_json_accum, real_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "accum_process: save_pending_cfg failed: %s",
                 esp_err_to_name(err));
        mqtt_json_accum_reset();
        if (s_mqtt_connected && g_mqtt_client)
            esp_mqtt_client_publish(g_mqtt_client, s_topic,
                                    "{\"error\":\"nvs_save\"}", 0, 1, 0);
        return;
    }

    int len = real_len;
    char *copy = malloc(len + 1);
    if (copy) {
        memcpy(copy, s_json_accum, len);
        copy[len] = '\0';
        ESP_LOGI(TAG, "accum_process: calling root_apply_new_config_from_mqtt "
                      "(%d bytes)", len);
        root_apply_new_config_from_mqtt(copy, len);
        free(copy);
    } else {
        ESP_LOGE(TAG, "accum_process: malloc(%d) failed", len);
    }

    if (s_mqtt_connected && g_mqtt_client) {
        esp_mqtt_client_publish(g_mqtt_client, s_topic,
            "{\"config\":\"updating\",\"yellow_blink\":true}", 0, 1, 0);
    }
    mqtt_json_accum_reset();
}

void mqtt_handle_json_config(const char *data, int data_len)
{
    if (data_len <= 0 || data_len > MQTT_JSON_INPUT_SIZE) return;
    if (!looks_like_config(data, data_len)) {
        ESP_LOGW(TAG, "/SetJson: not a config (%d bytes)", data_len);
        return;
    }
    static uint32_t s_last_ms = 0;
    static int      s_last_len = 0;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (s_last_len == data_len && s_last_ms != 0 &&
        (now_ms - s_last_ms) < 2000) {
        ESP_LOGW(TAG, "/SetJson: duplicate, skip");
        return;
    }
    s_last_ms = now_ms; s_last_len = data_len;

    esp_err_t err = save_pending_cfg_to_nvs(data, data_len);
    if (err != ESP_OK) {
        if (s_mqtt_connected && g_mqtt_client)
            esp_mqtt_client_publish(g_mqtt_client, s_topic,
                                    "{\"error\":\"nvs_save\"}", 0, 1, 0);
        return;
    }

    char *copy = malloc(data_len + 1);
    if (copy) {
        memcpy(copy, data, data_len);
        copy[data_len] = '\0';
        root_apply_new_config_from_mqtt(copy, data_len);
        free(copy);
    } else {
        ESP_LOGE(TAG, "/SetJson: malloc(%d) failed", data_len);
    }

    if (s_mqtt_connected && g_mqtt_client) {
        esp_mqtt_client_publish(g_mqtt_client, s_topic,
            "{\"config\":\"updating\",\"yellow_blink\":true}", 0, 1, 0);
    }
    ESP_LOGW(TAG, "/SetJson: config update queued");
}

/* ============================================================
   parse_hex — для rs485:send / can:send
   ============================================================ */
static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool parse_hex(const char *p, uint8_t *buf, size_t max, size_t *out_n)
{
    size_t hl = strlen(p);
    if (hl == 0 || (hl % 2) != 0 || hl / 2 > max) return false;
    size_t n = hl / 2;
    for (size_t i = 0; i < n; i++) {
        int hi = hex_nibble(p[i * 2]);
        int lo = hex_nibble(p[i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        buf[i] = (uint8_t)((hi << 4) | lo);
    }
    *out_n = n;
    return true;
}

/* ============================================================
   === FIX RT-CHANNELS ===
   root_start_traffic_rt — старт трафика на ROOT с одновременным
   включением каналов на ROOT и NODE по UTC.

   Используется:
     - config:reload с auto_start
     - traffic:start при восстановлении из ALL_OFF

   Логика:
     1. traffic_light_start_with_red() на ROOT — подготавливает
        состояние (STOPPED→RUNNING, phase=0), но НЕ включает
        каналы сразу (start_with_red на ROOT пока вызывает
        apply_stage_dd(0) — это первый локальный старт, который
        мы перезапишем ниже через root_apply_stage_sync_first).
        На NODE start_with_red уходит в STANDBY (follow_root).
     2. root_apply_stage_sync_first(0, dur):
        - вычисляет apply_utc_ms = now_utc + 150;
        - шлёт c:0:...:red с apply_utc_ms → NODE планирует;
        - ROOT ждёт до apply_utc_ms;
        - ROOT применяет маску 0 локально.
     3. Если UTC недоступен — fallback на lead_ms внутри
        root_apply_stage_sync_first.

   Внимание: root_apply_stage_sync_first — static в traffic_light.c.
   Чтобы вызвать его из mqtt_cmd.c, используем публичный
   force_phase(0, ...), который внутри также идёт через
   root_apply_stage_sync_first. Но force_phase выставляет
   s_phase_start_ms = now - phase_ms и s_state = RUNNING —
   это нам и нужно после start_with_red. */
static void root_start_traffic_rt(void)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return;
    if (!transport_is_lora()) {
        /* Без LoRa NODE не существует — просто локальный старт. */
        traffic_light_start_with_red();
        return;
    }

    /* Снимаем возможный fault. */
    s_fault_from_remote_ex = true;
    traffic_light_set_fault(TRAFFIC_FAULT_NONE);
    s_fault_from_remote_ex = false;

    /* Убеждаемся, что ROOT в состоянии RUNNING/phase=0. */
    traffic_light_start_with_red();

    /* Одновременное включение каналов по UTC. force_phase(0, 0, dur)
       внутри вызовет root_apply_stage_sync_first(0, dur): отправит
       c:0:...:red с apply_utc_ms на NODE, дождётся UTC и применит
       маску 0 локально. */
    uint32_t dur0 = traffic_light_stage_duration_ms(0);
    esp_err_t ferr = traffic_light_force_phase(0, 0, dur0);
    if (ferr != ESP_OK) {
        ESP_LOGW(TAG, "root_start_traffic_rt: force_phase(0) failed: %s "
                      "→ channels may be out of sync",
                 esp_err_to_name(ferr));
    } else {
        ESP_LOGI(TAG, "root_start_traffic_rt: c:0 (red) sent with "
                      "apply_utc_ms, ROOT+NODE channels synced");
    }
}
/* === END FIX RT-CHANNELS === */

/* ============================================================
   mqtt_handle_command
   === SIMPLIFIED-NO-CFG ===
   - config:reload: только локальное обновление, без transport_send_cfg
   - traffic:stop / traffic:mode: без сброса cfg-трекинга (его больше нет)
   === FIX LEAD-MS-RUNTIME ===
   - sync_lead / sync_lead:<ms>
   === FIX RT-CHANNELS ===
   - config:reload с auto_start → root_start_traffic_rt()
   - traffic:start из ALL_OFF → root_start_traffic_rt()
   ============================================================ */
void mqtt_handle_command(const char *data, int data_len)
{
    char cmd[256];
    int len = data_len < (int)sizeof(cmd) - 1 ? data_len : (int)sizeof(cmd) - 1;
    memcpy(cmd, data, len);
    cmd[len] = '\0';
    ESP_LOGI(TAG, "CMD: %s", cmd);

    /* ROOT транслирует почти все команды NODE. */
    if (g_config.role == DEVICE_ROLE_MASTER &&
        strncmp(cmd, "nvs:erase", 9) != 0 &&
        strncmp(cmd, "role:", 5) != 0 &&
        strncmp(cmd, "config:reload", 13) != 0 &&
        strncmp(cmd, "sync:", 5) != 0 &&
        strncmp(cmd, "sync_lead", 9) != 0 &&
        strncmp(cmd, "fault:", 6) != 0) {
        transport_send_cmd(cmd, strlen(cmd));
    }

    int relay = -1, state = -1;
    bool changed = false;

    if (strncmp(cmd, "key_pressed:", 12) == 0) { changed = true; goto publish; }

    if (strcmp(cmd, "net:info") == 0) {
        ESP_LOGI(TAG, "NET: ppp=%d ip=%s transport=%s",
                 modem_ppp_is_connected() ? 1 : 0,
                 modem_ppp_get_ip(),
                 transport_is_lora() ? "LoRa" : "TCP");
        changed = true; goto publish;
    }

    if (strcmp(cmd, "transport:lora") == 0) {
        transport_force_lora(); changed = true; goto publish;
    }
    if (strcmp(cmd, "transport:tcp") == 0) {
        transport_force_tcp(); changed = true; goto publish;
    }

    if (strcmp(cmd, "role:node") == 0) {
        esp_mqtt_client_publish(g_mqtt_client, s_topic,
                                "{\"role\":\"node\",\"reboot\":true}", 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
        config_save_role(DEVICE_ROLE_SLAVE);
        esp_restart();
    }
    if (strcmp(cmd, "role:root") == 0) {
        esp_mqtt_client_publish(g_mqtt_client, s_topic,
                                "{\"role\":\"root\",\"reboot\":true}", 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
        config_save_role(DEVICE_ROLE_MASTER);
        esp_restart();
    }
    if (sscanf(cmd, "relay:%d:%d", &relay, &state) == 2 ||
        sscanf(cmd, "sr:%d:%d",    &relay, &state) == 2) {
        if (relay >= 0 && relay < SR_OUTPUT_COUNT) {
            if (state) sr_output_on(relay); else sr_output_off(relay);
            changed = true;
        }
        goto publish;
    }
    if (strcmp(cmd, "all:on")  == 0) { sr_all_on();  changed = true; goto publish; }
    if (strcmp(cmd, "all:off") == 0) { sr_all_off(); changed = true; goto publish; }
    if (sscanf(cmd, "toggle:%d", &relay) == 1) {
        if (relay >= 0 && relay < SR_OUTPUT_COUNT) {
            sr_output_toggle(relay);
            changed = true;
        }
        goto publish;
    }
    if (strcmp(cmd, "traffic:start") == 0) {
        /* === FIX ALL-OFF-MANUAL-ONLY ===
           Если ROOT в ALL_OFF — сначала снять fault, потом
           start_with_red (восстановление с красного). Иначе — обычный
           traffic_light_start().

           === FIX RT-CHANNELS ===
           При восстановлении из ALL_OFF используем
           root_start_traffic_rt(): c:0:red с UTC-таймингом, чтобы
           ROOT и NODE включили красный ОДНОВРЕМЕННО. Раньше
           transport_send_cmd_noack("traffic:start") + локальный
           start_with_red давали рассинхрон до 300 мс. */
        if (traffic_light_get_fault() == TRAFFIC_FAULT_ALL_OFF) {
            ESP_LOGW(TAG, "traffic:start → recover from ALL_OFF (RT)");
            root_start_traffic_rt();
            wifi_mqtt_publish_event("recover", "manual:traffic_start", NULL, 0);
        } else {
            traffic_light_start();
        }
        /* === END FIX RT-CHANNELS === */
        changed = true;
        goto publish;
    }

    if (strcmp(cmd, "traffic:stop") == 0) {
        traffic_light_stop();
        changed = true;
        goto publish;
    }

    if (strcmp(cmd, "traffic:pause")  == 0) {
        traffic_light_pause();  changed = true; goto publish;
    }
    if (strcmp(cmd, "traffic:resume") == 0) {
        traffic_light_resume(); changed = true; goto publish;
    }
    if (strcmp(cmd, "traffic:reset")  == 0) {
        traffic_light_reset();  changed = true; goto publish;
    }
    if (strcmp(cmd, "traffic:next")   == 0) {
        traffic_light_next_phase(); changed = true; goto publish;
    }

    if (strcmp(cmd, "traffic:night:on") == 0) {
        traffic_light_set_night_mode(true);
        changed = true; goto publish;
    }
    if (strcmp(cmd, "traffic:night:off") == 0) {
        traffic_light_set_night_mode(false);
        changed = true; goto publish;
    }

    {
        int mph = -1;
        if (sscanf(cmd, "manual:override:phase:%d", &mph) == 1 &&
            mph >= 0 && mph < g_config.traffic.sequence_count) {
            traffic_light_manual_override((uint8_t)mph);
            changed = true; goto publish;
        }
    }
    if (strcmp(cmd, "manual:override:off") == 0) {
        traffic_light_manual_release();
        changed = true; goto publish;
    }

    {
        unsigned int pms = 0;
        if (sscanf(cmd, "priority:extend:%u", &pms) == 1 && pms > 0) {
            traffic_light_priority_extend(pms);
            changed = true; goto publish;
        }
        if (sscanf(cmd, "priority:shorten:%u", &pms) == 1 && pms > 0) {
            traffic_light_priority_shorten(pms);
            changed = true; goto publish;
        }
    }

    {
        int mode = 0;
        if (sscanf(cmd, "traffic:mode:%d", &mode) == 1) {
            if (mode >= 1 && mode <= 3) {
                if (traffic_light_set_mode((traffic_mode_t)mode) == ESP_OK) {
                    changed = true;
                } else {
                    if (s_mqtt_connected && g_mqtt_client)
                        esp_mqtt_client_publish(g_mqtt_client, s_topic,
                            "{\"error\":\"mode_while_running\"}", 0, 1, 0);
                }
            }
            goto publish;
        }
    }

    {
        int tch = -1;
        if (sscanf(cmd, "test:%d", &tch) == 1) {
            if (tch >= 0 && tch < SR_OUTPUT_COUNT)
                traffic_light_test_channel((uint8_t)tch);
            goto publish;
        }
    }
    if (strcmp(cmd, "pedestrian:request") == 0) {
        traffic_light_request_pedestrian();
        changed = true; goto publish;
    }
    if (strcmp(cmd, "rs485:stats") == 0) {
        rs485_stats_t st; rs485_get_stats(&st);
        ESP_LOGI(TAG, "RS485: rx_b=%lu rx_f=%lu ovf=%lu tx_b=%lu tx_err=%lu",
                 (unsigned long)st.rx_bytes, (unsigned long)st.rx_frames,
                 (unsigned long)st.rx_overflows, (unsigned long)st.tx_bytes,
                 (unsigned long)st.tx_errors);
        changed = true; goto publish;
    }
    if (strcmp(cmd, "rs485:reset_stats") == 0) {
        rs485_reset_stats(); changed = true; goto publish;
    }
    {
        int baud = 0;
        if (sscanf(cmd, "rs485:baud:%d", &baud) == 1 && baud > 0) {
            rs485_set_baudrate((uint32_t)baud); changed = true; goto publish;
        }
        if (strncmp(cmd, "rs485:send:", 11) == 0) {
            uint8_t buf[256]; size_t n = 0;
            if (parse_hex(cmd + 11, buf, sizeof(buf), &n)) rs485_send(buf, n);
            changed = true; goto publish;
        }
    }
    if (strcmp(cmd, "can:stats") == 0) {
        can_bus_stats_t cs; can_bus_get_stats(&cs);
        ESP_LOGI(TAG, "CAN: rx=%lu tx=%lu tx_err=%lu bus_off=%lu rec=%lu "
                 "err_p=%lu state=%lu",
                 (unsigned long)cs.rx_frames, (unsigned long)cs.tx_frames,
                 (unsigned long)cs.tx_errors, (unsigned long)cs.bus_off_count,
                 (unsigned long)cs.recovery_count,
                 (unsigned long)cs.error_passive_count,
                 (unsigned long)cs.last_state);
        changed = true; goto publish;
    }
    if (strcmp(cmd, "can:reset_stats") == 0) {
        can_bus_reset_stats(); changed = true; goto publish;
    }
    {
        unsigned int id = 0; char hex_data[32] = {0}; int ext_flag = 0;
        int matched = sscanf(cmd, "can:send:%x:%16[0-9a-fA-F]:%d",
                             &id, hex_data, &ext_flag);
        if (matched >= 2 ||
            (matched == 1 && sscanf(cmd, "can:send:%x", &id) == 1)) {
            uint8_t data[8] = {0}; size_t dlc = 0;
            parse_hex(hex_data, data, sizeof(data), &dlc);
            can_bus_send(id, data, (uint8_t)dlc, ext_flag != 0);
            changed = true; goto publish;
        }
    }
    if (strcmp(cmd, "can:standby") == 0) {
        can_bus_set_standby(true);  changed = true; goto publish;
    }
    if (strcmp(cmd, "can:normal")  == 0) {
        can_bus_set_standby(false); changed = true; goto publish;
    }

    if (strcmp(cmd, "rtt") == 0) {
        extern int32_t  blink_get_rtt_samples(void);
        extern uint32_t blink_get_rtt_us(void);
        extern uint32_t blink_get_rtt_delay_us(void);
        ESP_LOGI(TAG, "RTT: us=%lu delay_us=%lu samples=%ld",
                 (unsigned long)blink_get_rtt_us(),
                 (unsigned long)blink_get_rtt_delay_us(),
                 (long)blink_get_rtt_samples());
        changed = true; goto publish;
    }
    if (strcmp(cmd, "rtt:reset") == 0) {
        extern void blink_reset_rtt(void);
        blink_reset_rtt(); changed = true; goto publish;
    }

    /* === FIX LEAD-MS-RUNTIME ===
       sync_lead:<ms> — изменить fallback-задержку ROOT на случай,
       если UTC недоступен (apply_utc_ms == 0 в c:).
       При валидном UTC эта настройка НЕ используется —
       ROOT/NODE применяют маску по apply_utc_ms.
       sync_lead     — прочитать текущее значение (в лог).
       Локальная команда (не транслируется на NODE). */
    {
        unsigned int lead = 0;
        if (sscanf(cmd, "sync_lead:%u", &lead) == 1) {
            int r = sync_config_set_lead_ms((uint32_t)lead);
            if (r != ESP_OK) {
                if (s_mqtt_connected && g_mqtt_client) {
                    char err[96];
                    int en = snprintf(err, sizeof(err),
                        "{\"error\":\"sync_lead_range\","
                        "\"min\":%u,\"max\":%u}",
                        (unsigned)SYNC_LEAD_MS_MIN,
                        (unsigned)SYNC_LEAD_MS_MAX);
                    if (en > 0)
                        esp_mqtt_client_publish(g_mqtt_client, s_topic,
                                                err, en, 1, 0);
                }
            }
            changed = true;
            goto publish;
        }
        if (strcmp(cmd, "sync_lead") == 0) {
            ESP_LOGI(TAG, "sync_lead=%lu ms (fallback, active only if UTC "
                          "unavailable)",
                     (unsigned long)sync_config_get_lead_ms());
            changed = true;
            goto publish;
        }
    }
    /* === END FIX LEAD-MS-RUNTIME === */

    /* === SIMPLIFIED-NO-CFG ===
       config:reload — только локальное обновление ROOT.
       NODE получит новые маски через 'c:' на следующей смене фазы.

       === FIX RT-CHANNELS ===
       Если auto_start включён — используем root_start_traffic_rt():
       после перезагрузки конфига ROOT и NODE включат красный
       ОДНОВРЕМЕННО (c:0 с apply_utc_ms). Раньше был локальный
       start_with_red() на ROOT без c: — NODE получал маску только
       на следующем refresh от phase_sync_task (до 300 мс). */
    if (strcmp(cmd, "config:reload") == 0) {
        ESP_LOGI(TAG, "config:reload → reading NVS");
        traffic_light_stop();
        config_load();
        mqtt_apply_config_runtime();

        if (g_config.traffic.enabled && g_config.traffic.auto_start) {
            ESP_LOGW(TAG, "config:reload: auto_start → root_start_traffic_rt");
            root_start_traffic_rt();
        }

        ESP_LOGI(TAG, "config:reload done (NODE маски через c: с UTC)");
        changed = true;
        goto publish;
    }
    /* === END SIMPLIFIED-NO-CFG + FIX RT-CHANNELS === */

    if (strcmp(cmd, "nvs:erase") == 0) {
        ESP_LOGW(TAG, "nvs:erase → reboot");
        if (s_mqtt_connected && g_mqtt_client) {
            esp_mqtt_client_publish(g_mqtt_client, s_topic,
                "{\"nvs\":\"erasing\"}", 0, 1, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        esp_task_wdt_reset();
        nvs_flash_erase();
        esp_restart();
    }

    if (strcmp(cmd, "sync_lead:erase") == 0) {
        esp_err_t e = sync_config_nvs_erase();
        if (s_mqtt_connected && g_mqtt_client) {
            char resp[64];
            int rn = snprintf(resp, sizeof(resp),
                e == ESP_OK
                    ? "{\"sync_lead_erase\":\"ok\"}"
                    : "{\"error\":\"sync_lead_erase_fail\"}");
            if (rn > 0)
                esp_mqtt_client_publish(g_mqtt_client, s_topic,
                                        resp, rn, 1, 0);
        }
        changed = true;
        goto publish;
    }

    ESP_LOGW(TAG, "Unknown: %s", cmd);
    return;

publish:
    if (changed) wifi_mqtt_publish_now();
}