// config_manager.c
#include "config_manager.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "CFG";
#define NVS_NS       "app_cfg"
#define NVS_KEY      "cfg_blob"

#define NVS_NS_ROLE   "device_cfg"
#define NVS_KEY_ROLE  "role"
#define NVS_KEY_LOCK  "role_lock"

#define CFG_MIN_HEAP_FOR_PARSE  (48 * 1024)

app_config_t g_config;

/* === FIX STACK-OVERFLOW ===
   app_config_t ≈ 7.7 КБ. На стеке main_task (3584 байт default)
   это вызывает stack overflow → порча newlib lock → assert.
   Выносим tmp в static + mutex. */
static app_config_t      s_cfg_load_tmp;
static SemaphoreHandle_t s_cfg_load_mtx = NULL;

static SemaphoreHandle_t cfg_load_lock(void)
{
    if (!s_cfg_load_mtx) s_cfg_load_mtx = xSemaphoreCreateMutex();
    if (!s_cfg_load_mtx) return NULL;
    if (xSemaphoreTake(s_cfg_load_mtx, portMAX_DELAY) != pdTRUE) return NULL;
    return s_cfg_load_mtx;
}
static inline void cfg_load_unlock(void)
{
    if (s_cfg_load_mtx) xSemaphoreGive(s_cfg_load_mtx);
}
/* === END FIX STACK-OVERFLOW === */

static char  *s_cfg_json_cache = NULL;
static size_t s_cfg_json_len   = 0;

static void config_cache_invalidate(void)
{
    if (s_cfg_json_cache) {
        free(s_cfg_json_cache);
        s_cfg_json_cache = NULL;
        s_cfg_json_len = 0;
        ESP_LOGD(TAG, "config cache invalidated");
    }
}

static const char *duration_kind_to_str(uint8_t kind);
static const char *ped_request_mode_to_str(uint8_t m);
static const char *hold_on_ped_to_str(uint8_t m);

/* ============================================================
   set_defaults_to(cfg) — не трогает глобальный g_config.
   ============================================================ */
static void set_defaults_to(app_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    cfg->version = 1.0f;
    cfg->id = 0;
    strcpy(cfg->name, "default");

    cfg->role = DEVICE_ROLE_MASTER;

    cfg->espnow.enabled = false;
    cfg->espnow.role = 0;
    cfg->espnow.channel = 6;
    cfg->espnow.status_period_ms = 1000;
    memset(cfg->espnow.slave_mac, 0, 6);

    strcpy(cfg->wifi_ssid, "Keenetic-4250");
    strcpy(cfg->wifi_password, "VTuVMHPi");
    strcpy(cfg->ap_ssid, "");
    strcpy(cfg->ap_password, "");
    strcpy(cfg->timezone, "MSK-3");

    cfg->mqtt_enabled = true;
    strcpy(cfg->mqtt_uri, "mqtt://m2.wqtt.ru");
    cfg->mqtt_port = 13257;
    strcpy(cfg->mqtt_user, "u_GGENLB");
    strcpy(cfg->mqtt_password, "LTHNW22D");
    cfg->mqtt_timepause_min = 5;

    const char *names[] = { "solar", "bat", "load" };
    const int   pins[]  = { 18, 19, 20 };
    for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
        pwm_channel_config_t *c = &cfg->channels[i];
        strncpy(c->name, names[i], MAX_NAME_LEN - 1);
        c->gpio = pins[i];
        c->frequency_hz = 1000;
        c->duty_percent = 100;
        c->enabled = true;
        c->days_mask = 0x7F;
        c->start_hour = 8; c->end_hour = 20;
        c->duty_min = 0; c->duty_max = 100; c->duty_step = 10;
    }

    cfg->traffic.mode = 3;
    cfg->traffic.enabled = false;
    cfg->traffic.auto_start = true;

    cfg->traffic.pedestrian.button_enabled    = true;
    cfg->traffic.pedestrian.request_mode      = PED_REQUEST_MODE_STAGE;
    cfg->traffic.pedestrian.insert_timeout_ms = 30000;
    cfg->traffic.pedestrian.min_recall_s      = 5;

    /* === NEW: hold/insert defaults === */
    cfg->traffic.pedestrian.insert_count      = 0;
    cfg->traffic.pedestrian.insert_lead_ms    = 300;
    /* === FIX INSERT-RED-SHORT: 0 = использовать phases_ms.red === */
    cfg->traffic.pedestrian.insert_red_ms     = 0;
    /* === END FIX INSERT-RED-SHORT === */
    cfg->traffic.pedestrian.return_to_hold    = true;

    cfg->traffic.hold.enabled        = false;
    cfg->traffic.hold.stage_count    = 0;
    cfg->traffic.hold.max_hold_ms    = 0;
    cfg->traffic.hold.on_ped_request = HOLD_PED_INSERT;
    /* === END NEW: hold/insert === */

    cfg->traffic.pins.single = (traffic_color_pins_t){0,1,2};
    cfg->traffic.pins.cross.a_north = (traffic_color_pins_t){0,1,2};
    cfg->traffic.pins.cross.a_south = (traffic_color_pins_t){3,4,5};
    cfg->traffic.pins.cross.b_west  = (traffic_color_pins_t){6,7,8};
    cfg->traffic.pins.cross.b_east  = (traffic_color_pins_t){9,10,11};
    cfg->traffic.pins.cross_ped.axis_a  = (traffic_color_pins_t){0,1,2};
    cfg->traffic.pins.cross_ped.axis_a2 = (traffic_color_pins_t){3,4,5};
    cfg->traffic.pins.cross_ped.axis_a3 = (traffic_color_pins_t){6,7,8};
    cfg->traffic.pins.cross_ped.ped_red = 9;
    cfg->traffic.pins.cross_ped.ped_green = 10;

    cfg->traffic.phases_ms.red          = 8000;
    cfg->traffic.phases_ms.red_yellow   = 2000;
    cfg->traffic.phases_ms.green        = 10000;
    cfg->traffic.phases_ms.green_blink  = 3000;
    cfg->traffic.phases_ms.yellow       = 2000;
    cfg->traffic.phases_ms.yellow_blink = 2000;
    cfg->traffic.phases_ms.ped_green    = 10000;
    cfg->traffic.phases_ms.ped_blink    = 4000;
    cfg->traffic.phases_ms.fault_blink_period = 500;
    cfg->traffic.phases_ms.phase_blink_period = 500;

    cfg->traffic.fault.imin_mv = 10;
    cfg->traffic.fault.vin_on_min_v = 5.0f;
    cfg->traffic.fault.vin_off_min_v = 5.0f;
    cfg->traffic.fault.phase_grace_ms = 2500;
    cfg->traffic.fault.recovery_ms = 2000;
    cfg->traffic.fault.vin_off_confirmations = 3;

    /* Дефолтные stages: 4 фазы */
    traffic_stage_dd_t *st = cfg->traffic.stages;
    st[0] = (traffic_stage_dd_t){ .id=1, .name="Фаза.1", .duration_ms=8000,
        .duration_kind=DD_DUR_EXPLICIT, .outputs_on={0}, .on_count=1 };
    st[1] = (traffic_stage_dd_t){ .id=2, .name="Фаза.2", .duration_ms=8000,
        .duration_kind=DD_DUR_EXPLICIT, .outputs_on={3}, .on_count=1 };
    st[2] = (traffic_stage_dd_t){ .id=3, .name="Фаза.3", .duration_ms=10000,
        .duration_kind=DD_DUR_EXPLICIT, .outputs_on={6,9}, .on_count=2 };
    st[3] = (traffic_stage_dd_t){ .id=4, .name="Фаза.4", .duration_ms=10000,
        .duration_kind=DD_DUR_EXPLICIT, .outputs_on={9}, .on_count=1,
        .outputs_blink={10}, .blink_count=1 };
    cfg->traffic.stage_count = 4;

    cfg->traffic.sequence[0] = 1;
    cfg->traffic.sequence[1] = 2;
    cfg->traffic.sequence[2] = 3;
    cfg->traffic.sequence[3] = 4;
    cfg->traffic.sequence_count = 4;

    for (int i = 0; i < MAX_OUTPUTS; i++) {
        cfg->sr.outputs[i].ch = (uint8_t)i;
        snprintf(cfg->sr.outputs[i].name, sizeof(cfg->sr.outputs[i].name),
                 "%d", i + 1);
        cfg->sr.outputs[i].active = (i < 12);
    }
    cfg->sr.count = MAX_OUTPUTS;
}

static void set_defaults(void) { set_defaults_to(&g_config); }

/* ============================================================ */
void config_load(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS not opened (%s), defaults", esp_err_to_name(err));
        set_defaults();
        config_save();
        return;
    }
    size_t sz = sizeof(g_config);
    err = nvs_get_blob(nvs, NVS_KEY, &g_config, &sz);
    nvs_close(nvs);

    if (err != ESP_OK || sz != sizeof(g_config)) {
        ESP_LOGW(TAG, "NVS blob missing/bad size (got %u, want %u), defaults",
                 (unsigned)sz, (unsigned)sizeof(g_config));
        set_defaults();
        config_save();
    } else {
        ESP_LOGI(TAG, "Config loaded (version=%.2f, SSID='%s', traffic.mode=%u, "
                      "stages=%u, seq=%u, ped_mode=%u, hold=%d/%u, "
                      "insert_red_ms=%lu)",
                 g_config.version,
                 g_config.wifi_ssid, g_config.traffic.mode,
                 g_config.traffic.stage_count, g_config.traffic.sequence_count,
                 g_config.traffic.pedestrian.request_mode,
                 g_config.traffic.hold.enabled,
                 g_config.traffic.hold.stage_count,
                 (unsigned long)g_config.traffic.pedestrian.insert_red_ms);
    }
}

void config_save(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_blob(nvs, NVS_KEY, &g_config, sizeof(g_config));
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err == ESP_OK) {
        config_cache_invalidate();
        ESP_LOGI(TAG, "Config saved to NVS (version=%.2f, %u bytes, ped_mode=%u, "
                      "hold=%d, insert_red_ms=%lu)",
                 g_config.version, (unsigned)sizeof(g_config),
                 g_config.traffic.pedestrian.request_mode,
                 g_config.traffic.hold.enabled,
                 (unsigned long)g_config.traffic.pedestrian.insert_red_ms);
    } else {
        ESP_LOGE(TAG, "nvs_set_blob failed: %s", esp_err_to_name(err));
    }
}

void config_reset_defaults(void)
{
    set_defaults();
    config_save();
    config_apply_timezone();
}

void config_apply_timezone(void)
{
    setenv("TZ", g_config.timezone, 1);
    tzset();
    ESP_LOGI(TAG, "TZ=%s", g_config.timezone);
}

void config_init(void)
{
    config_load();
    config_apply_timezone();
    g_config.role = config_load_role();
}

/* ============================================================ */
esp_err_t config_save_role(device_role_t role)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS_ROLE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_u8(nvs, NVS_KEY_ROLE, (uint8_t)role);
    if (err == ESP_OK) err = nvs_set_u8(nvs, NVS_KEY_LOCK, 1);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err == ESP_OK) {
        g_config.role = role;
        config_cache_invalidate();
    }
    ESP_LOGI(TAG, "role saved: %s (locked)",
             role == DEVICE_ROLE_SLAVE ? "slave" : "master");
    return err;
}

device_role_t config_load_role(void)
{
    nvs_handle_t nvs;
    uint8_t v = 0, lock = 0;
    device_role_t role = DEVICE_ROLE_MASTER;
    bool locked = false;

    if (nvs_open(NVS_NS_ROLE, NVS_READONLY, &nvs) == ESP_OK) {
        if (nvs_get_u8(nvs, NVS_KEY_ROLE, &v) == ESP_OK) {
            role = (v == 1) ? DEVICE_ROLE_SLAVE : DEVICE_ROLE_MASTER;
        }
        if (nvs_get_u8(nvs, NVS_KEY_LOCK, &lock) == ESP_OK && lock == 1) {
            locked = true;
        }
        nvs_close(nvs);
    }

    ESP_LOGI(TAG, "role loaded: %s, locked=%d",
             role == DEVICE_ROLE_SLAVE ? "slave" : "master", locked);
    return role;
}

bool config_role_is_locked(void)
{
    nvs_handle_t nvs;
    uint8_t lock = 0;
    if (nvs_open(NVS_NS_ROLE, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u8(nvs, NVS_KEY_LOCK, &lock);
        nvs_close(nvs);
    }
    return lock == 1;
}

/* ============================================================ */
static void json_get_str(const cJSON *obj, const char *key, char *out, size_t sz)
{
    if (!obj) return;
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsString(item) && item->valuestring) {
        strncpy(out, item->valuestring, sz - 1);
        out[sz - 1] = '\0';
    }
}
static int json_get_int(const cJSON *obj, const char *key, int def)
{
    if (!obj) return def;
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    return cJSON_IsNumber(item) ? item->valueint : def;
}
static double json_get_double(const cJSON *obj, const char *key, double def)
{
    if (!obj) return def;
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    return cJSON_IsNumber(item) ? item->valuedouble : def;
}
static bool json_get_bool(const cJSON *obj, const char *key, bool def)
{
    if (!obj) return def;
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsBool(item)) return cJSON_IsTrue(item);
    return def;
}
static void parse_color_pins(const cJSON *p, const char *key,
                             traffic_color_pins_t *out)
{
    const cJSON *c = cJSON_GetObjectItemCaseSensitive(p, key);
    if (!cJSON_IsObject(c)) return;
    out->red    = (uint8_t)json_get_int(c, "red",    out->red);
    out->yellow = (uint8_t)json_get_int(c, "yellow", out->yellow);
    out->green  = (uint8_t)json_get_int(c, "green",  out->green);
}

static uint8_t parse_duration_kind(const cJSON *item, uint32_t fallback_ms,
                                   uint32_t *out_ms)
{
    *out_ms = (uint32_t)json_get_int(item, "duration_ms", (int)fallback_ms);

    const cJSON *k = cJSON_GetObjectItemCaseSensitive(item, "duration_kind");
    if (!k) return DD_DUR_EXPLICIT;

    if (cJSON_IsNumber(k)) {
        uint8_t v = (uint8_t)k->valueint;
        if (v > DD_DUR_PED_BLINK) v = DD_DUR_EXPLICIT;
        return v;
    }

    if (!cJSON_IsString(k) || !k->valuestring) return DD_DUR_EXPLICIT;

    char buf[32];
    size_t n = strlen(k->valuestring);
    if (n >= sizeof(buf)) n = sizeof(buf) - 1;
    for (size_t i = 0; i < n; i++)
        buf[i] = (k->valuestring[i] == ' ') ? '_' : k->valuestring[i];
    buf[n] = '\0';

    if (strcmp(buf, "red")          == 0) return DD_DUR_RED;
    if (strcmp(buf, "red_yellow")   == 0) return DD_DUR_RED_YELLOW;
    if (strcmp(buf, "green")        == 0) return DD_DUR_GREEN;
    if (strcmp(buf, "green_blink")  == 0) return DD_DUR_GREEN_BLINK;
    if (strcmp(buf, "yellow")       == 0) return DD_DUR_YELLOW;
    if (strcmp(buf, "yellow_blink") == 0) return DD_DUR_YELLOW_BLINK;
    if (strcmp(buf, "ped_green")    == 0) return DD_DUR_PED_GREEN;
    if (strcmp(buf, "ped_blink")    == 0) return DD_DUR_PED_BLINK;
    if (strcmp(buf, "explicit")     == 0) return DD_DUR_EXPLICIT;

    ESP_LOGW(TAG, "Unknown duration_kind='%s', using duration_ms",
             k->valuestring);
    return DD_DUR_EXPLICIT;
}

static void parse_stages_dd(const cJSON *arr, traffic_config_t *tc)
{
    if (!cJSON_IsArray(arr)) return;
    tc->stage_count = 0;
    const cJSON *item = NULL;
    cJSON_ArrayForEach(item, arr) {
        if (tc->stage_count >= MAX_DD_STAGES) break;
        traffic_stage_dd_t *st = &tc->stages[tc->stage_count];
        memset(st, 0, sizeof(*st));
        st->id = (uint8_t)json_get_int(item, "id", 0);
        json_get_str(item, "name", st->name, sizeof(st->name));
        st->duration_kind = parse_duration_kind(item, 1000, &st->duration_ms);

        const cJSON *g = cJSON_GetObjectItemCaseSensitive(item, "groups");
        if (cJSON_IsArray(g)) {
            const cJSON *gi = NULL;
            cJSON_ArrayForEach(gi, g) {
                if (st->group_count < MAX_STAGE_GROUPS && cJSON_IsNumber(gi))
                    st->groups[st->group_count++] = (uint8_t)gi->valueint;
            }
        }
        const cJSON *o = cJSON_GetObjectItemCaseSensitive(item, "outputs_on");
        if (cJSON_IsArray(o)) {
            const cJSON *oi = NULL;
            cJSON_ArrayForEach(oi, o) {
                if (st->on_count < MAX_ON_OUTPUTS && cJSON_IsNumber(oi))
                    st->outputs_on[st->on_count++] = (uint8_t)oi->valueint;
            }
        }
        const cJSON *b = cJSON_GetObjectItemCaseSensitive(item, "outputs_blink");
        if (cJSON_IsArray(b)) {
            const cJSON *bi = NULL;
            cJSON_ArrayForEach(bi, b) {
                if (st->blink_count < MAX_BLINK_OUTPUTS && cJSON_IsNumber(bi))
                    st->outputs_blink[st->blink_count++] = (uint8_t)bi->valueint;
            }
        }
        ESP_LOGI(TAG, "stage id=%u kind=%u dur_ms=%lu on=%u blink=%u",
                 st->id, st->duration_kind, (unsigned long)st->duration_ms,
                 st->on_count, st->blink_count);
        tc->stage_count++;
    }
    ESP_LOGI(TAG, "Parsed %u DD stages", tc->stage_count);
}

static void parse_transitions(const cJSON *arr, traffic_config_t *tc)
{
    if (!cJSON_IsArray(arr)) return;
    tc->transition_count = 0;
    const cJSON *item = NULL;
    cJSON_ArrayForEach(item, arr) {
        if (tc->transition_count >= MAX_TRANSITIONS) break;
        traffic_transition_t *t = &tc->transitions[tc->transition_count];
        t->clearing = (uint8_t)json_get_int(item, "clearing", 0);
        t->entering = (uint8_t)json_get_int(item, "entering", 0);
        t->time_cs  = (uint16_t)(json_get_double(item, "time", 0.0) * 100.0);
        tc->transition_count++;
    }
    ESP_LOGI(TAG, "Parsed %u transitions", tc->transition_count);
}

static void parse_pedestrian(const cJSON *t, traffic_config_t *tc)
{
    const cJSON *ped = cJSON_GetObjectItemCaseSensitive(t, "pedestrian");
    if (!cJSON_IsObject(ped)) {
        ESP_LOGI(TAG, "pedestrian: section missing → defaults");
        return;
    }

    tc->pedestrian.button_enabled =
        json_get_bool(ped, "button_enabled", true);
    tc->pedestrian.insert_timeout_ms =
        (uint32_t)json_get_int(ped, "insert_timeout_ms", 30000);
    tc->pedestrian.min_recall_s =
        (uint32_t)json_get_int(ped, "min_recall_s", 5);

    tc->pedestrian.insert_lead_ms =
        (uint32_t)json_get_int(ped, "insert_lead_ms", 300);
    /* === FIX INSERT-RED-SHORT: 0 = использовать phases_ms.red === */
    tc->pedestrian.insert_red_ms =
        (uint32_t)json_get_int(ped, "insert_red_ms", 0);
    /* === END FIX INSERT-RED-SHORT === */
    tc->pedestrian.return_to_hold =
        json_get_bool(ped, "return_to_hold", true);

    const cJSON *rm = cJSON_GetObjectItemCaseSensitive(ped, "request_mode");
    if (cJSON_IsString(rm) && rm->valuestring) {
        if (strcmp(rm->valuestring, "stage") == 0)
            tc->pedestrian.request_mode = PED_REQUEST_MODE_STAGE;
        else if (strcmp(rm->valuestring, "insert") == 0)
            tc->pedestrian.request_mode = PED_REQUEST_MODE_INSERT;
        else if (strcmp(rm->valuestring, "min_recall") == 0)
            tc->pedestrian.request_mode = PED_REQUEST_MODE_MIN_RECALL;
        else
            ESP_LOGW(TAG, "Unknown pedestrian.request_mode='%s', using 'stage'",
                     rm->valuestring);
    }

    const cJSON *is = cJSON_GetObjectItemCaseSensitive(ped, "insert_sequence");
    if (cJSON_IsArray(is)) {
        tc->pedestrian.insert_count = 0;
        const cJSON *it = NULL;
        cJSON_ArrayForEach(it, is) {
            if (tc->pedestrian.insert_count < MAX_INSERT_SEQ &&
                cJSON_IsNumber(it)) {
                tc->pedestrian.insert_sequence[tc->pedestrian.insert_count++] =
                    (uint8_t)it->valueint;
            }
        }
    }

    ESP_LOGI(TAG, "pedestrian: button=%d mode=%u insert=%lu min_recall=%lu "
                  "insert_seq=%u lead=%lu red=%lu return_hold=%d",
             tc->pedestrian.button_enabled,
             tc->pedestrian.request_mode,
             (unsigned long)tc->pedestrian.insert_timeout_ms,
             (unsigned long)tc->pedestrian.min_recall_s,
             tc->pedestrian.insert_count,
             (unsigned long)tc->pedestrian.insert_lead_ms,
             (unsigned long)tc->pedestrian.insert_red_ms,
             tc->pedestrian.return_to_hold);
}

static void parse_hold(const cJSON *t, traffic_config_t *tc)
{
    const cJSON *h = cJSON_GetObjectItemCaseSensitive(t, "hold");
    if (!cJSON_IsObject(h)) {
        ESP_LOGI(TAG, "hold: section missing → disabled");
        tc->hold.enabled = false;
        return;
    }

    tc->hold.enabled     = json_get_bool(h, "enabled", false);
    tc->hold.max_hold_ms = (uint32_t)json_get_int(h, "max_hold_ms", 0);

    const cJSON *ids = cJSON_GetObjectItemCaseSensitive(h, "stage_ids");
    if (cJSON_IsArray(ids)) {
        tc->hold.stage_count = 0;
        const cJSON *it = NULL;
        cJSON_ArrayForEach(it, ids) {
            if (tc->hold.stage_count < MAX_HOLD_STAGES && cJSON_IsNumber(it))
                tc->hold.stage_ids[tc->hold.stage_count++] = (uint8_t)it->valueint;
        }
    }

    const cJSON *opr = cJSON_GetObjectItemCaseSensitive(h, "on_ped_request");
    if (cJSON_IsString(opr) && opr->valuestring) {
        if (strcmp(opr->valuestring, "insert") == 0)
            tc->hold.on_ped_request = HOLD_PED_INSERT;
        else if (strcmp(opr->valuestring, "min_recall") == 0)
            tc->hold.on_ped_request = HOLD_PED_MIN_RECALL;
        else if (strcmp(opr->valuestring, "ignore") == 0)
            tc->hold.on_ped_request = HOLD_PED_IGNORE;
        else
            ESP_LOGW(TAG, "Unknown hold.on_ped_request='%s', using 'insert'",
                     opr->valuestring);
    }

    ESP_LOGI(TAG, "hold: enabled=%d stages=%u max_hold=%lu on_ped=%u",
             tc->hold.enabled, tc->hold.stage_count,
             (unsigned long)tc->hold.max_hold_ms,
             tc->hold.on_ped_request);
}

static void parse_traffic(const cJSON *t, traffic_config_t *tc)
{
    if (!cJSON_IsObject(t)) return;

    tc->mode       = (uint8_t)json_get_int(t, "mode", tc->mode);
    tc->enabled    = json_get_bool(t, "enabled", tc->enabled);
    tc->auto_start = json_get_bool(t, "auto_start", tc->auto_start);

    parse_pedestrian(t, tc);
    parse_hold(t, tc);

    const cJSON *pins = cJSON_GetObjectItemCaseSensitive(t, "pins");
    if (cJSON_IsObject(pins)) {
        const cJSON *single = cJSON_GetObjectItemCaseSensitive(pins, "single");
        if (cJSON_IsObject(single)) {
            tc->pins.single.red    = (uint8_t)json_get_int(single, "red", 0);
            tc->pins.single.yellow = (uint8_t)json_get_int(single, "yellow", 1);
            tc->pins.single.green  = (uint8_t)json_get_int(single, "green", 2);
        }
        const cJSON *cross = cJSON_GetObjectItemCaseSensitive(pins, "cross");
        if (cJSON_IsObject(cross)) {
            parse_color_pins(cross, "a_north", &tc->pins.cross.a_north);
            parse_color_pins(cross, "a_south", &tc->pins.cross.a_south);
            parse_color_pins(cross, "b_west",  &tc->pins.cross.b_west);
            parse_color_pins(cross, "b_east",  &tc->pins.cross.b_east);
        }
        const cJSON *cp = cJSON_GetObjectItemCaseSensitive(pins, "cross_ped");
        if (cJSON_IsObject(cp)) {
            parse_color_pins(cp, "axis_a",  &tc->pins.cross_ped.axis_a);
            parse_color_pins(cp, "axis_a2", &tc->pins.cross_ped.axis_a2);
            parse_color_pins(cp, "axis_a3", &tc->pins.cross_ped.axis_a3);
            const cJSON *ped = cJSON_GetObjectItemCaseSensitive(cp, "ped");
            if (cJSON_IsObject(ped)) {
                tc->pins.cross_ped.ped_red   = (uint8_t)json_get_int(ped, "red", 9);
                tc->pins.cross_ped.ped_green = (uint8_t)json_get_int(ped, "green", 10);
            }
        }
    }

    const cJSON *ph = cJSON_GetObjectItemCaseSensitive(t, "phases_ms");
    if (cJSON_IsObject(ph)) {
        tc->phases_ms.red          = (uint32_t)json_get_int(ph, "red", 8000);
        tc->phases_ms.red_yellow   = (uint32_t)json_get_int(ph, "red_yellow", 2000);
        tc->phases_ms.green        = (uint32_t)json_get_int(ph, "green", 10000);
        tc->phases_ms.green_blink  = (uint32_t)json_get_int(ph, "green_blink", 3000);
        tc->phases_ms.yellow       = (uint32_t)json_get_int(ph, "yellow", 2000);
        tc->phases_ms.yellow_blink = (uint32_t)json_get_int(ph, "yellow_blink", 2000);
        tc->phases_ms.ped_green    = (uint32_t)json_get_int(ph, "ped_green", 10000);
        tc->phases_ms.ped_blink    = (uint32_t)json_get_int(ph, "ped_blink", 4000);
        tc->phases_ms.fault_blink_period =
            (uint32_t)json_get_int(ph, "fault_blink_period", 500);
        tc->phases_ms.phase_blink_period =
            (uint32_t)json_get_int(ph, "phase_blink_period", 500);
    }

    const cJSON *f = cJSON_GetObjectItemCaseSensitive(t, "fault");
    if (cJSON_IsObject(f)) {
        tc->fault.imin_mv   = json_get_int(f, "imin_mv", 10);
        tc->fault.vin_on_min_v  = (float)json_get_double(f, "vin_on_min_v", 5.0);
        tc->fault.vin_off_min_v = (float)json_get_double(f, "vin_off_min_v", 5.0);
        tc->fault.phase_grace_ms = (uint32_t)json_get_int(f, "phase_grace_ms", 2500);
        tc->fault.recovery_ms    = (uint32_t)json_get_int(f, "recovery_ms", 2000);
        tc->fault.vin_off_confirmations = (uint8_t)json_get_int(f, "vin_off_confirmations", 3);
    }

    parse_transitions(cJSON_GetObjectItemCaseSensitive(t, "transitions"), tc);
    parse_stages_dd(cJSON_GetObjectItemCaseSensitive(t, "stages_dd"), tc);

    const cJSON *seq = cJSON_GetObjectItemCaseSensitive(t, "sequence");
    if (cJSON_IsArray(seq)) {
        tc->sequence_count = 0;
        const cJSON *si = NULL;
        cJSON_ArrayForEach(si, seq) {
            if (tc->sequence_count < MAX_SEQUENCE && cJSON_IsNumber(si))
                tc->sequence[tc->sequence_count++] = (uint8_t)si->valueint;
        }
    }

    if (tc->sequence_count > 0 && tc->stage_count > 0) {
        for (int i = 0; i < tc->sequence_count; i++) {
            bool found = false;
            for (int j = 0; j < tc->stage_count; j++) {
                if (tc->stages[j].id == tc->sequence[i]) { found = true; break; }
            }
            if (!found) {
                ESP_LOGE(TAG, "sequence[%d]=%u not in stages_dd (count=%u)!",
                         i, tc->sequence[i], tc->stage_count);
            }
        }
    }

    if (tc->hold.enabled) {
        if (tc->hold.stage_count == 0) {
            ESP_LOGW(TAG, "hold.enabled=true but stage_ids empty → disable");
            tc->hold.enabled = false;
        } else {
            for (int i = 0; i < tc->hold.stage_count; i++) {
                bool found = false;
                for (int j = 0; j < tc->stage_count; j++) {
                    if (tc->stages[j].id == tc->hold.stage_ids[i]) { found = true; break; }
                }
                if (!found) {
                    ESP_LOGW(TAG, "hold.stage_ids[%d]=%u not in stages_dd",
                             i, tc->hold.stage_ids[i]);
                }
            }
        }
    }
    if (tc->pedestrian.insert_count > 0) {
        for (int i = 0; i < tc->pedestrian.insert_count; i++) {
            bool found = false;
            for (int j = 0; j < tc->stage_count; j++) {
                if (tc->stages[j].id == tc->pedestrian.insert_sequence[i]) {
                    found = true; break;
                }
            }
            if (!found) {
                ESP_LOGW(TAG, "insert_sequence[%d]=%u not in stages_dd",
                         i, tc->pedestrian.insert_sequence[i]);
            }
        }
    }
}

static void parse_signal_groups(const cJSON *sg) { (void)sg; }
static void parse_igm(const cJSON *arr)            { (void)arr; }
static void parse_stages_ref(const cJSON *arr)     { (void)arr; }
static void parse_detectors(const cJSON *arr)      { (void)arr; }
static void parse_plans(const cJSON *arr)          { (void)arr; }
static void parse_schedule(const cJSON *s)         { (void)s; }
static void parse_control_sources(const cJSON *arr) { (void)arr; }
static void parse_modules(const cJSON *m)          { (void)m; }

/* ============================================================
   config_load_json (FULL)
   === FIX STACK-OVERFLOW: static tmp + mutex ===
   ============================================================ */
bool config_load_json(const char *json, size_t len)
{
    if (!json || len == 0) {
        ESP_LOGE(TAG, "config_load_json: empty input");
        return false;
    }

    size_t free_heap  = esp_get_free_heap_size();
    size_t min_heap   = esp_get_minimum_free_heap_size();
    size_t largest    = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    ESP_LOGI(TAG, "config_load_json: len=%u, free=%u, min=%u, largest=%u",
             (unsigned)len, (unsigned)free_heap,
             (unsigned)min_heap, (unsigned)largest);

    if (largest < CFG_MIN_HEAP_FOR_PARSE) {
        ESP_LOGE(TAG, "config_load_json: not enough continuous heap "
                      "(largest=%u < %u) — parse skipped",
                 (unsigned)largest, (unsigned)CFG_MIN_HEAP_FOR_PARSE);
        return false;
    }

    cJSON *root = cJSON_ParseWithLength(json, len);
    if (!root) {
        const char *err = cJSON_GetErrorPtr();
        size_t off = (err && err >= json) ? (size_t)(err - json) : 0;
        ESP_LOGE(TAG, "JSON parse error at offset %u / %u",
                 (unsigned)off, (unsigned)len);
        if (err) ESP_LOGE(TAG, "JSON error near: %.60s", err);
        return false;
    }

    ESP_LOGI(TAG, "Parsing JSON config (%u bytes)", (unsigned)len);

    /* === FIX STACK-OVERFLOW: static tmp + mutex === */
    if (!cfg_load_lock()) {
        ESP_LOGE(TAG, "config_load_json: mutex timeout");
        cJSON_Delete(root);
        return false;
    }
    app_config_t *tmp = &s_cfg_load_tmp;
    set_defaults_to(tmp);
    tmp->role = g_config.role;
    /* === END FIX STACK-OVERFLOW === */

    const cJSON *meta = cJSON_GetObjectItemCaseSensitive(root, "meta");
    if (cJSON_IsObject(meta)) {
        tmp->version = (float)json_get_double(meta, "version", 1.0);
        tmp->id = (uint32_t)json_get_int(meta, "id", 0);
        json_get_str(meta, "name", tmp->name, sizeof(tmp->name));
    }

    const cJSON *en = cJSON_GetObjectItemCaseSensitive(root, "espnow");
    if (cJSON_IsObject(en)) {
        tmp->espnow.enabled = json_get_bool(en, "enabled", false);
        tmp->espnow.channel = (uint8_t)json_get_int(en, "channel", 6);
        tmp->espnow.status_period_ms =
            (uint16_t)json_get_int(en, "status_period_ms", 1000);
        const cJSON *mac = cJSON_GetObjectItemCaseSensitive(en, "slave_mac");
        if (cJSON_IsString(mac) && mac->valuestring) {
            unsigned int b[6];
            if (sscanf(mac->valuestring, "%02x:%02x:%02x:%02x:%02x:%02x",
                       &b[0],&b[1],&b[2],&b[3],&b[4],&b[5]) == 6) {
                for (int i = 0; i < 6; i++)
                    tmp->espnow.slave_mac[i] = (uint8_t)b[i];
            }
        }
    }

    const cJSON *wifi = cJSON_GetObjectItemCaseSensitive(root, "wifi");
    if (cJSON_IsObject(wifi)) {
        json_get_str(wifi, "ssid", tmp->wifi_ssid, sizeof(tmp->wifi_ssid));
        json_get_str(wifi, "password", tmp->wifi_password, sizeof(tmp->wifi_password));
    }
    json_get_str(root, "timezone", tmp->timezone, sizeof(tmp->timezone));

    const cJSON *mqtt = cJSON_GetObjectItemCaseSensitive(root, "mqtt");
    if (cJSON_IsObject(mqtt)) {
        tmp->mqtt_enabled = json_get_bool(mqtt, "enabled", true);
        json_get_str(mqtt, "uri", tmp->mqtt_uri, sizeof(tmp->mqtt_uri));
        tmp->mqtt_port = (uint16_t)json_get_int(mqtt, "port", 13257);
        json_get_str(mqtt, "user", tmp->mqtt_user, sizeof(tmp->mqtt_user));
        json_get_str(mqtt, "password", tmp->mqtt_password, sizeof(tmp->mqtt_password));
        tmp->mqtt_timepause_min = (uint16_t)json_get_int(mqtt, "timepause_min", 5);
    }

    parse_traffic(cJSON_GetObjectItemCaseSensitive(root, "traffic"), &tmp->traffic);
    parse_signal_groups(cJSON_GetObjectItemCaseSensitive(root, "signalGroups"));
    parse_igm(cJSON_GetObjectItemCaseSensitive(root, "IGM"));
    parse_stages_ref(cJSON_GetObjectItemCaseSensitive(root, "stages"));
    parse_detectors(cJSON_GetObjectItemCaseSensitive(root, "detectors"));
    parse_plans(cJSON_GetObjectItemCaseSensitive(root, "plans"));
    parse_schedule(cJSON_GetObjectItemCaseSensitive(root, "schedule"));
    parse_control_sources(cJSON_GetObjectItemCaseSensitive(root, "controlSources"));
    parse_modules(cJSON_GetObjectItemCaseSensitive(root, "modules"));

    const cJSON *sr = cJSON_GetObjectItemCaseSensitive(root, "shift_register");
    if (cJSON_IsObject(sr)) {
        const cJSON *outs = cJSON_GetObjectItemCaseSensitive(sr, "outputs");
        if (cJSON_IsArray(outs)) {
            for (int i = 0; i < MAX_OUTPUTS; i++)
                tmp->sr.outputs[i].active = false;
            const cJSON *o = NULL;
            cJSON_ArrayForEach(o, outs) {
                int ch = json_get_int(o, "ch", -1);
                if (ch >= 0 && ch < MAX_OUTPUTS) {
                    tmp->sr.outputs[ch].ch = (uint8_t)ch;
                    json_get_str(o, "name", tmp->sr.outputs[ch].name,
                                 sizeof(tmp->sr.outputs[ch].name));
                    tmp->sr.outputs[ch].active = json_get_bool(o, "active", true);
                }
            }
        }
    }

    cJSON_Delete(root);

    /* === FIX: валидация перед коммитом === */
    if (tmp->traffic.enabled && tmp->traffic.stage_count == 0) {
        ESP_LOGE(TAG, "load_json: traffic.enabled=true but no stages_dd "
                      "→ reject (keep current config)");
        cfg_load_unlock();
        return false;
    }
    if (tmp->traffic.stage_count > 0 && tmp->traffic.sequence_count == 0) {
        ESP_LOGW(TAG, "load_json: stages_dd present but no sequence → auto-fill");
        for (int i = 0; i < tmp->traffic.stage_count && i < MAX_SEQUENCE; i++)
            tmp->traffic.sequence[i] = tmp->traffic.stages[i].id;
        tmp->traffic.sequence_count = tmp->traffic.stage_count;
    }
    /* === END FIX === */

    g_config = *tmp;
    config_save();

    ESP_LOGI(TAG, "JSON loaded (version=%.2f, role=%s, traffic stages=%u seq=%u, "
                  "ped_mode=%u, hold=%d/%u, insert_seq=%u, insert_red_ms=%lu)",
             g_config.version,
             g_config.role == DEVICE_ROLE_SLAVE ? "slave" : "master",
             g_config.traffic.stage_count,
             g_config.traffic.sequence_count,
             g_config.traffic.pedestrian.request_mode,
             g_config.traffic.hold.enabled,
             g_config.traffic.hold.stage_count,
             g_config.traffic.pedestrian.insert_count,
             (unsigned long)g_config.traffic.pedestrian.insert_red_ms);

    cfg_load_unlock();
    return true;
}

/* ============================================================ */
static const char *duration_kind_to_str(uint8_t kind)
{
    switch (kind) {
        case DD_DUR_RED:          return "red";
        case DD_DUR_RED_YELLOW:   return "red_yellow";
        case DD_DUR_GREEN:        return "green";
        case DD_DUR_GREEN_BLINK:  return "green_blink";
        case DD_DUR_YELLOW:       return "yellow";
        case DD_DUR_YELLOW_BLINK: return "yellow_blink";
        case DD_DUR_PED_GREEN:    return "ped_green";
        case DD_DUR_PED_BLINK:    return "ped_blink";
        default:                  return "explicit";
    }
}

static const char *ped_request_mode_to_str(uint8_t m)
{
    switch (m) {
        case PED_REQUEST_MODE_INSERT:     return "insert";
        case PED_REQUEST_MODE_MIN_RECALL: return "min_recall";
        default:                          return "stage";
    }
}

static const char *hold_on_ped_to_str(uint8_t m)
{
    switch (m) {
        case HOLD_PED_MIN_RECALL: return "min_recall";
        case HOLD_PED_IGNORE:     return "ignore";
        default:                  return "insert";
    }
}

/* ============================================================
   config_export_json (FULL) — выгрузка g_config в JSON (кэш).
   ============================================================ */
bool config_export_json(char *buf, size_t *len)
{
    if (!buf || !len || *len < 256) return false;

    if (s_cfg_json_cache) {
        if (s_cfg_json_len + 1 > *len) return false;
        memcpy(buf, s_cfg_json_cache, s_cfg_json_len);
        buf[s_cfg_json_len] = '\0';
        *len = s_cfg_json_len;
        return true;
    }

    cJSON *root = cJSON_CreateObject();
    if (!root) return false;

    cJSON *meta = cJSON_AddObjectToObject(root, "meta");
    if (meta) {
        cJSON_AddNumberToObject(meta, "version", g_config.version);
        cJSON_AddNumberToObject(meta, "id", g_config.id);
        cJSON_AddStringToObject(meta, "name", g_config.name);
    }

    cJSON *en = cJSON_AddObjectToObject(root, "espnow");
    if (en) {
        cJSON_AddBoolToObject(en, "enabled", g_config.espnow.enabled);
        cJSON_AddNumberToObject(en, "channel", g_config.espnow.channel);
        cJSON_AddNumberToObject(en, "status_period_ms",
                                g_config.espnow.status_period_ms);
        char mac[18];
        snprintf(mac, sizeof(mac), "%02x:%02x:%02x:%02x:%02x:%02x",
                 g_config.espnow.slave_mac[0], g_config.espnow.slave_mac[1],
                 g_config.espnow.slave_mac[2], g_config.espnow.slave_mac[3],
                 g_config.espnow.slave_mac[4], g_config.espnow.slave_mac[5]);
        cJSON_AddStringToObject(en, "slave_mac", mac);
    }

    cJSON *wifi = cJSON_AddObjectToObject(root, "wifi");
    if (wifi) {
        cJSON_AddStringToObject(wifi, "ssid", g_config.wifi_ssid);
        cJSON_AddStringToObject(wifi, "password", g_config.wifi_password);
    }
    cJSON_AddStringToObject(root, "timezone", g_config.timezone);

    cJSON *mqtt = cJSON_AddObjectToObject(root, "mqtt");
    if (mqtt) {
        cJSON_AddBoolToObject(mqtt, "enabled", g_config.mqtt_enabled);
        cJSON_AddStringToObject(mqtt, "uri", g_config.mqtt_uri);
        cJSON_AddNumberToObject(mqtt, "port", g_config.mqtt_port);
        cJSON_AddStringToObject(mqtt, "user", g_config.mqtt_user);
        cJSON_AddStringToObject(mqtt, "password", g_config.mqtt_password);
        cJSON_AddNumberToObject(mqtt, "timepause_min", g_config.mqtt_timepause_min);
    }

    cJSON *t = cJSON_AddObjectToObject(root, "traffic");
    if (t) {
        cJSON_AddNumberToObject(t, "mode", g_config.traffic.mode);
        cJSON_AddBoolToObject(t, "enabled", g_config.traffic.enabled);
        cJSON_AddBoolToObject(t, "auto_start", g_config.traffic.auto_start);

        cJSON *hold = cJSON_AddObjectToObject(t, "hold");
        if (hold) {
            cJSON_AddBoolToObject(hold, "enabled", g_config.traffic.hold.enabled);
            cJSON_AddNumberToObject(hold, "max_hold_ms",
                                    g_config.traffic.hold.max_hold_ms);
            cJSON_AddStringToObject(hold, "on_ped_request",
                                    hold_on_ped_to_str(
                                        g_config.traffic.hold.on_ped_request));
            cJSON *ids = cJSON_AddArrayToObject(hold, "stage_ids");
            for (int i = 0; i < g_config.traffic.hold.stage_count; i++)
                cJSON_AddItemToArray(ids,
                    cJSON_CreateNumber(g_config.traffic.hold.stage_ids[i]));
        }

        cJSON *ped = cJSON_AddObjectToObject(t, "pedestrian");
        if (ped) {
            cJSON_AddBoolToObject(ped, "button_enabled",
                                  g_config.traffic.pedestrian.button_enabled);
            cJSON_AddStringToObject(ped, "request_mode",
                                    ped_request_mode_to_str(
                                        g_config.traffic.pedestrian.request_mode));
            cJSON_AddNumberToObject(ped, "insert_timeout_ms",
                                    g_config.traffic.pedestrian.insert_timeout_ms);
            cJSON_AddNumberToObject(ped, "min_recall_s",
                                    g_config.traffic.pedestrian.min_recall_s);
            cJSON_AddNumberToObject(ped, "insert_lead_ms",
                                    g_config.traffic.pedestrian.insert_lead_ms);
            /* === FIX INSERT-RED-SHORT === */
            cJSON_AddNumberToObject(ped, "insert_red_ms",
                                    g_config.traffic.pedestrian.insert_red_ms);
            /* === END FIX INSERT-RED-SHORT === */
            cJSON_AddBoolToObject(ped, "return_to_hold",
                                  g_config.traffic.pedestrian.return_to_hold);
            cJSON *iseq = cJSON_AddArrayToObject(ped, "insert_sequence");
            for (int i = 0; i < g_config.traffic.pedestrian.insert_count; i++)
                cJSON_AddItemToArray(iseq,
                    cJSON_CreateNumber(
                        g_config.traffic.pedestrian.insert_sequence[i]));
        }

        cJSON *pins = cJSON_AddObjectToObject(t, "pins");
        if (pins) {
            cJSON *single = cJSON_AddObjectToObject(pins, "single");
            if (single) {
                cJSON_AddNumberToObject(single, "red",    g_config.traffic.pins.single.red);
                cJSON_AddNumberToObject(single, "yellow", g_config.traffic.pins.single.yellow);
                cJSON_AddNumberToObject(single, "green",  g_config.traffic.pins.single.green);
            }
            cJSON *cross = cJSON_AddObjectToObject(pins, "cross");
            if (cross) {
                const struct { const char *n; traffic_color_pins_t *p; } arr[] = {
                    {"a_north", &g_config.traffic.pins.cross.a_north},
                    {"a_south", &g_config.traffic.pins.cross.a_south},
                    {"b_west",  &g_config.traffic.pins.cross.b_west},
                    {"b_east",  &g_config.traffic.pins.cross.b_east},
                };
                for (size_t i = 0; i < sizeof(arr)/sizeof(arr[0]); i++) {
                    cJSON *o = cJSON_AddObjectToObject(cross, arr[i].n);
                    if (o) {
                        cJSON_AddNumberToObject(o, "red",    arr[i].p->red);
                        cJSON_AddNumberToObject(o, "yellow", arr[i].p->yellow);
                        cJSON_AddNumberToObject(o, "green",  arr[i].p->green);
                    }
                }
            }
            cJSON *cp = cJSON_AddObjectToObject(pins, "cross_ped");
            if (cp) {
                const struct { const char *n; traffic_color_pins_t *p; } arr[] = {
                    {"axis_a",  &g_config.traffic.pins.cross_ped.axis_a},
                    {"axis_a2", &g_config.traffic.pins.cross_ped.axis_a2},
                    {"axis_a3", &g_config.traffic.pins.cross_ped.axis_a3},
                };
                for (size_t i = 0; i < sizeof(arr)/sizeof(arr[0]); i++) {
                    cJSON *o = cJSON_AddObjectToObject(cp, arr[i].n);
                    if (o) {
                        cJSON_AddNumberToObject(o, "red",    arr[i].p->red);
                        cJSON_AddNumberToObject(o, "yellow", arr[i].p->yellow);
                        cJSON_AddNumberToObject(o, "green",  arr[i].p->green);
                    }
                }
                cJSON *ped2 = cJSON_AddObjectToObject(cp, "ped");
                if (ped2) {
                    cJSON_AddNumberToObject(ped2, "red",   g_config.traffic.pins.cross_ped.ped_red);
                    cJSON_AddNumberToObject(ped2, "green", g_config.traffic.pins.cross_ped.ped_green);
                }
            }
        }

        cJSON *ph = cJSON_AddObjectToObject(t, "phases_ms");
        if (ph) {
            cJSON_AddNumberToObject(ph, "red",          g_config.traffic.phases_ms.red);
            cJSON_AddNumberToObject(ph, "red_yellow",   g_config.traffic.phases_ms.red_yellow);
            cJSON_AddNumberToObject(ph, "green",        g_config.traffic.phases_ms.green);
            cJSON_AddNumberToObject(ph, "green_blink",  g_config.traffic.phases_ms.green_blink);
            cJSON_AddNumberToObject(ph, "yellow",       g_config.traffic.phases_ms.yellow);
            cJSON_AddNumberToObject(ph, "yellow_blink", g_config.traffic.phases_ms.yellow_blink);
            cJSON_AddNumberToObject(ph, "ped_green",    g_config.traffic.phases_ms.ped_green);
            cJSON_AddNumberToObject(ph, "ped_blink",    g_config.traffic.phases_ms.ped_blink);
            cJSON_AddNumberToObject(ph, "fault_blink_period", g_config.traffic.phases_ms.fault_blink_period);
            cJSON_AddNumberToObject(ph, "phase_blink_period", g_config.traffic.phases_ms.phase_blink_period);
        }

        cJSON *f = cJSON_AddObjectToObject(t, "fault");
        if (f) {
            cJSON_AddNumberToObject(f, "imin_mv",   g_config.traffic.fault.imin_mv);
            cJSON_AddNumberToObject(f, "vin_on_min_v",  g_config.traffic.fault.vin_on_min_v);
            cJSON_AddNumberToObject(f, "vin_off_min_v", g_config.traffic.fault.vin_off_min_v);
            cJSON_AddNumberToObject(f, "phase_grace_ms", g_config.traffic.fault.phase_grace_ms);
            cJSON_AddNumberToObject(f, "recovery_ms",    g_config.traffic.fault.recovery_ms);
            cJSON_AddNumberToObject(f, "vin_off_confirmations", g_config.traffic.fault.vin_off_confirmations);
        }

        cJSON *stages = cJSON_AddArrayToObject(t, "stages_dd");
        if (stages) {
            for (int i = 0; i < g_config.traffic.stage_count; i++) {
                const traffic_stage_dd_t *s = &g_config.traffic.stages[i];
                cJSON *st = cJSON_CreateObject();
                if (!st) continue;
                cJSON_AddNumberToObject(st, "id", s->id);
                cJSON_AddStringToObject(st, "name", s->name);
                cJSON_AddStringToObject(st, "duration_kind",
                                        duration_kind_to_str(s->duration_kind));
                cJSON_AddNumberToObject(st, "duration_ms", s->duration_ms);
                cJSON *g = cJSON_AddArrayToObject(st, "groups");
                for (int k = 0; k < s->group_count; k++)
                    cJSON_AddItemToArray(g, cJSON_CreateNumber(s->groups[k]));
                cJSON *o = cJSON_AddArrayToObject(st, "outputs_on");
                for (int k = 0; k < s->on_count; k++)
                    cJSON_AddItemToArray(o, cJSON_CreateNumber(s->outputs_on[k]));
                cJSON *b = cJSON_AddArrayToObject(st, "outputs_blink");
                for (int k = 0; k < s->blink_count; k++)
                    cJSON_AddItemToArray(b, cJSON_CreateNumber(s->outputs_blink[k]));
                cJSON_AddItemToArray(stages, st);
            }
        }

        cJSON *seq = cJSON_AddArrayToObject(t, "sequence");
        if (seq) {
            for (int i = 0; i < g_config.traffic.sequence_count; i++)
                cJSON_AddItemToArray(seq, cJSON_CreateNumber(g_config.traffic.sequence[i]));
        }

        cJSON *tr = cJSON_AddArrayToObject(t, "transitions");
        if (tr) {
            for (int i = 0; i < g_config.traffic.transition_count; i++) {
                const traffic_transition_t *tt = &g_config.traffic.transitions[i];
                cJSON *o = cJSON_CreateObject();
                if (!o) continue;
                cJSON_AddNumberToObject(o, "clearing", tt->clearing);
                cJSON_AddNumberToObject(o, "entering", tt->entering);
                cJSON_AddNumberToObject(o, "time", tt->time_cs / 100.0);
                cJSON_AddItemToArray(tr, o);
            }
        }
    }

    cJSON *sr = cJSON_AddObjectToObject(root, "shift_register");
    if (sr) {
        cJSON *outs = cJSON_AddArrayToObject(sr, "outputs");
        if (outs) {
            for (int i = 0; i < g_config.sr.count; i++) {
                cJSON *o = cJSON_CreateObject();
                if (!o) continue;
                cJSON_AddNumberToObject(o, "ch", g_config.sr.outputs[i].ch);
                cJSON_AddStringToObject(o, "name", g_config.sr.outputs[i].name);
                cJSON_AddBoolToObject(o, "active", g_config.sr.outputs[i].active);
                cJSON_AddItemToArray(outs, o);
            }
        }
    }

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!out) return false;

    size_t n = strlen(out);
    if (n + 1 > *len) { free(out); return false; }
    memcpy(buf, out, n);
    buf[n] = '\0';
    *len = n;

    s_cfg_json_cache = strdup(out);
    s_cfg_json_len = n;
    free(out);

    ESP_LOGI(TAG, "config_export_json: OK (%u bytes, cached)", (unsigned)n);
    return true;
}

/* ============================================================
   config_export_lora_json — короткий JSON для LoRa.
   === FIX INSERT-RED-SHORT: в "ped" добавлен insert_red_ms
       на позицию [6]. insert_count сдвинут на [7], sequence с [8].
   ============================================================ */
bool config_export_lora_json(char *buf, size_t *len)
{
    if (!buf || !len || *len < 256) return false;

    cJSON *root = cJSON_CreateObject();
    if (!root) return false;

    cJSON_AddNumberToObject(root, "v", g_config.version);
    cJSON_AddStringToObject(root, "tz", g_config.timezone);

    cJSON *t = cJSON_AddObjectToObject(root, "t");
    if (t) {
        cJSON_AddNumberToObject(t, "mode", g_config.traffic.mode);
        cJSON_AddBoolToObject(t, "en", g_config.traffic.enabled);
        cJSON_AddBoolToObject(t, "auto", g_config.traffic.auto_start);

        cJSON *pins = cJSON_AddObjectToObject(t, "pins");
        if (pins) {
            cJSON *single = cJSON_AddArrayToObject(pins, "single");
            cJSON_AddItemToArray(single, cJSON_CreateNumber(g_config.traffic.pins.single.red));
            cJSON_AddItemToArray(single, cJSON_CreateNumber(g_config.traffic.pins.single.yellow));
            cJSON_AddItemToArray(single, cJSON_CreateNumber(g_config.traffic.pins.single.green));

            cJSON *cross = cJSON_AddArrayToObject(pins, "cross");
            const traffic_color_pins_t cross_arr[] = {
                g_config.traffic.pins.cross.a_north,
                g_config.traffic.pins.cross.a_south,
                g_config.traffic.pins.cross.b_west,
                g_config.traffic.pins.cross.b_east,
            };
            for (int i = 0; i < 4; i++) {
                cJSON *c = cJSON_CreateArray();
                cJSON_AddItemToArray(c, cJSON_CreateNumber(cross_arr[i].red));
                cJSON_AddItemToArray(c, cJSON_CreateNumber(cross_arr[i].yellow));
                cJSON_AddItemToArray(c, cJSON_CreateNumber(cross_arr[i].green));
                cJSON_AddItemToArray(cross, c);
            }

            cJSON *cp = cJSON_AddArrayToObject(pins, "cross_ped");
            const traffic_color_pins_t cp_arr[] = {
                g_config.traffic.pins.cross_ped.axis_a,
                g_config.traffic.pins.cross_ped.axis_a2,
                g_config.traffic.pins.cross_ped.axis_a3,
            };
            for (int i = 0; i < 3; i++) {
                cJSON *c = cJSON_CreateArray();
                cJSON_AddItemToArray(c, cJSON_CreateNumber(cp_arr[i].red));
                cJSON_AddItemToArray(c, cJSON_CreateNumber(cp_arr[i].yellow));
                cJSON_AddItemToArray(c, cJSON_CreateNumber(cp_arr[i].green));
                cJSON_AddItemToArray(cp, c);
            }
            cJSON_AddItemToArray(cp, cJSON_CreateNumber(g_config.traffic.pins.cross_ped.ped_red));
            cJSON_AddItemToArray(cp, cJSON_CreateNumber(g_config.traffic.pins.cross_ped.ped_green));
        }

        cJSON *ph = cJSON_AddArrayToObject(t, "ph");
        if (ph) {
            cJSON_AddItemToArray(ph, cJSON_CreateNumber(g_config.traffic.phases_ms.red));
            cJSON_AddItemToArray(ph, cJSON_CreateNumber(g_config.traffic.phases_ms.red_yellow));
            cJSON_AddItemToArray(ph, cJSON_CreateNumber(g_config.traffic.phases_ms.green));
            cJSON_AddItemToArray(ph, cJSON_CreateNumber(g_config.traffic.phases_ms.green_blink));
            cJSON_AddItemToArray(ph, cJSON_CreateNumber(g_config.traffic.phases_ms.yellow));
            cJSON_AddItemToArray(ph, cJSON_CreateNumber(g_config.traffic.phases_ms.yellow_blink));
            cJSON_AddItemToArray(ph, cJSON_CreateNumber(g_config.traffic.phases_ms.ped_green));
            cJSON_AddItemToArray(ph, cJSON_CreateNumber(g_config.traffic.phases_ms.ped_blink));
            cJSON_AddItemToArray(ph, cJSON_CreateNumber(g_config.traffic.phases_ms.fault_blink_period));
            cJSON_AddItemToArray(ph, cJSON_CreateNumber(g_config.traffic.phases_ms.phase_blink_period));
        }

        cJSON *f = cJSON_AddArrayToObject(t, "f");
        if (f) {
            cJSON_AddItemToArray(f, cJSON_CreateNumber(g_config.traffic.fault.imin_mv));
            cJSON_AddItemToArray(f, cJSON_CreateNumber(g_config.traffic.fault.vin_on_min_v));
            cJSON_AddItemToArray(f, cJSON_CreateNumber(g_config.traffic.fault.vin_off_min_v));
            cJSON_AddItemToArray(f, cJSON_CreateNumber(g_config.traffic.fault.phase_grace_ms));
            cJSON_AddItemToArray(f, cJSON_CreateNumber(g_config.traffic.fault.recovery_ms));
            cJSON_AddItemToArray(f, cJSON_CreateNumber(g_config.traffic.fault.vin_off_confirmations));
        }

        cJSON *hold = cJSON_AddObjectToObject(t, "hold");
        if (hold) {
            cJSON_AddBoolToObject(hold, "en", g_config.traffic.hold.enabled);
            cJSON_AddNumberToObject(hold, "max_ms", g_config.traffic.hold.max_hold_ms);
            cJSON_AddNumberToObject(hold, "on_ped",
                                    g_config.traffic.hold.on_ped_request);
            cJSON *ids = cJSON_AddArrayToObject(hold, "ids");
            for (int i = 0; i < g_config.traffic.hold.stage_count; i++)
                cJSON_AddItemToArray(ids,
                    cJSON_CreateNumber(g_config.traffic.hold.stage_ids[i]));
        }

        /* === FIX INSERT-RED-SHORT: порядок элементов в "ped":
           [0]=button_enabled   [1]=request_mode
           [2]=insert_timeout   [3]=min_recall_s
           [4]=insert_lead_ms   [5]=return_to_hold
           [6]=insert_red_ms    [7]=insert_count
           [8..]=insert_sequence[]
        === */
        cJSON *ped = cJSON_AddArrayToObject(t, "ped");
        if (ped) {
            cJSON_AddItemToArray(ped, cJSON_CreateBool(g_config.traffic.pedestrian.button_enabled));
            cJSON_AddItemToArray(ped, cJSON_CreateNumber(g_config.traffic.pedestrian.request_mode));
            cJSON_AddItemToArray(ped, cJSON_CreateNumber(g_config.traffic.pedestrian.insert_timeout_ms));
            cJSON_AddItemToArray(ped, cJSON_CreateNumber(g_config.traffic.pedestrian.min_recall_s));
            cJSON_AddItemToArray(ped, cJSON_CreateNumber(g_config.traffic.pedestrian.insert_lead_ms));
            cJSON_AddItemToArray(ped, cJSON_CreateBool(g_config.traffic.pedestrian.return_to_hold));
            cJSON_AddItemToArray(ped, cJSON_CreateNumber(g_config.traffic.pedestrian.insert_red_ms));
            cJSON_AddItemToArray(ped, cJSON_CreateNumber(g_config.traffic.pedestrian.insert_count));
            for (int i = 0; i < g_config.traffic.pedestrian.insert_count; i++)
                cJSON_AddItemToArray(ped,
                    cJSON_CreateNumber(g_config.traffic.pedestrian.insert_sequence[i]));
        }
        /* === END FIX INSERT-RED-SHORT === */

        cJSON *st = cJSON_AddArrayToObject(t, "st");
        if (st) {
            for (int i = 0; i < g_config.traffic.stage_count; i++) {
                const traffic_stage_dd_t *s = &g_config.traffic.stages[i];
                cJSON *item = cJSON_CreateArray();
                cJSON_AddItemToArray(item, cJSON_CreateNumber(s->id));
                cJSON_AddItemToArray(item, cJSON_CreateString(s->name));
                cJSON *g = cJSON_CreateArray();
                for (int k = 0; k < s->group_count; k++)
                    cJSON_AddItemToArray(g, cJSON_CreateNumber(s->groups[k]));
                cJSON_AddItemToArray(item, g);
                cJSON_AddItemToArray(item, cJSON_CreateNumber(s->duration_kind));
                cJSON_AddItemToArray(item, cJSON_CreateNumber(s->duration_ms));
                cJSON *on = cJSON_CreateArray();
                for (int k = 0; k < s->on_count; k++)
                    cJSON_AddItemToArray(on, cJSON_CreateNumber(s->outputs_on[k]));
                cJSON_AddItemToArray(item, on);
                cJSON *blink = cJSON_CreateArray();
                for (int k = 0; k < s->blink_count; k++)
                    cJSON_AddItemToArray(blink, cJSON_CreateNumber(s->outputs_blink[k]));
                cJSON_AddItemToArray(item, blink);
                cJSON_AddItemToArray(st, item);
            }
        }

        cJSON *seq = cJSON_AddArrayToObject(t, "seq");
        if (seq) {
            for (int i = 0; i < g_config.traffic.sequence_count; i++)
                cJSON_AddItemToArray(seq, cJSON_CreateNumber(g_config.traffic.sequence[i]));
        }

        cJSON *tr = cJSON_AddArrayToObject(t, "tr");
        if (tr) {
            for (int i = 0; i < g_config.traffic.transition_count; i++) {
                const traffic_transition_t *tt = &g_config.traffic.transitions[i];
                cJSON *item = cJSON_CreateArray();
                cJSON_AddItemToArray(item, cJSON_CreateNumber(tt->clearing));
                cJSON_AddItemToArray(item, cJSON_CreateNumber(tt->entering));
                cJSON_AddItemToArray(item, cJSON_CreateNumber(tt->time_cs * 10));
                cJSON_AddItemToArray(tr, item);
            }
        }
    }

    cJSON *sr = cJSON_AddArrayToObject(root, "sr");
    if (sr) {
        for (int i = 0; i < g_config.sr.count; i++) {
            if (g_config.sr.outputs[i].active)
                cJSON_AddItemToArray(sr, cJSON_CreateNumber(g_config.sr.outputs[i].ch));
        }
    }

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!out) {
        ESP_LOGE(TAG, "config_export_lora_json: cJSON_PrintUnformatted failed");
        return false;
    }

    size_t n = strlen(out);
    if (n + 1 > *len) {
        ESP_LOGE(TAG, "config_export_lora_json: buffer too small (%u > %u)",
                 (unsigned)(n + 1), (unsigned)*len);
        free(out);
        return false;
    }
    memcpy(buf, out, n);
    buf[n] = '\0';
    *len = n;
    free(out);

    ESP_LOGI(TAG, "config_export_lora_json: OK (%u bytes)", (unsigned)n);
    return true;
}

/* ============================================================
   config_load_lora_json

   === FIX STACK-OVERFLOW: static tmp + mutex (как в config_load_json).
   === FIX INSERT-RED-SHORT: поддержка как нового формата "ped"
       (psz >= 8, insert_red_ms на [6], insert_count на [7],
       sequence с [8]), так и legacy (psz >= 7, без insert_red_ms).
   ============================================================ */
bool config_load_lora_json(const char *json, size_t len)
{
    if (!json || len == 0) {
        ESP_LOGE(TAG, "lora_json: empty input");
        return false;
    }

    cJSON *root = cJSON_ParseWithLength(json, len);
    if (!root) {
        const char *err = cJSON_GetErrorPtr();
        size_t off = (err && err >= json) ? (size_t)(err - json) : 0;
        ESP_LOGE(TAG, "lora_json: parse error at offset %u / %u",
                 (unsigned)off, (unsigned)len);
        return false;
    }

    /* === FIX STACK-OVERFLOW: static tmp + mutex === */
    if (!cfg_load_lock()) {
        ESP_LOGE(TAG, "config_load_lora_json: mutex timeout");
        cJSON_Delete(root);
        return false;
    }
    app_config_t *tmp = &s_cfg_load_tmp;
    set_defaults_to(tmp);
    tmp->role = g_config.role;
    /* === END FIX STACK-OVERFLOW === */

    const cJSON *v = cJSON_GetObjectItemCaseSensitive(root, "v");
    if (cJSON_IsNumber(v)) tmp->version = (float)v->valuedouble;

    const cJSON *tz = cJSON_GetObjectItemCaseSensitive(root, "tz");
    if (cJSON_IsString(tz) && tz->valuestring) {
        strncpy(tmp->timezone, tz->valuestring, sizeof(tmp->timezone) - 1);
        tmp->timezone[sizeof(tmp->timezone) - 1] = '\0';
    }

    const cJSON *t = cJSON_GetObjectItemCaseSensitive(root, "t");
    if (cJSON_IsObject(t)) {
        traffic_config_t *tc = &tmp->traffic;

        tc->mode       = (uint8_t)json_get_int(t, "mode", tc->mode);
        tc->enabled    = json_get_bool(t, "en", tc->enabled);
        tc->auto_start = json_get_bool(t, "auto", tc->auto_start);

        const cJSON *pins = cJSON_GetObjectItemCaseSensitive(t, "pins");
        if (cJSON_IsObject(pins)) {
            const cJSON *single = cJSON_GetObjectItemCaseSensitive(pins, "single");
            if (cJSON_IsArray(single) && cJSON_GetArraySize(single) >= 3) {
                tc->pins.single.red    = (uint8_t)cJSON_GetArrayItem(single, 0)->valueint;
                tc->pins.single.yellow = (uint8_t)cJSON_GetArrayItem(single, 1)->valueint;
                tc->pins.single.green  = (uint8_t)cJSON_GetArrayItem(single, 2)->valueint;
            }
            const cJSON *cross = cJSON_GetObjectItemCaseSensitive(pins, "cross");
            if (cJSON_IsArray(cross) && cJSON_GetArraySize(cross) >= 4) {
                traffic_color_pins_t *dst[] = {
                    &tc->pins.cross.a_north, &tc->pins.cross.a_south,
                    &tc->pins.cross.b_west,  &tc->pins.cross.b_east,
                };
                for (int i = 0; i < 4; i++) {
                    const cJSON *c = cJSON_GetArrayItem(cross, i);
                    if (cJSON_IsArray(c) && cJSON_GetArraySize(c) >= 3) {
                        dst[i]->red    = (uint8_t)cJSON_GetArrayItem(c, 0)->valueint;
                        dst[i]->yellow = (uint8_t)cJSON_GetArrayItem(c, 1)->valueint;
                        dst[i]->green  = (uint8_t)cJSON_GetArrayItem(c, 2)->valueint;
                    }
                }
            }
            const cJSON *cp = cJSON_GetObjectItemCaseSensitive(pins, "cross_ped");
            if (cJSON_IsArray(cp) && cJSON_GetArraySize(cp) >= 5) {
                traffic_color_pins_t *dst[] = {
                    &tc->pins.cross_ped.axis_a,
                    &tc->pins.cross_ped.axis_a2,
                    &tc->pins.cross_ped.axis_a3,
                };
                for (int i = 0; i < 3; i++) {
                    const cJSON *c = cJSON_GetArrayItem(cp, i);
                    if (cJSON_IsArray(c) && cJSON_GetArraySize(c) >= 3) {
                        dst[i]->red    = (uint8_t)cJSON_GetArrayItem(c, 0)->valueint;
                        dst[i]->yellow = (uint8_t)cJSON_GetArrayItem(c, 1)->valueint;
                        dst[i]->green  = (uint8_t)cJSON_GetArrayItem(c, 2)->valueint;
                    }
                }
                tc->pins.cross_ped.ped_red   = (uint8_t)cJSON_GetArrayItem(cp, 3)->valueint;
                tc->pins.cross_ped.ped_green = (uint8_t)cJSON_GetArrayItem(cp, 4)->valueint;
            }
        }

        const cJSON *ph = cJSON_GetObjectItemCaseSensitive(t, "ph");
        if (cJSON_IsArray(ph) && cJSON_GetArraySize(ph) >= 10) {
            tc->phases_ms.red          = (uint32_t)cJSON_GetArrayItem(ph, 0)->valueint;
            tc->phases_ms.red_yellow   = (uint32_t)cJSON_GetArrayItem(ph, 1)->valueint;
            tc->phases_ms.green        = (uint32_t)cJSON_GetArrayItem(ph, 2)->valueint;
            tc->phases_ms.green_blink  = (uint32_t)cJSON_GetArrayItem(ph, 3)->valueint;
            tc->phases_ms.yellow       = (uint32_t)cJSON_GetArrayItem(ph, 4)->valueint;
            tc->phases_ms.yellow_blink = (uint32_t)cJSON_GetArrayItem(ph, 5)->valueint;
            tc->phases_ms.ped_green    = (uint32_t)cJSON_GetArrayItem(ph, 6)->valueint;
            tc->phases_ms.ped_blink    = (uint32_t)cJSON_GetArrayItem(ph, 7)->valueint;
            tc->phases_ms.fault_blink_period = (uint32_t)cJSON_GetArrayItem(ph, 8)->valueint;
            tc->phases_ms.phase_blink_period = (uint32_t)cJSON_GetArrayItem(ph, 9)->valueint;
        }

        const cJSON *f = cJSON_GetObjectItemCaseSensitive(t, "f");
        if (cJSON_IsArray(f) && cJSON_GetArraySize(f) >= 6) {
            tc->fault.imin_mv   = (int)cJSON_GetArrayItem(f, 0)->valueint;
            tc->fault.vin_on_min_v  = (float)cJSON_GetArrayItem(f, 1)->valuedouble;
            tc->fault.vin_off_min_v = (float)cJSON_GetArrayItem(f, 2)->valuedouble;
            tc->fault.phase_grace_ms = (uint32_t)cJSON_GetArrayItem(f, 3)->valueint;
            tc->fault.recovery_ms    = (uint32_t)cJSON_GetArrayItem(f, 4)->valueint;
            tc->fault.vin_off_confirmations = (uint8_t)cJSON_GetArrayItem(f, 5)->valueint;
        }

        const cJSON *hold = cJSON_GetObjectItemCaseSensitive(t, "hold");
        if (cJSON_IsObject(hold)) {
            tc->hold.enabled     = json_get_bool(hold, "en", false);
            tc->hold.max_hold_ms = (uint32_t)json_get_int(hold, "max_ms", 0);
            tc->hold.on_ped_request = (uint8_t)json_get_int(hold, "on_ped",
                                                            HOLD_PED_INSERT);
            if (tc->hold.on_ped_request > HOLD_PED_IGNORE)
                tc->hold.on_ped_request = HOLD_PED_INSERT;
            const cJSON *ids = cJSON_GetObjectItemCaseSensitive(hold, "ids");
            if (cJSON_IsArray(ids)) {
                tc->hold.stage_count = 0;
                const cJSON *it = NULL;
                cJSON_ArrayForEach(it, ids) {
                    if (tc->hold.stage_count < MAX_HOLD_STAGES && cJSON_IsNumber(it))
                        tc->hold.stage_ids[tc->hold.stage_count++] =
                            (uint8_t)it->valueint;
                }
            }
        }

        /* ============================================================
           PED — FIX INSERT-RED-SHORT

           Новый формат (psz >= 8):
             [0]=button_enabled   [1]=request_mode
             [2]=insert_timeout   [3]=min_recall_s
             [4]=insert_lead_ms   [5]=return_to_hold
             [6]=insert_red_ms    [7]=insert_count
             [8..]=insert_sequence[]

           Legacy (psz >= 7, без insert_red_ms):
             [0..3] как выше, [4]=insert_lead_ms, [5]=return_to_hold,
             [6]=insert_count, [7..]=insert_sequence[]
           ============================================================ */
        const cJSON *ped = cJSON_GetObjectItemCaseSensitive(t, "ped");
        if (cJSON_IsArray(ped) && cJSON_GetArraySize(ped) >= 4) {
            tc->pedestrian.button_enabled    = cJSON_IsTrue(cJSON_GetArrayItem(ped, 0));
            tc->pedestrian.request_mode      = (uint8_t)cJSON_GetArrayItem(ped, 1)->valueint;
            tc->pedestrian.insert_timeout_ms = (uint32_t)cJSON_GetArrayItem(ped, 2)->valueint;
            tc->pedestrian.min_recall_s      = (uint32_t)cJSON_GetArrayItem(ped, 3)->valueint;

            int psz = cJSON_GetArraySize(ped);
            if (psz >= 8) {
                /* Новый формат: insert_red_ms присутствует */
                tc->pedestrian.insert_lead_ms = (uint32_t)cJSON_GetArrayItem(ped, 4)->valueint;
                tc->pedestrian.return_to_hold = cJSON_IsTrue(cJSON_GetArrayItem(ped, 5));
                tc->pedestrian.insert_red_ms  = (uint32_t)cJSON_GetArrayItem(ped, 6)->valueint;
                int cnt = cJSON_GetArrayItem(ped, 7)->valueint;
                if (cnt > MAX_INSERT_SEQ) cnt = MAX_INSERT_SEQ;
                if (cnt < 0) cnt = 0;
                tc->pedestrian.insert_count = 0;
                for (int k = 0; k < cnt && (8 + k) < psz; k++) {
                    const cJSON *it = cJSON_GetArrayItem(ped, 8 + k);
                    if (cJSON_IsNumber(it))
                        tc->pedestrian.insert_sequence[tc->pedestrian.insert_count++] =
                            (uint8_t)it->valueint;
                }
            } else if (psz >= 7) {
                /* Legacy без insert_red_ms */
                tc->pedestrian.insert_lead_ms = (uint32_t)cJSON_GetArrayItem(ped, 4)->valueint;
                tc->pedestrian.return_to_hold = cJSON_IsTrue(cJSON_GetArrayItem(ped, 5));
                tc->pedestrian.insert_red_ms  = 0;  /* default */
                int cnt = cJSON_GetArrayItem(ped, 6)->valueint;
                if (cnt > MAX_INSERT_SEQ) cnt = MAX_INSERT_SEQ;
                if (cnt < 0) cnt = 0;
                tc->pedestrian.insert_count = 0;
                for (int k = 0; k < cnt && (7 + k) < psz; k++) {
                    const cJSON *it = cJSON_GetArrayItem(ped, 7 + k);
                    if (cJSON_IsNumber(it))
                        tc->pedestrian.insert_sequence[tc->pedestrian.insert_count++] =
                            (uint8_t)it->valueint;
                }
                ESP_LOGW(TAG, "lora_json: legacy 'ped' format (no insert_red_ms)");
            }
        }
        /* === END FIX INSERT-RED-SHORT === */

        const cJSON *st = cJSON_GetObjectItemCaseSensitive(t, "st");
        if (cJSON_IsArray(st)) {
            tc->stage_count = 0;
            const cJSON *item = NULL;
            cJSON_ArrayForEach(item, st) {
                if (tc->stage_count >= MAX_DD_STAGES) break;
                if (!cJSON_IsArray(item) || cJSON_GetArraySize(item) < 7) continue;
                traffic_stage_dd_t *s = &tc->stages[tc->stage_count];
                memset(s, 0, sizeof(*s));
                s->id = (uint8_t)cJSON_GetArrayItem(item, 0)->valueint;
                const cJSON *nm = cJSON_GetArrayItem(item, 1);
                if (cJSON_IsString(nm) && nm->valuestring)
                    strncpy(s->name, nm->valuestring, sizeof(s->name) - 1);
                const cJSON *g = cJSON_GetArrayItem(item, 2);
                if (cJSON_IsArray(g)) {
                    const cJSON *gi = NULL;
                    cJSON_ArrayForEach(gi, g) {
                        if (s->group_count < MAX_STAGE_GROUPS && cJSON_IsNumber(gi))
                            s->groups[s->group_count++] = (uint8_t)gi->valueint;
                    }
                }
                s->duration_kind = (uint8_t)cJSON_GetArrayItem(item, 3)->valueint;
                s->duration_ms   = (uint32_t)cJSON_GetArrayItem(item, 4)->valueint;
                const cJSON *on = cJSON_GetArrayItem(item, 5);
                if (cJSON_IsArray(on)) {
                    const cJSON *oi = NULL;
                    cJSON_ArrayForEach(oi, on) {
                        if (s->on_count < MAX_ON_OUTPUTS && cJSON_IsNumber(oi))
                            s->outputs_on[s->on_count++] = (uint8_t)oi->valueint;
                    }
                }
                const cJSON *blink = cJSON_GetArrayItem(item, 6);
                if (cJSON_IsArray(blink)) {
                    const cJSON *bi = NULL;
                    cJSON_ArrayForEach(bi, blink) {
                        if (s->blink_count < MAX_BLINK_OUTPUTS && cJSON_IsNumber(bi))
                            s->outputs_blink[s->blink_count++] = (uint8_t)bi->valueint;
                    }
                }
                tc->stage_count++;
            }
        }

        const cJSON *seq = cJSON_GetObjectItemCaseSensitive(t, "seq");
        if (cJSON_IsArray(seq)) {
            tc->sequence_count = 0;
            const cJSON *si = NULL;
            cJSON_ArrayForEach(si, seq) {
                if (tc->sequence_count < MAX_SEQUENCE && cJSON_IsNumber(si))
                    tc->sequence[tc->sequence_count++] = (uint8_t)si->valueint;
            }
        }

        const cJSON *tr = cJSON_GetObjectItemCaseSensitive(t, "tr");
        if (cJSON_IsArray(tr)) {
            tc->transition_count = 0;
            const cJSON *ti = NULL;
            cJSON_ArrayForEach(ti, tr) {
                if (tc->transition_count >= MAX_TRANSITIONS) break;
                if (!cJSON_IsArray(ti) || cJSON_GetArraySize(ti) < 3) continue;
                traffic_transition_t *tt = &tc->transitions[tc->transition_count];
                tt->clearing = (uint8_t)cJSON_GetArrayItem(ti, 0)->valueint;
                tt->entering = (uint8_t)cJSON_GetArrayItem(ti, 1)->valueint;
                tt->time_cs  = (uint16_t)(cJSON_GetArrayItem(ti, 2)->valueint / 10);
                tc->transition_count++;
            }
        }
    }

    const cJSON *sr = cJSON_GetObjectItemCaseSensitive(root, "sr");
    if (cJSON_IsArray(sr)) {
        for (int i = 0; i < tmp->sr.count; i++)
            tmp->sr.outputs[i].active = false;
        const cJSON *ch = NULL;
        cJSON_ArrayForEach(ch, sr) {
            if (!cJSON_IsNumber(ch)) continue;
            int c = ch->valueint;
            if (c >= 0 && c < tmp->sr.count)
                tmp->sr.outputs[c].active = true;
        }
    }

    cJSON_Delete(root);

    /* === FIX: валидация перед коммитом === */
    if (tmp->traffic.enabled && tmp->traffic.stage_count == 0) {
        ESP_LOGE(TAG, "lora_json: traffic.enabled=true but no stages → reject");
        cfg_load_unlock();
        return false;
    }
    if (tmp->traffic.stage_count > 0 && tmp->traffic.sequence_count == 0) {
        ESP_LOGW(TAG, "lora_json: stages present but no sequence → auto-fill");
        for (int i = 0; i < tmp->traffic.stage_count && i < MAX_SEQUENCE; i++)
            tmp->traffic.sequence[i] = tmp->traffic.stages[i].id;
        tmp->traffic.sequence_count = tmp->traffic.stage_count;
    }
    /* === END FIX === */

    g_config = *tmp;
    config_save();

    ESP_LOGI(TAG, "lora_json loaded (version=%.2f, stages=%u seq=%u, "
                  "ped_mode=%u, hold=%d/%u, insert_seq=%u, insert_red_ms=%lu, "
                  "%u bytes)",
             g_config.version, g_config.traffic.stage_count,
             g_config.traffic.sequence_count,
             g_config.traffic.pedestrian.request_mode,
             g_config.traffic.hold.enabled,
             g_config.traffic.hold.stage_count,
             g_config.traffic.pedestrian.insert_count,
             (unsigned long)g_config.traffic.pedestrian.insert_red_ms,
             (unsigned)len);

    cfg_load_unlock();
    return true;
}