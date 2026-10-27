// config_manager.c
#include "config_manager.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

static const char *TAG = "CFG";
#define NVS_NS       "app_cfg"
#define NVS_KEY      "cfg_blob"

#define NVS_NS_ROLE   "device_cfg"
#define NVS_KEY_ROLE  "role"
#define NVS_KEY_LOCK  "role_lock"

app_config_t g_config;

/* ============================================================ */
static void set_defaults(void)
{
    memset(&g_config, 0, sizeof(g_config));

    g_config.version = 1.0f;
    g_config.id = 0;
    strcpy(g_config.name, "default");

    g_config.role = DEVICE_ROLE_MASTER;

    g_config.espnow.enabled = false;
    g_config.espnow.role = 0;
    g_config.espnow.channel = 6;
    g_config.espnow.status_period_ms = 1000;
    memset(g_config.espnow.slave_mac, 0, 6);

    strcpy(g_config.wifi_ssid, "Keenetic-4250");
    strcpy(g_config.wifi_password, "VTuVMHPi");
    strcpy(g_config.ap_ssid, "");
    strcpy(g_config.ap_password, "");
    strcpy(g_config.timezone, "MSK-3");

    g_config.mqtt_enabled = true;
    strcpy(g_config.mqtt_uri, "mqtt://m2.wqtt.ru");
    g_config.mqtt_port = 13257;
    strcpy(g_config.mqtt_user, "u_GGENLB");
    strcpy(g_config.mqtt_password, "LTHNW22D");
    g_config.mqtt_timepause_min = 5;

    const char *names[] = { "solar", "bat", "load" };
    const int   pins[]  = { 18, 19, 20 };
    for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
        pwm_channel_config_t *c = &g_config.channels[i];
        strncpy(c->name, names[i], MAX_NAME_LEN - 1);
        c->gpio = pins[i];
        c->frequency_hz = 1000;
        c->duty_percent = 100;
        c->enabled = true;
        c->days_mask = 0x7F;
        c->start_hour = 8; c->end_hour = 20;
        c->duty_min = 0; c->duty_max = 100; c->duty_step = 10;
    }

    g_config.traffic.mode = 3;
    g_config.traffic.enabled = false;
    g_config.traffic.auto_start = true;

    g_config.traffic.pedestrian.button_enabled    = true;
    g_config.traffic.pedestrian.request_mode      = PED_REQUEST_MODE_STAGE;
    g_config.traffic.pedestrian.insert_timeout_ms = 30000;
    g_config.traffic.pedestrian.min_recall_s      = 5;

    g_config.traffic.pins.single = (traffic_color_pins_t){0,1,2};
    g_config.traffic.pins.cross.a_north = (traffic_color_pins_t){0,1,2};
    g_config.traffic.pins.cross.a_south = (traffic_color_pins_t){3,4,5};
    g_config.traffic.pins.cross.b_west  = (traffic_color_pins_t){6,7,8};
    g_config.traffic.pins.cross.b_east  = (traffic_color_pins_t){9,10,11};
    g_config.traffic.pins.cross_ped.axis_a  = (traffic_color_pins_t){0,1,2};
    g_config.traffic.pins.cross_ped.axis_a2 = (traffic_color_pins_t){3,4,5};
    g_config.traffic.pins.cross_ped.axis_a3 = (traffic_color_pins_t){6,7,8};
    g_config.traffic.pins.cross_ped.ped_red = 9;
    g_config.traffic.pins.cross_ped.ped_green = 10;

    g_config.traffic.phases_ms.red          = 8000;
    g_config.traffic.phases_ms.red_yellow   = 2000;
    g_config.traffic.phases_ms.green        = 10000;
    g_config.traffic.phases_ms.green_blink  = 3000;
    g_config.traffic.phases_ms.yellow       = 2000;
    g_config.traffic.phases_ms.yellow_blink = 2000;
    g_config.traffic.phases_ms.ped_green    = 10000;
    g_config.traffic.phases_ms.ped_blink    = 4000;
    g_config.traffic.phases_ms.fault_blink_period = 500;
    g_config.traffic.phases_ms.phase_blink_period = 500;

    g_config.traffic.fault.imin_mv = 10;
    g_config.traffic.fault.vin_on_min_v = 5.0f;
    g_config.traffic.fault.vin_off_min_v = 5.0f;
    g_config.traffic.fault.phase_grace_ms = 2500;
    g_config.traffic.fault.recovery_ms = 2000;
    g_config.traffic.fault.vin_off_confirmations = 3;

    traffic_stage_dd_t *st = g_config.traffic.stages;
    st[0] = (traffic_stage_dd_t){ .id=1, .name="Фаза.1", .duration_ms=8000,
        .duration_kind=DD_DUR_EXPLICIT, .outputs_on={0}, .on_count=1 };
    st[1] = (traffic_stage_dd_t){ .id=2, .name="Фаза.2", .duration_ms=8000,
        .duration_kind=DD_DUR_EXPLICIT, .outputs_on={3}, .on_count=1 };
    st[2] = (traffic_stage_dd_t){ .id=3, .name="Фаза.3", .duration_ms=10000,
        .duration_kind=DD_DUR_EXPLICIT, .outputs_on={6,9}, .on_count=2 };
    st[3] = (traffic_stage_dd_t){ .id=4, .name="Фаза.4", .duration_ms=10000,
        .duration_kind=DD_DUR_EXPLICIT, .outputs_on={9}, .on_count=1,
        .outputs_blink={10}, .blink_count=1 };
    g_config.traffic.stage_count = 4;

    g_config.traffic.sequence[0] = 1;
    g_config.traffic.sequence[1] = 2;
    g_config.traffic.sequence[2] = 3;
    g_config.traffic.sequence[3] = 4;
    g_config.traffic.sequence_count = 4;

    for (int i = 0; i < MAX_OUTPUTS; i++) {
        g_config.sr.outputs[i].ch = (uint8_t)i;
        snprintf(g_config.sr.outputs[i].name, sizeof(g_config.sr.outputs[i].name),
                 "%d", i + 1);
        g_config.sr.outputs[i].active = (i < 12);
    }
    g_config.sr.count = MAX_OUTPUTS;
}

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
                      "stages=%u, seq=%u, ped_mode=%u)",
                 g_config.version,
                 g_config.wifi_ssid, g_config.traffic.mode,
                 g_config.traffic.stage_count, g_config.traffic.sequence_count,
                 g_config.traffic.pedestrian.request_mode);
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
        ESP_LOGI(TAG, "Config saved to NVS (version=%.2f, %u bytes, ped_mode=%u)",
                 g_config.version, (unsigned)sizeof(g_config),
                 g_config.traffic.pedestrian.request_mode);
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
        ESP_LOGI(TAG, "pedestrian: section missing → defaults (enabled, stage)");
        return;
    }

    tc->pedestrian.button_enabled =
        json_get_bool(ped, "button_enabled", true);
    tc->pedestrian.insert_timeout_ms =
        (uint32_t)json_get_int(ped, "insert_timeout_ms", 30000);
    tc->pedestrian.min_recall_s =
        (uint32_t)json_get_int(ped, "min_recall_s", 5);

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

    ESP_LOGI(TAG, "pedestrian: button=%d mode=%u insert=%lu min_recall=%lu",
             tc->pedestrian.button_enabled,
             tc->pedestrian.request_mode,
             (unsigned long)tc->pedestrian.insert_timeout_ms,
             (unsigned long)tc->pedestrian.min_recall_s);
}

static void parse_traffic(const cJSON *t)
{
    if (!cJSON_IsObject(t)) return;
    traffic_config_t *tc = &g_config.traffic;

    tc->mode       = (uint8_t)json_get_int(t, "mode", tc->mode);
    tc->enabled    = json_get_bool(t, "enabled", tc->enabled);
    tc->auto_start = json_get_bool(t, "auto_start", tc->auto_start);

    parse_pedestrian(t, tc);

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
}

static void parse_signal_groups(const cJSON *sg) { /* ваш код */ }
static void parse_igm(const cJSON *arr)            { /* ваш код */ }
static void parse_stages_ref(const cJSON *arr)     { /* ваш код */ }
static void parse_detectors(const cJSON *arr)      { /* ваш код */ }
static void parse_plans(const cJSON *arr)          { /* ваш код */ }
static void parse_schedule(const cJSON *s)         { /* ваш код */ }
static void parse_control_sources(const cJSON *arr) { /* ваш код */ }
static void parse_modules(const cJSON *m)          { /* ваш код */ }

/* ============================================================ */
bool config_load_json(const char *json, size_t len)
{
    if (!json || len == 0) {
        ESP_LOGE(TAG, "config_load_json: empty input");
        return false;
    }

    cJSON *root = cJSON_ParseWithLength(json, len);
    if (!root) {
        ESP_LOGE(TAG, "JSON parse error");
        return false;
    }
    ESP_LOGI(TAG, "Parsing JSON config (%u bytes)", (unsigned)len);

    device_role_t saved_role = g_config.role;

    set_defaults();
    g_config.role = saved_role;

    const cJSON *meta = cJSON_GetObjectItemCaseSensitive(root, "meta");
    if (cJSON_IsObject(meta)) {
        g_config.version = (float)json_get_double(meta, "version", 1.0);
        g_config.id = (uint32_t)json_get_int(meta, "id", 0);
        json_get_str(meta, "name", g_config.name, sizeof(g_config.name));
    }

    /* espnow, wifi, timezone, mqtt */
    const cJSON *en = cJSON_GetObjectItemCaseSensitive(root, "espnow");
    if (cJSON_IsObject(en)) {
        g_config.espnow.enabled = json_get_bool(en, "enabled", false);
        g_config.espnow.channel = (uint8_t)json_get_int(en, "channel", 6);
        g_config.espnow.status_period_ms =
            (uint16_t)json_get_int(en, "status_period_ms", 1000);

        const cJSON *mac = cJSON_GetObjectItemCaseSensitive(en, "slave_mac");
        if (cJSON_IsString(mac) && mac->valuestring) {
            unsigned int b[6];
            if (sscanf(mac->valuestring, "%02x:%02x:%02x:%02x:%02x:%02x",
                       &b[0],&b[1],&b[2],&b[3],&b[4],&b[5]) == 6) {
                for (int i = 0; i < 6; i++)
                    g_config.espnow.slave_mac[i] = (uint8_t)b[i];
            }
        }
    }

    const cJSON *wifi = cJSON_GetObjectItemCaseSensitive(root, "wifi");
    if (cJSON_IsObject(wifi)) {
        json_get_str(wifi, "ssid", g_config.wifi_ssid, sizeof(g_config.wifi_ssid));
        json_get_str(wifi, "password", g_config.wifi_password, sizeof(g_config.wifi_password));
    }
    json_get_str(root, "timezone", g_config.timezone, sizeof(g_config.timezone));

    const cJSON *mqtt = cJSON_GetObjectItemCaseSensitive(root, "mqtt");
    if (cJSON_IsObject(mqtt)) {
        g_config.mqtt_enabled = json_get_bool(mqtt, "enabled", true);
        json_get_str(mqtt, "uri", g_config.mqtt_uri, sizeof(g_config.mqtt_uri));
        g_config.mqtt_port = (uint16_t)json_get_int(mqtt, "port", 13257);
        json_get_str(mqtt, "user", g_config.mqtt_user, sizeof(g_config.mqtt_user));
        json_get_str(mqtt, "password", g_config.mqtt_password, sizeof(g_config.mqtt_password));
        g_config.mqtt_timepause_min = (uint16_t)json_get_int(mqtt, "timepause_min", 5);
    }

    parse_traffic(cJSON_GetObjectItemCaseSensitive(root, "traffic"));
    parse_signal_groups(cJSON_GetObjectItemCaseSensitive(root, "signalGroups"));
    parse_igm(cJSON_GetObjectItemCaseSensitive(root, "IGM"));
    parse_stages_ref(cJSON_GetObjectItemCaseSensitive(root, "stages"));
    parse_detectors(cJSON_GetObjectItemCaseSensitive(root, "detectors"));
    parse_plans(cJSON_GetObjectItemCaseSensitive(root, "plans"));
    parse_schedule(cJSON_GetObjectItemCaseSensitive(root, "schedule"));
    parse_control_sources(cJSON_GetObjectItemCaseSensitive(root, "controlSources"));
    parse_modules(cJSON_GetObjectItemCaseSensitive(root, "modules"));

    cJSON_Delete(root);

    /* FIX: config_load_json ВСЕГДА сохраняет в NVS. */
    config_save();

    ESP_LOGI(TAG, "JSON loaded (version=%.2f, role=%s, traffic stages=%u seq=%u, "
                  "ped_mode=%u)",
             g_config.version,
             g_config.role == DEVICE_ROLE_SLAVE ? "slave" : "master",
             g_config.traffic.stage_count,
             g_config.traffic.sequence_count,
             g_config.traffic.pedestrian.request_mode);
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

bool config_export_json(char *buf, size_t *len)
{
    if (!buf || !len || *len < 256) return false;

    cJSON *root = cJSON_CreateObject();
    if (!root) {
        ESP_LOGE(TAG, "config_export_json: cJSON_CreateObject failed");
        return false;
    }

    cJSON *meta = cJSON_CreateObject();
    cJSON_AddNumberToObject(meta, "version", g_config.version);
    cJSON_AddNumberToObject(meta, "id", g_config.id);
    cJSON_AddStringToObject(meta, "name", g_config.name);
    cJSON_AddItemToObject(root, "meta", meta);

    cJSON *en = cJSON_CreateObject();
    cJSON_AddBoolToObject(en, "enabled", g_config.espnow.enabled);
    cJSON_AddNumberToObject(en, "channel", g_config.espnow.channel);
    char mac_str[20];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
             g_config.espnow.slave_mac[0], g_config.espnow.slave_mac[1],
             g_config.espnow.slave_mac[2], g_config.espnow.slave_mac[3],
             g_config.espnow.slave_mac[4], g_config.espnow.slave_mac[5]);
    cJSON_AddStringToObject(en, "slave_mac", mac_str);
    cJSON_AddNumberToObject(en, "status_period_ms", g_config.espnow.status_period_ms);
    cJSON_AddItemToObject(root, "espnow", en);

    cJSON *wifi = cJSON_CreateObject();
    cJSON_AddStringToObject(wifi, "ssid", g_config.wifi_ssid);
    cJSON_AddStringToObject(wifi, "password", g_config.wifi_password);
    cJSON_AddItemToObject(root, "wifi", wifi);

    cJSON_AddStringToObject(root, "timezone", g_config.timezone);

    cJSON *mqtt = cJSON_CreateObject();
    cJSON_AddBoolToObject(mqtt, "enabled", g_config.mqtt_enabled);
    cJSON_AddStringToObject(mqtt, "uri", g_config.mqtt_uri);
    cJSON_AddNumberToObject(mqtt, "port", g_config.mqtt_port);
    cJSON_AddStringToObject(mqtt, "user", g_config.mqtt_user);
    cJSON_AddStringToObject(mqtt, "password", g_config.mqtt_password);
    cJSON_AddNumberToObject(mqtt, "timepause_min", g_config.mqtt_timepause_min);
    cJSON_AddItemToObject(root, "mqtt", mqtt);

    /* ===== traffic ===== */
    cJSON *t = cJSON_CreateObject();
    cJSON_AddNumberToObject(t, "mode", g_config.traffic.mode);
    cJSON_AddBoolToObject(t, "enabled", g_config.traffic.enabled);
    cJSON_AddBoolToObject(t, "auto_start", g_config.traffic.auto_start);

    cJSON *ped = cJSON_CreateObject();
    cJSON_AddBoolToObject(ped, "button_enabled",
                          g_config.traffic.pedestrian.button_enabled);
    cJSON_AddStringToObject(ped, "request_mode",
                            ped_request_mode_to_str(g_config.traffic.pedestrian.request_mode));
    cJSON_AddNumberToObject(ped, "insert_timeout_ms",
                            g_config.traffic.pedestrian.insert_timeout_ms);
    cJSON_AddNumberToObject(ped, "min_recall_s",
                            g_config.traffic.pedestrian.min_recall_s);
    cJSON_AddItemToObject(t, "pedestrian", ped);

    cJSON *ph = cJSON_CreateObject();
    cJSON_AddNumberToObject(ph, "red",          g_config.traffic.phases_ms.red);
    cJSON_AddNumberToObject(ph, "red_yellow",   g_config.traffic.phases_ms.red_yellow);
    cJSON_AddNumberToObject(ph, "green",        g_config.traffic.phases_ms.green);
    cJSON_AddNumberToObject(ph, "green_blink",  g_config.traffic.phases_ms.green_blink);
    cJSON_AddNumberToObject(ph, "yellow",       g_config.traffic.phases_ms.yellow);
    cJSON_AddNumberToObject(ph, "yellow_blink", g_config.traffic.phases_ms.yellow_blink);
    cJSON_AddNumberToObject(ph, "ped_green",    g_config.traffic.phases_ms.ped_green);
    cJSON_AddNumberToObject(ph, "ped_blink",    g_config.traffic.phases_ms.ped_blink);
    cJSON_AddNumberToObject(ph, "fault_blink_period",
                            g_config.traffic.phases_ms.fault_blink_period);
    cJSON_AddNumberToObject(ph, "phase_blink_period",
                            g_config.traffic.phases_ms.phase_blink_period);
    cJSON_AddItemToObject(t, "phases_ms", ph);

    cJSON *flt = cJSON_CreateObject();
    cJSON_AddNumberToObject(flt, "imin_mv",   g_config.traffic.fault.imin_mv);
    cJSON_AddNumberToObject(flt, "vin_on_min_v",  g_config.traffic.fault.vin_on_min_v);
    cJSON_AddNumberToObject(flt, "vin_off_min_v", g_config.traffic.fault.vin_off_min_v);
    cJSON_AddNumberToObject(flt, "phase_grace_ms",
                            g_config.traffic.fault.phase_grace_ms);
    cJSON_AddNumberToObject(flt, "recovery_ms",
                            g_config.traffic.fault.recovery_ms);
    cJSON_AddNumberToObject(flt, "vin_off_confirmations",
                            g_config.traffic.fault.vin_off_confirmations);
    cJSON_AddItemToObject(t, "fault", flt);

    cJSON *pins = cJSON_CreateObject();
    cJSON *single = cJSON_CreateObject();
    cJSON_AddNumberToObject(single, "red",    g_config.traffic.pins.single.red);
    cJSON_AddNumberToObject(single, "yellow", g_config.traffic.pins.single.yellow);
    cJSON_AddNumberToObject(single, "green",  g_config.traffic.pins.single.green);
    cJSON_AddItemToObject(pins, "single", single);

    cJSON *cross = cJSON_CreateObject();
    {
        const char *keys[] = {"a_north", "a_south", "b_west", "b_east"};
        const traffic_color_pins_t *vals[] = {
            &g_config.traffic.pins.cross.a_north,
            &g_config.traffic.pins.cross.a_south,
            &g_config.traffic.pins.cross.b_west,
            &g_config.traffic.pins.cross.b_east,
        };
        for (int i = 0; i < 4; i++) {
            cJSON *o = cJSON_CreateObject();
            cJSON_AddNumberToObject(o, "red",    vals[i]->red);
            cJSON_AddNumberToObject(o, "yellow", vals[i]->yellow);
            cJSON_AddNumberToObject(o, "green",  vals[i]->green);
            cJSON_AddItemToObject(cross, keys[i], o);
        }
    }
    cJSON_AddItemToObject(pins, "cross", cross);

    cJSON *cp = cJSON_CreateObject();
    {
        const char *keys[] = {"axis_a", "axis_a2", "axis_a3"};
        const traffic_color_pins_t *vals[] = {
            &g_config.traffic.pins.cross_ped.axis_a,
            &g_config.traffic.pins.cross_ped.axis_a2,
            &g_config.traffic.pins.cross_ped.axis_a3,
        };
        for (int i = 0; i < 3; i++) {
            cJSON *o = cJSON_CreateObject();
            cJSON_AddNumberToObject(o, "red",    vals[i]->red);
            cJSON_AddNumberToObject(o, "yellow", vals[i]->yellow);
            cJSON_AddNumberToObject(o, "green",  vals[i]->green);
            cJSON_AddItemToObject(cp, keys[i], o);
        }
    }
    cJSON *pedpin = cJSON_CreateObject();
    cJSON_AddNumberToObject(pedpin, "red",   g_config.traffic.pins.cross_ped.ped_red);
    cJSON_AddNumberToObject(pedpin, "green", g_config.traffic.pins.cross_ped.ped_green);
    cJSON_AddItemToObject(cp, "ped", pedpin);
    cJSON_AddItemToObject(pins, "cross_ped", cp);

    cJSON_AddItemToObject(t, "pins", pins);

    cJSON *stages = cJSON_CreateArray();
    for (int i = 0; i < g_config.traffic.stage_count; i++) {
        const traffic_stage_dd_t *st = &g_config.traffic.stages[i];
        cJSON *so = cJSON_CreateObject();
        cJSON_AddNumberToObject(so, "id", st->id);
        cJSON_AddStringToObject(so, "name", st->name);
        cJSON_AddStringToObject(so, "duration_kind",
                                duration_kind_to_str(st->duration_kind));
        cJSON_AddNumberToObject(so, "duration_ms", st->duration_ms);

        cJSON *gr = cJSON_CreateArray();
        for (int k = 0; k < st->group_count; k++)
            cJSON_AddItemToArray(gr, cJSON_CreateNumber(st->groups[k]));
        cJSON_AddItemToObject(so, "groups", gr);

        cJSON *on = cJSON_CreateArray();
        for (int k = 0; k < st->on_count; k++)
            cJSON_AddItemToArray(on, cJSON_CreateNumber(st->outputs_on[k]));
        cJSON_AddItemToObject(so, "outputs_on", on);

        cJSON *bl = cJSON_CreateArray();
        for (int k = 0; k < st->blink_count; k++)
            cJSON_AddItemToArray(bl, cJSON_CreateNumber(st->outputs_blink[k]));
        cJSON_AddItemToObject(so, "outputs_blink", bl);

        cJSON_AddItemToArray(stages, so);
    }
    cJSON_AddItemToObject(t, "stages_dd", stages);

    cJSON *seq = cJSON_CreateArray();
    for (int i = 0; i < g_config.traffic.sequence_count; i++)
        cJSON_AddItemToArray(seq, cJSON_CreateNumber(g_config.traffic.sequence[i]));
    cJSON_AddItemToObject(t, "sequence", seq);

    cJSON_AddItemToObject(root, "traffic", t);

    cJSON *sr = cJSON_CreateObject();
    cJSON *outputs = cJSON_CreateArray();
    for (int i = 0; i < g_config.sr.count; i++) {
        cJSON *o = cJSON_CreateObject();
        cJSON_AddNumberToObject(o, "ch", g_config.sr.outputs[i].ch);
        cJSON_AddStringToObject(o, "name", g_config.sr.outputs[i].name);
        cJSON_AddBoolToObject(o, "active", g_config.sr.outputs[i].active);
        cJSON_AddItemToArray(outputs, o);
    }
    cJSON_AddItemToObject(sr, "outputs", outputs);
    cJSON_AddItemToObject(root, "shift_register", sr);

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!out) {
        ESP_LOGE(TAG, "config_export_json: cJSON_PrintUnformatted failed");
        return false;
    }

    size_t n = strlen(out);
    if (n + 1 > *len) {
        ESP_LOGE(TAG, "config_export_json: buffer too small (%u > %u)",
                 (unsigned)(n + 1), (unsigned)*len);
        free(out);
        return false;
    }
    memcpy(buf, out, n);
    buf[n] = '\0';
    *len = n;
    free(out);

    ESP_LOGI(TAG, "config_export_json: OK (%u bytes, ped_mode=%u)",
             (unsigned)n, g_config.traffic.pedestrian.request_mode);
    return true;
}