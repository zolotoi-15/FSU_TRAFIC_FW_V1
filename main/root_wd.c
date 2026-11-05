/* ============================================================
   root_wd.c — ROOT: CFG на NODE → cfg:ok → START ROOT+NODE.

   === FIX SYNC-PM + SYNC-MASK ===
       ROOT шлёт sync:ph:pm_target:next:on_mask:blink_mask.
       pm_target = pm + SYNC_LEAD_MS (pm НА МОМЕНТ применения).
       on_mask/blink_mask — маски каналов из stage_dd ROOT
       (через ПУБЛИЧНЫЙ traffic_light_get_stage_masks).

       ROOT ждёт SYNC_LEAD_MS после отправки, затем применяет
       force_phase(pm_target + lag, dur).

       NODE применяет ту же маску через apply_mask().

   === FIX NO-DOUBLE-SYNC ===
       root_node_watchdog_task re-send traffic:start БЕЗ sync:.
       Раньше: watchdog слал sync: каждые 2 с пока s_node_running
               не обновится → ДУБЛИ (sync:ph:300 и sync:ph:600
               с интервалом 500 мс), pm «прыгал» 600→300→600,
               фазы не переключались.
       Теперь: sync: шлёт ТОЛЬКО phase_sync_task (из pending
               phase_change, force_resync, fallback 30s).
               Watchdog шлёт только traffic:start.

   === FIX SYNC-ASYNC ===
       send_sync_to_node_async — только отправка.
       apply_sync_at_utc — ждёт + применяет.

   === FIX REQUEUE ===
       При ошибке send_sync pending-фаза возвращается в очередь.

   === FIX ROOT-START-NO-TIME ===
       traffic:start ВСЕГДА, sync: — только когда time валиден.
       При отсутствии time: phase_sync_task дошлёт sync: при
       появлении времени.
   ============================================================ */

#include "root_wd.h"
#include "app_shared.h"
#include "transport.h"
#include "traffic_light.h"
#include "wifi_mqtt.h"
#include "time_sync.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "root_wd";

#define ROOT_NODE_CHECK_MS          2000
#define NODE_COMM_LOST_MS           20000
#define LORA_SYNC_INTERVAL_MS       30000
#define CFG_PUSH_RX_WAIT_MS         3000
#define CFG_RETRY_PERIOD_MS         10000
#define NO_CFG_ALL_OFF_MS           (60 * 1000)

#define CFG_INITIAL_DELAY_MS        5000
#define CFG_MAX_ATTEMPTS            10
#define CFG_RETRY_COOLDOWN_MS       2000
#define CFG_RESET_ACK_DELAY_MS      500
#define HEARTBEAT_INTERVAL_MS       10000

#define CFG_LORA_BUF_SIZE           2048

/* === FIX SYNC-PM: SYNC_LEAD_MS = запас на ToA (112) + ack_tx. === */
#define SYNC_LEAD_MS                300
#define SYNC_TOA_EST_MS             112
/* === END FIX SYNC-PM === */

#define TRAFFIC_START_MAX_RETRIES   3
#define TRAFFIC_START_RETRY_MS      2000
#define FRESH_STATUS_MS             5000

#define TIME_PUSH_PERIOD_MS         30000
#define SYNC_PRE_DELAY_MS           50
#define TIME_PUSH_BEFORE_START_MS   300

extern volatile uint32_t s_last_fault_clear_ms;
extern volatile uint32_t s_last_node_msg_ms;
extern volatile bool s_node_running;

volatile bool s_cfg_ack = false;
volatile bool s_cfg_pushing = false;
volatile bool s_cfg_applied_and_started = false;
volatile bool s_cfg_req_pending = false;
volatile bool s_force_resync_pending = false;

#define SET_REMOTE_FLAG(v) do { s_fault_from_remote_ex = (v); } while (0)

extern uint8_t phase_change_take_pending(uint32_t *dur);
extern void    phase_change_requeue(uint8_t phase, uint32_t dur_ms);

/* ============================================================
   send_sync_to_node_async — только отправка sync:.
   Формат: sync:ph:pm_target:next:on_mask:blink_mask
     ph         — номер фазы (для логов)
     pm_target  — pm НА МОМЕНТ применения (= pm + lead)
     next       — следующая фаза
     on_mask    — 16-битная маска включённых каналов (hex)
     blink_mask — 16-битная маска мигающих каналов (hex)

   Маски получаются через ПУБЛИЧНЫЙ
   traffic_light_get_stage_masks() — БЕЗ traffic_internal.h.

   Возвращает:
     > 0  — apply_ms (момент применения)
     0    — ошибка/пропуск (pending можно вернуть в очередь)
   ============================================================ */
uint32_t send_sync_to_node_async(uint8_t phase, uint32_t phase_ms,
                                 uint32_t duration_ms,
                                 uint8_t *out_next_phase)
{
    (void)duration_ms;
    if (g_config.role != DEVICE_ROLE_MASTER) return 0;
    if (!transport_is_lora()) return 0;
    if (s_cfg_pushing) return 0;

    if (g_config.traffic.sequence_count == 0) {
        ESP_LOGW(TAG, "send_sync: sequence_count=0, skip");
        return 0;
    }
    if (phase >= g_config.traffic.sequence_count) {
        ESP_LOGW(TAG, "send_sync: phase=%u >= seq_count=%u, skip",
                 phase, g_config.traffic.sequence_count);
        return 0;
    }

    /* pm_target = pm + SYNC_LEAD_MS (pm НА МОМЕНТ применения) */
    uint32_t pm_target = phase_ms + SYNC_LEAD_MS;
    uint8_t next_phase = (phase + 1) % g_config.traffic.sequence_count;

    /* === FIX SYNC-MASK: маски из stage_dd ROOT (публичный геттер) === */
    uint16_t on_mask = 0;
    uint16_t blink_mask = 0;
    traffic_light_get_stage_masks(phase, &on_mask, &blink_mask);
    /* === END FIX SYNC-MASK === */

    char cmd[80];
    int n = snprintf(cmd, sizeof(cmd), "sync:%u:%lu:%u:0x%04X:0x%04X",
                     phase,
                     (unsigned long)pm_target,
                     next_phase,
                     (unsigned)on_mask,
                     (unsigned)blink_mask);
    if (n <= 0) return 0;

    ESP_LOGD(TAG, "sync: TX '%s' (pm=%lu + lead=%u = %lu, "
                  "on=0x%04X blink=0x%04X)",
             cmd, (unsigned long)phase_ms, SYNC_LEAD_MS,
             (unsigned long)pm_target,
             (unsigned)on_mask, (unsigned)blink_mask);

    esp_err_t err = transport_send_cmd(cmd, n);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "send_sync: %s failed: %s", cmd, esp_err_to_name(err));
        return 0;
    }

    if (out_next_phase) *out_next_phase = next_phase;

    /* apply_ms — момент применения. esp_timer — монотонный. */
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t apply_ms = now_ms + SYNC_LEAD_MS;

    ESP_LOGI(TAG, "sync: ph=%u pm_target=%lu apply_ms=%lu next=%u "
                  "on=0x%04X blink=0x%04X",
             phase, (unsigned long)pm_target,
             (unsigned long)apply_ms, next_phase,
             (unsigned)on_mask, (unsigned)blink_mask);

    return apply_ms;
}

/* ============================================================
   apply_sync_at_utc — ждём apply_ms, применяем force_phase.
   На ROOT force_phase использует stage_dd ROOT — это нормально,
   ROOT — источник правды. NODE применяет ту же маску через
   apply_mask() (в node_proto.c).
   ============================================================ */
uint32_t apply_sync_at_utc(uint8_t phase, uint32_t phase_ms,
                           uint32_t duration_ms, uint32_t apply_ms)
{
    if (apply_ms == 0) return 0;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    int32_t wait_ms = (int32_t)(apply_ms - now_ms);
    if (wait_ms > 2000) {
        ESP_LOGW(TAG, "apply_sync: wait_ms=%ld > 2000 → clamp", (long)wait_ms);
        wait_ms = 2000;
    }
    if (wait_ms > 0) vTaskDelay(pdMS_TO_TICKS(wait_ms));

    now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    int32_t lag = (int32_t)(now_ms - apply_ms);
    if (lag < 0) lag = 0;

    uint32_t pm_target = phase_ms + SYNC_LEAD_MS;
    uint32_t pm_apply = pm_target + (uint32_t)lag;
    if (duration_ms > 0 && pm_apply >= duration_ms) pm_apply = duration_ms - 1;

    traffic_light_force_phase(phase, pm_apply, duration_ms);

    ESP_LOGI(TAG, "sync applied (ROOT): ph=%u pm=%lu lag=%ld dur=%lu",
             phase, (unsigned long)pm_apply, (long)lag,
             (unsigned long)duration_ms);
    return 1;
}

/* Синхронная обёртка (для watchdog / fallback 30s) */
uint32_t send_sync_to_node(uint8_t phase, uint32_t phase_ms,
                           uint32_t duration_ms)
{
    uint32_t apply_ms = send_sync_to_node_async(phase, phase_ms,
                                                duration_ms, NULL);
    if (apply_ms == 0) return 0;
    return apply_sync_at_utc(phase, phase_ms, duration_ms, apply_ms);
}

void apply_sync_at(uint8_t phase, uint32_t phase_ms,
                   uint32_t duration_ms, uint32_t apply_ms)
{
    (void)apply_ms;
    traffic_light_force_phase(phase, phase_ms, duration_ms);
}

/* ============================================================
   root_node_watchdog_task
   === FIX NO-DOUBLE-SYNC ===
       Watchdog НЕ шлёт sync: при re-send traffic:start.
       sync: шлёт ТОЛЬКО phase_sync_task (из pending phase_change,
       force_resync, fallback 30s). Иначе — ДУБЛИ: sync:ph:300
       и sync:ph:600 с интервалом 500 мс, pm «прыгает»
       600→300→600, фазы не переключаются.
   ============================================================ */
static void root_node_watchdog_task(void *arg)
{
    ESP_LOGI(TAG, "root_node_watchdog_task started (SYNC-MASK, lead=%d ms, "
                  "NO-DOUBLE-SYNC)", SYNC_LEAD_MS);
    bool     was_connected = false;
    uint32_t lost_since_ms = 0;
    bool     in_comm_lost_all_off = false;
    uint32_t boot_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t node_seen_ms = 0;
    bool     no_cfg_all_off = false;
    uint32_t last_start_ms = 0;
    uint32_t start_retry_count = 0;

    if (!s_cfg_ack) {
        ESP_LOGW(TAG, "root_wd: no cfg:ok at start → ALL_OFF");
        traffic_light_stop();
        SET_REMOTE_FLAG(true);
        traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
        SET_REMOTE_FLAG(false);
        wifi_mqtt_set_root_comm_lost(true);
    }

    while (1) {
        if (g_config.role != DEVICE_ROLE_MASTER) {
            vTaskDelay(pdMS_TO_TICKS(ROOT_NODE_CHECK_MS));
            continue;
        }

        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        bool has_node = false;

        if (transport_is_lora() && s_last_node_msg_ms != 0) {
            has_node = !link_monitor_root_lost();
        }
        if (s_cfg_pushing) has_node = true;

        if (has_node && node_seen_ms == 0) {
            node_seen_ms = now;
        }
        if (s_cfg_ack) no_cfg_all_off = false;

        if (s_cfg_ack && has_node && !s_cfg_applied_and_started) {
            ESP_LOGW(TAG, "cfg:ok + NODE present → START на ROOT и NODE");

            if (traffic_light_get_fault() == TRAFFIC_FAULT_ALL_OFF) {
                SET_REMOTE_FLAG(true);
                traffic_light_set_fault(TRAFFIC_FAULT_NONE);
                SET_REMOTE_FLAG(false);
            }

            /* time: первым, если валиден */
            uint32_t utc_sec = 0; uint16_t utc_ms = 0;
            bool time_ok = time_sync_get(&utc_sec, &utc_ms);
            if (time_ok) {
                time_sync_root_push_force(0);
                vTaskDelay(pdMS_TO_TICKS(TIME_PUSH_BEFORE_START_MS));
                ESP_LOGI(TAG, "ROOT: time: pushed → starting traffic");
            } else {
                ESP_LOGW(TAG, "ROOT: no valid time yet → traffic:start "
                              "без time: (sync: отложен)");
            }

            /* traffic:start — ВСЕГДА */
            transport_send_cmd("traffic:start", strlen("traffic:start"));

            traffic_light_set_follow_root(false);
            traffic_light_start();

            /* sync: с pm=0 сразу после старта, если time валиден.
               Если time нет — phase_sync_task дошлёт при появлении. */
            if (time_ok) {
                if (traffic_light_get_state() == TRAFFIC_STATE_RUNNING) {
                    uint8_t phase = traffic_light_get_phase();
                    uint32_t dur = traffic_light_get_phase_duration_ms();
                    send_sync_to_node(phase, 0, dur);
                }
            }

            s_cfg_applied_and_started = true;
            was_connected = true;
            lost_since_ms = 0;
            in_comm_lost_all_off = false;
            last_start_ms = now;
            start_retry_count = 0;
            wifi_mqtt_reset_node_fault_tracking();
            wifi_mqtt_set_root_comm_lost(false);

            ESP_LOGI(TAG, "ROOT: cfg applied + traffic:start%s",
                     time_ok ? " + sync:" : " (sync: deferred)");
        }

        if (s_cfg_applied_and_started && has_node && !s_node_running) {
            bool fresh_status = (s_last_node_msg_ms != 0) &&
                                ((now - s_last_node_msg_ms) < FRESH_STATUS_MS);
            if (!fresh_status) {
                vTaskDelay(pdMS_TO_TICKS(ROOT_NODE_CHECK_MS));
                continue;
            }
            if (start_retry_count < TRAFFIC_START_MAX_RETRIES &&
                (now - last_start_ms) >= TRAFFIC_START_RETRY_MS) {
                ESP_LOGI(TAG, "ROOT: NODE not RUNNING → re-send traffic:start "
                              "(%u/%u) — БЕЗ sync: (NO-DOUBLE-SYNC)",
                         start_retry_count + 1, TRAFFIC_START_MAX_RETRIES);
                transport_send_cmd("traffic:start", strlen("traffic:start"));
                last_start_ms = now;
                start_retry_count++;
                /* === FIX NO-DOUBLE-SYNC ===
                   НЕ шлём sync: — phase_sync_task дошлёт из pending
                   или fallback (30 с). Иначе — ДУБЛИ sync: каждые
                   2 с: sync:ph:300 и sync:ph:600, pm «прыгает»
                   600→300→600, фазы не переключаются. === */
            } else if (start_retry_count >= TRAFFIC_START_MAX_RETRIES) {
                ESP_LOGW(TAG, "ROOT: traffic:start retries exhausted (%u)",
                         TRAFFIC_START_MAX_RETRIES);
                start_retry_count = TRAFFIC_START_MAX_RETRIES + 1;
            }
        }

        if (has_node && !was_connected && !s_cfg_applied_and_started) {
            ESP_LOGI(TAG, "ROOT: NODE on LoRa, waiting for cfg:ok → ALL_OFF");
            traffic_light_stop();
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
            was_connected = true;
        }

        if (!has_node && was_connected) {
            ESP_LOGW(TAG, "ROOT: NODE lost → STOP + ALL_OFF");
            traffic_light_stop();
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
            transport_send_cmd("traffic:stop", strlen("traffic:stop"));
            was_connected = false;
            lost_since_ms = now;
            s_cfg_applied_and_started = false;
            s_cfg_ack = false;
            start_retry_count = 0;
        }

        if (has_node && !s_cfg_ack && node_seen_ms != 0 && !no_cfg_all_off &&
            (now - node_seen_ms) >= NO_CFG_ALL_OFF_MS) {
            ESP_LOGE(TAG, "ROOT: NODE present but cfg not ack'd > %d ms → ALL_OFF",
                     NO_CFG_ALL_OFF_MS);
            traffic_light_stop();
            SET_REMOTE_FLAG(true);
            traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
            SET_REMOTE_FLAG(false);
            no_cfg_all_off = true;
        }

        if (!has_node && !in_comm_lost_all_off && !was_connected) {
            uint32_t since = (lost_since_ms != 0)
                             ? (now - lost_since_ms)
                             : (now - boot_ms);
            if (since >= (uint32_t)NODE_COMM_LOST_MS) {
                if (traffic_light_get_fault() != TRAFFIC_FAULT_ALL_OFF) {
                    ESP_LOGE(TAG, "ROOT: NODE absent > %d ms → ALL_OFF",
                             NODE_COMM_LOST_MS);
                    SET_REMOTE_FLAG(true);
                    traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF);
                    SET_REMOTE_FLAG(false);
                    wifi_mqtt_set_root_comm_lost(true);
                    in_comm_lost_all_off = true;
                }
                s_cfg_applied_and_started = false;
                start_retry_count = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(ROOT_NODE_CHECK_MS));
    }
}

/* ============================================================
   phase_sync_task
   ЕДИНСТВЕННЫЙ источник sync: на ROOT.

   === FIX SYNC-MASK: pm в пакете = pm_target (pm + lead),
       on_mask/blink_mask — из stage_dd ROOT (публичный геттер).

   === FIX NO-DOUBLE-SYNC: sync: шлётся ТОЛЬКО отсюда:
       1) первый sync после старта, когда появится time:
       2) phase_change из fault_handler.c (P7-sync)
       3) force_resync_pending (NODE перезапустился)
       4) fallback 30 с (если фаза не менялась)
   ============================================================ */
static void phase_sync_task(void *arg)
{
    ESP_LOGI(TAG, "phase_sync_task started (SYNC-MASK, lead=%d ms, "
                  "NO-DOUBLE-SYNC)", SYNC_LEAD_MS);

    uint32_t last_ms = 0;
    uint32_t last_time_push_ms = 0;
    bool     time_pushed_since_start = false;

    while (1) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        if (g_config.role == DEVICE_ROLE_MASTER) {
            if (s_cfg_pushing || !s_cfg_ack || !s_cfg_applied_and_started) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            traffic_state_t st = traffic_light_get_state();
            bool in_all_off = (traffic_light_get_fault() == TRAFFIC_FAULT_ALL_OFF);
            bool has_node = transport_is_lora() && s_last_node_msg_ms != 0 &&
                            !link_monitor_root_lost();
            if (s_cfg_pushing) has_node = true;

            /* 1) Первый sync после старта, как только появится время */
            if (has_node && st == TRAFFIC_STATE_RUNNING && !in_all_off &&
                !time_pushed_since_start) {
                uint32_t ts = 0; uint16_t tm = 0;
                if (time_sync_get(&ts, &tm)) {
                    time_pushed_since_start = true;
                    last_time_push_ms = now;
                    time_sync_root_push_force(0);
                    ESP_LOGI(TAG, "phase_sync: first time: pushed → "
                                  "sync:current phase now");
                    uint8_t phase = traffic_light_get_phase();
                    uint32_t dur = traffic_light_get_phase_duration_ms();
                    uint32_t pm  = traffic_light_get_phase_ms();
                    uint32_t apply_ms = send_sync_to_node_async(
                        phase, pm, dur, NULL);
                    if (apply_ms != 0) {
                        apply_sync_at_utc(phase, pm, dur, apply_ms);
                    }
                }
            }

            /* time: периодически */
            if (has_node && st == TRAFFIC_STATE_RUNNING && !in_all_off &&
                time_pushed_since_start) {
                if (last_time_push_ms == 0 ||
                    (now - last_time_push_ms) >= TIME_PUSH_PERIOD_MS) {
                    last_time_push_ms = now;
                    time_sync_root_push(0);
                }
            }

            /* 2) phase_change из fault_handler.c (P7-sync) */
            if (has_node && st == TRAFFIC_STATE_RUNNING && !in_all_off) {
                uint32_t dur_pending = 0;
                uint8_t ph_pending = phase_change_take_pending(&dur_pending);
                if (ph_pending != 0xFF) {
                    vTaskDelay(pdMS_TO_TICKS(SYNC_PRE_DELAY_MS));

                    uint32_t apply_ms = send_sync_to_node_async(
                        ph_pending, 0, dur_pending, NULL);
                    if (apply_ms == 0) {
                        ESP_LOGW(TAG, "phase_change → sync:%u TX failed, "
                                      "re-queue", ph_pending);
                        phase_change_requeue(ph_pending, dur_pending);
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(SYNC_LEAD_MS));
                        apply_sync_at_utc(ph_pending, 0, dur_pending, apply_ms);
                        ESP_LOGI(TAG, "phase_change → sync:%u:lead:%lu (r=1)",
                                 ph_pending, (unsigned long)dur_pending);
                    }
                    last_ms = (uint32_t)(esp_timer_get_time() / 1000);
                }
            }

            /* 3) force_resync_pending (NODE перезапустился) */
            if (s_force_resync_pending) {
                s_force_resync_pending = false;
                if (traffic_light_get_state() == TRAFFIC_STATE_RUNNING) {
                    uint8_t phase = traffic_light_get_phase();
                    uint32_t dur = traffic_light_get_phase_duration_ms();
                    uint32_t pm  = traffic_light_get_phase_ms();
                    ESP_LOGW(TAG, "force resync: phase=%u pm=%lu dur=%lu",
                             phase, (unsigned long)pm, (unsigned long)dur);
                    send_sync_to_node(phase, pm, dur);
                    last_ms = (uint32_t)(esp_timer_get_time() / 1000);
                }
            }

            /* 4) fallback (30 с) — если фаза не менялась */
            if (has_node && st == TRAFFIC_STATE_RUNNING && !in_all_off) {
                if (last_ms != 0 && (now - last_ms >= LORA_SYNC_INTERVAL_MS)) {
                    uint8_t  phase = traffic_light_get_phase();
                    uint32_t dur   = traffic_light_get_phase_duration_ms();
                    uint32_t pm    = traffic_light_get_phase_ms();
                    vTaskDelay(pdMS_TO_TICKS(SYNC_PRE_DELAY_MS));
                    send_sync_to_node(phase, pm, dur);
                    last_ms = (uint32_t)(esp_timer_get_time() / 1000);
                }
            }
        } else {
            time_pushed_since_start = false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ============================================================
   cfg_push_task
   ============================================================ */
static void cfg_push_task(void *arg)
{
    ESP_LOGI(TAG, "cfg_push_task started (short JSON for LoRa)");

    uint32_t waited = 0;
    while (!transport_is_lora() && waited < 10000) {
        vTaskDelay(pdMS_TO_TICKS(500));
        waited += 500;
    }
    if (!transport_is_lora()) {
        ESP_LOGE(TAG, "cfg_push: LoRa not ready");
    }

    ESP_LOGI(TAG, "cfg_push: wait for cfg:req from NODE");
    vTaskDelay(pdMS_TO_TICKS(CFG_INITIAL_DELAY_MS));

    while (1) {
        if (g_config.role != DEVICE_ROLE_MASTER) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (!transport_is_lora()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (s_cfg_pushing) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (s_cfg_ack) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (!s_cfg_req_pending) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        s_cfg_req_pending = false;
        ESP_LOGI(TAG, "cfg_push: cfg:req from NODE → start push");

        extern bool config_export_lora_json(char *b, size_t *l);
        char *buf = malloc(CFG_LORA_BUF_SIZE);
        size_t blen = CFG_LORA_BUF_SIZE;
        if (!buf || !config_export_lora_json(buf, &blen)) {
            ESP_LOGE(TAG, "cfg_push: short JSON export failed");
            if (buf) free(buf);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "cfg_push: short JSON ready (%u bytes)", (unsigned)blen);

        s_cfg_pushing = true;
        vTaskDelay(pdMS_TO_TICKS(CFG_PUSH_RX_WAIT_MS));

        for (int attempt = 0; attempt < CFG_MAX_ATTEMPTS; attempt++) {
            if (s_cfg_ack) break;

            transport_send_cmd("cfg:reset", strlen("cfg:reset"));
            vTaskDelay(pdMS_TO_TICKS(CFG_RESET_ACK_DELAY_MS));

            ESP_LOGI(TAG, "cfg_push → LoRa attempt %d/%d (%u bytes)",
                     attempt + 1, CFG_MAX_ATTEMPTS, (unsigned)blen);

            esp_err_t err = transport_send_cfg(buf, blen);

            if (err == ESP_OK) {
                for (int w = 0; w < 50; w++) {
                    if (s_cfg_ack) break;
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                if (s_cfg_ack) {
                    ESP_LOGI(TAG, "cfg_push: cfg:ok received");
                    break;
                }
                ESP_LOGW(TAG, "cfg_push: TX OK but no cfg:ok within 5s");
            } else {
                ESP_LOGW(TAG, "cfg_push: failed (attempt %d): %s",
                         attempt + 1, esp_err_to_name(err));
            }

            if (attempt < CFG_MAX_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(CFG_RETRY_COOLDOWN_MS));
            }
        }

        s_cfg_pushing = false;
        free(buf);

        if (s_cfg_ack) {
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            ESP_LOGE(TAG, "cfg_push: NODE did not confirm cfg after %d attempts",
                     CFG_MAX_ATTEMPTS);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

/* ============================================================
   root_heartbeat_task
   ============================================================ */
static void root_heartbeat_task(void *arg)
{
    ESP_LOGI(TAG, "root_heartbeat_task started (periodic ping every %d ms)",
             HEARTBEAT_INTERVAL_MS);
    vTaskDelay(pdMS_TO_TICKS(CFG_INITIAL_DELAY_MS));
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
        if (g_config.role != DEVICE_ROLE_MASTER) continue;
        if (!transport_is_lora()) continue;
        if (s_cfg_pushing) continue;
        esp_err_t err = transport_send_cmd("ping", 4);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ROOT: heartbeat ping failed: %s", esp_err_to_name(err));
        }
    }
}

void root_wd_start_tasks(void)
{
    ESP_LOGI(TAG, "Starting ROOT watchdog tasks (SYNC-MASK, lead=%d ms, "
                  "NO-DOUBLE-SYNC)...", SYNC_LEAD_MS);
    xTaskCreate(phase_sync_task,          "phase_sync",    4096, NULL, 4, NULL);
    xTaskCreate(root_node_watchdog_task,  "root_node_wd",  8192, NULL, 3, NULL);
    xTaskCreate(cfg_push_task,            "cfg_push",      8192, NULL, 3, NULL);
    xTaskCreate(root_heartbeat_task,      "root_hb",       3072, NULL, 3, NULL);
}