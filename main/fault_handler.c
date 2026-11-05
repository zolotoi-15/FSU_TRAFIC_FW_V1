/* ============================================================
   fault_handler.c — зеркалирование fault между ROOT и NODE
   + отложенная регистрация смены фазы (pending sync).

   === FIX P7-sync: on_phase_change НЕ шлёт sync: напрямую
   (был бы вызов под s_mutex из apply_stage_dd → блокировка).
   Вместо этого ставит флаг, который разбирает
   phase_sync_task (root_wd.c) раз в 20 мс.

   === FIX REQUEUE: если phase_sync_task не смог отправить sync:
   (нет времени / LoRa / cfg_push), он возвращает pending-фазу
   через phase_change_requeue(), чтобы не потерять.
   ============================================================ */

#include "fault_handler.h"
#include "app_shared.h"
#include "traffic_light.h"
#include "transport.h"
#include "wifi_mqtt.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "fault_handler";

extern volatile uint32_t s_last_node_msg_ms;
extern volatile uint32_t s_last_fault_clear_ms;

#define SYNC_REMOTE_FLAG() do { s_fault_from_remote = s_fault_from_remote_ex; } while (0)
#define SET_REMOTE_FLAG(v) do { s_fault_from_remote = (v); s_fault_from_remote_ex = (v); } while (0)

static volatile bool s_fault_from_remote = false;

/* === pending sync: ставится on_phase_change, разбирается root_wd === */
static volatile uint8_t  s_pending_sync_phase = 0xFF;
static volatile uint32_t s_pending_sync_dur   = 0;

uint8_t phase_change_take_pending(uint32_t *dur)
{
    uint8_t p = s_pending_sync_phase;
    if (p == 0xFF) return 0xFF;
    s_pending_sync_phase = 0xFF;
    if (dur) *dur = s_pending_sync_dur;
    return p;
}

/* === FIX REQUEUE: вернуть pending-фазу в очередь при ошибке send_sync === */
void phase_change_requeue(uint8_t phase, uint32_t dur_ms)
{
    if (phase == 0xFF) return;
    /* не перезаписываем, если уже есть более свежая pending */
    if (s_pending_sync_phase == 0xFF) {
        s_pending_sync_phase = phase;
        s_pending_sync_dur   = dur_ms;
        ESP_LOGD(TAG, "phase_change requeue phase=%u dur=%lu",
                 phase, (unsigned long)dur_ms);
    } else {
        ESP_LOGD(TAG, "phase_change requeue skipped (already pending %u)",
                 s_pending_sync_phase);
    }
}

/* === callback при смене фазы (только ROOT, follow_root=false) === */
static void on_phase_change(uint8_t phase, uint32_t dur_ms)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return;
    /* Не блокируем apply_stage_dd — только флаг */
    s_pending_sync_phase = phase;
    s_pending_sync_dur   = dur_ms;
    ESP_LOGD(TAG, "phase_change → pending sync phase=%u dur=%lu",
             phase, (unsigned long)dur_ms);
}

static void on_fault_changed(traffic_fault_t fault)
{
    SYNC_REMOTE_FLAG();
    if (s_fault_from_remote) {
        SET_REMOTE_FLAG(false);
        ESP_LOGI(TAG, "fault changed (from remote) → no echo");
        if (g_config.role == DEVICE_ROLE_MASTER && fault == TRAFFIC_FAULT_NONE) {
            s_last_fault_clear_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
        return;
    }

    if (g_config.role == DEVICE_ROLE_MASTER && fault == TRAFFIC_FAULT_NONE) {
        traffic_fault_t node_fault = wifi_mqtt_get_node_fault();
        if (node_fault != TRAFFIC_FAULT_NONE) {
            uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            uint32_t since  = now_ms - s_last_node_msg_ms;
            if (s_last_node_msg_ms != 0 && since < 5000) {
                ESP_LOGI(TAG, "root fault NONE, but node still in fault (%d, %lu ms ago) → keep",
                         (int)node_fault, (unsigned long)since);
                return;
            }
            ESP_LOGW(TAG, "root fault NONE, node_fault=%d stale (%lu ms) → clear anyway",
                     (int)node_fault, (unsigned long)since);
        }
    }

    if (g_config.role == DEVICE_ROLE_MASTER && fault == TRAFFIC_FAULT_NONE) {
        s_last_fault_clear_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }

    char buf[256];
    size_t n = traffic_light_build_fault_report(buf, sizeof(buf));
    if (n == 0) {
        ESP_LOGW(TAG, "fault changed but report is empty");
        return;
    }

    ESP_LOGW(TAG, "fault changed → '%s'", buf);

    if (g_config.role == DEVICE_ROLE_MASTER) {
        esp_err_t err = transport_send_cmd(buf, n);
        if (err == ESP_OK) {
            ESP_LOGW(TAG, "root fault → node: '%s'", buf);
        } else {
            ESP_LOGW(TAG, "root fault → node FAILED: %s", esp_err_to_name(err));
        }
        wifi_mqtt_publish_node_fault(buf, n);
    } else {
        if (!transport_is_lora()) {
            ESP_LOGW(TAG, "node fault → root: LoRa not ready, skip '%s'", buf);
            return;
        }
        esp_err_t err = transport_send_evt(buf, n);
        if (err == ESP_OK) {
            ESP_LOGW(TAG, "node fault → root: '%s'", buf);
        } else {
            ESP_LOGE(TAG, "node fault → root: send failed '%s' (%s)",
                     buf, esp_err_to_name(err));
        }
    }
}

void fault_handler_register(void)
{
    traffic_light_set_fault_notify(on_fault_changed);
    /* === FIX P7-sync: регистрируем on_phase_change (только флаг) === */
    traffic_light_set_phase_change_cb(on_phase_change);
}