/* ============================================================
   fault_handler.c — mirror fault между ROOT и NODE.

   === FIX FAULT-FROM-NODE ===
   NODE при ЛОКАЛЬНОЙ аварии (или её устранении):
     - traffic_light_set_fault() → s_fault_notify(on_fault_changed)
     - on_fault_changed: g_config.role != MASTER → 
       transport_send_evt("fault:local:<mode>:<chs>:<reasons>")
   ROOT получает в node_proto.c on_evt_from_node → 
   traffic_light_set_fault(YELLOW_BLINK) → root_push_fault_to_node
   отправит NODE fault:yellow_blink:<mask>.

   === FIX NODE-FOLLOW-ROOT-NO-FAULT-EVT ===
   NODE в follow_root=true НЕ отправляет fault:yellow_blink /
   fault:all_off в ROOT с маской. Раньше он слал
   fault:yellow_blink:0x0000 (маска = 0, если конфиг пустой) —
   ROOT получал пустую маску. Теперь шлёт fault:local:<mode>
   без маски. ROOT сам применит YELLOW_BLINK и пришлёт NODE
   fault:yellow_blink:<mask> (маска по s_mode ROOT).

   === FIX NO-ECHO-ON-ROOT ===
   На ROOT on_fault_changed с s_fault_from_remote=true → return.
   Иначе двойная отправка: и из traffic_light_set_fault, и из
   fault_handler.

   === FIX NO-DOUBLE-FAULT-EVT ===
   on_fault_changed вызывается из traffic_light_set_fault
   (через s_fault_notify). Если авария уже отправлена в ROOT
   (s_fault_notify вызван повторно с тем же режимом) — не
   спамим. Используем статический s_last_sent_mode.
   ============================================================ */

#include "fault_handler.h"
#include "app_shared.h"
#include "traffic_light.h"
#include "traffic_internal.h"
#include "transport.h"
#include "wifi_mqtt.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "fault_handler";

extern volatile uint32_t s_last_node_msg_ms;
extern volatile uint32_t s_last_fault_clear_ms;

#define SYNC_REMOTE_FLAG() do { s_fault_from_remote = s_fault_from_remote_ex; } while (0)
#define SET_REMOTE_FLAG(v) do { s_fault_from_remote = (v); s_fault_from_remote_ex = (v); } while (0)

static volatile bool s_fault_from_remote = false;

/* === FIX NO-DOUBLE-FAULT-EVT ===
   Защита от повторной отправки того же режима в ROOT. */
static volatile traffic_fault_t s_last_sent_mode = TRAFFIC_FAULT_NONE;
/* === END FIX NO-DOUBLE-FAULT-EVT === */

/* === SIMPLIFIED-NO-CFG ===
   s_pending_sync_phase читается traffic_light.c, но pending-логики
   в NO-CFG нет — всегда 0xFF. */
volatile uint8_t s_pending_sync_phase = 0xFF;

uint8_t phase_change_take_pending(uint32_t *dur)
{
    if (dur) *dur = 0;
    return 0xFF;
}

void phase_change_requeue(uint8_t phase, uint32_t dur_ms)
{
    (void)phase;
    (void)dur_ms;
}

void phase_change_cancel_pending(void)
{
    /* no-op */
}
/* === END SIMPLIFIED-NO-CFG === */

/* ============================================================
   on_fault_changed
   ============================================================ */
static void on_fault_changed(traffic_fault_t fault)
{
    SYNC_REMOTE_FLAG();

    if (s_fault_from_remote) {
        SET_REMOTE_FLAG(false);
        ESP_LOGI(TAG, "fault changed (from remote) -> no echo");
        if (g_config.role == DEVICE_ROLE_MASTER && fault == TRAFFIC_FAULT_NONE) {
            s_last_fault_clear_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
        return;
    }

    /* ============================================================
       ROOT: не отправлять fault в NODE из fault_handler — это
       делает root_push_fault_to_node() (вызывается из
       traffic_light_set_fault). fault_handler на ROOT только
       логирует и обновляет last_fault_clear_ms.
       ============================================================ */
    if (g_config.role == DEVICE_ROLE_MASTER) {
        if (fault == TRAFFIC_FAULT_NONE) {
            s_last_fault_clear_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
        char buf[256];
        size_t n = traffic_light_build_fault_report(buf, sizeof(buf));
        if (n > 0) {
            ESP_LOGW(TAG, "ROOT fault changed -> '%s' (NODE notified by "
                          "root_push_fault_to_node)", buf);
        }
        if (n > 0) wifi_mqtt_publish_node_fault(buf, n);
        return;
    }

    /* ============================================================
       === FIX NODE-FOLLOW-ROOT-NO-FAULT-EVT ===
       NODE: ЛОКАЛЬНАЯ авария.
       В follow_root=true NODE шлёт в ROOT ТОЛЬКО уведомление
       fault:local:<mode> — БЕЗ маски и без каналов. ROOT сам
       применит YELLOW_BLINK и пришлёт NODE fault:yellow_blink:<mask>
       (маска по s_mode ROOT — корректная).

       Раньше NODE слал fault:yellow_blink:<mask> с маской из
       своего конфига. Если конфиг NVS на NODE пустой (yellow=0),
       маска была 0x0000 — ROOT получал пустую маску.
       ============================================================ */
    if (s_follow_root) {
        /* Не спамим одинаковыми уведомлениями */
        if (fault == s_last_sent_mode) {
            ESP_LOGD(TAG, "node fault (follow_root) %d -> no change",
                     (int)fault);
            return;
        }
        s_last_sent_mode = fault;

        char buf[64];
        const char *mode_str = traffic_light_fault_str();
        int n = snprintf(buf, sizeof(buf), "fault:local:%s", mode_str);
        if (n <= 0) return;

        if (!transport_is_lora()) {
            ESP_LOGW(TAG, "node fault (follow_root) -> root: LoRa not ready, "
                          "skip '%s'", buf);
            return;
        }
        esp_err_t err = transport_send_evt(buf, n);
        if (err == ESP_OK) {
            ESP_LOGW(TAG, "node fault (follow_root) -> root: '%s'", buf);
        } else {
            ESP_LOGE(TAG, "node fault (follow_root) -> root: send failed "
                          "'%s' (%s)", buf, esp_err_to_name(err));
        }
        return;
    }
    /* === END FIX NODE-FOLLOW-ROOT-NO-FAULT-EVT === */

    /* ============================================================
       NODE в follow_root=false (не должно происходить на NODE,
       но на всякий случай — отправляем короткое уведомление).
       ============================================================ */
    if (fault == TRAFFIC_FAULT_NONE) {
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t since  = now_ms - s_last_node_msg_ms;
        if (s_last_node_msg_ms != 0 && since < 5000) {
            ESP_LOGI(TAG, "node fault NONE, ROOT recent (%lu ms) -> send anyway",
                     (unsigned long)since);
        }
    }

    char buf[64];
    const char *mode_str = traffic_light_fault_str();
    int n = snprintf(buf, sizeof(buf), "fault:local:%s", mode_str);
    if (n > 0 && transport_is_lora()) {
        transport_send_evt(buf, n);
        ESP_LOGW(TAG, "node fault (no follow_root) -> root: '%s'", buf);
    }
}

/* ============================================================
   fault_handler_register
   ============================================================ */
void fault_handler_register(void)
{
    s_last_sent_mode = TRAFFIC_FAULT_NONE;
    traffic_light_set_fault_notify(on_fault_changed);
    traffic_light_set_phase_change_cb(NULL);
    ESP_LOGI(TAG, "registered (fault_notify only, phase_change_cb=NULL, "
                  "s_pending_sync_phase=0xFF, NODE follow_root → fault:local)");
}