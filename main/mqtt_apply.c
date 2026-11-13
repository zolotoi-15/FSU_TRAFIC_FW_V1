/* ============================================================
   mqtt_apply.c — применение конфига к runtime.

   === FIX SYNC-LEAD-CONSISTENT ===
   === FIX LEAD-MS-RUNTIME ===
       Задержка ROOT между отправкой c: и локальным
       apply_stage_dd читается через sync_config_get_lead_ms()
       (runtime, g_sync_lead_ms, дефолт 150 мс).

   === FIX STOP-CLEARS-FAULT ===
       SET_REMOTE_FLAG — локальный макрос root_wd.c/node_proto.c.
       В mqtt_apply.c его нет. Присваиваем s_fault_from_remote_ex
       напрямую (extern в app_shared.h).

       s_last_sync_ms объявлен extern в app_shared.h и определён
       в blink_example_main.c. Здесь НЕ переопределяем — вводим
       локальный s_last_sync_sent_ms для троттлинга sync: на ROOT
       (глобальная s_last_sync_ms — это «время последнего ping/CMD
       на NODE», её использует link_monitor_task).

   === FIX ALL-OFF-IS-NOT-FAULT ===
       КЛЮЧЕВАЯ ПРАВКА.
       Раньше на старте (ROOT и NODE) вызывался
         traffic_light_set_fault(TRAFFIC_FAULT_ALL_OFF)
       Это ставило s_fault = ALL_OFF → traffic_light_fault_has_priority()
       = true → send_channels_to_node() блокировался, ROOT не мог
       послать c: на NODE. NODE в свою очередь тоже в ALL_OFF
       (получил fault:all_off от ROOT). Замкнутый круг.

       ALL_OFF на старте — это standby, НЕ авария.

   === FIX PED-DISPLAY-MQTT ===
       .ped_display обязательно копируется, иначе C99 зануляет
       поле, и ped_display = 0 → «ЖДИТЕ» включается на ch00.

   === FIX NODE-BLINK-AND-ON ===
       mqtt_apply_config_runtime() НЕ вызывает traffic_light_start().
       ROOT стартует через traffic_light_start_with_red() из
       root_node_watchdog_task после NODE-ack. NODE — через
       traffic:start от ROOT.
   ============================================================ */

#include "mqtt_internal.h"
#include "app_shared.h"          /* s_fault_from_remote_ex, s_last_sync_ms */
#include "config_manager.h"
#include "traffic_light.h"
#include "transport.h"
#include "root_wd.h"
#include "sync_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>

static const char *TAG = "mqtt_apply";

/* === FIX STOP-CLEARS-FAULT ===
   Локальный троттлинг sync: на ROOT. НЕ путать с глобальной
   s_last_sync_ms из app_shared.h (та — «время последнего
   ping/CMD на NODE», обновляется в link_monitor_note_ping_rx
   и используется link_monitor_task). */
static volatile uint8_t  s_last_sync_phase    = 0xFF;
static volatile uint32_t s_last_sync_sent_ms  = 0;
/* === END FIX STOP-CLEARS-FAULT === */

/* ============================================================
   mqtt_apply_config_runtime
   ============================================================ */
void mqtt_apply_config_runtime(void)
{
    /* --- 1. Применяем пины к runtime из g_config --- */
    traffic_pins_single_t ps = {
        .pin_red    = g_config.traffic.pins.single.red,
        .pin_yellow = g_config.traffic.pins.single.yellow,
        .pin_green  = g_config.traffic.pins.single.green,
    };
    traffic_light_set_pins_single(&ps);

    traffic_pins_cross_t pc = {
        .a_north_red    = g_config.traffic.pins.cross.a_north.red,
        .a_north_yellow = g_config.traffic.pins.cross.a_north.yellow,
        .a_north_green  = g_config.traffic.pins.cross.a_north.green,
        .a_south_red    = g_config.traffic.pins.cross.a_south.red,
        .a_south_yellow = g_config.traffic.pins.cross.a_south.yellow,
        .a_south_green  = g_config.traffic.pins.cross.a_south.green,
        .b_west_red     = g_config.traffic.pins.cross.b_west.red,
        .b_west_yellow  = g_config.traffic.pins.cross.b_west.yellow,
        .b_west_green   = g_config.traffic.pins.cross.b_west.green,
        .b_east_red     = g_config.traffic.pins.cross.b_east.red,
        .b_east_yellow  = g_config.traffic.pins.cross.b_east.yellow,
        .b_east_green   = g_config.traffic.pins.cross.b_east.green,
    };
    traffic_light_set_pins_cross(&pc);

    /* === FIX PED-DISPLAY-MQTT === */
    traffic_pins_cross_ped_t pp = {
        .axis_a_red     = g_config.traffic.pins.cross_ped.axis_a.red,
        .axis_a_yellow  = g_config.traffic.pins.cross_ped.axis_a.yellow,
        .axis_a_green   = g_config.traffic.pins.cross_ped.axis_a.green,
        .axis_a2_red    = g_config.traffic.pins.cross_ped.axis_a2.red,
        .axis_a2_yellow = g_config.traffic.pins.cross_ped.axis_a2.yellow,
        .axis_a2_green  = g_config.traffic.pins.cross_ped.axis_a2.green,
        .axis_a3_red    = g_config.traffic.pins.cross_ped.axis_a3.red,
        .axis_a3_yellow = g_config.traffic.pins.cross_ped.axis_a3.yellow,
        .axis_a3_green  = g_config.traffic.pins.cross_ped.axis_a3.green,
        .ped_red        = g_config.traffic.pins.cross_ped.ped_red,
        .ped_green      = g_config.traffic.pins.cross_ped.ped_green,
        .ped_display    = g_config.traffic.pins.cross_ped.ped_display,
    };
    traffic_light_set_pins_cross_ped(&pp);
    /* === END FIX PED-DISPLAY-MQTT === */

    /* --- 2. Останавливаем трафик --- */
    /* === FIX STOP-CLEARS-FAULT ===
       Перед stop() явно снимаем возможный fault. Если оставить
       s_fault = ALL_OFF, traffic_light_stop() не тронет
       fault-каналы. Присваиваем s_fault_from_remote_ex напрямую
       (без SET_REMOTE_FLAG), чтобы fault_handler не отправил
       эхо fault:none в NODE через root_push_fault_to_node. */
    s_fault_from_remote_ex = true;
    traffic_light_set_fault(TRAFFIC_FAULT_NONE);
    s_fault_from_remote_ex = false;
    /* === END FIX STOP-CLEARS-FAULT === */

    traffic_light_stop();

    esp_err_t mr = traffic_light_set_mode((traffic_mode_t)g_config.traffic.mode);
    if (mr != ESP_OK) {
        ESP_LOGW(TAG, "set_mode: %s", esp_err_to_name(mr));
    }
    traffic_light_reset();

    /* --- 3. Стартовое состояние --- */
    if (!g_config.traffic.enabled || !g_config.traffic.auto_start) {
        ESP_LOGI(TAG, "auto_start disabled → traffic STOPPED (channels OFF)");
        return;
    }

    /* === FIX ALL-OFF-IS-NOT-FAULT ===
       ROOT и NODE НЕ ставят TRAFFIC_FAULT_ALL_OFF. Это standby,
       не авария. Каналы уже OFF после stop()+reset(). */
    if (g_config.role == DEVICE_ROLE_MASTER) {
        ESP_LOGI(TAG, "ROOT: STOPPED + channels OFF at start "
                      "(wait for NODE ack → start_with_red)");
        return;
    } else {
        ESP_LOGI(TAG, "NODE: STOPPED + channels OFF at start "
                      "(wait for CFG + traffic:start from ROOT)");
        return;
    }
    /* === END FIX ALL-OFF-IS-NOT-FAULT === */
}

/* ============================================================
   mqtt_notify_phase — sync: из MQTT (только ROOT).
   === FIX LEAD-MS-RUNTIME ===
   === FIX STOP-CLEARS-FAULT (s_last_sync_sent_ms) ===
   ============================================================ */
void mqtt_notify_phase(uint8_t phase, uint32_t phase_ms, uint32_t duration_ms)
{
    if (g_config.role != DEVICE_ROLE_MASTER) return;
    if (!transport_is_lora()) return;
    if (s_cfg_pushing) return;
    if (!s_cfg_ack) return;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    if (phase == s_last_sync_phase &&
        (now_ms - s_last_sync_sent_ms) < SYNC_MIN_INTERVAL_MS) {
        return;
    }
    s_last_sync_phase    = phase;
    s_last_sync_sent_ms  = now_ms;

    /* === FIX LEAD-MS-RUNTIME === */
    uint32_t apply_ms = now_ms + sync_config_get_lead_ms();
    /* === END FIX LEAD-MS-RUNTIME === */

    char cmd[128];
    int n = snprintf(cmd, sizeof(cmd), "sync:%u:%lu:%lu:%lu:%lu",
                     phase,
                     (unsigned long)phase_ms,
                     (unsigned long)duration_ms,
                     (unsigned long)now_ms,
                     (unsigned long)apply_ms);
    if (n <= 0) return;

    esp_err_t err = transport_send_cmd(cmd, n);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sync TX failed: %s", esp_err_to_name(err));
    }
}