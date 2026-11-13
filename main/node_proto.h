#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
   node_proto.h — NODE/ROOT протокол.

   === SIMPLIFIED-NO-CFG ===
       NODE больше не загружает конфигурацию. ROOT шлёт готовые
       маски каналов через 'c:<phase>:<pm>:<on_hex>:<blink_hex>:
       <dur>[:<end_utc>[:<apply_utc>]]'.
       Все функции cfg:* удалены/стали stub'ами для совместимости.

   === FIX LORA-LINK-STATS ===
       Качество последнего RX от NODE (на стороне ROOT).
       Заполняется в on_node_status_from_transport при разборе
       LINK из STATUS. Публикуется в mqtt_build_json → LINK.node.

   === FIX T_CHANNELS ===
       NODE раз в NODE_CHANNELS_PERIOD_MS (60 с) шлёт отдельный
       T_CHANNELS со всеми 16 каналами (V_IN, I_LOAD, V_IMON,
       STATE, SR, DIAG). ROOT принимает, публикует в
       <base>/Node/Channels. STATUS остаётся лёгким
       (SR/TRAFFIC/FAULT/LINK).

   === FIX RT-CHANNELS ===
       c: содержит apply_utc_ms — момент UTC, когда ROOT и NODE
       ОДНОВРЕМЕННО переключают каналы. ToA LoRa и джиттер
       фрагментации не влияют на синхронность.
       NODE ставит маску в pending (node_schedule_mask_apply) и
       применяет её в node_mask_scheduler_task ровно в apply_utc_ms.
       Пока pending — node_mask_pending() == true; mux_task и
       traffic_light_check_faults должны пропускать проверку
       каналов, чтобы не поймать ложный SHORT_GND (SR ещё в старой
       маске, ожидаемая — уже новая).

       apply_utc_ms == 0 или поле отсутствует → fallback: применяем
       сразу (refresh c: / старое поведение).
   ============================================================ */

/* --- Инициализация/запуск --- */
void node_proto_register_callbacks(void);
void node_proto_client_start(const char *host, uint16_t port);
void node_proto_start_tasks(void);
void node_proto_start_root_tasks(void);
void node_handle_local_cmd(const char *cmd);

/* === FIX FAST-STATUS === */
void node_send_status_now(void);
void node_send_status_for_ms(uint32_t ms);
/* === END FIX FAST-STATUS === */

/* === SIMPLIFIED-NO-CFG: stub'ы для совместимости ===
   Внешние модули могут проверять «получен ли CFG», но в новой
   архитектуре CFG не передаётся вовсе — NODE всегда «готов». */
bool node_cfg_received(void);
bool node_cfg_confirmed_by_root(void);
/* === END SIMPLIFIED-NO-CFG === */

/* === FIX RT-CHANNELS ===
   TRUE, пока на NODE есть отложенная (pending) маска каналов,
   которая будет применена в apply_utc_ms.
   Используется в mux_task.c и traffic_fault.c (через
   traffic_light_check_faults) для пропуска проверки каналов:
   иначе за 150 мс до применения новой маски SR ещё в старой,
   а s_expected_mask_from_root уже новая → ложный SHORT_GND.

   На ROOT всегда false (ROOT не планирует маску — он её
   применяет сам через root_apply_stage_sync_first). */
bool node_mask_pending(void);
/* === END FIX RT-CHANNELS === */

/* === FIX LORA-LINK-STATS ===
   Качество последнего RX от NODE (на стороне ROOT).
   Заполняется в on_node_status_from_transport при разборе LINK
   из STATUS (rssi/snr/sig_rssi/age_ms на стороне NODE).
   Публикуется в mqtt_build_json → LINK.node. */
extern volatile float    s_node_link_rssi;
extern volatile float    s_node_link_snr;
extern volatile float    s_node_link_sig_rssi;
extern volatile uint32_t s_node_link_age_ms;
extern volatile bool     s_node_link_valid;
/* === END FIX LORA-LINK-STATS === */

/* === FIX T_CHANNELS ===
   ROOT: последний снимок каналов NODE (заполняется при приёме
   T_CHANNELS, публикуется в <base>/Node/Channels). */
extern char s_node_channels_json[2048];
extern volatile bool s_node_channels_valid;
/* === END FIX T_CHANNELS === */

#ifdef __cplusplus
}
#endif