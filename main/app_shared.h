#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "board_pins.h"
#include "traffic_light.h"
#include "pi4ioe5v6408.h"
#include "wifi_ap.h"

#ifdef __cplusplus
extern "C" {
#endif

extern pi4ioe5v6408_config_t expander_a;
extern pi4ioe5v6408_config_t expander_b;

extern volatile bool s_fault_from_remote_ex;

int32_t  blink_get_rtt_samples(void);
uint32_t blink_get_rtt_us(void);
uint32_t blink_get_rtt_delay_us(void);
void     blink_reset_rtt(void);

extern volatile uint32_t s_last_sync_ms;
extern volatile bool     s_comm_lost;
extern volatile uint32_t s_last_pong_ms;

esp_err_t board_diag_en_set(uint8_t channel, bool enable);
void      board_diag_en_all_off(void);
int       board_read_iload_raw(uint8_t channel,
                               imon_state_t *out_state,
                               int *out_mv);

/* ============================================================
   === LINK LOSS ===
   Связь считается потерянной, если LINK_LOST_MISSED_PKTS
   ожидаемых пакетов подряд не пришли.

   ROOT ждёт STATUS от NODE (период NODE_STATUS_PERIOD_MS = 10 с)
   NODE ждёт CMD/ping/sync/time от ROOT (период HEARTBEAT = 10 с)

   Счётчики инкрементит link_monitor_task (blink_example_main.c)
   раз в LINK_*_EXPECT_MS (15 с = 1.5× период). Сбрасываются при
   приёме пакета:
     ROOT: on_node_status_from_transport → link_monitor_note_status_rx()
     NODE: on_cmd_from_root            → link_monitor_note_ping_rx()
   ============================================================ */
#define LINK_LOST_MISSED_PKTS   5

extern volatile uint8_t s_root_missed_status;  /* ROOT: пропущено STATUS */
extern volatile uint8_t s_node_missed_ping;    /* NODE: пропущено CMD/ping */

void link_monitor_note_status_rx(void);        /* ROOT: STATUS от NODE */
void link_monitor_note_ping_rx(void);          /* NODE: CMD от ROOT */

bool link_monitor_root_lost(void);             /* s_root_missed_status >= 5 */
bool link_monitor_node_lost(void);             /* s_node_missed_ping   >= 5 */
/* === END LINK LOSS === */

/* ============================================================
   === FIX P7-sync: pending phase из fault_handler.c ===
   on_phase_change() только ставит флаг, phase_sync_task (root_wd.c)
   его разбирает → send_sync_to_node. Так apply_stage_dd
   не блокируется на 750 мс (SYNC_LEAD_MS) под s_mutex.

   === FIX REQUEUE: phase_sync_task при ошибке send_sync возвращает
   pending-фазу через phase_change_requeue(), чтобы не потерять.
   ============================================================ */
uint8_t phase_change_take_pending(uint32_t *dur);
void    phase_change_requeue(uint8_t phase, uint32_t dur_ms);
/* === END FIX P7-sync === */

#ifdef __cplusplus
}
#endif