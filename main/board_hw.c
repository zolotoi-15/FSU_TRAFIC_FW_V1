#include "board_hw.h"
#include "app_shared.h"
#include "board_pins.h"
#include "i2c_bus.h"
#include "pi4ioe5v6408.h"
#include "shift_register.h"
#include "mux.h"
#include "mux_data.h"
#include "config_manager.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "board_hw";

pi4ioe5v6408_config_t expander_a = {
    .device_addr = 0x43, .clk_speed = 100000, .initialized = false,
};
pi4ioe5v6408_config_t expander_b = {
    .device_addr = 0x44, .clk_speed = 100000, .initialized = false,
};

/* === FIX I2C-EXPANDER-MUTEX ===
   PI4IOE5V6408 set_pin_output = read-modify-write регистра
   OUTPUT_PORT (2 I2C транзакции). Если две задачи одновременно
   дёргают board_diag_en_set для разных каналов на одном
   экспандере — одна установка теряется.

   === FIX I2C-EXPANDER-MUTEX-P7 ===
   button_p7_task читает INPUT_STATUS expander_b без мьютекса.
   В момент read-modify-write OUTPUT_PORT пара transmit+receive
   для read_register может вернуть мусор → ложное «нажатие P7».
   Экспортируем board_expander_mutex() — buttons.c берёт его
   вокруг p7_read_raw. */
static SemaphoreHandle_t s_diag_en_mtx = NULL;

SemaphoreHandle_t board_expander_mutex(void) { return s_diag_en_mtx; }
/* === END FIX I2C-EXPANDER-MUTEX-P7 === */

typedef struct { uint8_t expander; uint8_t pin; uint8_t channel; } diag_en_map_t;
static const diag_en_map_t s_diag_en_map[] = {
    {0,0,11},{0,1,10},{0,2,7},{0,3,1},{0,4,2},{0,5,8},{0,6,5},{0,7,4},
    {1,1,9},{1,2,6},{1,3,3},{1,4,0},
};
#define DIAG_EN_MAP_SIZE (sizeof(s_diag_en_map)/sizeof(s_diag_en_map[0]))

esp_err_t board_diag_en_set(uint8_t channel, bool enable) {
    for (size_t i = 0; i < DIAG_EN_MAP_SIZE; i++) {
        const diag_en_map_t *m = &s_diag_en_map[i];
        if (m->channel != channel) continue;
        pi4ioe5v6408_config_t *exp = (m->expander == 0) ? &expander_a : &expander_b;

        if (s_diag_en_mtx) xSemaphoreTake(s_diag_en_mtx, portMAX_DELAY);
        esp_err_t ret = pi4ioe5v6408_set_pin_output(exp, m->pin, enable);
        if (s_diag_en_mtx) xSemaphoreGive(s_diag_en_mtx);
        return ret;
    }
    return ESP_ERR_NOT_FOUND;
}

void board_diag_en_all_off(void) {
    for (size_t i = 0; i < DIAG_EN_MAP_SIZE; i++) {
        const diag_en_map_t *m = &s_diag_en_map[i];
        pi4ioe5v6408_config_t *exp = (m->expander == 0) ? &expander_a : &expander_b;

        if (s_diag_en_mtx) xSemaphoreTake(s_diag_en_mtx, portMAX_DELAY);
        pi4ioe5v6408_set_pin_output(exp, m->pin, false);
        if (s_diag_en_mtx) xSemaphoreGive(s_diag_en_mtx);
    }
}

static void configure_diag_en_outputs(void) {
    bool used_a[8] = {0}, used_b[8] = {0};
    for (size_t i = 0; i < DIAG_EN_MAP_SIZE; i++) {
        const diag_en_map_t *m = &s_diag_en_map[i];
        if (m->expander == 0) {
            if (used_a[m->pin]) continue;
            used_a[m->pin] = true;
            pi4ioe5v6408_set_pin_direction(&expander_a, m->pin, false);
            pi4ioe5v6408_set_pin_output(&expander_a, m->pin, false);
        } else {
            if (used_b[m->pin]) continue;
            used_b[m->pin] = true;
            pi4ioe5v6408_set_pin_direction(&expander_b, m->pin, false);
            pi4ioe5v6408_set_pin_output(&expander_b, m->pin, false);
        }
    }
}

static imon_state_t classify_imon(int mv) {
    return imon_classify(mv, g_config.traffic.fault.imin_mv);
}

int board_read_iload_raw(uint8_t channel, imon_state_t *out_state, int *out_mv) {
    if (out_state) *out_state = IMON_STATE_UNKNOWN;
    if (out_mv)    *out_mv    = 0;
    if (channel >= MUX_CHANNEL_COUNT) return -1;
    if (!sr_get_output(channel)) {
        if (out_state) *out_state = IMON_STATE_OK;
        return 0;
    }

    esp_err_t ret = board_diag_en_set(channel, true);
    if (ret == ESP_ERR_NOT_FOUND) { if (out_state) *out_state = IMON_STATE_NO_DIAG; return 0; }
    if (ret != ESP_OK) return -1;
    esp_rom_delay_us(200);

    mux_select_channel(channel);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* V_IN проверяется ДО V_IMON (UVLO) */
    int mv_vin = mux_read_sig1_avg_mv(4);
    if (mv_vin < 0) {
        board_diag_en_set(channel, false);
        if (out_state) *out_state = IMON_STATE_UNKNOWN;
        return -1;
    }
    float v_in = (float)mv_vin * MUX1_DIVIDER_RATIO / 1000.0f;
    if (v_in < g_config.traffic.fault.vin_on_min_v) {
        board_diag_en_set(channel, false);
        if (out_state) *out_state = IMON_STATE_UNKNOWN;
        ESP_LOGW(TAG, "ch%02d: V_IN=%.2f V < %.2f V (UVLO), skip IMON",
                 channel, v_in, g_config.traffic.fault.vin_on_min_v);
        return -1;
    }

    int mv = mux_read_sig2_avg_mv(8);
    board_diag_en_set(channel, false);

    if (mv < 0) return -1;
    imon_state_t st = classify_imon(mv);
    if (out_state) *out_state = st;
    if (out_mv)    *out_mv    = mv;
    if (st == IMON_STATE_FAULT)   return 0;
    if (st == IMON_STATE_SUSPECT) return 0;
    if (st == IMON_STATE_NO_DIAG) return 0;
    if (st == IMON_STATE_UNKNOWN) return 0;
    int offset = mux_get_imon_offset_mv(channel);
    int corr = mv - offset;
    if (corr < 0) corr = 0;
    return (int)(((float)corr * TPS27S100B_K_IMON) / TPS27S100B_R_IMON);
}

esp_err_t board_hw_init(void) {
    if (!s_diag_en_mtx) s_diag_en_mtx = xSemaphoreCreateMutex();
    if (!s_diag_en_mtx) return ESP_ERR_NO_MEM;

    if (i2c_bus_init() != ESP_OK) return ESP_FAIL;
    i2c_bus_scan();
    i2c_dump_all();

    esp_err_t ea = pi4ioe5v6408_init_with_bus(&expander_a, g_i2c_bus);
    esp_err_t eb = pi4ioe5v6408_init_with_bus(&expander_b, g_i2c_bus);
    if (ea != ESP_OK) ESP_LOGE(TAG, "expander A init: %s", esp_err_to_name(ea));
    if (eb != ESP_OK) ESP_LOGE(TAG, "expander B init: %s", esp_err_to_name(eb));

    pi4ioe5v6408_set_pin_direction(&expander_a, 0, false);
    pi4ioe5v6408_set_pin_output(&expander_a, 0, true);
    configure_diag_en_outputs();

    /* P7 expander_b: input, pull-up. Кнопка к GND → нажатие = 0. */
    pi4ioe5v6408_set_pin_direction(&expander_b, 7, true);
    pi4ioe5v6408_set_pin_pull(&expander_b, 7, 1, true);   /* pull-up */
    pi4ioe5v6408_set_pin_high_z(&expander_b, 7, false);

    bool init_state = false;
    if (pi4ioe5v6408_get_pin_input(&expander_b, 7, &init_state) == ESP_OK) {
        ESP_LOGI(TAG, "Button P7 initial state: %d (raw)", (int)init_state);
    }
    return (ea == ESP_OK && eb == ESP_OK) ? ESP_OK : ESP_FAIL;
}

void board_diag_en_self_test(void) {
    const uint8_t test_ch = 5;
    board_diag_en_all_off();
    vTaskDelay(pdMS_TO_TICKS(10));

    SemaphoreHandle_t mux_mtx = mux_get_mutex();
    if (mux_mtx) xSemaphoreTake(mux_mtx, portMAX_DELAY);
    mux_select_channel(test_ch);
    vTaskDelay(pdMS_TO_TICKS(5));
    int mv_low = mux_read_sig2_avg_mv(8);
    if (mux_mtx) xSemaphoreGive(mux_mtx);

    if (board_diag_en_set(test_ch, true) != ESP_OK) return;
    vTaskDelay(pdMS_TO_TICKS(10));

    if (mux_mtx) xSemaphoreTake(mux_mtx, portMAX_DELAY);
    mux_select_channel(test_ch);
    vTaskDelay(pdMS_TO_TICKS(5));
    int mv_high = mux_read_sig2_avg_mv(8);
    if (mux_mtx) xSemaphoreGive(mux_mtx);

    board_diag_en_set(test_ch, false);
    ESP_LOGI(TAG, "self-test ch%02d: LOW=%d, HIGH=%d", test_ch, mv_low, mv_high);
}