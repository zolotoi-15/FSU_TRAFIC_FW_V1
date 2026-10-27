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

/* Extern через app_shared.h */
pi4ioe5v6408_config_t expander_a = {
    .device_addr = 0x43, .clk_speed = 100000, .initialized = false,
};
pi4ioe5v6408_config_t expander_b = {
    .device_addr = 0x44, .clk_speed = 100000, .initialized = false,
};

typedef struct { uint8_t expander; uint8_t pin; uint8_t channel; } diag_en_map_t;
static const diag_en_map_t s_diag_en_map[] = {
    {0,0,11},{0,1,10},{0,2,7},{0,3,1},{0,4,2},{0,5,8},{0,6,5},{0,7,4},
    {1,1,9},{1,2,6},{1,3,3},{1,4,0},
};
#define DIAG_EN_MAP_SIZE (sizeof(s_diag_en_map)/sizeof(s_diag_en_map[0]))

esp_err_t board_diag_en_set(uint8_t channel, bool enable)
{
    for (size_t i = 0; i < DIAG_EN_MAP_SIZE; i++) {
        const diag_en_map_t *m = &s_diag_en_map[i];
        if (m->channel != channel) continue;
        pi4ioe5v6408_config_t *exp = (m->expander == 0) ? &expander_a : &expander_b;
        return pi4ioe5v6408_set_pin_output(exp, m->pin, enable);
    }
    return ESP_ERR_NOT_FOUND;
}

void board_diag_en_all_off(void)
{
    for (size_t i = 0; i < DIAG_EN_MAP_SIZE; i++) {
        const diag_en_map_t *m = &s_diag_en_map[i];
        pi4ioe5v6408_config_t *exp = (m->expander == 0) ? &expander_a : &expander_b;
        pi4ioe5v6408_set_pin_output(exp, m->pin, false);
    }
}

static void configure_diag_en_outputs(void)
{
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

static imon_state_t classify_imon(int mv)
{
    if (mv < 0) return IMON_STATE_UNKNOWN;
    if (mv < g_config.traffic.fault.imin_mv) return IMON_STATE_NO_DIAG;
    if (mv >= IMON_TH_FAULT_MV)   return IMON_STATE_FAULT;
    if (mv >= IMON_TH_SUSPECT_MV) return IMON_STATE_SUSPECT;
    return IMON_STATE_OK;
}

int board_read_iload_raw(uint8_t channel, imon_state_t *out_state, int *out_mv)
{
    if (out_state) *out_state = IMON_STATE_UNKNOWN;
    if (out_mv)    *out_mv    = 0;
    if (!sr_get_output(channel)) {
        if (out_state) *out_state = IMON_STATE_OK;
        return 0;
    }
    esp_rom_delay_us(20);
    esp_err_t ret = board_diag_en_set(channel, true);
    if (ret == ESP_ERR_NOT_FOUND) {
        if (out_state) *out_state = IMON_STATE_NO_DIAG;
        return 0;
    }
    if (ret != ESP_OK) return -1;
    esp_rom_delay_us(50);

    SemaphoreHandle_t mux_mtx = mux_get_mutex();
    if (mux_mtx) xSemaphoreTake(mux_mtx, portMAX_DELAY);
    mux_select_channel(channel);
    vTaskDelay(pdMS_TO_TICKS(5));
    int mv = mux_read_sig2_avg_mv(8);
    if (mux_mtx) xSemaphoreGive(mux_mtx);

    if (mv < 0) return -1;
    imon_state_t st = classify_imon(mv);
    if (out_state) *out_state = st;
    if (out_mv)    *out_mv    = mv;
    if (st == IMON_STATE_FAULT)   return -2;
    if (st == IMON_STATE_SUSPECT) return -3;
    if (st == IMON_STATE_NO_DIAG) return 0;
    if (st == IMON_STATE_UNKNOWN) return -1;
    return (int)(((float)mv * TPS27S100B_K_IMON) / TPS27S100B_R_IMON);
}

esp_err_t board_hw_init(void)
{
    if (i2c_bus_init() != ESP_OK) return ESP_FAIL;
    i2c_bus_scan();
    i2c_dump_all();

    pi4ioe5v6408_init_with_bus(&expander_a, g_i2c_bus);
    pi4ioe5v6408_init_with_bus(&expander_b, g_i2c_bus);
    pi4ioe5v6408_set_pin_direction(&expander_a, 0, false);
    pi4ioe5v6408_set_pin_output(&expander_a, 0, true);
    configure_diag_en_outputs();

    /* P7 input */
    pi4ioe5v6408_set_pin_direction(&expander_b, 7, true);
    pi4ioe5v6408_set_pin_pull(&expander_b, 7, 1, true);
    pi4ioe5v6408_set_pin_high_z(&expander_b, 7, false);

    bool init_state = false;
    if (pi4ioe5v6408_get_pin_input(&expander_b, 7, &init_state) == ESP_OK) {
        ESP_LOGI(TAG, "Button P7 initial state: %d", (int)init_state);
    }

    return ESP_OK;
}

void board_diag_en_self_test(void)
{
    const uint8_t test_ch = 5;
    board_diag_en_all_off();
    vTaskDelay(pdMS_TO_TICKS(10));
    mux_select_channel(test_ch);
    vTaskDelay(pdMS_TO_TICKS(5));
    int mv_low = mux_read_sig2_avg_mv(8);
    if (board_diag_en_set(test_ch, true) != ESP_OK) return;
    vTaskDelay(pdMS_TO_TICKS(10));
    mux_select_channel(test_ch);
    vTaskDelay(pdMS_TO_TICKS(5));
    int mv_high = mux_read_sig2_avg_mv(8);
    board_diag_en_set(test_ch, false);
    ESP_LOGI(TAG, "self-test ch%02d: LOW=%d, HIGH=%d", test_ch, mv_low, mv_high);
}