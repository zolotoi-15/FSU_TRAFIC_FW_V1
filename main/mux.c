#include "mux.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "mux";

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t         s_cali_handle = NULL;
static bool                      s_cali_ok = false;

/* FIX #2: один мьютекс защищает и выбор канала мультиплексора,
   и ADC-чтения. Раньше был только s_adc_mutex на adc_read_oneshot,
   но mux_select_channel был вне критической секции → mux_task
   мог переключить канал между выбором и чтением ADC. */
static SemaphoreHandle_t         s_mux_mutex = NULL;

static adc_channel_t s_ch1 = ADC_CHANNEL_4;
static adc_channel_t s_ch2 = ADC_CHANNEL_3;

static int s_imon_offset_mv[MUX_CHANNEL_COUNT] = {0};

static bool mux_cali_init(void)
{
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration: Curve Fitting");
        return true;
    }
    ESP_LOGW(TAG, "Curve Fitting calibration not available: %s", esp_err_to_name(ret));
    return false;
}

/* FIX #2: экспорт мьютекса для board_read_iload_raw / mux_task. */
SemaphoreHandle_t mux_get_mutex(void)
{
    return s_mux_mutex;
}

esp_err_t mux_init(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << MUX_S0) | (1ULL << MUX_S1) |
                        (1ULL << MUX_S2) | (1ULL << MUX_S3) |
                        (1ULL << MUX_EN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));

    gpio_set_level(MUX_EN2, 0);

    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, s_ch1, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, s_ch2, &chan_cfg));

    s_cali_ok = mux_cali_init();

    s_mux_mutex = xSemaphoreCreateMutex();
    if (!s_mux_mutex) {
        ESP_LOGE(TAG, "Failed to create mux mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "MUX initialized: S0=%d S1=%d S2=%d S3=%d EN2=%d",
             MUX_S0, MUX_S1, MUX_S2, MUX_S3, MUX_EN2);
    return ESP_OK;
}

/* FIX #2: adc_read_oneshot больше НЕ берёт мьютекс сам —
   вызывающий код обязан держать mux_get_mutex() вокруг
   «mux_select_channel + mux_read_*». */
static int adc_read_oneshot_unlocked(adc_channel_t ch)
{
    if (!s_adc_handle) return -1;
    int v = 0;
    esp_err_t err = adc_oneshot_read(s_adc_handle, ch, &v);
    return (err == ESP_OK) ? v : -1;
}

void mux_enable_mux2(bool enable)
{
    gpio_set_level(MUX_EN2, enable ? 0 : 1);
}

/* FIX #2: mux_select_channel БЕЗ мьютекса — вызывающий держит. */
void mux_select_channel(uint8_t channel)
{
    if (channel > 15) channel = 15;
    gpio_set_level(MUX_S0, (channel >> 0) & 1);
    gpio_set_level(MUX_S1, (channel >> 1) & 1);
    gpio_set_level(MUX_S2, (channel >> 2) & 1);
    gpio_set_level(MUX_S3, (channel >> 3) & 1);
    esp_rom_delay_us(MUX_SETTLE_DELAY_US);
}

void mux_calibrate_imon_offset(int samples)
{
    if (samples <= 0) samples = MUX_DEFAULT_AVG_SAMPLES;
    ESP_LOGI(TAG, "=== IMON OFFSET CALIBRATION ===");
    for (uint8_t ch = 0; ch < MUX_CHANNEL_COUNT; ch++) {
        if (s_mux_mutex) xSemaphoreTake(s_mux_mutex, portMAX_DELAY);
        mux_select_channel(ch);
        vTaskDelay(pdMS_TO_TICKS(10));
        int mv = mux_read_sig2_avg_mv(samples);
        if (s_mux_mutex) xSemaphoreGive(s_mux_mutex);
        if (mv < 0) mv = 0;
        s_imon_offset_mv[ch] = mv;
        ESP_LOGI(TAG, "%02d  | %4d mV", ch, mv);
    }
    ESP_LOGI(TAG, "=== END ===");
}

int mux_get_imon_offset_mv(uint8_t channel)
{
    if (channel >= MUX_CHANNEL_COUNT) return 0;
    return s_imon_offset_mv[channel];
}

void mux_set_imon_offset_mv(uint8_t channel, int offset_mv)
{
    if (channel >= MUX_CHANNEL_COUNT) return;
    s_imon_offset_mv[channel] = offset_mv;
}

int mux_read_sig1(void) { return adc_read_oneshot_unlocked(s_ch1); }
int mux_read_sig2(void) { return adc_read_oneshot_unlocked(s_ch2); }

void mux_read_both(int *sig1, int *sig2)
{
    if (sig1) *sig1 = adc_read_oneshot_unlocked(s_ch1);
    if (sig2) *sig2 = adc_read_oneshot_unlocked(s_ch2);
}

int mux_read_sig1_avg(int n)
{
    if (n <= 0) n = MUX_DEFAULT_AVG_SAMPLES;
    int32_t sum = 0;
    int count = 0;
    for (int i = 0; i < n; i++) {
        int v = adc_read_oneshot_unlocked(s_ch1);
        if (v >= 0) { sum += v; count++; }
    }
    return (count > 0) ? (int)(sum / count) : -1;
}

int mux_read_sig2_avg(int n)
{
    if (n <= 0) n = 16;
    int32_t sum = 0;
    int count = 0;
    for (int i = 0; i < n; i++) {
        int v = adc_read_oneshot_unlocked(s_ch2);
        if (v >= 0) { sum += v; count++; }
    }
    return (count > 0) ? (int)(sum / count) : -1;
}

void mux_read_both_avg(int *sig1, int *sig2, int n)
{
    if (n <= 0) n = MUX_DEFAULT_AVG_SAMPLES;
    int32_t s1 = 0, s2 = 0;
    int c1 = 0, c2 = 0;
    for (int i = 0; i < n; i++) {
        int v1 = adc_read_oneshot_unlocked(s_ch1);
        int v2 = adc_read_oneshot_unlocked(s_ch2);
        if (v1 >= 0) { s1 += v1; c1++; }
        if (v2 >= 0) { s2 += v2; c2++; }
    }
    if (sig1) *sig1 = (c1 > 0) ? (int)(s1 / c1) : -1;
    if (sig2) *sig2 = (c2 > 0) ? (int)(s2 / c2) : -1;
}

static int mux_raw_to_mv(int raw)
{
    if (raw < 0) return -1;
    if (!s_cali_ok || !s_cali_handle) return -1;
    int mv = 0;
    esp_err_t ret = adc_cali_raw_to_voltage(s_cali_handle, raw, &mv);
    if (ret != ESP_OK) return -1;
    return mv;
}

int mux_read_sig1_mv(void) { return mux_raw_to_mv(mux_read_sig1()); }
int mux_read_sig2_mv(void) { return mux_raw_to_mv(mux_read_sig2()); }

void mux_read_both_mv(int *mv1, int *mv2)
{
    int r1 = 0, r2 = 0;
    mux_read_both(&r1, &r2);
    if (mv1) *mv1 = mux_raw_to_mv(r1);
    if (mv2) *mv2 = mux_raw_to_mv(r2);
}

int mux_read_sig1_avg_mv(int n) { return mux_raw_to_mv(mux_read_sig1_avg(n)); }
int mux_read_sig2_avg_mv(int n) { return mux_raw_to_mv(mux_read_sig2_avg(n)); }

void mux_read_both_avg_mv(int *mv1, int *mv2, int n)
{
    int r1 = 0, r2 = 0;
    mux_read_both_avg(&r1, &r2, n);
    if (mv1) *mv1 = mux_raw_to_mv(r1);
    if (mv2) *mv2 = mux_raw_to_mv(r2);
}

int mux_read_vin_mv(int n)
{
    int mv_adc = mux_read_sig1_avg_mv(n);
    if (mv_adc < 0) return -1;
    return (int)((float)mv_adc * MUX1_DIVIDER_RATIO);
}

int mux_read_iload_ma(int n)
{
    int mv = mux_read_sig2_avg_mv(n);
    if (mv < 0) return -1;
    if (mv >= TPS27S100B_IMON_FAULT_MV) return -2;
    return (int)(((float)mv * TPS27S100B_K_IMON) / TPS27S100B_R_IMON);
}

int mux_read_iload_ma_ch(uint8_t channel, int n)
{
    if (channel >= MUX_CHANNEL_COUNT) return -1;
    if (s_mux_mutex) xSemaphoreTake(s_mux_mutex, portMAX_DELAY);
    mux_select_channel(channel);
    vTaskDelay(pdMS_TO_TICKS(5));
    int mv = mux_read_sig2_avg_mv(n);
    if (s_mux_mutex) xSemaphoreGive(s_mux_mutex);
    if (mv < 0) return -1;
    if (mv >= TPS27S100B_IMON_FAULT_MV) return -2;
    int offset = s_imon_offset_mv[channel];
    int corr = mv - offset;
    if (corr < 0) corr = 0;
    return (int)(((float)corr * TPS27S100B_K_IMON) / TPS27S100B_R_IMON);
}

bool mux_imon_is_fault(int mv) { return (mv >= TPS27S100B_IMON_FAULT_MV); }