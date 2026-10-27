#include "mux_data.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

mux_data_t g_mux_data = {0};
static SemaphoreHandle_t s_mutex = NULL;

const char *imon_state_str(imon_state_t s)
{
    switch (s) {
        case IMON_STATE_OK:      return "OK";
        case IMON_STATE_FAULT:   return "FAULT";
        case IMON_STATE_SUSPECT: return "SUSPECT";
        case IMON_STATE_NO_DIAG: return "NO_DIAG";
        default:                 return "UNKNOWN";
    }
}

void mux_data_init(void)
{
    if (s_mutex != NULL) return;
    s_mutex = xSemaphoreCreateMutex();
}

void mux_data_set(const mux_data_t *d)
{
    if (d == NULL || s_mutex == NULL) return;
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&g_mux_data, d, sizeof(mux_data_t));
        xSemaphoreGive(s_mutex);
    }
}

bool mux_data_get(mux_data_t *out)
{
    if (out == NULL || s_mutex == NULL) return false;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    memcpy(out, &g_mux_data, sizeof(mux_data_t));
    bool valid = g_mux_data.valid;
    xSemaphoreGive(s_mutex);
    return valid;
}