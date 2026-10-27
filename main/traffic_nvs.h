#pragma once

#include "esp_err.h"
#include "traffic_light.h"
#include "config_manager.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Сохранить режим + enabled (для совместимости) */
esp_err_t traffic_nvs_save(traffic_mode_t mode, bool enabled);
void      traffic_nvs_load(traffic_mode_t *mode, bool *enabled);
esp_err_t traffic_nvs_reset(void);

/* DD: сохранить/загрузить полный traffic_config_t в отдельный namespace */
esp_err_t traffic_nvs_save_full(const traffic_config_t *tc);
esp_err_t traffic_nvs_load_full(traffic_config_t *tc);

#ifdef __cplusplus
}
#endif