#pragma once
#include "traffic_light.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void mqtt_fault_apply_remote(const char *fault_str, size_t len);
void mqtt_fault_publish_node(const char *fault_str, size_t len);
void mqtt_fault_publish_node_json(const char *json, size_t len);
void mqtt_fault_reset_tracking(void);
int  mqtt_fault_priority(traffic_fault_t f);
traffic_fault_t mqtt_fault_get_node(void);

/* === FIX T_CHANNELS ===
   ROOT: публикация снимка 16 каналов NODE в <base>/Node/Channels. */
void mqtt_fault_publish_node_channels(const char *json, size_t len);
/* === END FIX T_CHANNELS === */

#ifdef __cplusplus
}
#endif