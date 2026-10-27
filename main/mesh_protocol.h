#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
   Mesh-протокол между root (master) и node (slave).
   Транспорт: esp_mesh_send / esp_mesh_recv.
   ============================================================ */

#define MESH_MAGIC          0x4D534831  /* "MSH1" */
#define MESH_MAX_PAYLOAD    180         /* < mesh data max (около 200) */

typedef enum {
    MSG_HELLO           = 0x01,  /* node → root: я жив */
    MSG_HELLO_ACK       = 0x02,  /* root → node: принял */
    MSG_CMD_TRAFFIC     = 0x10,  /* root → node: start/stop/pause/... */
    MSG_CMD_RELAY       = 0x11,  /* root → node: sr:N:S */
    MSG_CMD_CONFIG      = 0x12,  /* root → node: JSON конфиг (чанками) */
    MSG_CMD_CONFIG_END  = 0x13,  /* root → node: конец JSON конфига */
    MSG_STATUS          = 0x20,  /* node → root: краткий статус */
    MSG_ACK             = 0x21,  /* общий ACK */

    /* ✅ JSON node → root → MQTT */
    MSG_JSON_REPORT     = 0x30,  /* node → root: JSON-статус (чанками) */
    MSG_JSON_REPORT_END = 0x31,  /* node → root: конец JSON */
} mesh_msg_type_t;

#define MESH_FLAG_NONE        0x00

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  type;
    uint8_t  seq;
    uint8_t  flags;
    uint8_t  payload_len;
    uint8_t  payload[MESH_MAX_PAYLOAD];
} mesh_packet_t;

/* CMD_TRAFFIC */
typedef struct __attribute__((packed)) {
    uint8_t action;   /* 0=start, 1=stop, 2=pause, 3=resume, 4=reset, 5=next */
    uint8_t mode;     /* 1..3, 0 = не менять */
} mesh_cmd_traffic_t;

/* CMD_RELAY */
typedef struct __attribute__((packed)) {
    uint8_t channel;
    uint8_t state;
} mesh_cmd_relay_t;

/* CMD_CONFIG — заголовок + часть JSON (root → node) */
typedef struct __attribute__((packed)) {
    uint16_t chunk_idx;
    uint16_t chunk_total;
    uint16_t chunk_len;
} mesh_cmd_config_hdr_t;

/* ✅ JSON-репорт — заголовок + часть JSON (node → root) */
typedef struct __attribute__((packed)) {
    uint16_t chunk_idx;
    uint16_t chunk_total;
    uint16_t chunk_len;
} mesh_json_hdr_t;

/* STATUS — краткий статус node → root */
typedef struct __attribute__((packed)) {
    uint8_t  state;
    uint8_t  mode;
    uint8_t  phase;
    uint8_t  fault;
    uint8_t  fault_count;
    uint16_t sr_state;
    uint32_t phase_ms;
    uint32_t uptime_ms;
    int8_t   rssi;
} mesh_status_t;