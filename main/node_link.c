#include "node_link.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>

static const char *TAG = "node_link";

static node_json_cb_t s_cb     = NULL;
static node_cfg_cb_t  s_cfg_cb = NULL;
static node_cmd_cb_t  s_cmd_cb = NULL;
static node_evt_cb_t  s_evt_cb = NULL;

static int  s_server_sock      = -1;
static int  s_client_sock      = -1;
static int  s_root_client_sock = -1;
static volatile bool s_client_connected = false;
static char     s_client_host[32] = {0};
static uint16_t s_client_port     = 0;
static volatile bool s_running    = false;

static SemaphoreHandle_t s_send_mutex        = NULL;
static SemaphoreHandle_t s_client_send_mutex = NULL;

// === ВАРИАНТ B: семафор для ожидания подключения node (root) ===
static SemaphoreHandle_t s_client_connected_sem = NULL;

#define NODE_LINK_MAX_MSG 32768

#define NODE_LINK_SNDTIMEO_S  2
#define NODE_LINK_RCVTIMEO_MS 100

#define SEND_RETRY_MAX_MS     100
#define SEND_RETRY_STEP_MS    20

#define SEND_LOCK_TIMEOUT_MS  500

static void set_socket_timeouts(int fd, int snd_s, int rcv_ms)
{
    struct timeval tv_snd = { .tv_sec = snd_s, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv_snd, sizeof(tv_snd));

    struct timeval tv_rcv = { .tv_sec = rcv_ms / 1000,
                              .tv_usec = (rcv_ms % 1000) * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv_rcv, sizeof(tv_rcv));
}

void node_link_set_cb(node_json_cb_t cb)     { s_cb     = cb; }
void node_link_set_cfg_cb(node_cfg_cb_t cb)  { s_cfg_cb = cb; }
void node_link_set_cmd_cb(node_cmd_cb_t cb)  { s_cmd_cb = cb; }
void node_link_set_evt_cb(node_evt_cb_t cb)  { s_evt_cb = cb; }

static esp_err_t send_line_to_node(const char *prefix,
                                   const char *payload, size_t len,
                                   uint32_t lock_timeout_ms);

static void dispatch_line(const char *line, size_t len)
{
    if (len == 0) return;

    if (!(len >= 9 && strncmp(line, "cmd:sync:", 9) == 0)) {
        ESP_LOGI(TAG, "RX: %.*s", (int)len, line);
    }

    if (len > 4 && strncmp(line, "cfg:", 4) == 0) {
        if (s_cfg_cb) s_cfg_cb(line + 4, len - 4);
        else ESP_LOGW(TAG, "cfg: but s_cfg_cb == NULL");
        return;
    }
    if (len > 4 && strncmp(line, "cmd:", 4) == 0) {
        if (s_cmd_cb) s_cmd_cb(line + 4, len - 4);
        else ESP_LOGW(TAG, "cmd: but s_cmd_cb == NULL");
        return;
    }
    if (len > 4 && strncmp(line, "evt:", 4) == 0) {
        if (s_evt_cb) s_evt_cb(line + 4, len - 4);
        else ESP_LOGE(TAG, "EVT but s_evt_cb == NULL!");
        return;
    }
    if (len > 7 && strncmp(line, "status:", 7) == 0) {
        if (s_cb) s_cb(line + 7, len - 7);
        return;
    }

    ESP_LOGW(TAG, "dispatch_line: unknown line, drop '%.*s'", (int)len, line);
}

/* ============================================================
   ROOT: TCP-сервер. При accept — СРАЗУ cfg: (initial config).
   ============================================================ */
static void server_task(void *arg)
{
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(NODE_LINK_PORT);

    s_server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (s_server_sock < 0) { vTaskDelete(NULL); return; }
    int opt = 1;
    setsockopt(s_server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(s_server_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind failed errno=%d", errno);
        close(s_server_sock); s_server_sock = -1; vTaskDelete(NULL); return;
    }
    if (listen(s_server_sock, 5) < 0) {
        ESP_LOGE(TAG, "listen failed errno=%d", errno);
        close(s_server_sock); s_server_sock = -1; vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG, "TCP server listening on :%d", NODE_LINK_PORT);

    char *buf = malloc(NODE_LINK_MAX_MSG);
    if (!buf) { vTaskDelete(NULL); return; }

    while (s_running) {
        struct sockaddr_in cli = {0};
        socklen_t cli_len = sizeof(cli);
        int cfd = accept(s_server_sock, (struct sockaddr *)&cli, &cli_len);
        if (cfd < 0) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }

        int nodelay = 1;
        setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
        set_socket_timeouts(cfd, NODE_LINK_SNDTIMEO_S, NODE_LINK_RCVTIMEO_MS);

        char ip[16]; inet_ntoa_r(cli.sin_addr, ip, sizeof(ip));
        ESP_LOGI(TAG, "node connected from %s:%d", ip, ntohs(cli.sin_port));
        s_root_client_sock = cfd;

        /* ===== FIX: initial cfg ===== */
        {
            extern bool config_export_json(char *b, size_t *l);
            char  *cbuf = malloc(16384);
            size_t clen = 16384;
            esp_err_t cfg_err = ESP_FAIL;
            if (cbuf && config_export_json(cbuf, &clen)) {
                ESP_LOGI(TAG, "→ node: initial cfg %u bytes", (unsigned)clen);
                cfg_err = send_line_to_node("cfg:", cbuf, clen, SEND_LOCK_TIMEOUT_MS);
            } else {
                ESP_LOGW(TAG, "initial cfg export failed");
            }
            free(cbuf);

            if (cfg_err != ESP_OK) {
                ESP_LOGE(TAG, "initial cfg send failed (%s) → close, wait reconnect",
                         esp_err_to_name(cfg_err));
                close(cfd);
                s_root_client_sock = -1;
                continue;
            }
        }

        // === ВАРИАНТ B: сигналим, что node подключён и cfg отправлен ===
        if (!s_client_connected_sem)
            s_client_connected_sem = xSemaphoreCreateBinary();
        if (s_client_connected_sem)
            xSemaphoreGive(s_client_connected_sem);

        size_t acc = 0;
        while (s_running) {
            int n = recv(cfd, buf + acc, NODE_LINK_MAX_MSG - acc - 1, 0);
            if (n == 0) {
                ESP_LOGW(TAG, "node closed connection");
                break;
            }
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                ESP_LOGW(TAG, "recv errno=%d", errno);
                break;
            }
            acc += n;
            buf[acc] = '\0';
            char *line = buf;
            char *nl;
            while ((nl = memchr(line, '\n', acc - (line - buf))) != NULL) {
                size_t llen = nl - line;
                if (llen > 0) dispatch_line(line, llen);
                line = nl + 1;
            }
            size_t rest = acc - (line - buf);
            if (rest > 0) memmove(buf, line, rest);
            acc = rest;
            if (acc >= NODE_LINK_MAX_MSG - 1) acc = 0;
        }
        ESP_LOGW(TAG, "node disconnected");
        s_root_client_sock = -1;
        close(cfd);
    }
    free(buf);
    s_server_sock = -1;
    vTaskDelete(NULL);
}

esp_err_t node_link_server_start(uint16_t port)
{
    (void)port;
    if (s_running) return ESP_OK;
    if (!s_send_mutex) s_send_mutex = xSemaphoreCreateMutex();
    if (!s_send_mutex) return ESP_ERR_NO_MEM;
    if (!s_client_send_mutex) s_client_send_mutex = xSemaphoreCreateMutex();
    // === ВАРИАНТ B: создаём семафор ожидания node ===
    if (!s_client_connected_sem) s_client_connected_sem = xSemaphoreCreateBinary();
    s_running = true;
    xTaskCreate(server_task, "node_srv", 6144, NULL, 6, NULL);
    return ESP_OK;
}

bool node_link_server_has_client(void)
{
    return s_root_client_sock >= 0;
}

static esp_err_t send_line_to_node(const char *prefix,
                                   const char *payload, size_t len,
                                   uint32_t lock_timeout_ms)
{
    if (s_root_client_sock < 0) return ESP_ERR_INVALID_STATE;
    if (!s_send_mutex) return ESP_ERR_INVALID_STATE;

    TickType_t wait = (lock_timeout_ms == 0) ? 0 : pdMS_TO_TICKS(lock_timeout_ms);
    if (xSemaphoreTake(s_send_mutex, wait) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    size_t plen = strlen(prefix);
    char *out = malloc(plen + len + 2);
    if (!out) { xSemaphoreGive(s_send_mutex); return ESP_ERR_NO_MEM; }
    memcpy(out, prefix, plen);
    memcpy(out + plen, payload, len);
    out[plen + len] = '\n';
    int total = (int)(plen + len + 1);

    int sent = 0;
    int waited_ms = 0;
    esp_err_t result = ESP_OK;

    while (sent < total) {
        int n = send(s_root_client_sock, out + sent, total - sent, 0);
        if (n <= 0) {
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                if (waited_ms >= SEND_RETRY_MAX_MS) {
                    ESP_LOGW(TAG, "send to node: EAGAIN after %d ms (sent=%d/%d)",
                             waited_ms, sent, total);
                    result = ESP_FAIL;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(SEND_RETRY_STEP_MS));
                waited_ms += SEND_RETRY_STEP_MS;
                continue;
            }
            ESP_LOGW(TAG, "send to node failed (sent=%d/%d errno=%d)",
                     sent, total, errno);
            close(s_root_client_sock);
            s_root_client_sock = -1;
            result = ESP_FAIL;
            break;
        }
        sent += n;
        waited_ms = 0;
    }
    free(out);
    xSemaphoreGive(s_send_mutex);
    return result;
}

esp_err_t node_link_server_send_config(const char *json, size_t len)
{
    ESP_LOGI(TAG, "→ node: cfg %u bytes", (unsigned)len);
    return send_line_to_node("cfg:", json, len, SEND_LOCK_TIMEOUT_MS);
}

esp_err_t node_link_server_send_cmd(const char *cmd, size_t len)
{
    ESP_LOGI(TAG, "→ node: cmd '%.*s'", (int)len, cmd);
    return send_line_to_node("cmd:", cmd, len, SEND_LOCK_TIMEOUT_MS);
}

esp_err_t node_link_server_send_cmd_nonblock(const char *cmd, size_t len)
{
    return send_line_to_node("cmd:", cmd, len, 0);
}

/* ============================================================
   NODE: TCP-клиент.
   ============================================================ */
static bool connect_once(void)
{
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(s_client_port);
    if (inet_pton(AF_INET, s_client_host, &addr.sin_addr) != 1) return false;
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return false;
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd); return false;
    }
    int nodelay = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    set_socket_timeouts(fd, NODE_LINK_SNDTIMEO_S, NODE_LINK_RCVTIMEO_MS);
    s_client_sock = fd;
    s_client_connected = true;
    ESP_LOGI(TAG, "client: connected to %s:%d", s_client_host, s_client_port);
    return true;
}

static void client_task(void *arg)
{
    char *buf = malloc(NODE_LINK_MAX_MSG);
    if (!buf) { vTaskDelete(NULL); return; }

    size_t acc = 0;

    while (s_running) {
        if (!s_client_connected) {
            acc = 0;
            if (!connect_once()) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }
        int n = recv(s_client_sock, buf + acc, NODE_LINK_MAX_MSG - acc - 1, 0);
        if (n == 0) {
            ESP_LOGW(TAG, "client: peer closed, reconnect");
            s_client_connected = false;
            close(s_client_sock);
            s_client_sock = -1;
            acc = 0;
            continue;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            ESP_LOGW(TAG, "client: recv errno=%d, reconnect", errno);
            s_client_connected = false;
            close(s_client_sock);
            s_client_sock = -1;
            acc = 0;
            continue;
        }
        acc += n;
        buf[acc] = '\0';

        char *line = buf;
        char *nl;
        while ((nl = memchr(line, '\n', acc - (line - buf))) != NULL) {
            size_t llen = nl - line;
            if (llen > 0) dispatch_line(line, llen);
            line = nl + 1;
        }
        size_t rest = acc - (line - buf);
        if (rest > 0 && line != buf) memmove(buf, line, rest);
        acc = rest;
        if (acc >= NODE_LINK_MAX_MSG - 1) {
            ESP_LOGE(TAG, "client: RX buffer overflow, reset");
            acc = 0;
        }
    }
    free(buf);
    vTaskDelete(NULL);
}

esp_err_t node_link_client_start(const char *host, uint16_t port)
{
    if (s_running) return ESP_OK;
    snprintf(s_client_host, sizeof(s_client_host), "%s",
             host ? host : "192.168.4.1");
    s_client_port = port ? port : NODE_LINK_PORT;
    if (!s_client_send_mutex) s_client_send_mutex = xSemaphoreCreateMutex();
    if (!s_client_send_mutex) return ESP_ERR_NO_MEM;
    s_running = true;
    xTaskCreate(client_task, "node_cli", 6144, NULL, 6, NULL);
    return ESP_OK;
}

esp_err_t node_link_client_send(const char *json, size_t len)
{
    if (!s_client_connected || s_client_sock < 0) return ESP_ERR_INVALID_STATE;
    if (!s_client_send_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_client_send_mutex, pdMS_TO_TICKS(500)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    char *out = malloc(len + 8);
    if (!out) { xSemaphoreGive(s_client_send_mutex); return ESP_ERR_NO_MEM; }
    int n = snprintf(out, len + 8, "status:%.*s\n", (int)len, json);
    int sent = 0;
    int waited_ms = 0;
    esp_err_t result = ESP_OK;
    while (sent < n) {
        int r = send(s_client_sock, out + sent, n - sent, 0);
        if (r <= 0) {
            if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                if (waited_ms >= SEND_RETRY_MAX_MS) {
                    ESP_LOGW(TAG, "send_status: EAGAIN after %d ms", waited_ms);
                    s_client_connected = false;
                    close(s_client_sock); s_client_sock = -1;
                    result = ESP_FAIL;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(SEND_RETRY_STEP_MS));
                waited_ms += SEND_RETRY_STEP_MS;
                continue;
            }
            ESP_LOGW(TAG, "send_status failed: sent=%d/%d errno=%d",
                     sent, n, errno);
            s_client_connected = false;
            close(s_client_sock); s_client_sock = -1;
            result = ESP_FAIL;
            break;
        }
        sent += r;
        waited_ms = 0;
    }
    free(out);
    xSemaphoreGive(s_client_send_mutex);
    return result;
}

esp_err_t node_link_client_send_evt(const char *evt, size_t len)
{
    if (!s_client_connected || s_client_sock < 0) return ESP_ERR_INVALID_STATE;
    if (!s_client_send_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_client_send_mutex, pdMS_TO_TICKS(500)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    char *out = malloc(len + 8);
    if (!out) { xSemaphoreGive(s_client_send_mutex); return ESP_ERR_NO_MEM; }
    int n = snprintf(out, len + 8, "evt:%.*s\n", (int)len, evt);
    int sent = 0;
    int waited_ms = 0;
    esp_err_t result = ESP_OK;
    while (sent < n) {
        int r = send(s_client_sock, out + sent, n - sent, 0);
        if (r <= 0) {
            if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                if (waited_ms >= SEND_RETRY_MAX_MS) {
                    ESP_LOGW(TAG, "send_evt: EAGAIN after %d ms", waited_ms);
                    s_client_connected = false;
                    close(s_client_sock); s_client_sock = -1;
                    result = ESP_FAIL;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(SEND_RETRY_STEP_MS));
                waited_ms += SEND_RETRY_STEP_MS;
                continue;
            }
            ESP_LOGW(TAG, "send_evt failed: sent=%d/%d errno=%d",
                     sent, n, errno);
            s_client_connected = false;
            close(s_client_sock); s_client_sock = -1;
            result = ESP_FAIL;
            break;
        }
        sent += r;
        waited_ms = 0;
    }
    free(out);
    xSemaphoreGive(s_client_send_mutex);
    return result;
}

bool node_link_client_connected(void) { return s_client_connected; }

void node_link_stop(void)
{
    s_running = false;
    if (s_server_sock >= 0) { close(s_server_sock); s_server_sock = -1; }
    if (s_client_sock >= 0) { close(s_client_sock); s_client_sock = -1; }
    if (s_root_client_sock >= 0) { close(s_root_client_sock); s_root_client_sock = -1; }
    s_client_connected = false;
}

// === ВАРИАНТ B: новые функции ===

bool node_link_wait_for_client(uint32_t timeout_ms)
{
    if (s_root_client_sock >= 0) return true;
    if (!s_client_connected_sem)
        s_client_connected_sem = xSemaphoreCreateBinary();
    if (!s_client_connected_sem) return false;
    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(s_client_connected_sem, ticks) == pdTRUE;
}

void node_link_force_reconnect(void)
{
    ESP_LOGW(TAG, "force_reconnect: closing node socket");
    if (s_root_client_sock >= 0) {
        close(s_root_client_sock);
        s_root_client_sock = -1;
    }
}