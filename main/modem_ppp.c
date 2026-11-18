/* ============================================================
   modem_ppp.c — PPPoS через A7680C (SIMCom A76XX, Cat.1)
   esp_modem 2.x API.

   UART: TX=GPIO21 (ESP32 -> модем RX), RX=GPIO10 (модем TX -> ESP32).
   RST:  GPIO36 — input-only, НЕ используется (MODEM_USE_RST=0).
   Baud: 115200, fallback 9600.

   Команды (A76XX):
     AT                          — sync
     ATE0                        — выключить эхо
     AT+CGDCONT=1,"IP","<APN>"   — PDP context (делает esp_modem)
     ATD*99#                     — PPP (делает esp_modem)

   esp_modem 2.x: DCE+DTE создаются одной функцией esp_modem_new_dev().
   ============================================================ */
#define _GNU_SOURCE
#include "modem_ppp.h"
#include "board_pins.h"

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "esp_event.h"
#include "esp_modem_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "modem_ppp";

/* GPIO36 input-only — RST через него невозможен.
   Если RST перепаян на выходной GPIO — задайте MODEM_RST в board_pins.h
   и выставьте MODEM_USE_RST=1. */
#define MODEM_USE_RST   1

#define MODEM_BAUD_DEFAULT   115200
#define MODEM_BAUD_FALLBACK  57600
#define MODEM_BOOT_DELAY_MS  15000
#define MODEM_SYNC_RETRIES   5

#define PPP_GOT_IP_BIT   BIT0
#define PPP_LOST_IP_BIT  BIT1

/* esp_modem 2.x: s_dte не нужен — DTE внутри DCE */
static esp_modem_dce_t   *s_dce        = NULL;
static esp_netif_t       *s_ppp_netif  = NULL;
static EventGroupHandle_t s_evt        = NULL;
static SemaphoreHandle_t  s_state_mtx  = NULL;

static volatile bool s_connected = false;
static char          s_ip[16]    = "0.0.0.0";
static char          s_apn[64]   = {0};

// === SIM READY: флаг, что модем ответил +CPIN: READY ===
static volatile bool s_sim_ready = false;


/* Прямой UART-тест: шлём "AT\r\n", ждём "OK".
   Возвращает baud, на котором модем ответил OK, или 0. */
static uint32_t modem_uart_find_baud(void)
{
    /*static const uint32_t bauds[] = {
        115200, 9600, 57600, 230400, 460800, 38400, 19200, 74880
    };*/
    static const uint32_t bauds[] = {
        115200, 57600
    };
    const uart_port_t uart_num = MODEM_UART_NUM;

    ESP_LOGW(TAG, "UART baud scan (TX=%d RX=%d)...", MODEM_TX, MODEM_RX);

    for (size_t b = 0; b < sizeof(bauds) / sizeof(bauds[0]); b++) {
        uint32_t baud = bauds[b];
        ESP_LOGW(TAG, "  trying %lu baud", (unsigned long)baud);

        uart_config_t cfg = {
            .baud_rate  = baud,
            .data_bits  = UART_DATA_8_BITS,
            .parity     = UART_PARITY_DISABLE,
            .stop_bits  = UART_STOP_BITS_1,
            .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        uart_driver_install(uart_num, 1024, 1024, 0, NULL, 0);
        uart_param_config(uart_num, &cfg);
        uart_set_pin(uart_num, MODEM_TX, MODEM_RX,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_flush_input(uart_num);
        vTaskDelay(pdMS_TO_TICKS(50));

        bool ok = false;
        for (int attempt = 0; attempt < 3 && !ok; attempt++) {
            const char *at = "AT\r\n";
            uart_write_bytes(uart_num, at, strlen(at));
            uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));

            uint8_t rx[128];
            int n = uart_read_bytes(uart_num, rx, sizeof(rx),
                                    pdMS_TO_TICKS(800));
            if (n > 0) {
                ESP_LOGW(TAG, "  rx %d bytes: '%.*s'",
                         n, n, (char *)rx);

                // === SIM READY: парсим +CPIN: READY в ответе модема ===
                // Ответ может быть: "AT\r\nOK\r\n\r\n+CPIN: READY\r\n"
                // Используем memmem, т.к. rx может содержать \0.
                // modem_ppp.c, функция modem_uart_find_baud()
if (memmem(rx, n, "+CPIN: READY", 12) != NULL) {
    if (!s_sim_ready) {
        ESP_LOGI(TAG, "SIM ready (+CPIN: READY) → PPPoS preferred over WiFi STA");
        s_sim_ready = true;
        
        // === ДОБАВЬТЕ ЭТО: ===
        extern void wifi_stop(void); // Или wifi_ap_stop(), если нужно оставить AP
        ESP_LOGW(TAG, "Stopping WiFi STA to free RAM for PPPoS");
        // wifi_stop(); // Раскомментируйте, если хотите полностью убить STA при наличии SIM
    }
}

                if (memmem(rx, n, "OK", 2)) {
                    ok = true;
                }
            }
        }
        uart_driver_delete(uart_num);

        if (ok) {
            ESP_LOGW(TAG, "  ** modem responds OK at %lu baud **",
                     (unsigned long)baud);
            return baud;
        }
    }
    ESP_LOGE(TAG, "modem not responding at any baud");
    return 0;
}

/* Прямой UART-тест: шлём "AT\r\n", ждём ответ 2 с.
   Показывает, отвечает ли модем физически, минуя esp_modem.
   Возвращает число принятых байт. */
static int modem_uart_probe(uint32_t baud)
{
    ESP_LOGW(TAG, "UART probe @ %lu baud (TX=%d RX=%d)",
             (unsigned long)baud, MODEM_TX, MODEM_RX);

    const uart_port_t uart_num = MODEM_UART_NUM;

    uart_config_t cfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* Установить драйвер UART (или переустановить с новым baud) */
    uart_driver_install(uart_num, 1024, 1024, 0, NULL, 0);
    uart_param_config(uart_num, &cfg);
    uart_set_pin(uart_num, MODEM_TX, MODEM_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    /* Очистить буфер */
    uart_flush_input(uart_num);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Шлём AT несколько раз — модем может быть в середине загрузки */
    int total_rx = 0;
    uint8_t rx[256];

    for (int attempt = 0; attempt < 5; attempt++) {
        const char *at = "AT\r\n";
        uart_write_bytes(uart_num, at, strlen(at));
        uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));

        /* Ждём ответ до 1 с */
        int n = uart_read_bytes(uart_num, rx, sizeof(rx),
                                pdMS_TO_TICKS(1000));
        if (n > 0) {
            total_rx += n;
            ESP_LOGW(TAG, "probe attempt %d: rx %d bytes: '%.*s'",
                     attempt + 1, n, n, (char *)rx);

            // === SIM READY: тоже парсим здесь (на случай, если
            // modem_uart_find_baud не увидел +CPIN) ===
            if (memmem(rx, n, "+CPIN: READY", 12) != NULL) {
                if (!s_sim_ready) {
                    ESP_LOGI(TAG, "SIM ready (+CPIN: READY) → "
                                  "PPPoS preferred over WiFi STA");
                    s_sim_ready = true;
                }
            }

            if (memchr(rx, 'O', n) || memchr(rx, 'K', n)) {
                ESP_LOGW(TAG, "probe: OK detected at baud=%lu",
                         (unsigned long)baud);
                break;
            }
        } else {
            ESP_LOGW(TAG, "probe attempt %d: no response", attempt + 1);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    uart_driver_delete(uart_num);
    return total_rx;
}

/* ============================================================
   Event handlers — ДОЛЖНЫ БЫТЬ ДО modem_ppp_start
   ============================================================ */
static void ppp_netif_event_handler(void *arg, esp_event_base_t base,
                                    int32_t id, void *data)
{
    switch (id) {
    case NETIF_PPP_ERRORUSER:
        ESP_LOGW(TAG, "PPP: error user");
        break;
    case NETIF_PPP_ERRORCONNECT:
        ESP_LOGW(TAG, "PPP: connect error");
        break;
    case NETIF_PPP_ERRORAUTHFAIL:
        ESP_LOGE(TAG, "PPP: auth fail (проверьте APN / user / pass)");
        break;
    case NETIF_PPP_PHASE_DEAD:
        ESP_LOGD(TAG, "PPP: phase dead");
        break;
    case NETIF_PPP_PHASE_NETWORK:
        ESP_LOGD(TAG, "PPP: phase network");
        break;
    default:
        ESP_LOGD(TAG, "PPP: status id=%d", (int)id);
        break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t base,
                             int32_t id, void *data)
{
    if (id == IP_EVENT_PPP_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        if (s_state_mtx && xSemaphoreTake(s_state_mtx, pdMS_TO_TICKS(100)) == pdTRUE) {
            snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
            s_connected = true;
            xSemaphoreGive(s_state_mtx);
        }
        ESP_LOGI(TAG, "PPP got IP: %s", s_ip);
        if (s_evt) xEventGroupSetBits(s_evt, PPP_GOT_IP_BIT);
    } else if (id == IP_EVENT_PPP_LOST_IP) {
        if (s_state_mtx && xSemaphoreTake(s_state_mtx, pdMS_TO_TICKS(100)) == pdTRUE) {
            s_connected = false;
            snprintf(s_ip, sizeof(s_ip), "0.0.0.0");
            xSemaphoreGive(s_state_mtx);
        }
        ESP_LOGW(TAG, "PPP lost IP");
        if (s_evt) xEventGroupSetBits(s_evt, PPP_LOST_IP_BIT);
    }
}

/* ============================================================
   HW reset (опционально)
   ============================================================ */
static void modem_hw_reset(void)
{
    s_sim_ready = false;
#if MODEM_USE_RST
    ESP_LOGI(TAG, "HW reset модема on GPIO%d", MODEM_RST);
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << MODEM_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(MODEM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(MODEM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(1500));
#else
    ESP_LOGI(TAG, "RST не используется (GPIO36 input-only). "
                  "Ждём загрузку модема %d мс", MODEM_BOOT_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(MODEM_BOOT_DELAY_MS));
#endif
}

/* ============================================================
   FIX: создание DCE+DTE+netif одной функцией esp_modem_new_dev,
   sync с retries. Возвращает DCE или NULL.
   ============================================================ */
static esp_modem_dce_t *create_dce_synced(uint32_t baud, const char *apn)
{
    esp_modem_dte_config_t dte_cfg = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_cfg.uart_config.port_num     = MODEM_UART_NUM;
    dte_cfg.uart_config.tx_io_num    = MODEM_TX;   /* GPIO21 */
    dte_cfg.uart_config.rx_io_num    = MODEM_RX;   /* GPIO10 */
    dte_cfg.uart_config.rts_io_num   = -1;
    dte_cfg.uart_config.cts_io_num   = -1;
    dte_cfg.uart_config.baud_rate    = baud;
    dte_cfg.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_NONE;

    esp_modem_dce_config_t dce_cfg = ESP_MODEM_DCE_DEFAULT_CONFIG(apn);

    esp_modem_dce_t *dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM7600,
                                             &dte_cfg, &dce_cfg, s_ppp_netif);
    if (!dce) {
        ESP_LOGE(TAG, "esp_modem_new_dev failed (baud=%lu)",
                 (unsigned long)baud);
        return NULL;
    }

    for (int i = 0; i < MODEM_SYNC_RETRIES; i++) {
        if (esp_modem_sync(dce) == ESP_OK) {
            ESP_LOGI(TAG, "sync OK at baud=%lu", (unsigned long)baud);
            return dce;
        }
        ESP_LOGW(TAG, "sync %d/%d failed (baud=%lu)",
                 i + 1, MODEM_SYNC_RETRIES, (unsigned long)baud);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGE(TAG, "sync FAILED at baud=%lu", (unsigned long)baud);
    esp_modem_destroy(dce);
    return NULL;
}


/* === FIX SIM-REG-CEREG ===
   A76XX — LTE Cat.1: статус регистрации живёт в +CEREG,
   а не в +CGREG (тот для LTE может оставаться 0,2 навсегда,
   даже когда модем в сети). Опрашиваем оба, каждый ответ
   логируем. Заодно обновляем s_sim_ready по факту CPIN? —
   тупик «SIM not ready» исчезает. */
static bool modem_wait_registered(esp_modem_dce_t *dce, uint32_t timeout_ms)
{
    char resp[128];
    uint32_t waited = 0;

    while (waited < timeout_ms) {
        if (esp_modem_at(dce, "AT+CPIN?", resp, sizeof(resp)) == ESP_OK &&
            strstr(resp, "READY")) {
            if (!s_sim_ready) {
                s_sim_ready = true;
                ESP_LOGI(TAG, "SIM ready (+CPIN: READY)");
            }
        }

        bool reg_ok = false;
        if (esp_modem_at(dce, "AT+CEREG?", resp, sizeof(resp)) == ESP_OK) {
            ESP_LOGI(TAG, "reg poll %lus: %s",
                     (unsigned long)(waited / 1000), resp);
            if (strstr(resp, "0,1") || strstr(resp, "1,1") ||
                strstr(resp, "0,5") || strstr(resp, "1,5")) {
                reg_ok = true;
            }
        }
        if (!reg_ok &&
            esp_modem_at(dce, "AT+CGREG?", resp, sizeof(resp)) == ESP_OK) {
            ESP_LOGI(TAG, "cgreg poll %lus: %s",
                     (unsigned long)(waited / 1000), resp);
            if (strstr(resp, "0,1") || strstr(resp, "1,1") ||
                strstr(resp, "0,5") || strstr(resp, "1,5")) {
                reg_ok = true;
            }
        }

        if (reg_ok) {
            ESP_LOGI(TAG, "registered after %lu ms",
                     (unsigned long)waited);
            return true;
        }

        if ((waited % 20000) == 0 &&
            esp_modem_at(dce, "AT+CSQ", resp, sizeof(resp)) == ESP_OK) {
            ESP_LOGI(TAG, "signal: %s", resp);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
        waited += 5000;
    }
    return false;
}

/* ============================================================
   Public API
   ============================================================ */
esp_err_t modem_ppp_start(const char *apn, uint32_t timeout_ms)
{
    /* === FIX PPP-SINGLE-FLIGHT ===
       Попытка ещё идёт — не наслаиваемся. connectivity
       получает ESP_ERR_INVALID_STATE и НЕ увеличивает
       fail_count (см. FIX в connectivity.c). */
    if (!s_state_mtx) s_state_mtx = xSemaphoreCreateMutex();
    if (s_state_mtx &&
        xSemaphoreTake(s_state_mtx, 0) != pdTRUE) {
        ESP_LOGW(TAG, "modem_ppp_start: attempt already in progress → skip");
        return ESP_ERR_INVALID_STATE;
    }

    /* === FIX PPP-STALE-STATE (сохранён) === */
    if ((s_dce != NULL || s_ppp_netif != NULL) && !modem_ppp_is_connected()) {
        ESP_LOGW(TAG, "PPP stale state → clean stop before re-dial");
        modem_ppp_stop();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (s_dce) {                       /* уже запущен и жив */
        if (s_state_mtx) xSemaphoreGive(s_state_mtx);
        return ESP_ERR_INVALID_STATE;
    }

    if (!apn || strlen(apn) == 0) {
        if (s_state_mtx) xSemaphoreGive(s_state_mtx);
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_state_mtx) s_state_mtx = xSemaphoreCreateMutex();
    if (!s_evt)       s_evt = xEventGroupCreate();
    if (!s_state_mtx || !s_evt) {
        if (s_state_mtx) xSemaphoreGive(s_state_mtx);
        return ESP_ERR_NO_MEM;
    }
    snprintf(s_apn, sizeof(s_apn), "%s", apn);

    /* === FIX SET_MODE-DEBUG ===
       Видим точный AT-обмен на dial: команду и ответ модема. */
    esp_log_level_set("esp_modem",     ESP_LOG_DEBUG);
    esp_log_level_set("esp_modem_dce", ESP_LOG_DEBUG);
    esp_log_level_set("esp_modem_dte", ESP_LOG_DEBUG);

    esp_netif_init();
    esp_event_loop_create_default();

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_PPP();
    s_ppp_netif = esp_netif_new(&netif_cfg);
    if (!s_ppp_netif) {
        ESP_LOGE(TAG, "esp_netif_new failed");
        goto fail;
    }

    /* === FIX DOUBLE-RESET: один reset на попытку === */
    modem_hw_reset();

    uint32_t baud = modem_uart_find_baud();
    if (baud == 0) {
        ESP_LOGE(TAG, "no working baud");
        goto fail;
    }
    ESP_LOGI(TAG, "using baud=%lu", (unsigned long)baud);

    s_dce = create_dce_synced(baud, apn);
    if (!s_dce) {
        ESP_LOGE(TAG, "create_dce_synced failed");
        goto fail;
    }

    esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID,
                               &ppp_netif_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP,
                               &ip_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_LOST_IP,
                               &ip_event_handler, NULL);

    esp_modem_at(s_dce, "ATE0", NULL, 0);

    if (!modem_wait_registered(s_dce, 60000)) {
        ESP_LOGE(TAG, "not registered after 60 s → fail attempt");
        goto fail;
    }

    /* === FIX PPP-ATTACH-WAIT ===
       set_mode(DATA) падал за 10-20 мс: ATD получает ERROR,
       пока не завершены GPRS-attach и PDP-activation.
       CEREG 0,1 — только регистрация, НЕ готовность к данным. */

    /* 1) GPRS attach */
    bool attached = false;
    for (int i = 0; i < 15 && !attached; i++) {
        char resp[64];
        if (esp_modem_at(s_dce, "AT+CGATT?", resp, sizeof(resp)) == ESP_OK) {
            ESP_LOGI(TAG, "cgatt: %s", resp);
            if (strstr(resp, "+CGATT: 1")) attached = true;
        }
        if (!attached) {
            esp_modem_at(s_dce, "AT+CGATT=1", NULL, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    if (!attached) {
        ESP_LOGE(TAG, "GPRS attach not ready → fail attempt");
        goto fail;
    }

    /* 2) PDP context */
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", apn);
    esp_modem_at(s_dce, cmd, NULL, 0);

    /* 3) PDP activation */
    esp_modem_at(s_dce, "AT+CGACT=1,1", NULL, 0);
    bool active = false;
    for (int i = 0; i < 5 && !active; i++) {
        char resp[64];
        if (esp_modem_at(s_dce, "AT+CGACT?", resp, sizeof(resp)) == ESP_OK) {
            ESP_LOGI(TAG, "cgact: %s", resp);
            if (strstr(resp, "1,1")) active = true;
        }
        if (!active) vTaskDelay(pdMS_TO_TICKS(2000));
    }
    if (!active) {
        ESP_LOGE(TAG, "PDP activation not ready → fail attempt");
        goto fail;
    }
    /* === END FIX PPP-ATTACH-WAIT === */

    /* === FIX STALE-EVENT-BIT (сохранён) ===
       GOT_IP может остаться с прошлого успешного dial —
       чистим, иначе WaitBits вернёт мгновенный фейковый UP. */
    xEventGroupClearBits(s_evt, PPP_GOT_IP_BIT | PPP_LOST_IP_BIT);

    ESP_LOGI(TAG, "entering PPP data mode...");
    esp_err_t err = esp_modem_set_mode(s_dce, ESP_MODEM_MODE_DATA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_mode(DATA): %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGI(TAG, "waiting for PPP IP (up to %lu ms)...",
             (unsigned long)timeout_ms);
    EventBits_t bits = xEventGroupWaitBits(s_evt,
        PPP_GOT_IP_BIT | PPP_LOST_IP_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
    if (!(bits & PPP_GOT_IP_BIT)) {
        ESP_LOGE(TAG, "PPP timeout");
        goto fail;
    }

    ESP_LOGI(TAG, "PPPoS UP: IP=%s APN=%s", s_ip, s_apn);
    if (s_state_mtx) xSemaphoreGive(s_state_mtx);
    return ESP_OK;

fail:
    if (s_dce)       { esp_modem_destroy(s_dce); s_dce = NULL; }
    if (s_ppp_netif) { esp_netif_destroy(s_ppp_netif); s_ppp_netif = NULL; }
    if (s_state_mtx) xSemaphoreGive(s_state_mtx);
    return ESP_FAIL;
}

esp_err_t modem_ppp_stop(void)
{
    s_sim_ready = false;
    if (!s_dce) return ESP_OK;

    ESP_LOGI(TAG, "stopping PPP...");
    esp_modem_set_mode(s_dce, ESP_MODEM_MODE_COMMAND);
    esp_modem_destroy(s_dce);
    s_dce = NULL;

    if (s_ppp_netif) {
        esp_netif_destroy(s_ppp_netif);
        s_ppp_netif = NULL;
    }

    if (s_state_mtx && xSemaphoreTake(s_state_mtx, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_connected = false;
        snprintf(s_ip, sizeof(s_ip), "0.0.0.0");
        xSemaphoreGive(s_state_mtx);
    }
    ESP_LOGI(TAG, "PPPoS stopped");
    return ESP_OK;
}

bool modem_ppp_is_connected(void)
{
    bool v = false;
    if (s_state_mtx && xSemaphoreTake(s_state_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
        v = s_connected;
        xSemaphoreGive(s_state_mtx);
    }
    return v;
}

const char *modem_ppp_get_ip(void)
{
    return s_ip;
}

// === SIM READY: публичный геттер ===
bool modem_ppp_sim_ready(void)
{
    return s_sim_ready;
}

esp_netif_t *modem_ppp_get_netif(void)
{
    return s_ppp_netif;
}