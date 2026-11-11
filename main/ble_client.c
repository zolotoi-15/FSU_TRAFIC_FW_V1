// ble_client.c (для NODE)
#include "ble_client.h"
#include "transport.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "BLE_CLIENT";
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_char_handle; // Handle для приема уведомлений от ROOT

// UUID сервиса, который мы ищем (должен совпадать с ROOT)
static const ble_uuid128_t s_svc_uuid = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

// Обработчик GAP-событий (сканирование, подключение)
static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_DISC: { // Нашли устройство при сканировании
            struct ble_gap_disc_desc *disc = &event->disc;
            // Проверяем, содержит ли реклама наш сервис
            struct ble_hs_adv_fields fields;
            if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) == 0) {
                if (fields.uuids128 != NULL) {
                    for (int i = 0; i < fields.num_uuids128; i++) {
                        if (ble_uuid_cmp(&fields.uuids128[i].u, &s_svc_uuid.u) == 0) {
                            ble_gap_disc_cancel(); // Нашли, прекращаем скан
                            ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &disc->addr, 30000, NULL, gap_event_handler, NULL);
                            return 0;
                        }
                    }
                }
            }
            break;
        }
        case BLE_GAP_EVENT_CONNECT: {
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
                // После подключения нужно обнаружить сервисы и характеристики
                ble_gattc_disc_all_svcs(s_conn_handle, svc_disc_cb, NULL);
            }
            break;
        }
        case BLE_GAP_EVENT_NOTIFY_RX: { // Пришли данные от ROOT
            uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
            if (len > 0 && len <= 512) {
                uint8_t buf[512];
                os_mbuf_copydata(event->notify_rx.om, 0, len, buf);
                transport_handle_raw_rx(buf, len); // Передаем в транспорт
            }
            break;
        }
        // ... обработка DISCONNECT и других событий ...
    }
    return 0;
}

// Коллбэк для поиска сервиса и характеристики TX
static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service, void *arg) {
    // ... логика поиска сервиса по UUID и вызова ble_gattc_disc_all_chrs ...
    return 0;
}

// Функция для отправки данных через BLE (вызывается из transport.c)
bool ble_client_send(const uint8_t *data, size_t len) {
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return false;
    // Для отправки данных от клиента серверу используется Write
    return ble_gattc_write_no_rsp(s_conn_handle, s_tx_char_handle, 
                                  OS_MBUF_FROM_FLAT(data, len)) == 0;
}