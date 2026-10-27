# FSU Trafic FW v1 — контроллер светофорного объекта (root + node)

Прошивка для ESP32-S3, реализующая распределённый дорожный контроллер:
**root** (мастер) и **node** (ведомый), связанные по Wi-Fi (SoftAP + TCP),
с синхронизацией фаз, зеркалированием аварий, синхронизацией времени,
диагностикой каналов (V_IN / I_LOAD / V_IMON) и управлением через MQTT.

---

## 1. Состав проекта

| Файл | Назначение |
|---|---|
| `blink_example_main.c` | Точка входа `app_main()`, все фоновые задачи, обработчики команд/событий root↔node |
| `traffic_light.c/.h` | Data-driven сценарий светофора (фазы, аварии, recovery, пешеходная кнопка) |
| `mux.c/.h` | Аналоговый мультиплексор + ADC: V_IN, V_IMON, I_LOAD |
| `mux_data.c/.h` | Общий снимок измерений (mutex-protected) |
| `shift_register.c/.h` | 2×SN74HC595 — 16 выходных каналов (лампы) |
| `board_pins.h` | Распиновка ESP32-S3 (GPIO) |
| `pi4ioe5v6408.c/.h` | I2C-расширитель (expander_a 0x43, expander_b 0x44) |
| `i2c_bus.c/.h` | Общая I2C-шина |
| `config_manager.c/.h` | Конфиг в NVS + загрузка/экспорт JSON |
| `traffic_nvs.c/.h` | Сохранение traffic-конфига в отдельный namespace |
| `wifi_ap.c/.h` | STA/AP Wi-Fi, скан, watchdog |
| `wifi_mqtt.c/.h` | MQTT-клиент, публикация JSON, приём команд, зеркалирование fault |
| `node_link.c/.h` | TCP-канал root↔node (`cmd:`, `evt:`, `cfg:`, `status:`) |
| `time_sync.c/.h` | Синхронизация даты/времени root↔node, DS1307, SNTP |
| `rs485.c/.h` | RS-485 (UART2, RE через shift register) |
| `can_bus.c/.h` | TWAI (CAN) — новый драйвер IDF |
| `modem_ppp.c/.h` | PPPoS через A7680C (резервный интернет) |
| `eth_w5500.c/.h` | Ethernet W5500 (SPI3) |
| `lora.c/.h` | LLCC68 / Ra-01 (SPI2) — не используется в основном сценарии |
| `mesh_link.c/.h`, `mesh_protocol.h` | ESP-MESH — **отключено** (временами убрано из сборки) |
| `config.json` | Встроенный конфиг (binary embed через `target_add_binary_data`) |
| `CMakeLists.txt`, `partitions.csv`, `sdkconfig.defaults`, `idf_component.yml` | Сборка |

---

## 2. Роли устройств

Роль определяется в NVS (`device_cfg/role`, `device_cfg/role_lock`) и
**переживает** `config:reload`. Меняется кнопкой BOOT (long press ≥ 3 с).

| Роль | Поведение |
|---|---|
| **ROOT** (master) | SoftAP `FSU_Traffic_AP`, TCP-сервер `:3333`, MQTT-клиент, SNTP, источник фаз |
| **NODE** (slave)  | STA к AP root, TCP-клиент к `192.168.4.1:3333`, ведомый фаз, отдаёт телеметрию |

---

## 3. Взаимодействие root ↔ node

Транспорт — **TCP** (порт `3333`) поверх Wi-Fi SoftAP root'а.
Построчный протокол (`\n` в конце строки):

| Префикс | Направление | Назначение |
|---|---|---|
| `cfg:<json>` | root → node | начальный/обновлённый конфиг |
| `cmd:traffic:start` / `stop` / `pause` / `resume` / `reset` / `next` | root → node | управление светофором |
| `cmd:traffic:mode:<1..3>` | root → node | смена режима |
| `cmd:relay:<N>:<0/1>` / `cmd:sr:<N>:<0/1>` | root → node | прямое управление выходом |
| `cmd:fault:<mode>:<ch>,<..>:<reason>,<..>` | root ↔ node | зеркало аварии |
| `cmd:fault:none` | root → node | снятие аварии (после reconnect) |
| `cmd:sync:<phase>:<pm>:<dur>:<ts>[:<blink_on>]` | root → node | синхронизация фазы (10 Гц для YELLOW_BLINK, 4 Гц иначе) |
| `cmd:ping` / `cmd:ping:<us>` | root → node | ping без/с RTT-замером |
| `cmd:delay:<us>` | root → node | RTT/2 — компенсация односторонней задержки |
| `cmd:time:<sec>:<ms>:<rtt_us>` | root → node | установка времени |
| `cmd:timeq` | root → node | запрос времени node |
| `cmd:pedestrian:request` | root → node | запрос пешеходной фазы |
| `cmd:rs485:*`, `cmd:can:*` | root → node | прокси команд в локальные шины |
| `evt:pong:<ts>:<t_recv>:<t_send>` | node → root | ответ на ping |
| `evt:time:<sec>:<ms>:<uptime>` | node → root | отчёт о времени (для контроля дрейфа) |
| `evt:fault:<mode>:<ch>,<..>:<reason>,<..>` | node → root | авария с node |
| `evt:pedestrian:request` | node → root | нажатие P7 на node |
| `status:<json>` | node → root | JSON-телеметрия (1 Гц) |

---

## 4. Логика работы светофора (`traffic_light.c`)

### 4.1. Data-driven сценарий

Конфиг содержит массив `traffic.stages_dd[]`:

```jsonc
{
  "id": 3,
  "name": "Фаза.3",
  "groups": [3],
  "duration_kind": "green",       // или duration_ms для explicit
  "outputs_on":    [2, 5, 8, 9],
  "outputs_blink": []
}
duration_kind связывает длительность стадии с полем traffic.phases_ms:

duration_kind	Поле phases_ms
red	red
red_yellow	red_yellow
green	green
green_blink	green_blink
yellow	yellow
yellow_blink	yellow_blink
ped_green	ped_green
ped_blink	ped_blink
explicit	duration_ms из стадии
Порядок фаз — traffic.sequence[] (например [1,2,3,4,5,6,7,8]).

4.2. traffic_task (20 мс)
блинк outputs_blink — с периодом phases_ms.phase_blink_period (500 мс);

смена фазы по stage_duration_ms();

node (s_follow_root = true) не двигает фазу сам — только sync: от root;

root двигает и рассылает sync: через wifi_mqtt_notify_phase();

пешеходная кнопка (P7) — см. §6.

4.3. Аварии (fault)
Значение	Что означает
NONE	норма
YELLOW_BLINK	мигание жёлтым (1 Гц, настраивается fault_blink_period)
ALL_OFF	все выходы выключены (короткое/питание)
Авария ставится при:

пропаже тока (I_LOAD < imin_mv, imon_state == NO_LOAD/SUSPECT);

V_IN present while OFF (vin_off_confirmations подряд);

перегруз (I_LOAD > TRAFFIC_OVERLOAD_MA mA);

залипании фазы (elapsed > dur + TRAFFIC_PHASE_OVERDUE_MS);

команда извне (MQTT fault:..., sync: с pm >> dur).

Снимается:

try_recovery на node: 3 подряд успешных цикла mux_data → NONE;

cmd:fault:none от root после reconnect;

traffic_light_set_fault(NONE) вручную.

4.4. Зеркалирование fault root ↔ node
on_fault_changed на root → cmd:fault:... node + MQTT;

on_fault_changed на node → evt:fault:... root;

root применяет через wifi_mqtt_apply_remote_fault (приоритет: ALL_OFF > YELLOW_BLINK > NONE);

защита от эха: s_fault_from_remote_ex, s_root_comm_lost_all_off.

5. COMM_LOST: ALL_OFF при потере связи > 30 с
Оба устройства переходят в ALL_OFF, если связь отсутствует дольше
NODE_COMM_LOST_MS = 30000.

Root
root_ap_client_watchdog_task:

нет AP-клиента → traffic_light_stop(), lost_since_ms = now;

через 30 с нет клиента → traffic_light_set_fault(ALL_OFF),
wifi_mqtt_set_root_comm_lost(true);

клиент подключился →

traffic_light_set_fault(NONE);

cmd:fault:none + cmd:traffic:start;

sync: (immediate);

time: (immediate);

wifi_mqtt_set_root_comm_lost(false).

Node
node_link_watchdog_task:

нет sync:/ping: от root > 30 с → traffic_light_stop(),
traffic_light_set_fault(ALL_OFF);

снова пришёл sync: → traffic_light_set_fault(NONE),
traffic_light_start(), s_comm_lost = false.

Пока s_root_comm_lost_all_off = true, wifi_mqtt.c игнорирует
evt:fault:none и NODE.FAULT=NONE от node — иначе node своим
эхом снимал бы ALL_OFF на root раньше времени.

6. Вызывная кнопка пешехода (P7, expander_b pin 7)
Конфигурируется через traffic.pedestrian:

jsonc
"pedestrian": {
  "button_enabled": true,        // false → таск не создаётся, команды игнорируются
  "request_mode": "stage",       // "stage" | "insert" | "min_recall"
  "insert_timeout_ms": 30000,
  "min_recall_s": 5
}
request_mode	Поведение
stage	кнопка вставляет пешеходную фазу следующей (как раньше)
insert	запрос вставляется только если текущая транспортная фаза отработала minGreen; иначе — ждём до insert_timeout_ms, потом отмена
min_recall	кнопка не вставляет фазу, а гарантирует, что следующий пешеходный зелёный будет не короче min_recall_s (эффективная длительность через effective_stage_duration_ms)
Если button_enabled = false:

button_p7_task не создаётся в app_main;

traffic_light_request_pedestrian() → лог «button disabled in config» и return;

cmd:pedestrian:request / evt:pedestrian:request — игнорируются
(on_cmd_from_root, on_evt_from_node).

Пешеходные стадии 7/8 в sequence работают всегда — это независимо от
кнопки. Если пешеходной фазы не нужно вообще — уберите 7/8 из sequence.

7. Диагностика каналов (mux_task)
mux_task раз в 1 с:

проходит 16 каналов;

для активных: mux_select_channel, mux_read_vin_mv — V_IN;

board_read_iload_raw — DIAG_EN + 8× ADC → V_IMON + I_LOAD;

в аварии: fault-канал включается только на время своего чтения
(MUX_FAULT_SETTLE_MS = 12), затем сразу гасится — визуально не мигает;

traffic_light_check_faults(&local) — проверка порогов;

mux_data_set(&local) — публикация снимка.

board_read_iload_raw и mux_task берут общий мьютекс mux_get_mutex()
вокруг «select channel + ADC» — иначе mux_task и try_recovery могут
получить ложный OK.

8. MQTT
Топики (база — mqtt.base_topic + MAC):

Топик	Направление	Содержимое
<base>_<MAC>	root → MQTT	JSON-телеметрия (1 Гц)
<base>_<MAC>/LWT	—	online / offline
<base>_<MAC>/Set	MQTT → root	текстовые команды
<base>_<MAC>/SetJson	MQTT → root	новый JSON-конфиг
<base>_<MAC>/Layer	root → MQTT	role:root IP:… rssi:… ap:… clients:…
<base>_<MAC>/key_pressed	root → MQTT	MAC при нажатии BOOT
<base>_<MAC>/Node	root → MQTT	JSON от node (проксирование)
Команды (/Set)
text
traffic:start|stop|pause|resume|reset|next
traffic:mode:1..3
relay:N:0|1        (или sr:N:0|1)
toggle:N
all:on | all:off
fault:yellow_blink:…
fault:none
test:N
pedestrian:request
rs485:baud:115200
rs485:send:<hex>
rs485:stats | rs485:reset_stats
can:send:<id>:<hex>:<ext>
can:standby | can:normal
can:stats   | can:reset_stats
rtt | rtt:reset
net:info | wifi:reconnect
role:root | role:node          (reboot)
config:reload | nvs:erase      (reboot)
key_pressed:<MAC>
JSON-телеметрия root
jsonc
{
  "SR":  {"state": 123, "hex": "0x007B", "outputs": [0,1,3,4,5,6]},
  "NET": {"role":"root", "rssi":-55, "channel":6, "ip":"192.168.4.1",
          "ap_running":1, "ap_ssid":"FSU_Traffic_AP",
          "ap_clients":1, "sta_connected":1, "has_ip":1},
  "TRAFFIC": {"state":"RUNNING", "mode":3, "phase":2,
              "phase_ms":1234, "phase_duration_ms":10000, "phase_count":8},
  "FAULT": {"mode":"NONE", "count":0, "channels":[], "reasons":[]},
  "NODE": null,
  "NODE_FAULT": null,
  "RS485": {"rx_bytes":…,"rx_frames":…,"rx_overflows":…,"tx_bytes":…,"tx_errors":…},
  "CAN":   {"rx_frames":…,"tx_frames":…,"tx_errors":…,
            "bus_off":…,"recovery":…,"error_passive":…,"state":…},
  "RTT":   {"us":…,"delay_us":…,"samples":…},
  "ch00":  {"V_IN":12.1,"I_LOAD":250,"V_IMON":500,"STATE":"OK","SR":1,"DIAG":1},
  …
  "ch15":  {…}
}
Node дополнительно шлёт "TIME": {"unix":…, "ms":…} и "LOST": 0|1.

9. Синхронизация времени (time_sync.c)
Root — источник времени. При старте: DS1307 → time_sync_init();
при появлении IP — time_sync_start_sntp() (SNTP + запись в DS1307).

Node — получает cmd:time:<sec>:<ms>:<rtt_us>. RTT/2 прибавляется
к ms (компенсация односторонней задержки).

Root раз в 30 с шлёт time:, node раз в 5 с отчитывается
evt:time:<sec>:<ms>:<uptime> — root логирует дрейф > 2 с.

DS1307 best-effort: если RTC не отвечает — лог один раз, время
остаётся валидным в системных часах (settimeofday).

10. RTT-компенсация фаз
Root шлёт cmd:ping:<us> каждые 2 с (RTT_PING_PERIOD_MS).

Node отвечает evt:pong:<ts>:<t_recv>:<t_send>.

Root считает rtt = now_us - ts, delay = rtt / 2, сглаживает
((old*7 + new*3) / 10) → шлёт cmd:delay:<us>.

Node в on_cmd_from_root (sync:) прибавляет delay_us/1000 мс
к pm при вызове traffic_light_force_phase().

Если s_last_pong_ms > 10 с — delay_us = 0.

11. Конфиг (config.json → NVS)
Загружается при старте: если в NVS пусто — config_load_json(embedded).
Далее управляется через /SetJson (MQTT) или cmd:cfg: (от root к node).

Ключевые секции:

Секция	Содержимое
meta	version, id, name
wifi	ssid, password (STA к роутеру)
mqtt	uri, port, user, password, base_topic
traffic	mode, enabled, auto_start, pedestrian, pins, phases_ms, fault, stages_dd[], sequence[], transitions[]
shift_register.outputs[]	ch, name, active
IGM, stages, plans, schedule, controlSources, modules	справочные (парсятся, но не участвуют в DD-логике)
NVS namespace:

Namespace	Ключ	Что
app_cfg	cfg_blob	весь app_config_t (blob)
device_cfg	role, role_lock	роль устройства (переживает config:reload)
traffic_cfg	mode, enabled, full	legacy + traffic_config_t
mqtt_cfg	broker_url, port, user, password, base_topic, lwt_msg	MQTT
12. Сборка
bash
. $IDF_PATH/export.sh
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
CMakeLists.txt (проект) использует MINIMAL_BUILD. Партиции —
partitions.csv (nvs 128K, phy_init 4K, factory 3M). Встроенный
config.json пакуется через target_add_binary_data(... TEXT).

Зависимости (idf_component.yml):

text
idf: '>=6.1'
espressif/led_strip: ~3.0.3
espressif/cjson: ^1.7.19
espressif/mqtt: ^1.1.0
espressif/w5500: ^2.0.1
larryli/ds1307: ^1.0.1
espressif/esp_modem: "^2.0.0"
13. Отладка
Логи
board — задачи верхнего уровня (app_main, watchdog, phase_sync, кнопки);

traffic — фазы, аварии, recovery;

wifi_mqtt — MQTT-события, команды, зеркалирование fault;

node_link — TCP-канал (RX/TX строки);

time_sync — синхронизация времени;

mux, sr, can_bus, rs485, CFG, WIFI, modem_ppp.

Команды диагностики (MQTT /Set)
rtt — текущий RTT и delay_us;

rs485:stats / can:stats — счётчики;

net:info — Wi-Fi STA + AP clients;

test:N — включить канал N и прочитать V_IMON/I_LOAD.

Инварианты
s_mqtt_connected == false → публикация дропается, wifi_mqtt_restart() через 30 с;

traffic_light_get_state() == STOPPED → phase_sync не шлёт sync:;

traffic_light_get_fault() == ALL_OFF → phase_sync тоже не шлёт sync:;

mux_task в аварии: traffic_light_set_mux_reading(true) → fault_blink_yellow() не мигает (визуально не дёргается).

14. Известные ограничения
remote_fault_watchdog удалён — его роль выполняет root_ap_client_watchdog_task (ALL_OFF при lost > 30 с).

mesh_link.c/.h — в проекте есть, но не собирается (закомментировано в CMakeLists.txt). Транспорт root↔node — TCP.

lora.c/.h — компилируется, но не участвует в основном сценарии.

DS1307 может быть не распаян — тогда запись в RTC логируется один раз и игнорируется (best-effort).

picolibc: timegm() отсутствует → используется timegm_compat() в time_sync.c.

15. Быстрый старт
Собрать и прошить root (роль по умолчанию — master, если NVS пуст).

Прошить node; удерживать BOOT ≥ 3 с до подтверждающего мигания → reboot → станет slave.

На root: лог AP ready: SSID='FSU_Traffic_AP' IP=192.168.4.1.

На node: лог NODE: connecting to 'FSU_Traffic_AP' → phase_sync: node connected.

В MQTT: топики <base>_<MAC> и <base>_<MAC>/Node с телеметрией 1 Гц.

Команды: <base>_<MAC>/Set → traffic:start, traffic:mode:3, fault:none и т.д.

text

---

## Сводка ключевых изменений, отражённых в README

| Механизм | Где в коде | Где в README |
|---|---|---|
| COMM_LOST > 30 с → ALL_OFF | `node_link_watchdog_task`, `root_ap_client_watchdog_task` | §5 |
| Восстановление при reconnect | `traffic_light_set_fault(NONE)`, `cmd:fault:none` | §5 |
| `wifi_mqtt_set_root_comm_lost()` | `wifi_mqtt.c/.h` | §5, §8 |
| TIME_SYNC (DS1307, SNTP, RTT/2) | `time_sync.c`, `on_cmd_from_root`, `on_evt_from_node` | §9 |
| RTT-компенсация фаз (`ping`/`pong`/`delay`) | `phase_sync_task`, `on_evt_from_node`, `on_cmd_from_root` | §10 |
| Вызывная кнопка P7 (3 режима + `button_enabled`) | `traffic.pedestrian`, `traffic_light_request_pedestrian`, `button_p7_task` | §6 |
| Тихий `mux_task` (12 мс settle, канал только на чтение) | `mux_task` | §7 |
| Удаление `remote_fault_watchdog` | `blink_example_main.c` | §14 |
| `timegm_compat()` для picolibc | `time_sync.c` | §14 |
