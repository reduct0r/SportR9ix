# SportR9ix

**SmartPort → MAVLink: телеметрия FrSky R9 в Mission Planner**

**English documentation:** [README.en.md](README.en.md)

**SportR9ix** — открытая прошивка для **ESP32** и набор Python-утилит. Проект превращает **ArduPilot FrSky Passthrough** с заднего **SmartPort** наземного модуля **R9M** в поток **MAVLink 2** для **Mission Planner** (USB или WiFi UDP). Позволяет смотреть attitude, GPS, батарею и другие поля по радиолинии 900 MHz без прямого USB к полётному контроллеру.

Репозиторий: [github.com/reduct0r/SportR9ix](https://github.com/reduct0r/SportR9ix)

---

## Содержание

1. [Назначение](#назначение)
2. [Совместимое оборудование](#совместимое-оборудование)
3. [Возможности и ограничения](#возможности-и-ограничения)
4. [Архитектура](#архитектура)
5. [Схема подключения](#схема-подключения)
6. [Настройка ArduPilot](#настройка-ardupilot)
7. [Установка и сборка](#установка-и-сборка)
8. [Прошивка ESP32](#прошивка-esp32)
9. [Подключение Mission Planner](#подключение-mission-planner)
10. [Справочник `config.h`](#справочник-configh)
11. [Декодирование Passthrough и MAVLink](#декодирование-passthrough-и-mavlink)
12. [Структура прошивки](#структура-прошивки)
13. [Доработка под себя](#доработка-под-себя)
14. [Инструменты тестирования](#инструменты-тестирования)
15. [Устранение неполадок](#устранение-неполадок)

---

## Назначение

На борту ArduPilot отдаёт телеметрию в приёмник FrSky в формате **Passthrough** (`SERIALx_PROTOCOL = 10`). Пульт (ER9X / OpenTX / EdgeTX) опрашивает шину SmartPort модуля **R9M**; ответы с борта проходят по RF и появляются на заднем разъёме S.Port модуля.

ESP32 **пассивно слушает** эту шину (или сам опрашивает модуль в режиме стенда), **декодирует** пакеты `0x5000`–`0x50FF`, **формирует MAVLink 2** и передаёт в Mission Planner по **USB** или **WiFi UDP**.

Это **мост телеметрии для отображения**, а не полноценная GCS-связь с автопилотом.

---

## Совместимое оборудование

### Полётный контроллер

| Компонент | Требования |
|-----------|------------|
| Автопилот | **ArduPilot** (ArduPlane, ArduCopter, ArduRover) с поддержкой FrSky Passthrough |
| Порт к приёмнику | `SERIALx_PROTOCOL = 10`, `SERIALx_BAUD = 57` (57600) |
| Проводка борта | Инвертор UART между FC и приёмником (как в штатной схеме FrSky) |

### Приёмник и передатчик (RF)

| Компонент | Статус |
|-----------|--------|
| FrSky **R9 Slim+** / **R9 Mini** (борт) | Проверено в проекте SportR9ix |
| FrSky **R9M** / **R9M Lite** (наземный модуль, задний S.Port) | Проверено |
| Другие приёмники с ArduPilot Passthrough на SmartPort | Ожидаемо совместимы при `FRSKY_DNLINK_ID = 27` (0x1B) |

### Пульт (мастер шины SmartPort)

| Прошивка | Режим в `config.h` |
|----------|-------------------|
| **ER9X** (Turnigy 9X и аналоги) | `SPORT_ACTIVE_POLL 0` — listen-only, пульт опрашивает шину |
| **OpenTX / EdgeTX** | `SPORT_ACTIVE_POLL 0` или `1` (стенд без пульта) |
| Без пульта (стенд) | `SPORT_ACTIVE_POLL 1` — ESP опрашивает R9M |

### Микроконтроллер

| Параметр | Значение |
|----------|----------|
| Чип | **ESP32** (тест: ESP32-D0WD-V3, DevKit 30 pin) |
| Framework | Arduino (PlatformIO `espressif32`) |
| Flash / RAM | ~744 KB flash, ~46 KB RAM (типичная сборка) |
| USB-UART | CH9102 / CP2102 / аналог, **57600** для MAVLink |

**Не использовать GPIO12** (D12) — при сбросе должен быть LOW.

### Наземная станция

| ПО | Подключение |
|----|-------------|
| **Mission Planner** | USB COM @ 57600 или UDP @ 14550 |
| Другие GCS | Не тестировались; возможна совместимость по MAVLink UDP |

---

## Возможности и ограничения

### Что работает

- Attitude (roll, pitch, yaw)
- GPS (координаты, fix, HDOP, высота AMSL при наличии fix)
- Высота над home, дистанция/курс до home (из passthrough)
- Вертикальная и горизонтальная скорость (с квантованием passthrough)
- Батарея: напряжение, ток, mAh, остаток %
- Режим полёта, armed, failsafe-флаги (отображение)
- Текст STATUSTEXT с FC (чанки 0x5000)
- Точка маршрута (MISSION_CURRENT)
- Одновременный вывод: **USB + WiFi**

### Что не поддерживается

- Загрузка миссий, изменение параметров, арм/дизарм через этот канал
- Полная частота и точность USB-MAVLink (passthrough квантует, напр. pitch 0.2°, высота 0.1 m)
- Управление камерой, gimbal, MAVLink-команды к FC
- Прямое подключение к **COM полётника** — MP должен быть подключён к **ESP**, не к Pixhawk

---

## Архитектура

```
ArduPilot (SERIALx_PROTOCOL=10)
    → [инвертор] → FrSky приёмник (борт)
        → RF 900 MHz → R9M (модуль пульта)
            → S.Port+ ── GPIO16 (RX2) ESP32
            → GND ──── GND ESP32
                → [SportParser] → [PassthroughDecoder] → [MavlinkSender]
                    → USB Serial 57600  ─┐
                    → WiFi UDP 14550   ─┴→ Mission Planner
```

**Два уровня инверсии UART (не путать):**

| Участок | Инверсия |
|---------|----------|
| FC → приёмник (борт) | Аппаратный инвертор |
| R9M → ESP32 (земля) | Программная (`SPORT_UART_INVERT`) |

---

## Схема подключения

### R9M → ESP32

| R9M (задний разъём) | ESP32 DevKit |
|---------------------|--------------|
| **GND** | **GND** |
| **S.Port+** | **GPIO16** (RX2, `SPORT_PIN`) |
| S.Port− | не подключать |
| +5V | не подключать (ESP питается от USB) |

SmartPort: **57600 baud**, однопроводная линия, **инвертированный UART**.

---

## Настройка ArduPilot

На серийном порту к FrSky-приёмнику:

```text
SERIALx_PROTOCOL = 10    # FrSky Passthrough
SERIALx_BAUD = 57        # 57600
```

Убедитесь, что downlink ID соответствует опросу (по умолчанию в прошивке: физический ID **27** → wire **0x1B**).

Телеметрия на пульте ER9X/OpenTX должна показывать passthrough-данные с борта при включённом RF.

---

## Установка и сборка

### Требования

- [PlatformIO](https://platformio.org/) (CLI или VS Code / Cursor)
- Python 3 + `pyserial` (для скриптов в `tools/`)
- Драйвер USB-UART для ESP32

### Клонирование и сборка

```bash
git clone git@github.com:reduct0r/SportR9ix.git
cd SportR9ix
pio run
```

Укажите порт прошивки в `platformio.ini` (раскомментируйте `upload_port`) или через CLI:

```bash
pio run -t upload --upload-port COM8
```

---

## Прошивка ESP32

```bash
pio run -t upload
```

После загрузки в Mission Planner (Messages) появятся:

- `SKAT telem bridge ready`
- `SKAT WiFi: SKAT-TELEM UDP 14550` (если `WIFI_ENABLED 1`)

При отсутствии passthrough с борта (>15 с, но шина активна):

- `SKAT: no FC passthrough - aircraft+RF?`

---

## Подключение Mission Planner

### USB (рекомендуется для отладки)

1. Подключите ESP32 по USB.
2. Mission Planner → **Connect** → COM-порт ESP, **57600**.
3. **Не подключайте** одновременно USB к полётнику.

### WiFi UDP

1. ESP поднимает AP: **`SKAT-TELEM`** / пароль **`skat12345`**, IP **`192.168.4.1`**.
2. Закройте все окна Mission Planner (освободите порт **14550**).
3. ПК → WiFi **`SKAT-TELEM`**, тип сети **Private**.
4. Разрешите `MissionPlanner.exe` в брандмауэре (UDP 14550, частная сеть).
5. MP → **UDP** → порт **14550** → Remote host **пустой** (Listen) или **`192.168.4.1`** → **Connect**.
6. Не используйте USB COM и UDP одновременно к одному мосту.

**Ошибка «порт занят»:** `taskkill /IM MissionPlanner.exe /F`, перезапустите MP.

**Ошибка «socket closed»:** Disconnect → закройте MP → подключитесь снова; проверьте WiFi и ping `192.168.4.1`.

---

## Справочник `config.h`

Все настройки — в файле [`include/config.h`](include/config.h).

### SmartPort (вход)

| Макрос | По умолчанию | Описание |
|--------|--------------|----------|
| `SPORT_PIN` | `16` | GPIO RX SmartPort (RX2) |
| `SPORT_BAUD` | `57600` | Скорость SmartPort |
| `SPORT_UART_INVERT` | `true` | Программная инверсия UART (R9M → ESP) |
| `SPORT_ACTIVE_POLL` | `0` | `0` = listen-only (ER9X мастер), `1` = ESP опрашивает R9M |
| `SPORT_OPENTX_ROTATE_POLL` | `1` | Ротация 28 wire ID (как OpenTX) при active poll |
| `SPORT_POLL_ID_COUNT` | `28` | Число слотов опроса |
| `SPORT_POLL_INTERVAL_MS` | `12` | Период опроса (active poll) |
| `SPORT_POLL_RX_WINDOW_MS` | `8` | Окно приёма после poll |
| `SPORT_DEFAULT_SENSOR_ID` | `0x1B` | Sensor ID при опросе без ротации (ID 27) |
| `SPORT_SKIP_CRC` | `0` | `1` = принимать кадры без CRC (отладка) |
| `SPORT_AUTO_POLL_FALLBACK` | `0` | Авто-переход в poll при тишине (осторожно с пультом) |
| `SPORT_PERIODIC_SCAN` | `0` | Периодическое сканирование ID (конфликт с пультом) |
| `SPORT_SCAN_*` | см. файл | Параметры сканирования |
| `DEBUG_USB` | `0` | `1` = отладочный текст в USB (ломает MP на том же порту) |

### MAVLink (выход)

| Макрос | По умолчанию | Описание |
|--------|--------------|----------|
| `MAVLINK_BAUD` | `57600` | Скорость USB Serial |
| `MAVLINK_SYSTEM_ID` | `1` | MAVLink system ID |
| `MAVLINK_COMPONENT_ID` | `1` | MAVLink component ID |
| `MAVLINK_DIAG` | `0` | `1` = STATUSTEXT с диагностикой шины (ids, hex, sync) |
| `SPORT_RAW_DIAG_INTERVAL_MS` | `10000` | Интервал hex-дампа (только при `MAVLINK_DIAG=1`) |

### WiFi

| Макрос | По умолчанию | Описание |
|--------|--------------|----------|
| `WIFI_ENABLED` | `1` | Точка доступа + UDP MAVLink |
| `WIFI_AP_SSID` | `SKAT-TELEM` | Имя сети |
| `WIFI_AP_PASSWORD` | `skat12345` | Пароль WPA2 |
| `WIFI_AP_IP_OCTETS` | `192,168,4,1` | IP ESP в режиме AP |
| `WIFI_AP_BCAST_OCTETS` | `192,168,4,255` | Broadcast подсети для UDP |
| `WIFI_UDP_PORT` | `14550` | Порт MAVLink UDP (Mission Planner) |
| `WIFI_UDP_BROADCAST` | `1` | Fallback broadcast (если клиент ещё не известен) |

### Батарея

| Макрос | По умолчанию | Описание |
|--------|--------------|----------|
| `BATT_CAPACITY_MAH` | `0` | Резервная ёмкость mAh; `0` = ждать 0x5007 с FC |

---

## Декодирование Passthrough и MAVLink

### App ID ArduPilot (вход)

| App ID | Содержимое |
|--------|------------|
| `0x5000` | STATUSTEXT (чанки) |
| `0x5001` | Режим, armed, failsafe, throttle, IMU temp |
| `0x5002` | GPS fix, sats, HDOP, AMSL alt |
| `0x5003` / `0x5008` | Батарея 1 / 2 |
| `0x5004` | Дистанция/курс/высота над home |
| `0x5005` | Vario, yaw, ground/airspeed |
| `0x5006` | Roll, pitch, rangefinder |
| `0x5007` | Параметры (ёмкость АКБ и др.) |
| `0x5009` / `0x500D` | Waypoint |
| `0x500A`–`0x500C` | RPM, terrain, wind (декодируются, не все поля в MP) |
| `0x0800` | GPS lat/lon |

### Сообщения MAVLink (выход)

| MAVLink | Источник / назначение в MP |
|---------|---------------------------|
| `HEARTBEAT` | Тип коптера, armed bit |
| `ATTITUDE` | Roll, pitch, yaw |
| `GLOBAL_POSITION_INT` | GPS, relative alt, vz |
| `GPS_RAW_INT` | При валидном GPS |
| `VFR_HUD` | Airspeed, groundspeed, heading, throttle, alt, climb |
| `SYS_STATUS` | Напряжение, ток, remaining % |
| `BATTERY_STATUS` | Детали АКБ для MP |
| `MISSION_CURRENT` | Номер waypoint |
| `STATUSTEXT` | FC + служебные сообщения моста |
| `PARAM_VALUE` | Заглушка `SKAT_TELEM` (ответ MP на запрос параметров) |

---

## Структура прошивки

```text
include/config.h          — все настройки
src/main.cpp              — цикл: SmartPort → decode → MAVLink
src/SportParser.cpp       — приём кадров (sequential + sliding window)
src/PassthroughDecoder.cpp — декод App ID → TelemetryState
src/MavlinkSender.cpp     — MAVLink 2, USB + WiFi UDP
src/SportDiag.cpp         — диагностика App ID (MAVLINK_DIAG)
tools/                    — Python-скрипты проверки
platformio.ini            — сборка ESP32
```

**Listen mode** (`SPORT_ACTIVE_POLL 0`): скользящее окно CRC извлекает кадры из потока с байтами опроса `0x7E` от пульта.

**Active poll** (`SPORT_ACTIVE_POLL 1`): классический последовательный парсер; ESP сам шлёт poll на R9M.

---

## Доработка под себя

1. Скопируйте репозиторий, измените [`include/config.h`](include/config.h) под свою проводку и режим пульта.
2. Другой GPIO: измените `SPORT_PIN` (избегайте GPIO12).
3. Другой WiFi: `WIFI_AP_SSID`, `WIFI_AP_PASSWORD`, при необходимости `WIFI_UDP_PORT`.
4. Новый датчик passthrough: добавьте case в `PassthroughDecoder::handlePacket`, поле в `TelemetryState`, отправку в `MavlinkSender::update`.
5. Отладка шины: `MAVLINK_DIAG 1`, пересоберите; смотрите STATUSTEXT в MP или `tools/monitor_full_com8.py`.
6. Сборка: `pio run`; прошивка: `pio run -t upload`.

Декодирование соответствует [ArduPilot AP_Frsky_SPort_Passthrough](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp).

---

## Инструменты тестирования

Скрипты в [`tools/`](tools/), зависимость: `pip install pyserial`.

| Скрипт | Назначение | Пример |
|--------|------------|--------|
| [`verify_mp_telemetry.py`](tools/verify_mp_telemetry.py) | Проверка полей MP (ATTITUDE, VFR_HUD, BATTERY) | `python tools/verify_mp_telemetry.py COM8 40` |
| [`live_mavlink_watch.py`](tools/live_mavlink_watch.py) | Потоковый монитор alt/climb/roll + статистика | `python tools/live_mavlink_watch.py COM8 60` |
| [`monitor_full_com8.py`](tools/monitor_full_com8.py) | Полный захват + App ID histogram (нужен `MAVLINK_DIAG=1`) | `python tools/monitor_full_com8.py COM8 55` |
| [`analyze_com8_mavlink.py`](tools/analyze_com8_mavlink.py) | Разбор счётчиков MAVLink-сообщений | `python tools/analyze_com8_mavlink.py COM8` |
| [`sport_listen_com8.py`](tools/sport_listen_com8.py) | Прослушивание сырого потока | `python tools/sport_listen_com8.py COM8` |
| [`sniff_com7.py`](tools/sniff_com7.py) | Сравнение с прямым MAVLink FC по USB | `python tools/sniff_com7.py COM7` |
| [`test_mp_params.py`](tools/test_mp_params.py) | Проверка ответа на PARAM_REQUEST | `python tools/test_mp_params.py COM8` |
| [`monitor_esp.py`](tools/monitor_esp.py) | Монитор USB при `DEBUG_USB=1` | `python tools/monitor_esp.py COM8` |

**Типичная проверка после прошивки:**

```bash
python tools/verify_mp_telemetry.py COM8 45
```

Ожидается `VERDICT: PASS` при включённом борте и RF-линке. Счётчики `ATTITUDE` и `VFR_HUD` должны расти; при `pt=0` в diag — нет passthrough с FC.

---

## Устранение неполадок

| Симптом | Причина | Действие |
|---------|---------|----------|
| В MP только HEARTBEAT | Нет passthrough на шине | Включите борт + RF; проверьте `SERIALx_PROTOCOL=10` |
| `F101`/`F104`, нет `0x500x` | Только телеметрия модуля R9M | Нет линка с бортом или FC не шлёт passthrough |
| Alt/climb = 0 на столе | Квантование passthrough | Норма; USB к FC показывает точнее |
| Pitch дёргается по WiFi | Дубли UDP (исправлено) | Обновите прошивку; один UDP-адрес на пакет |
| MP: порт 14550 занят | Старый процесс MP | Закройте все окна MP / `taskkill` |
| Нет WiFi `SKAT-TELEM` | ESP не прошит / нет питания | Перепрошейте, проверьте USB |

---

## Лицензия

Проект распространяется как есть, для образовательных и любительских целей. ArduPilot и FrSky — торговые марки соответствующих правообладателей.

---

**SportR9ix** — SmartPort MAVLink telemetry bridge for long-range FrSky R9 systems.
