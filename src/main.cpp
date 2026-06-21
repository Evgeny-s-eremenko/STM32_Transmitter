#include <Wire.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_VEML7700.h"
#include <SPI.h>
#include <RH_NRF905.h>
#include <IWatchdog.h>

// ---------------------------- Пины и устройства ----------------------------
#define NRF905_SPI_SCK      PA5
#define NRF905_SPI_MISO     PA6
#define NRF_SPI_MOSI        PA7
#define NRF905_CE           PB0
#define NRF905_TX_EN        PB2
#define NRF905_CS           PA4
#define NRF905_PWR_UP_PIN   PA3
#define NRF905_DR_PIN       PB1

#define UV_PIN              PA0     // Аналоговый вход S12SD
#define ZH07_PWR_SAVE_PIN   PA8    // Управление питанием ZH07
#define FAN_PIN             PA11   // Управление вентилятором через MOSFET

#define RF_CHANNEL          175    // 439.9 МГц

// -------- Флаги статуса (отправляются в пакете) --------
#define ST_NORMAL   0x01
#define ST_HEATER   0x02
#define ST_COOLING  0x03
#define ST_FAN_OFF  0x00
#define ST_FAN_ON   0x01

// [FIX 7] Именованные константы для смещений в пакете ZH07B
// Источник: ZH07B datasheet, раздел "Output data format", стандартная атмосфера
#define ZH07_PM25_HIGH  6
#define ZH07_PM25_LOW   7
#define ZH07_PM10_HIGH  8
#define ZH07_PM10_LOW   9

// -------- Пороги для управления вентилятором (LUX) --------
const float LUX_THRESHOLD_ON  = 8000.0f;
const float LUX_THRESHOLD_OFF = 4000.0f;

// -------- Пороги температуры для датчика PM --------
const float PM_TEMP_THRESHOLD_STOP   = -10.0f;
const float PM_TEMP_THRESHOLD_RESUME =  -8.0f;

RH_NRF905         driver(NRF905_CE, NRF905_TX_EN, NRF905_CS);
Adafruit_SHT31    sht31(&Wire);
Adafruit_VEML7700 veml = Adafruit_VEML7700();

// ------------------------ Прототипы ----------------------------------------
uint8_t calcChecksum(const uint8_t *buf, size_t len);
void    pack16(uint8_t *dst, uint16_t v);
void    pack32(uint8_t *dst, uint32_t v);
void    sendBinaryPacket();
void    processCommand(const char *cmd);
void    checkIncoming();

// ----------------------- Флаги прерываний ----------------------------------
volatile bool drTriggered = false;

// [FIX 3] Флаг безопасного перезапуска nRF905.
// delay() и driver.init() нельзя вызывать из контекста обработчика команд
// (который вызывается из checkIncoming() внутри ISR-цепочки).
// Устанавливаем флаг здесь — выполняем в loop().
volatile bool nrfRestartRequested = false;

// --------------------------- Временные константы ---------------------------
const unsigned long DRYER_DURATION_MS    = 5UL * 60UL * 1000UL;
const unsigned long COOL_DOWN_MS         = 1UL * 60UL * 1000UL;
const unsigned long DAY_INTERVAL_MS      = 24UL * 60UL * 60UL * 1000UL;
const float         HUM_THRESHOLD        = 98.0f;

// ------------------------- Глобальные переменные ---------------------------
float lastPM25 = 0.0f;
float lastPM10 = 0.0f;
float avgT     = 0.0f;
float avgH     = 0.0f;
float avgUV    = 0.0f;
float avgLux   = 0.0f;

bool vemlFound = false;

// ----------------------- Состояния сушки / вентилятора --------------------
bool          dryer_triggered_today = false;
bool          dryer_active          = false;
bool          cooldown_active       = false;
bool          fan_active            = false;
unsigned long dryer_start_time      = 0;
unsigned long cooldown_start_time   = 0;
unsigned long last_day_check        = 0;

// ----------------------- Управление датчиком ZH07 -------------------------
// [FIX 2] Добавлены состояния Z_COLLECTING и Z_PROCESS.
// Было: Z_READ блокировал весь цикл на while(...millis()...) до 1 секунды,
// не давая watchdog.reload() и checkIncoming() выполняться.
// Стало: сбор данных разбит на несколько вызовов manageZH07() без блокировки.
enum ZH07State { Z_IDLE, Z_WARMING_UP, Z_COLLECTING, Z_PROCESS };
ZH07State     zh07_state       = Z_IDLE;
unsigned long zh07_timer_start = 0;
unsigned long zh07CollectStart = 0;
bool          pm_sensor_enabled = true;

// Буфер для неблокирующего сбора 32-байтного пакета ZH07
uint8_t zh07Buffer[32];
int     zh07BufIndex = 0;

const unsigned long ZH07_COLLECT_TIMEOUT_MS      = 2000UL;            // Таймаут сбора пакета
// [FIX 10] Исправлен комментарий: 30 секунд (не "1 минута")
const unsigned long ZH07_WARMUP_DURATION_MS      = 30UL * 1000UL;     // 30 секунд на прогрев
const unsigned long ZH07_MEASUREMENT_INTERVAL_MS = 5UL * 60UL * 1000UL; // Раз в 5 минут

// ------------------------- Фильтр UV-индекса ------------------------------
const int UV_FILTER_SIZE = 10;
float     uvHistory[UV_FILTER_SIZE];
int       uvHistIndex = 0;
int       uvHistCount = 0;

// [FIX 8] static const — гарантирует размещение в .rodata (flash), не в RAM
static const int uvTable[12][2] = {
  {50, 0}, {227, 1}, {318, 2},  {408, 3},
  {503, 4}, {606, 5}, {696, 6}, {795, 7},
  {881, 8}, {976, 9}, {1079, 10}, {1170, 11}
};

// ---------------------------------------------------------------------------
// Чтение UV-индекса с ночным обнулением
// ---------------------------------------------------------------------------
float readUVIndex(float &vout_mV) {
  if (vemlFound && avgLux < 10.0f) {
    vout_mV = 0;
    return 0;
  }
  int raw = analogRead(UV_PIN);
  vout_mV = raw * (3252.0f / 4095.0f);   // 12-bit ADC: 0–4095

  if (vout_mV < uvTable[0][0])  return 0;
  if (vout_mV >= uvTable[11][0]) return 11;

  for (int i = 0; i < 11; i++) {
    int v1 = uvTable[i][0],   v2 = uvTable[i + 1][0];
    int u1 = uvTable[i][1],   u2 = uvTable[i + 1][1];
    if (vout_mV >= v1 && vout_mV < v2) {
      return u1 + (vout_mV - v1) * float(u2 - u1) / float(v2 - v1);
    }
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Проверка присутствия устройства на I2C без риска зависания библиотеки
// ---------------------------------------------------------------------------
bool pingI2C(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

// ---------------------------------------------------------------------------
// Скользящее среднее для UV
// ---------------------------------------------------------------------------
float filterUV(float uv) {
  uvHistory[uvHistIndex] = uv;
  uvHistIndex = (uvHistIndex + 1) % UV_FILTER_SIZE;
  if (uvHistCount < UV_FILTER_SIZE) uvHistCount++;

  float sum = 0;
  for (int i = 0; i < uvHistCount; i++) sum += uvHistory[i];
  return sum / uvHistCount;
}

// ---------------------------------------------------------------------------
// [FIX 2] Неблокирующий менеджер ZH07
// Z_READ → Z_COLLECTING + Z_PROCESS
// ---------------------------------------------------------------------------
void manageZH07() {
  if (!pm_sensor_enabled) {
    digitalWrite(ZH07_PWR_SAVE_PIN, LOW);
    return;
  }

  switch (zh07_state) {

    case Z_IDLE:
      if (millis() - zh07_timer_start >= ZH07_MEASUREMENT_INTERVAL_MS) {
        Serial.println("[ZH07] Начало цикла. Питание включено, прогрев...");
        digitalWrite(ZH07_PWR_SAVE_PIN, HIGH);
        zh07_timer_start = millis();
        zh07_state = Z_WARMING_UP;
      }
      break;

    case Z_WARMING_UP:
      if (millis() - zh07_timer_start >= ZH07_WARMUP_DURATION_MS) {
        Serial.println("[ZH07] Прогрев завершён. Начинаем сбор данных...");
        // Сбрасываем входной буфер UART и счётчики перед сбором
        while (Serial1.available()) Serial1.read();
        zh07BufIndex    = 0;
        zh07CollectStart = millis();
        zh07_state = Z_COLLECTING;
      }
      break;

    case Z_COLLECTING:
      // Читаем доступные байты без блокировки — каждый вызов добирает кусок пакета
      while (Serial1.available() && zh07BufIndex < 32) {
        zh07Buffer[zh07BufIndex++] = (uint8_t)Serial1.read();
      }

      if (zh07BufIndex >= 32) {
        // Пакет собран целиком — переходим к обработке
        zh07_state = Z_PROCESS;
      } else if (millis() - zh07CollectStart >= ZH07_COLLECT_TIMEOUT_MS) {
        // Не дождались полного пакета — таймаут
        Serial.println("[ZH07] Таймаут сбора данных! Попытка через 5 мин.");
        digitalWrite(ZH07_PWR_SAVE_PIN, LOW);
        zh07_timer_start = millis();
        zh07_state = Z_IDLE;
      }
      break;

    case Z_PROCESS:
      // Проверяем заголовок пакета ZH07B
      if (zh07Buffer[0] == 0x42 &&
          zh07Buffer[1] == 0x4D &&
          zh07Buffer[2] == 0x00 &&
          zh07Buffer[3] == 0x1C)
      {
        // Контрольная сумма: сумма байт 0..29
        uint16_t checksum = 0;
        for (int i = 0; i < 30; i++) checksum += zh07Buffer[i];
        uint16_t received_checksum = ((uint16_t)zh07Buffer[30] << 8) | zh07Buffer[31];

        if (checksum == received_checksum) {
          // [FIX 7] Именованные константы вместо магических смещений 6,7,8,9
          lastPM25 = (((uint16_t)zh07Buffer[ZH07_PM25_HIGH] << 8) | zh07Buffer[ZH07_PM25_LOW]) / 10.0f;
          lastPM10 = (((uint16_t)zh07Buffer[ZH07_PM10_HIGH] << 8) | zh07Buffer[ZH07_PM10_LOW]) / 10.0f;
          Serial.print("[ZH07] PM2.5: "); Serial.print(lastPM25);
          Serial.print(" μg/m³, PM10: "); Serial.print(lastPM10); Serial.println(" μg/m³");
        } else {
          Serial.println("[ZH07] Ошибка контрольной суммы!");
        }
      } else {
        // Пакет собран не с начала (мы попали в середину потока).
        // Результат будет отброшен. Следующая попытка через 5 минут.
        Serial.println("[ZH07] Неверный заголовок пакета!");
      }

      digitalWrite(ZH07_PWR_SAVE_PIN, LOW);
      zh07_timer_start = millis();
      zh07_state = Z_IDLE;
      Serial.println("[ZH07] Измерение завершено. Датчик выключен.");
      break;
  }
}

// ---------------------------------------------------------------------------
// Чтение и усреднение T, H, UV, Lux (8 замеров за ~10 с)
// ---------------------------------------------------------------------------
void updateAveragedSensors() {
  float sumT = 0, sumH = 0, sumUV = 0, sumLux = 0;
  int cntSHT = 0, cntLux = 0, cntUV = 0;

  const uint8_t ADDR_SHT31 = 0x44;
  const uint8_t ADDR_VEML  = 0x10;

  Serial.println("[SENS] Начинаем серию измерений...");

  for (int i = 0; i < 8; i++) {

    // 1. SHT31 с предварительным ping
    if (pingI2C(ADDR_SHT31)) {
      float t = sht31.readTemperature();
      float h = sht31.readHumidity();
      if (!isnan(t) && !isnan(h) && t > -100.0f && t < 100.0f) {
        sumT += t; sumH += h; cntSHT++;
      }
    }

    // 2. VEML7700 с предварительным ping
    if (vemlFound) {
      if (pingI2C(ADDR_VEML)) {
        float lux = veml.readLux(VEML_LUX_NORMAL);
        if (lux >= 0.0f) { sumLux += lux; cntLux++; }
      } else {
        Serial.println("[ERR] VEML перестал отвечать!");
      }
    }

    // 3. UV-индекс (аналоговый, без риска зависания)
    float voutUV;
    float fuv = filterUV(readUVIndex(voutUV));
    if (isfinite(fuv)) { sumUV += fuv; cntUV++; }

    // Проверяем радио во время паузы между замерами
    checkIncoming();
    for (int d = 0; d < 125; d++) {
      delay(10);         // 125 × 10 мс = 1250 мс
      checkIncoming();
    }
  }

  if (cntSHT > 0) {
    avgT = sumT / cntSHT;
    avgH = sumH / cntSHT;
  } else {
    Serial.println("[ERR] SHT31 не ответил ни разу за 8 попыток!");
  }
  avgLux = (cntLux > 0) ? (sumLux / cntLux) : 0.0f;
  if (cntUV  > 0) avgUV = sumUV / cntUV;

  Serial.print("T: "); Serial.print(avgT);
  Serial.print(" H: "); Serial.print(avgH);
  Serial.print(" Lux: "); Serial.println(avgLux);
}

// ---------------------------------------------------------------------------
// Прерывание DR nRF905
// ---------------------------------------------------------------------------
void onNRF905DataReady() {
  drTriggered = true;
}

// ---------------------------------------------------------------------------
// Обработка входящих команд
// ---------------------------------------------------------------------------
void processCommand(const char *cmd) {

  if (strcmp(cmd, "HEATER") == 0) {
    Serial.println("[CMD] Форсируем цикл просушки");
    sht31.heater(true);
    dryer_active          = true;
    dryer_start_time      = millis();
    dryer_triggered_today = true;
    return;
  }

  // [FIX 3] Больше не вызываем delay() и driver.init() прямо здесь:
  // processCommand() может быть вызван из checkIncoming() внутри
  // updateAveragedSensors(), где watchdog не перезагружается.
  // Флаг обрабатывается безопасно в начале loop().
  if (strcmp(cmd, "NRF_REST") == 0) {
    Serial.println("[CMD] Запрос перезапуска nRF905 (выполнится в loop)");
    nrfRestartRequested = true;
    return;
  }

  if (strcmp(cmd, "REST") == 0) {
    Serial.println("[CMD] Перезапуск STM32");
    NVIC_SystemReset();
  }
}

// ---------------------------------------------------------------------------
// [FIX 4] Приём входящего пакета
// Было: drTriggered сбрасывался только при успешном recv().
//       При ошибке приёма флаг оставался true навсегда → driver.available()
//       опрашивался при каждом вызове без реального события DR.
// Стало: сбрасываем drTriggered сразу, до вызова recv().
// ---------------------------------------------------------------------------
void checkIncoming() {
  if (!drTriggered) return;
  drTriggered = false;   // Сбрасываем немедленно — битый пакет не блокирует

  if (driver.available()) {
    uint8_t buf[RH_NRF905_MAX_MESSAGE_LEN + 1];
    uint8_t len = sizeof(buf) - 1;
    if (driver.recv(buf, &len)) {
      buf[len] = '\0';
      Serial.print("[RX] Команда: ");
      Serial.println((char *)buf);
      processCommand((char *)buf);
    } else {
      Serial.println("[RX] Ошибка приёма пакета");
    }
  }
}

// ---------------------------------------------------------------------------
// Контрольная сумма (XOR)
// ---------------------------------------------------------------------------
uint8_t calcChecksum(const uint8_t *buf, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; i++) cs ^= buf[i];
  return cs;
}

void pack16(uint8_t *dst, uint16_t v) {
  dst[0] = v & 0xFF;
  dst[1] = v >> 8;
}

void pack32(uint8_t *dst, uint32_t v) {
  dst[0] =  v        & 0xFF;
  dst[1] = (v >>  8) & 0xFF;
  dst[2] = (v >> 16) & 0xFF;
  dst[3] = (v >> 24) & 0xFF;
}

// ---------------------------------------------------------------------------
// Инициализация
// ---------------------------------------------------------------------------
void setup() {
  last_day_check = millis();
  Serial.begin(115200);

  // [FIX 1] Исправлено значение watchdog: 30 000 000 мкс = 30 секунд.
  // Предыдущее значение 60 000 000 мкс (60 с) превышало аппаратный максимум
  // STM32 IWDG (~32 с при 32 кГц LSI) и не совпадало с комментарием "10 секунд".
  // Цикл loop() занимает ~10 с → 30 с даёт трёхкратный запас.
  IWatchdog.begin(30000000);

  // Первое измерение ZH07 через 15 секунд после старта
  zh07_timer_start = millis() - ZH07_MEASUREMENT_INTERVAL_MS + 15000UL;

  // Вентилятор
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);

  // ZH07
  pinMode(ZH07_PWR_SAVE_PIN, OUTPUT);
  digitalWrite(ZH07_PWR_SAVE_PIN, LOW);
  Serial1.begin(9600);

  // nRF905
  pinMode(NRF905_PWR_UP_PIN, OUTPUT);
  digitalWrite(NRF905_PWR_UP_PIN, HIGH);
  pinMode(NRF905_DR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(NRF905_DR_PIN), onNRF905DataReady, RISING);

  if (!driver.init()) {
    Serial.println("[ERR] Ошибка инициализации nRF905!");
  }
  driver.setChannel(RF_CHANNEL, false);
  driver.setRF(RH_NRF905::TransmitPower10dBm);
  Serial.println("[OK] nRF905 готов.");

  // I2C → SHT31
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.setClock(100000);
  Wire.setTimeout(50);
  Wire.begin();
  if (!sht31.begin(0x44)) {
    Serial.println("[ERR] SHT31 не найден!");
  }
  Serial.print("Heater: ");
  Serial.println(sht31.isHeaterEnabled() ? "ON" : "OFF");

  // VEML7700
  if (veml.begin()) {
    Serial.println("[OK] VEML7700 найден.");
    vemlFound = true;
    veml.setGain(VEML7700_GAIN_1_8);
    veml.setIntegrationTime(VEML7700_IT_50MS);
  } else {
    Serial.println("[ERR] VEML7700 не найден! Пропускаем.");
    vemlFound = false;
  }

  analogReadResolution(12);
  pinMode(UV_PIN, INPUT);
}

// ---------------------------------------------------------------------------
// Основной цикл
// ---------------------------------------------------------------------------
void loop() {
  IWatchdog.reload();

  unsigned long now = millis();
  checkIncoming();

  // [FIX 3] Безопасный перезапуск nRF905 в основном цикле.
  // delay() здесь допустим: watchdog только что перезагружен выше,
  // 100 мс << 30 сек таймаут. driver.init() вызывается когда
  // checkIncoming() точно не выполняется.
  if (nrfRestartRequested) {
    nrfRestartRequested = false;
    Serial.println("[CMD] Перезапуск nRF905...");
    digitalWrite(NRF905_PWR_UP_PIN, LOW);
    delay(100);
    digitalWrite(NRF905_PWR_UP_PIN, HIGH);
    driver.init();
    driver.setChannel(RF_CHANNEL, false);
    driver.setRF(RH_NRF905::TransmitPower10dBm);
    Serial.println("[CMD] nRF905 перезапущен.");
  }

  // 1) Суточный сброс флага просушки
  if (now - last_day_check >= DAY_INTERVAL_MS) {
    dryer_triggered_today = false;
    last_day_check = now;
  }

  // 2) Авто-старт нагревателя по влажности
  // [FIX 5] Добавлен pingI2C перед прямым чтением: без него при отвалившемся
  // SHT31 библиотека зависает на Wire.endTransmission() до истечения timeout.
  if (!dryer_triggered_today && !dryer_active && !cooldown_active) {
    if (pingI2C(0x44)) {
      float h0 = sht31.readHumidity();
      if (!isnan(h0) && h0 >= HUM_THRESHOLD) {
        sht31.heater(true);
        dryer_active          = true;
        dryer_triggered_today = true;
        dryer_start_time      = now;
      }
    }
  }

  // 3) Обновление фаз сушки / охлаждения
  if (dryer_active && now - dryer_start_time >= DRYER_DURATION_MS) {
    sht31.heater(false);
    dryer_active        = false;
    cooldown_active     = true;
    cooldown_start_time = now;
  }
  if (cooldown_active && now - cooldown_start_time >= COOL_DOWN_MS) {
    cooldown_active = false;
  }

  // 4) Усреднение SHT31, VEML и UV (~10 с)
  updateAveragedSensors();
  IWatchdog.reload(); // Явная перезагрузка после длинного блока

  // 5) Управление датчиком PM по температуре
  if (pm_sensor_enabled && avgT < PM_TEMP_THRESHOLD_STOP) {
    pm_sensor_enabled = false;
    Serial.println("[PM] Температура слишком низкая. Отключаем датчик PM.");
    digitalWrite(ZH07_PWR_SAVE_PIN, LOW);
    zh07_state = Z_IDLE;
    lastPM25 = 0.0f;
    lastPM10 = 0.0f;
  } else if (!pm_sensor_enabled && avgT > PM_TEMP_THRESHOLD_RESUME) {
    pm_sensor_enabled = true;
    Serial.println("[PM] Температура в норме. Включаем датчик PM.");
    zh07_timer_start = millis() - ZH07_MEASUREMENT_INTERVAL_MS + 15000UL;
  }

  // 6) Неблокирующий менеджер ZH07
  manageZH07();

  // 7) Управление вентилятором по освещённости с гистерезисом
  if (!fan_active && avgLux > LUX_THRESHOLD_ON) {
    fan_active = true;
    digitalWrite(FAN_PIN, HIGH);
    Serial.println("[FAN] Вентилятор включен (превышен порог LUX)");
  } else if (fan_active && avgLux < LUX_THRESHOLD_OFF) {
    fan_active = false;
    digitalWrite(FAN_PIN, LOW);
    Serial.println("[FAN] Вентилятор выключен (LUX ниже порога гистерезиса)");
  }

  // 8) Отправка пакета
  checkIncoming();
  sendBinaryPacket();
  checkIncoming();
}

// ---------------------------------------------------------------------------
// Сборка и отправка бинарного пакета
// ---------------------------------------------------------------------------
void sendBinaryPacket() {
  // [FIX 9] Не отправляем пакет при невалидных данных T/H.
  // isnan возвращает true если SHT31 отвалился и avgT/avgH не обновлялись.
  if (isnan(avgT) || isnan(avgH)) {
    Serial.println("[TX] Пропуск пакета: невалидные данные T/H");
    return;
  }

  uint8_t buf[32];
  size_t  pos = 0;

  // Байт 0: статус нагревателя
  uint8_t heater_status = ST_NORMAL;
  if      (dryer_active)    heater_status = ST_HEATER;
  else if (cooldown_active) heater_status = ST_COOLING;
  buf[pos++] = heater_status;

  // Байт 1: статус вентилятора
  buf[pos++] = fan_active ? ST_FAN_ON : ST_FAN_OFF;

  // Данные: упаковка с фиксированной точкой
  int16_t  t_packed    = (int16_t)round(avgT   * 100.0f);
  uint16_t h_packed    = (uint16_t)(avgH   * 100.0f + 0.5f);
  uint16_t uv_packed   = (uint16_t)(avgUV  * 100.0f + 0.5f);
  uint32_t lux_packed  = (uint32_t)(avgLux * 100.0f + 0.5f);
  uint16_t pm25_packed = (uint16_t)(lastPM25 * 10.0f + 0.5f);
  uint16_t pm10_packed = (uint16_t)(lastPM10 * 10.0f + 0.5f);

  // [FIX 6] Явное приведение (uint16_t) для t_packed устраняет
  // предупреждение компилятора "-Wconversion int16_t → uint16_t".
  // На приёмной стороне (ESP32) читается обратно как int16_t — корректно.
  pack16(buf + pos, (uint16_t)t_packed);  pos += 2;
  pack16(buf + pos, h_packed);            pos += 2;
  pack16(buf + pos, uv_packed);           pos += 2;
  pack32(buf + pos, lux_packed);          pos += 4;
  pack16(buf + pos, pm25_packed);         pos += 2;
  pack16(buf + pos, pm10_packed);         pos += 2;

  buf[pos++] = calcChecksum(buf, pos);

  // Отладочный дамп в HEX
  Serial.print("[TX] BINARY: ");
  for (size_t i = 0; i < pos; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  driver.send(buf, pos);
  driver.waitPacketSent();
}