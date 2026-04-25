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

#define UV_PIN              PA0     // АНАЛОГОВЫЙ вход S12SD
#define ZH07_PWR_SAVE_PIN   PA8     // Пин управления питанием ZH07
#define FAN_PIN             PA11    // Пин управления вентилятором через мосфет

#define RF_CHANNEL          175     // 439.9 МГц канал

// -------- Флаги статуса (отправляются в пакете) --------------
#define ST_NORMAL   0x01
#define ST_HEATER   0x02
#define ST_COOLING  0x03
#define ST_FAN_OFF  0x00
#define ST_FAN_ON   0x01

// -------- Пороги для управления вентилятором (LUX) -----------
const float LUX_THRESHOLD_ON  = 8000.0f; // Порог включения вентилятора
const float LUX_THRESHOLD_OFF = 4000.0f; // Порог выключения (гистерезис)

// -------- Пороги температуры для датчика PM -----------
const float PM_TEMP_THRESHOLD_STOP = -10.0f;
const float PM_TEMP_THRESHOLD_RESUME = -8.0f; // -10 + 2 градуса гистерезиса

RH_NRF905 driver(NRF905_CE, NRF905_TX_EN, NRF905_CS);
Adafruit_SHT31 sht31(&Wire);
Adafruit_VEML7700 veml = Adafruit_VEML7700();

// ------------------------ Прототипы ------------------------------------------
uint8_t calcChecksum(const uint8_t *buf, size_t len);
void pack16(uint8_t *dst, uint16_t v);
void pack32(uint8_t *dst, uint32_t v);
void sendBinaryPacket();
void processCommand(const char *cmd);
void checkIncoming();
void readZH07();

// ------------------------- Состояние DR nRF905 -------------------------------
volatile bool drTriggered = false;

// ---------------------------- Временные константы ----------------------------
const unsigned long DRYER_DURATION_MS = 5UL * 60UL * 1000UL;   // 5 минут
const unsigned long COOL_DOWN_MS      = 1UL * 60UL * 1000UL;   // 1 минута
const unsigned long DAY_INTERVAL_MS   = 24UL * 60UL * 60UL * 1000UL; // 24 часа
const float HUM_THRESHOLD             = 98.0;                  // %

// --------------------------- Глобальные переменные ---------------------------
uint8_t  cycleCount = 0;
float    lastPM25   = 0.0f;
float    lastPM10   = 0.0f;
float avgT = 0.0f;
float avgH = 0.0f;
float avgUV = 0.0f;
float avgLux = 0.0f;

// ---------------------------- Состояния датчиков -----------------------------
bool vemlFound = false; // Флаг: true - датчик есть и работает, false - датчика нет

// ---------------------------- Состояния сушки и вентилятора ------------------
bool  dryer_triggered_today = false;
bool  dryer_active          = false;
bool  cooldown_active       = false;
bool  fan_active            = false; // НОВОЕ: состояние вентилятора
unsigned long dryer_start_time    = 0;
unsigned long cooldown_start_time = 0;
unsigned long last_day_check      = 0;

// ---------------------------- НОВОЕ: Управление датчиком ZH07 ----------------
enum ZH07State { Z_IDLE, Z_WARMING_UP, Z_READ };
ZH07State zh07_state = Z_IDLE;
unsigned long zh07_timer_start = 0;
bool pm_sensor_enabled = true; // Глобальный флаг разрешения работы датчика

const unsigned long ZH07_WARMUP_DURATION_MS = 30UL * 1000UL;             // 1 минута на прогрев
const unsigned long ZH07_MEASUREMENT_INTERVAL_MS = 5UL * 30UL * 1000UL; // Измеряем раз в 5 минут

// ---------------------------- Параметры фильтра UV ----------------------------
const int   UV_FILTER_SIZE = 10;
float       uvHistory[UV_FILTER_SIZE];
int         uvHistIndex  = 0;
int         uvHistCount  = 0;

// Таблица для перевода mV → UV index
const int uvTable[12][2] = {
  {50, 0}, {227, 1}, {318, 2}, {408, 3},
  {503, 4}, {606, 5}, {696, 6}, {795, 7},
  {881, 8}, {976, 9}, {1079, 10}, {1170, 11}
};

// ----------------------------------------------------------------------------
// Функция чтения UV-индекса и параллельно mV на выходе датчика
// ----------------------------------------------------------------------------
float readUVIndex(float &vout_mV) {
  // Если датчик освещенности видит, что сейчас ночь (например, меньше 10 люкс)
  // то принудительно обнуляем УФ, так как солнца нет.
  if (vemlFound && avgLux < 10.0f) {
    vout_mV = 0;
    return 0;
  }
  int raw = analogRead(UV_PIN);
  vout_mV = raw * (3252.0f / 4095.0f);  // 12-bit ADC на STM32F4: 0–4095

  if (vout_mV < uvTable[0][0]) return 0;
  if (vout_mV >= uvTable[11][0]) return 11;

  // линейная интерполяция между точками таблицы
  for (int i = 0; i < 11; i++) {
    int v1 = uvTable[i][0], v2 = uvTable[i+1][0];
    int u1 = uvTable[i][1], u2 = uvTable[i+1][1];
    if (vout_mV >= v1 && vout_mV < v2) {
      return u1 + (vout_mV - v1) * float(u2 - u1) / float(v2 - v1);
    }
  }
  return 0;
}

// ----------------------------------------------------------------------------
// Вспомогательная функция для проверки, "жив" ли датчик на шине
// ----------------------------------------------------------------------------
bool pingI2C(uint8_t address) {
  Wire.beginTransmission(address);
  uint8_t error = Wire.endTransmission();
  return (error == 0); // 0 = успех (ACK получен)
}

// ----------------------------------------------------------------------------
// Простая функция добавления в скользящий фильтр и получения усреднённого UV
// ----------------------------------------------------------------------------
float filterUV(float uv) {
  uvHistory[uvHistIndex] = uv;
  uvHistIndex = (uvHistIndex + 1) % UV_FILTER_SIZE;
  if (uvHistCount < UV_FILTER_SIZE) uvHistCount++;

  float sum = 0;
  for (int i = 0; i < uvHistCount; i++) sum += uvHistory[i];
  return sum / uvHistCount;
}

// ----------------------------------------------------------------------------
// НОВОЕ: Неблокирующая функция управления ZH07
// ----------------------------------------------------------------------------
void manageZH07() {
  if (!pm_sensor_enabled) {
    digitalWrite(ZH07_PWR_SAVE_PIN, LOW); // Гарантированно выключаем питание
    return;
  }

  switch (zh07_state) {
    case Z_IDLE:
      if (millis() - zh07_timer_start >= ZH07_MEASUREMENT_INTERVAL_MS) {
        Serial.println("[ZH07] Начало цикла измерения. Включаем питание и прогрев...");
        digitalWrite(ZH07_PWR_SAVE_PIN, HIGH);
        zh07_timer_start = millis();
        zh07_state = Z_WARMING_UP;
      }
      break;

    case Z_WARMING_UP:
      if (millis() - zh07_timer_start >= ZH07_WARMUP_DURATION_MS) {
        Serial.println("[ZH07] Прогрев завершен. Чтение данных...");
        zh07_state = Z_READ;
      }
      break;

    case Z_READ:
      {
        // Очистка буфера перед чтением
        while (Serial1.available()) Serial1.read();
        
        // Ожидание данных с таймаутом
        uint8_t buffer[32];
        int index = 0;
        unsigned long start_time = millis();
        
        while (index < 32 && millis() - start_time < 1000) {
          if (Serial1.available()) {
            buffer[index++] = Serial1.read();
          }
        }

        // Анализ полученных данных
        if (index == 32 && 
            buffer[0] == 0x42 && 
            buffer[1] == 0x4D && 
            buffer[2] == 0x00 && 
            buffer[3] == 0x1C) 
        {
          // Проверка контрольной суммы
          uint16_t checksum = 0;
          for (int i = 0; i < 30; i++) {
            checksum += buffer[i];
          }
          
          uint16_t received_checksum = (buffer[30] << 8) | buffer[31];
          
          if (checksum == received_checksum) {
            // Извлечение значений PM (стандартные позиции для ZH07B)
            lastPM25 = ((buffer[6] << 8) | buffer[7]) / 10.0f;
            lastPM10 = ((buffer[8] << 8) | buffer[9]) / 10.0f;
            Serial.print("[ZH07] PM2.5: ");
            Serial.print(lastPM25);
            Serial.print(" μg/m³, PM10: ");
            Serial.print(lastPM10);
            Serial.println(" μg/m³");
          } else {
            Serial.println("[ZH07] Ошибка контрольной суммы!");
          }
        } else {
          Serial.println("[ZH07] Неверный пакет данных!");
        }
        
        digitalWrite(ZH07_PWR_SAVE_PIN, LOW);
        zh07_timer_start = millis();
        zh07_state = Z_IDLE;
        Serial.println("[ZH07] Измерение завершено. Датчик выключен.");
      }
      break;
  }
}

// ----------------------------------------------------------------------------
// Чтение и усреднение T, H, UV и Lux
// ----------------------------------------------------------------------------
// Обновленная, "неубиваемая" функция чтения
void updateAveragedSensors() {
  float sumT = 0, sumH = 0, sumUV = 0, sumLux = 0;
  int cntSHT = 0;   // Отдельный счетчик успешных чтений для SHT
  int cntLux = 0;   // Отдельный счетчик для VEML
  int cntUV = 0;    // Счетчик для аналогового UV

  // Адреса датчиков (стандартные)
  const uint8_t ADDR_SHT31 = 0x44; 
  const uint8_t ADDR_VEML  = 0x10; 

  Serial.println("[SENS] Начинаем серию измерений...");

  for (int i = 0; i < 8; i++) {
    // 1. Безопасное чтение SHT31
    // Сначала "пингуем" датчик. Если он не отвечает, пропускаем чтение, чтобы не тревожить библиотеку.
    if (pingI2C(ADDR_SHT31)) {
      float t = sht31.readTemperature();
      float h = sht31.readHumidity();
      
      if (!isnan(t) && !isnan(h) && t > -100.0 && t < 100.0) { // Фильтр явного бреда
        sumT += t;
        sumH += h;
        cntSHT++;
      }
    } else {
      // Если нужно, можно раскомментировать для отладки, но лучше не спамить в эфир
      // Serial.println("[ERR] SHT31 не отвечает!"); 
    }

    // 2. Безопасное чтение VEML7700
    if (vemlFound) {
      if (pingI2C(ADDR_VEML)) { // 0x10 - адрес VEML7700
          float lux = veml.readLux(VEML_LUX_NORMAL);
          // Если вернулось -1 или другое ошибочное значение - фильтруем
          if (lux >= 0.0) { 
             sumLux += lux;
             cntLux++; // Увеличиваем счетчик успешных замеров только при успехе
          }
      } else {
          // Если пинг не прошел, можно попробовать сбросить флаг, чтобы не спамить в шину
          // vemlFound = false; // Раскомментируйте, если хотите отключить его навсегда до перезагрузки
          Serial.println("[ERR] VEML перестал отвечать!");
      }
  }

    // 3. Чтение UV (аналоговый пин, тут зависать нечему, но усредняем)
    float voutUV;
    float rawUV = readUVIndex(voutUV);
    float fuv = filterUV(rawUV);
    if (isfinite(fuv)) {
      sumUV += fuv;
      cntUV++;
    }

    // Проверяем входящие и ждем
    checkIncoming();
    // Делим задержку на мелкие части, чтобы чаще проверять радио
    for(int d=0; d<125; d++) { 
        delay(10); // 125 * 10 = 1250мс (примерно то же самое, что 10000/8)
        checkIncoming(); 
    }
  }

  // 4. Расчет средних значений с защитой от деления на ноль
  if (cntSHT > 0) {
    avgT = sumT / cntSHT;
    avgH = sumH / cntSHT;
  } else {
    // Если датчик умер совсем, можно либо оставить старое значение,
    // либо выставить флаг ошибки. Пока оставляем старое, чтобы график не падал в 0.
    Serial.println("[ERR] SHT31 отвалился во всех 8 попытках!");
  }

  if (cntLux > 0) {
    avgLux = sumLux / cntLux;
  } else {
    avgLux = 0.0; // Если темно или датчик умер - света нет
  }

  if (cntUV > 0) {
    avgUV = sumUV / cntUV;
  }
  
  Serial.print("T: "); Serial.print(avgT);
  Serial.print(" H: "); Serial.print(avgH);
  Serial.print(" Lux: "); Serial.println(avgLux);
}

// ----------------------------------------------------------------------------
// Обработка прерывания DR
// ----------------------------------------------------------------------------
void onNRF905DataReady() {
  drTriggered = true;
}

// ----------------------------------------------------------------------------
// Обработка входящих команд
// ----------------------------------------------------------------------------
void processCommand(const char *cmd) {
  if (strcmp(cmd, "HEATER") == 0) {
    Serial.println("[CMD] Форсируем цикл просушки");
    sht31.heater(true);
    dryer_active = true;
    dryer_start_time = millis();
    dryer_triggered_today = true;
    return;
  }

  if (strcmp(cmd, "NRF_REST") == 0) {
    Serial.println("[CMD] Перезапуск модуля nRF905");
    digitalWrite(NRF905_PWR_UP_PIN, LOW);
    delay(100);
    digitalWrite(NRF905_PWR_UP_PIN, HIGH);
    driver.init(); // повторная инициализация
    driver.setChannel(RF_CHANNEL, false);
    driver.setRF(RH_NRF905::TransmitPower10dBm);
    return;
  }

  if (strcmp(cmd, "REST") == 0) {
    Serial.println("[CMD] Перезапуск STM32");
    NVIC_SystemReset();
  }
}

// ----------------------------------------------------------------------------
// Чтение входящего пакета
// ----------------------------------------------------------------------------
void checkIncoming()
{
  if (drTriggered)
  {
    if (driver.available())
    {
      uint8_t buf[RH_NRF905_MAX_MESSAGE_LEN + 1];
      uint8_t len = sizeof(buf) - 1;
      if (driver.recv(buf, &len))
      {
        buf[len] = '\0';
        Serial.print("[RX] Команда: ");
        Serial.println((char *)buf);
        processCommand((char *)buf);
        // Только после успешного recv сбрасываем флаг
        drTriggered = false;
      }
    }
  }
}

// ---------------------------расчет checksum --------------------------------
uint8_t calcChecksum(const uint8_t *buf, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; i++) cs ^= buf[i];
  return cs;
}

void pack16(uint8_t *dst, uint16_t v) {
  dst[0] = v & 0xFF;  // Младший байт
  dst[1] = v >> 8;    // Старший байт
}

void pack32(uint8_t* buf, uint32_t val) {
  buf[0] = val & 0xFF;
  buf[1] = (val >> 8) & 0xFF;
  buf[2] = (val >> 16) & 0xFF;
  buf[3] = (val >> 24) & 0xFF;
}

// ----------------------------------------------------------------------------
// Настройка
// ----------------------------------------------------------------------------
void setup() {
  last_day_check = millis();
  Serial.begin(115200);
  IWatchdog.begin(60000000); // Таймер на 10 секунд

  // ИЗМЕНЕНО: Инициализируем таймер ZH07 так, чтобы первое измерение началось через 15 секунд
  zh07_timer_start = millis() - ZH07_MEASUREMENT_INTERVAL_MS + 15000UL;

  // Вентилятор
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW); // Убедимся, что вентилятор выключен при старте

  // ZH07
  pinMode(ZH07_PWR_SAVE_PIN, OUTPUT);       // PWR_SAVE
  digitalWrite(ZH07_PWR_SAVE_PIN, LOW);     // по умолчанию в спящем режиме
  Serial1.begin(9600);       // UART1 на PA9(TX1), PA10(RX1)

  // Включение nRF905
  pinMode(NRF905_PWR_UP_PIN, OUTPUT);
  digitalWrite(NRF905_PWR_UP_PIN, HIGH);

  // Инициализация пина DR
  pinMode(NRF905_DR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(NRF905_DR_PIN), onNRF905DataReady, RISING);

  // nRF905
  if (!driver.init()) {
    Serial.println("Ошибка инициализации nRF905!");
  }
  driver.setChannel(RF_CHANNEL, false);
  driver.setRF(RH_NRF905::TransmitPower10dBm);
  Serial.println("nRF905 готов.");

  // I2C → SHT31
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.setClock(100000); // 100kHz
  Wire.setTimeout(50);
  Wire.begin();
  if (!sht31.begin(0x44)) {
    Serial.println("SHT31 не найден!");
  }
  Serial.print("Heater: ");
  Serial.println(sht31.isHeaterEnabled() ? "ON" : "OFF");

  // VEML7700
  if (veml.begin()) {
    Serial.println("[OK] VEML7700 найден.");
    vemlFound = true;  // Ставим флаг - датчик жив
    
    // Выполняем настройку ТОЛЬКО если датчик найден
    veml.setGain(VEML7700_GAIN_1_8);
    veml.setIntegrationTime(VEML7700_IT_50MS);
  
  } else {
    Serial.println("[ERR] VEML7700 не найден! Пропускаем.");
    vemlFound = false; // Ставим флаг - датчика нет
    // Мы НЕ вызываем setGain и прочее. Это предотвратит Hard Fault.
  }
  
  analogReadResolution(12);

  // Аналоговый вход
  pinMode(UV_PIN, INPUT);
}

// ----------------------------------------------------------------------------
// Основной цикл
// ----------------------------------------------------------------------------
void loop() {
  IWatchdog.reload();

  unsigned long now = millis();
  checkIncoming();

  // 1) Суточный сброс флага просушки
  if (now - last_day_check >= DAY_INTERVAL_MS) {
    dryer_triggered_today = false;
    last_day_check = now;
  }

  // 2) Авто-старт HEATER по влажности
  if (!dryer_triggered_today && !dryer_active && !cooldown_active) {
    float h0 = sht31.readHumidity();
    if (!isnan(h0) && h0 >= HUM_THRESHOLD) {
      sht31.heater(true);
      dryer_active          = true;
      dryer_triggered_today = true;
      dryer_start_time      = now;
    }
  }

  // 3) Обновление фаз сушки/охлаждения
  if (dryer_active && now - dryer_start_time >= DRYER_DURATION_MS) {
    sht31.heater(false);
    dryer_active        = false;
    cooldown_active     = true;
    cooldown_start_time = now;
  }
  if (cooldown_active && now - cooldown_start_time >= COOL_DOWN_MS) {
    cooldown_active = false;
  }

  // 4) Усреднение SHT31, VEML и UV (8 замеров за ~10 с)
  updateAveragedSensors();

  // 5) НОВОЕ: Логика отключения датчика PM при низкой температуре
  if (pm_sensor_enabled && avgT < PM_TEMP_THRESHOLD_STOP) {
    pm_sensor_enabled = false;
    Serial.println("[PM] Температура слишком низкая. Отключаем датчик PM.");
    // Принудительно выключаем питание и сбрасываем состояние
    digitalWrite(ZH07_PWR_SAVE_PIN, LOW);
    zh07_state = Z_IDLE;
    // Обнуляем последние показания
    lastPM25 = 0.0f;
    lastPM10 = 0.0f;
  } else if (!pm_sensor_enabled && avgT > PM_TEMP_THRESHOLD_RESUME) {
    pm_sensor_enabled = true;
    Serial.println("[PM] Температура в норме. Включаем датчик PM.");
    // Сбрасываем таймер, чтобы следующее измерение началось скорее
    zh07_timer_start = millis() - ZH07_MEASUREMENT_INTERVAL_MS + 15000UL;
  }

  // 6) ИЗМЕНЕНО: Вызываем неблокирующий менеджер ZH07
  manageZH07();
  
  // 7) НОВОЕ: Управление вентилятором по уровню освещенности с гистерезисом
  if (!fan_active && avgLux > LUX_THRESHOLD_ON) {
    fan_active = true;
    digitalWrite(FAN_PIN, HIGH);
    Serial.println("[FAN] Вентилятор включен (превышен порог LUX)");
  } else if (fan_active && avgLux < LUX_THRESHOLD_OFF) {
    fan_active = false;
    digitalWrite(FAN_PIN, LOW);
    Serial.println("[FAN] Вентилятор выключен (LUX ниже порога гистерезиса)");
  }

  // 8) Отправляем единый бинарный пакет со всеми данными
  checkIncoming();
  sendBinaryPacket();
  checkIncoming();
}

// ----------------------------------------------------------------------------
// ИЗМЕНЕНО: Отправка единого бинарного пакета
// ----------------------------------------------------------------------------
void sendBinaryPacket() {
  uint8_t buf[32];
  size_t  pos = 0;

  // 1) Байт статуса нагревателя
  uint8_t heater_status = ST_NORMAL;
  if (dryer_active)       heater_status = ST_HEATER;
  else if (cooldown_active) heater_status = ST_COOLING;
  buf[pos++] = heater_status;

  // 2) Байт статуса вентилятора
  buf[pos++] = fan_active ? ST_FAN_ON : ST_FAN_OFF;

  // 3) Подготовка и упаковка всех данных
  int16_t t_packed = (int16_t)round(avgT * 100.0f);
  uint16_t h_packed   = (uint16_t)(avgH * 100.0f + 0.5f);
  uint16_t uv_packed  = (uint16_t)(avgUV * 100.0f + 0.5f);
  uint32_t lux_packed = (uint32_t)(avgLux * 100.0f + 0.5f);
  uint16_t pm25_packed = (uint16_t)(lastPM25 * 10.0f + 0.5f);
  uint16_t pm10_packed = (uint16_t)(lastPM10 * 10.0f + 0.5f);

  pack16(buf + pos, t_packed);    pos += 2; // Температура
  pack16(buf + pos, h_packed);    pos += 2; // Влажность
  pack16(buf + pos, uv_packed);   pos += 2; // УФ-индекс
  pack32(buf + pos, lux_packed);  pos += 4; // Освещенность
  pack16(buf + pos, pm25_packed); pos += 2; // PM2.5
  pack16(buf + pos, pm10_packed); pos += 2; // PM10

  // 4) Контрольная сумма
  buf[pos++] = calcChecksum(buf, pos);

  // 5) Отладочный вывод в HEX
  Serial.print("[TX] BINARY: ");
  for (size_t i = 0; i < pos; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  // 6) Отправка
  driver.send(buf, pos);
  driver.waitPacketSent();
}