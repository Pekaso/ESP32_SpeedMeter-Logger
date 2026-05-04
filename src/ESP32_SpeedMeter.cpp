#define sample 30

#define switch1 23
#define sensor 32

#include <Arduino.h>
#include "I2C_AXP192.h"
#include <SPI.h>
#include <SPIFFS.h>
#include <U8g2lib.h>
#include <limits.h>
#include <donguri_bitmap.h>

#include "esp32_e220900t22s_jp_lib.h"

CLoRa lora;
LoRaConfigItem_t config;

// ===== Node settings =====
static const uint16_t PARENT_ADDR = 0x0001;
static const uint16_t CHILD_ADDR  = 0x0010;
static const uint8_t  LORA_CH     = 0x00;

// true: broadcast, false: unicast to CHILD_ADDR
static const bool USE_BROADCAST = false;
static const uint16_t BROADCAST_ADDR = 0xFFFF;

// ===== Telemetry protocol settings =====
static const size_t PAYLOAD_LEN = 29;
static const size_t APP_DATA_LEN = 13;
static const uint8_t MAGIC = 0xE2;
static const uint8_t PROTOCOL_VER = 0x01;
static const uint8_t APP_DATA_TYPE_VEHICLE = 0x01;
static const uint8_t STATUS_MEASURING = 0x01;
static const uint8_t THROTTLE_NOT_IMPL = 0xFF;
static const int16_t TEMP_NOT_IMPL = INT16_MIN;

// ===== Binary log settings =====
static const char *LOG_FILENAME = "/speedlog.scub";
static const size_t LOG_HEADER_LEN = 32;
static const size_t LOG_RECORD_LEN = 24;
static const size_t LOG_SUMMARY_LEN = 32;
static const uint16_t LOG_SAMPLE_PERIOD_MS = 1000;
static const uint32_t LOG_MAX_SECONDS = 60UL * 60UL;
static const uint32_t LOG_MAX_RECORDS = LOG_MAX_SECONDS * 1000UL / LOG_SAMPLE_PERIOD_MS;
static const uint32_t LOG_FLAG_HAS_SUMMARY = 0x00000001UL;
static const uint32_t LOG_FLAG_CLOSED_CLEANLY = 0x00000004UL;
static const uint8_t LOG_RECORD_FLAG_CRC_OK = 0x01;
static const uint8_t LOG_RECORD_FLAG_APP_DATA_VALID = 0x08;

uint32_t seq = 0;
uint16_t appCounter = 0;
uint32_t lastSendMs = 0;
bool serialDebugEnabled = true;

U8G2_ST7565_ERC12864_F_4W_SW_SPI u8g2(U8G2_R0,/* clock=*/ 33, /* data=*/ 14, /* cs=*/ 15, /* dc=*/ 2, /* reset=*/ 13);

//Update display
unsigned long dispInterval = 100;
unsigned long dispCurrTime = 0;
unsigned long dispPrevTime = 0;

//Analog Meter
const int needleWid = 4;
const int meterMax = 195;
const int meterMin = -15;
const int meterPosX = 64;
const int meterPosY = 50;
const int meterRad = 50;
const int meterWid = 10;

//Update Battery
bool battIsCharge = false;
unsigned long battInterval = 10000;
unsigned long battCurrTime = 0;
unsigned long battPrevTime = 0;

const float maxBatVol = 4200.0f;
const float minBatVol = 3000.0f;
float batVoltage = 0.0f;
float batCapacity = 0.0f;
float batVbusVol = 0.0f;

//Calc wheel speed
const float spdMax = 70.0f;
const float spdMin = 0.0f;
const float WHEEL_DIAMETER_M = 0.55f;
const float WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * PI;
const unsigned long SPEED_TIMEOUT_MS = 2500;
const float SPEED_TREND_EPSILON_KMH = 0.5f;

volatile unsigned long spdCurrTime = 0;
unsigned long spdPrevTime = 0;
unsigned long spdDiffTime = 0;
float wheelSpeed = 0.0f;
float WheelAvg = 0.0f;
float spdAvg[sample] = {0};
char spdTexbuf[3];

//Timer
bool isTimerStart = false;
int timeInihold = 0;
int timeElapsed = 0;
int timeMinhold = 0;
int timeSechold = 0;

char timeTextbuf[6];

uint8_t *logBuffer = nullptr;
uint32_t logRecordCount = 0;
uint32_t logLostRecordCount = 0;
bool logActive = false;
bool logOverflow = false;
bool logSaved = false;
bool logUsePsram = false;
uint32_t logSavedBytes = 0;
uint16_t logMinSpeedX10 = UINT16_MAX;
uint16_t logMaxSpeedX10 = 0;
uint16_t logMinRpmX100 = UINT16_MAX;
uint16_t logMaxRpmX100 = 0;
uint16_t logTurnMinSpeedX10 = UINT16_MAX;
uint16_t logTurnMaxSpeedX10 = 0;
bool logHasTurnMinSpeed = false;
bool logHasTurnMaxSpeed = false;

enum SpeedTrend {
  SPEED_TREND_UNKNOWN,
  SPEED_TREND_ACCEL,
  SPEED_TREND_DECEL
};

SpeedTrend speedTrend = SPEED_TREND_UNKNOWN;
bool speedTrendHasSample = false;
float speedTrendPrevKmh = 0.0f;
float speedTurnCandidateKmh = 0.0f;

I2C_AXP192 axp192(I2C_AXP192_DEFAULT_ADDRESS, Wire1);

void IRAM_ATTR timeInterval(){
  spdCurrTime = millis();
}

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;

    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static void put_u16_be(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFF);
}

static void put_i16_be(uint8_t *p, int16_t v) {
  put_u16_be(p, (uint16_t)v);
}

static void put_u32_be(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)(v & 0xFF);
}

static uint8_t crc8_atm(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;

  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];

    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x07);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
  crc = ~crc;

  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];

    for (int bit = 0; bit < 8; bit++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      } else {
        crc >>= 1;
      }
    }
  }

  return ~crc;
}

static bool build_payload(uint8_t payload[PAYLOAD_LEN], const uint8_t *appData, size_t appLen) {
  if (appLen != APP_DATA_LEN) return false;

  memset(payload, 0, PAYLOAD_LEN);

  payload[0] = MAGIC;
  payload[1] = PROTOCOL_VER;

  put_u16_be(&payload[2], PARENT_ADDR);
  put_u32_be(&payload[4], seq);
  put_u32_be(&payload[8], millis());
  put_u16_be(&payload[12], appCounter);

  memcpy(&payload[14], appData, appLen);

  uint16_t crc = crc16_ccitt_false(payload, 27);
  put_u16_be(&payload[27], crc);

  return true;
}

static uint16_t encode_speed_x10(float speedKmh) {
  if (speedKmh <= 0.0f) return 0;

  const float encoded = speedKmh * 10.0f + 0.5f;
  if (encoded >= 65535.0f) return 65535;

  return (uint16_t)encoded;
}

static void reset_speed_turn_summary() {
  logTurnMinSpeedX10 = UINT16_MAX;
  logTurnMaxSpeedX10 = 0;
  logHasTurnMinSpeed = false;
  logHasTurnMaxSpeed = false;

  speedTrend = SPEED_TREND_UNKNOWN;
  speedTrendHasSample = false;
  speedTrendPrevKmh = 0.0f;
  speedTurnCandidateKmh = 0.0f;
}

static void record_turn_min_speed(float speedKmh) {
  const uint16_t speedX10 = encode_speed_x10(speedKmh);
  if (!logHasTurnMinSpeed || speedX10 < logTurnMinSpeedX10) {
    logTurnMinSpeedX10 = speedX10;
    logHasTurnMinSpeed = true;
  }
}

static void record_turn_max_speed(float speedKmh) {
  const uint16_t speedX10 = encode_speed_x10(speedKmh);
  if (!logHasTurnMaxSpeed || speedX10 > logTurnMaxSpeedX10) {
    logTurnMaxSpeedX10 = speedX10;
    logHasTurnMaxSpeed = true;
  }
}

static void update_speed_turn_summary(float speedKmh) {
  if (!isTimerStart) return;

  if (!speedTrendHasSample) {
    speedTrendHasSample = true;
    speedTrendPrevKmh = speedKmh;
    speedTurnCandidateKmh = speedKmh;
    return;
  }

  const float deltaKmh = speedKmh - speedTrendPrevKmh;

  if (deltaKmh > SPEED_TREND_EPSILON_KMH) {
    if (speedTrend == SPEED_TREND_DECEL) {
      record_turn_min_speed(speedTurnCandidateKmh);
    }

    if (speedTrend != SPEED_TREND_ACCEL) {
      speedTurnCandidateKmh = speedKmh;
    } else if (speedKmh > speedTurnCandidateKmh) {
      speedTurnCandidateKmh = speedKmh;
    }

    speedTrend = SPEED_TREND_ACCEL;
  } else if (deltaKmh < -SPEED_TREND_EPSILON_KMH) {
    if (speedTrend == SPEED_TREND_ACCEL) {
      record_turn_max_speed(speedTurnCandidateKmh);
    }

    if (speedTrend != SPEED_TREND_DECEL) {
      speedTurnCandidateKmh = speedKmh;
    } else if (speedKmh < speedTurnCandidateKmh) {
      speedTurnCandidateKmh = speedKmh;
    }

    speedTrend = SPEED_TREND_DECEL;
  } else {
    if ((speedTrend == SPEED_TREND_ACCEL) && (speedKmh > speedTurnCandidateKmh)) {
      speedTurnCandidateKmh = speedKmh;
    } else if ((speedTrend == SPEED_TREND_DECEL) && (speedKmh < speedTurnCandidateKmh)) {
      speedTurnCandidateKmh = speedKmh;
    }
  }

  speedTrendPrevKmh = speedKmh;
}

static void pack_vehicle_app_data(
  uint8_t appData[APP_DATA_LEN],
  uint16_t speedX10,
  uint16_t rpmX100,
  bool measuring
) {
  memset(appData, 0, APP_DATA_LEN);

  appData[0] = APP_DATA_TYPE_VEHICLE;
  appData[1] = measuring ? STATUS_MEASURING : 0;
  put_u16_be(&appData[2], speedX10);
  put_u16_be(&appData[4], rpmX100);
  appData[6] = THROTTLE_NOT_IMPL;
  put_i16_be(&appData[7], TEMP_NOT_IMPL);
  put_i16_be(&appData[9], TEMP_NOT_IMPL);
  put_u16_be(&appData[11], 0x0000);
}

static void print_hex(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    Serial.printf("%02X ", data[i]);
  }
  Serial.println();
}

static uint32_t log_capacity_bytes() {
  return LOG_MAX_RECORDS * LOG_RECORD_LEN;
}

static uint8_t log_used_percent() {
  return (uint8_t)((logRecordCount * 100UL + LOG_MAX_RECORDS - 1) / LOG_MAX_RECORDS);
}

static size_t log_total_file_bytes(uint32_t records) {
  return LOG_HEADER_LEN + (records * LOG_RECORD_LEN) + LOG_SUMMARY_LEN;
}

static void reset_log_summary() {
  logMinSpeedX10 = UINT16_MAX;
  logMaxSpeedX10 = 0;
  logMinRpmX100 = UINT16_MAX;
  logMaxRpmX100 = 0;
  logLostRecordCount = 0;
  reset_speed_turn_summary();
}

static void begin_log_session() {
  logRecordCount = 0;
  logOverflow = false;
  logSaved = false;
  logSavedBytes = 0;
  reset_log_summary();
  SPIFFS.remove(LOG_FILENAME);
  logActive = (logBuffer != nullptr);

  if (!logActive) {
    Serial.println("Log buffer is not available");
  }
}

static void append_log_record(uint32_t sequenceNumber, uint32_t recordMillis, const uint8_t appData[APP_DATA_LEN]) {
  if (!logActive || logBuffer == nullptr) return;

  if (logRecordCount >= LOG_MAX_RECORDS) {
    logOverflow = true;
    return;
  }

  uint8_t *record = logBuffer + (logRecordCount * LOG_RECORD_LEN);
  memset(record, 0, LOG_RECORD_LEN);

  put_u32_be(&record[0], sequenceNumber);
  put_u32_be(&record[4], recordMillis);
  record[8] = 0; // RSSI is unavailable on the parent-side local logger.
  record[9] = LOG_RECORD_FLAG_CRC_OK | LOG_RECORD_FLAG_APP_DATA_VALID;
  memcpy(&record[10], appData, APP_DATA_LEN);
  record[23] = crc8_atm(record, 23);

  const uint16_t speedX10 = ((uint16_t)appData[2] << 8) | appData[3];
  const uint16_t rpmX100 = ((uint16_t)appData[4] << 8) | appData[5];
  if (speedX10 < logMinSpeedX10) logMinSpeedX10 = speedX10;
  if (speedX10 > logMaxSpeedX10) logMaxSpeedX10 = speedX10;
  if (rpmX100 < logMinRpmX100) logMinRpmX100 = rpmX100;
  if (rpmX100 > logMaxRpmX100) logMaxRpmX100 = rpmX100;

  logRecordCount++;
}

static void pack_log_header(uint8_t header[LOG_HEADER_LEN]) {
  memset(header, 0, LOG_HEADER_LEN);
  header[0] = 'S';
  header[1] = 'C';
  header[2] = 'U';
  header[3] = 'B';
  header[4] = 0x01;
  header[5] = LOG_RECORD_LEN;
  put_u16_be(&header[6], LOG_SAMPLE_PERIOD_MS);
  put_u32_be(&header[8], 0);
  put_u32_be(&header[12], logRecordCount);
  put_u32_be(&header[16], logLostRecordCount);
  put_u32_be(&header[20], LOG_FLAG_HAS_SUMMARY | LOG_FLAG_CLOSED_CLEANLY);
}

static void pack_log_summary(uint8_t summary[LOG_SUMMARY_LEN]) {
  memset(summary, 0, LOG_SUMMARY_LEN);
  summary[0] = 'S';
  summary[1] = 'U';
  summary[2] = 'M';
  summary[3] = 'M';

  const uint16_t minSpeed = logHasTurnMinSpeed ? logTurnMinSpeedX10 : ((logRecordCount > 0) ? logMinSpeedX10 : 0);
  const uint16_t maxSpeed = logHasTurnMaxSpeed ? logTurnMaxSpeedX10 : logMaxSpeedX10;
  const uint16_t minRpm = (logRecordCount > 0) ? logMinRpmX100 : 0;

  put_u16_be(&summary[4], minSpeed);
  put_u16_be(&summary[6], maxSpeed);
  put_u16_be(&summary[8], minRpm);
  put_u16_be(&summary[10], logMaxRpmX100);
  put_i16_be(&summary[12], TEMP_NOT_IMPL);
  put_i16_be(&summary[14], TEMP_NOT_IMPL);
  put_i16_be(&summary[16], TEMP_NOT_IMPL);
  put_i16_be(&summary[18], TEMP_NOT_IMPL);
  put_u32_be(&summary[20], logRecordCount);
  put_u32_be(&summary[24], logLostRecordCount);

  const uint32_t bodyCrc = crc32_update(0, logBuffer, logRecordCount * LOG_RECORD_LEN);
  put_u32_be(&summary[28], bodyCrc);
}

static bool save_log_to_spiffs() {
  if (logBuffer == nullptr) return false;

  File file = SPIFFS.open(LOG_FILENAME, FILE_WRITE);
  if (!file) {
    Serial.println("Log file open failed");
    return false;
  }

  uint8_t header[LOG_HEADER_LEN];
  uint8_t summary[LOG_SUMMARY_LEN];
  pack_log_header(header);
  pack_log_summary(summary);

  bool ok = true;
  ok = ok && (file.write(header, LOG_HEADER_LEN) == LOG_HEADER_LEN);
  ok = ok && (file.write(logBuffer, logRecordCount * LOG_RECORD_LEN) == (logRecordCount * LOG_RECORD_LEN));
  ok = ok && (file.write(summary, LOG_SUMMARY_LEN) == LOG_SUMMARY_LEN);
  file.close();

  logSaved = ok;
  logSavedBytes = ok ? log_total_file_bytes(logRecordCount) : 0;

  Serial.printf("Log saved: ok=%d records=%lu bytes=%lu used=%u%% overflow=%d\n",
                ok ? 1 : 0,
                (unsigned long)logRecordCount,
                (unsigned long)logSavedBytes,
                log_used_percent(),
                logOverflow ? 1 : 0);

  return ok;
}

static void end_log_session() {
  if (!logActive) return;

  logActive = false;
  save_log_to_spiffs();
}

static void dump_log_file_hex() {
  File file = SPIFFS.open(LOG_FILENAME, FILE_READ);
  if (!file) {
    Serial.println("SCUB_LOG_NOT_FOUND");
    return;
  }

  Serial.printf("SCUB_HEX_BEGIN bytes=%u\n", (unsigned int)file.size());
  uint8_t line[32];
  while (file.available()) {
    const size_t n = file.read(line, sizeof(line));
    for (size_t i = 0; i < n; i++) {
      Serial.printf("%02X", line[i]);
    }
    Serial.println();
  }
  Serial.println("SCUB_HEX_END");
  file.close();
}

static void print_log_info() {
  const size_t total = SPIFFS.totalBytes();
  const size_t used = SPIFFS.usedBytes();
  const size_t expected = log_total_file_bytes(LOG_MAX_RECORDS);
  const uint8_t expectedPercent = total > 0 ? (uint8_t)((expected * 100UL + total - 1) / total) : 0;

  Serial.printf("LOGINFO file=%s exists=%d saved=%d records=%lu max_records=%lu current_used=%u%% max_file_bytes=%u spiffs_total=%u spiffs_used=%u max_spiffs_use=%u%% psram=%d overflow=%d\n",
                LOG_FILENAME,
                SPIFFS.exists(LOG_FILENAME) ? 1 : 0,
                logSaved ? 1 : 0,
                (unsigned long)logRecordCount,
                (unsigned long)LOG_MAX_RECORDS,
                log_used_percent(),
                (unsigned int)expected,
                (unsigned int)total,
                (unsigned int)used,
                expectedPercent,
                logUsePsram ? 1 : 0,
                logOverflow ? 1 : 0);
}

static void handle_serial_commands() {
  static String command;

  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;

    if (ch == '\n') {
      command.trim();
      command.toUpperCase();

      if (command == "LOGINFO") {
        print_log_info();
      } else if (command == "LOGDUMP") {
        serialDebugEnabled = false;
        dump_log_file_hex();
      } else if (command == "DEBUG") {
        serialDebugEnabled = !serialDebugEnabled;
        Serial.printf("DEBUG %s\n", serialDebugEnabled ? "ON" : "OFF");
      } else if (command == "LOGERASE") {
        SPIFFS.remove(LOG_FILENAME);
        logSaved = false;
        logSavedBytes = 0;
        Serial.println("LOGERASE OK");
      } else if (command.length() > 0) {
        Serial.println("Commands: LOGINFO, LOGDUMP, DEBUG, LOGERASE");
      }

      command = "";
    } else if (command.length() < 32) {
      command += ch;
    }
  }
}

static void update_timer_state(uint32_t now) {
  const bool switchActive = (digitalRead(switch1) == 0);

  if (switchActive) {
    if (isTimerStart) {
      timeElapsed = now - timeInihold;
      timeSechold = timeElapsed / 1000;
      timeMinhold = timeSechold / 60;
    } else {
      isTimerStart = true;
      timeElapsed = 0;
      timeMinhold = 0;
      timeSechold = 0;
      timeInihold = now;
      if (serialDebugEnabled) {
        Serial.println("Begin log session");
      }
      begin_log_session();
    }
  } else {
    if (isTimerStart) {
      if (serialDebugEnabled) {
        Serial.println("Begin log end");
      }
      end_log_session();
    }
    isTimerStart = false;
  }
}

static void setup_lora_config() {
  lora.SetDefaultConfigValue(config);

  config.own_address = PARENT_ADDR;

  // Keep UART at 9600 because the attached library starts SerialLoRa with LoRa_BaudRate=9600.
  config.baud_rate = 0b011;       // 9600 bps

  // Fast air rate: SF5 / BW500kHz
  // config.air_data_rate = 0b00010;
  // Default: SF9 / BW125kHz
  config.air_data_rate = 0b10000;

  // Payload/sub-packet size 32 bytes
  config.subpacket_size = 0b11;   // 32 bytes

  // RSSI ambient noise measurement disabled
  config.rssi_ambient_noise_flag = 0b0;

  // 13 dBm
  config.transmitting_power = 0b01;

  config.own_channel = LORA_CH;

  // Keep enabled because receiveFrame() in the attached library expects RSSI byte.
  config.rssi_byte_flag = 0b1;

  // Fixed-block mode
  config.transmission_method_type = 0b1;

  // WOR is not used in normal mode; keep a valid value.
  config.wor_cycle = 0b001;       // 1000 ms

  // Same key must be set on parent and child.
  config.encryption_key = 0x0000;

  config.target_address = USE_BROADCAST ? BROADCAST_ADDR : CHILD_ADDR;
  config.target_channel = LORA_CH;
}

int rotX(int cx, int r, int deg) {
    return cx + r * cos(2*PI * (deg / 360.00) - PI);
}
int rotY(int cy, int r, int deg) {
    return cy + r * sin(2*PI * (deg / 360.00) - PI);
}

void drawMeter(int speed){
  int needlePos = map(speed, spdMin, spdMax, meterMin, meterMax);
  u8g2.drawCircle(meterPosX, meterPosY, meterRad, U8G2_DRAW_ALL);
  u8g2.drawCircle(meterPosX, meterPosY, meterRad - meterWid, U8G2_DRAW_ALL);
  
  int x1 = rotX(meterPosX, meterRad, needlePos-needleWid);
  int y1 = rotY(meterPosY, meterRad, needlePos-needleWid);
  int x2 = rotX(meterPosX, meterRad, needlePos+needleWid);
  int y2 = rotY(meterPosY, meterRad, needlePos+needleWid);

  u8g2.drawTriangle(meterPosX, meterPosY, x1, y1, x2, y2);

  u8g2.setDrawColor(0);
  u8g2.drawDisc(meterPosX, meterPosY, meterRad - (meterWid + 1), U8G2_DRAW_ALL);
  u8g2.setDrawColor(1);

  for(int n = meterMin; n <= meterMax; n += 30){
    int xn1 = rotX(meterPosX, meterRad - meterWid, n);
    int yn1 = rotY(meterPosY, meterRad - meterWid, n);
    int xn2 = rotX(meterPosX, meterRad - (meterWid + 4), n);
    int yn2 = rotY(meterPosY, meterRad - (meterWid + 4), n);

    u8g2.drawLine(xn1, yn1, xn2, yn2);
  }

}

void drawBattery(int x, int y, bool isCharge, int percent){
  u8g2.setFont(u8g2_font_battery19_tn);
  if(isCharge){
    u8g2.drawGlyph(x,y, 0x0036);
  }else{
    if(percent >= 80){
      u8g2.drawGlyph(x, y, 0x0035);
    }else if(percent >= 60){
      u8g2.drawGlyph(x, y, 0x0034);
    }else if(percent >= 40){
      u8g2.drawGlyph(x, y, 0x0033);
    }else if(percent >= 20){
      u8g2.drawGlyph(x, y, 0x0032);
    }else if(percent >= 10){
      u8g2.drawGlyph(x, y, 0x0031);
    }else if(10 > percent){
      u8g2.drawGlyph(x, y, 0x0030);
    }
  }
}

void setup() {
  u8g2.begin();
  u8g2.setContrast(15);
  u8g2.clearBuffer();

  u8g2.setBitmapMode(false /* solid */);
  u8g2.setDrawColor(0);
  u8g2.drawXBM( 0, 0, 128, 47, epd_bitmap_donguri);
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_samim_12_t_all);
  u8g2.setCursor(5,60);
  u8g2.print("AXP192 init");
  u8g2.nextPage();

  pinMode(switch1, INPUT_PULLUP);
  
  I2C_AXP192_InitDef initDef = {
    .EXTEN  = true,
    .BACKUP = true,
    .DCDC1  = 0,
    .DCDC2  = 0,
    .DCDC3  = 3300,
    .LDO2   = 3000,
    .LDO3   = 3000,
    .GPIO0  = 2800,
    .GPIO1  = -1,
    .GPIO2  = -1,
    .GPIO3  = -1,
    .GPIO4  = -1,
  };
  noInterrupts();
  Wire1.begin(21, 22);
  axp192.begin(initDef);
  interrupts();

  delay(100);

  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 48, 128, 16);
  u8g2.setDrawColor(1);
  u8g2.setCursor(5,60);
  u8g2.print("Serial init");
  u8g2.nextPage();

  Serial.begin(115200);
  delay(900);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, 48, 128, 16);
    u8g2.setDrawColor(1);
    u8g2.setCursor(5,60);
    u8g2.print("SPIFFS Mount Failed");
    u8g2.nextPage();
    while(1){};
  }

  const size_t logBytes = log_capacity_bytes();
  if (psramFound()) {
    logBuffer = (uint8_t *)ps_malloc(logBytes);
    logUsePsram = (logBuffer != nullptr);
  }
  if (logBuffer == nullptr) {
    logBuffer = (uint8_t *)malloc(logBytes);
    logUsePsram = false;
  }
  Serial.printf("Log buffer: bytes=%u psram=%d max_records=%lu max_file_bytes=%u\n",
                (unsigned int)logBytes,
                logUsePsram ? 1 : 0,
                (unsigned long)LOG_MAX_RECORDS,
                (unsigned int)log_total_file_bytes(LOG_MAX_RECORDS));

  pinMode(sensor, INPUT);
  attachInterrupt(sensor, timeInterval, RISING);

  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 48, 128, 16);
  u8g2.setDrawColor(1);
  u8g2.setCursor(5,60);
  u8g2.print("LoRa module init");
  u8g2.nextPage();

  Serial.println();
  Serial.println("E220-900T22S(JP) parent transmitter");

  pinMode(LoRa_AUXPin, INPUT);

  setup_lora_config();

  delay(100);

  int ret = lora.InitLoRaModule(config);
  Serial.printf("InitLoRaModule ret=%d\n", ret);

  lora.SwitchToNormalMode();

  Serial.print("Target: ");
  if (USE_BROADCAST) {
    Serial.println("broadcast");
  } else {
    Serial.printf("unicast 0x%04X\n", CHILD_ADDR);
  }

}

void loop() {
  dispCurrTime = millis();
  battCurrTime = millis();

  const uint32_t now = millis();
  handle_serial_commands();
  update_timer_state(now);

  if (now - lastSendMs >= 1000) {
    lastSendMs = now;

    uint8_t payload[PAYLOAD_LEN];
    uint8_t appData[APP_DATA_LEN];

    const uint16_t speedX10 = encode_speed_x10(WheelAvg);
    const uint16_t rpmX100 = 0;
    pack_vehicle_app_data(appData, speedX10, rpmX100, isTimerStart);

    if (!build_payload(payload, appData, sizeof(appData))) {
      Serial.println("build_payload failed");
      return;
    }

    if (isTimerStart) {
      append_log_record(seq, now, appData);
    }

    if (serialDebugEnabled) {
      Serial.printf("TX seq=%lu appCounter=%u payload=",
                    (unsigned long)seq,
                    appCounter);
      print_hex(payload, PAYLOAD_LEN);
    }

    int ret = lora.SendFrame(config, payload, PAYLOAD_LEN);
    if ((ret != 0) && serialDebugEnabled) {
      Serial.printf("SendFrame failed ret=%d\n", ret);
    }

    seq++;
    appCounter++;
  }

  if((dispCurrTime - dispPrevTime) >= dispInterval){
    u8g2.firstPage();
    do {
      noInterrupts();
      const unsigned long latestPulseTime = spdCurrTime;
      interrupts();

      if(latestPulseTime != spdPrevTime){
        if(spdPrevTime > 0){
          spdDiffTime = latestPulseTime - spdPrevTime;

          // Wheel speed calculation:
          // - Tire diameter is 550 mm = 0.55 m.
          // - The sensor emits 1 pulse per wheel rotation.
          // - One pulse interval therefore represents one tire circumference.
          // - circumference[m] = diameter[m] * PI = 0.55 * PI.
          // - speed[m/s] = circumference[m] / (pulse_interval_ms / 1000).
          // - speed[km/h] = speed[m/s] * 3.6.
          // Combined:
          //   speed[km/h] = circumference[m] * 1000 * 3.6 / pulse_interval_ms
          //                = circumference[m] * 3600 / pulse_interval_ms
          wheelSpeed = (WHEEL_CIRCUMFERENCE_M * 3600.0f) / spdDiffTime;
          if(wheelSpeed >= 100){
            wheelSpeed = 99;
          }
        }
        spdPrevTime = latestPulseTime;
      }else if((spdPrevTime == 0) || ((dispCurrTime - spdPrevTime) > SPEED_TIMEOUT_MS)){
        wheelSpeed = 0;
      }

      for(int i = sample - 1; i > 0; i--){
        spdAvg[i] = spdAvg[i-1];
      }
      spdAvg[0] = wheelSpeed;

      float speedSum = 0.0f;
      for(int i = 0; i < sample; i++){
        speedSum += spdAvg[i];
      }
      WheelAvg = speedSum / sample;
      update_speed_turn_summary(WheelAvg);

      sprintf(timeTextbuf, "%02d:%02d", timeMinhold, (timeSechold)%60);

      drawMeter(WheelAvg);
      dtostrf(WheelAvg, 2, 0, spdTexbuf);

      u8g2.setFont(u8g2_font_logisoso28_tn);
      u8g2.drawStr(39, 47, spdTexbuf);
      u8g2.setFont(u8g2_font_t0_11b_te);
      u8g2.setCursor(76,47);
      u8g2.print("km/h");

      u8g2.setFont(u8g2_font_mercutio_sc_nbp_tn);
      u8g2.drawStr(56, 62, timeTextbuf);

      drawBattery(119, 22, battIsCharge, batCapacity);
      u8g2.setFont(u8g2_font_5x7_mr);
      u8g2.setCursor(118,30);
      u8g2.print(batCapacity, 0);
      u8g2.setCursor(101, 39);
      u8g2.print("L");
      u8g2.print(log_used_percent());

      u8g2.setFont(u8g2_font_open_iconic_play_1x_t);
      if(isTimerStart){
        u8g2.drawGlyph(42, 61,0x0045); //Started
      }else{
        u8g2.drawGlyph(42, 61,0x0044); //paused
      }

      } while ( u8g2.nextPage() );

      dispPrevTime = dispCurrTime;
  }

  if(((battCurrTime - battPrevTime) >= battInterval)){
    batVoltage = axp192.getBatteryVoltage();
    batVbusVol = axp192.getVbusVoltage();
    if(batVbusVol >= 200){
      battIsCharge = true;
    }else{
      battIsCharge = false;
    }
    batCapacity = 100 * (batVoltage - minBatVol)/(maxBatVol - minBatVol);
    battPrevTime = battCurrTime;
  }

}
