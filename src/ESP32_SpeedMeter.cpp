#define sample 30

#define switch1 23
#define sensor 32

#include <Arduino.h>
#include "I2C_AXP192.h"
#include <SPI.h>
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

uint32_t seq = 0;
uint16_t appCounter = 0;
uint32_t lastSendMs = 0;

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

    Serial.printf("TX seq=%lu appCounter=%u payload=",
                  (unsigned long)seq,
                  appCounter);
    print_hex(payload, PAYLOAD_LEN);

    int ret = lora.SendFrame(config, payload, PAYLOAD_LEN);
    if (ret != 0) {
      Serial.printf("SendFrame failed ret=%d\n", ret);
    }

    seq++;
    appCounter++;
  }

  if((dispCurrTime - dispPrevTime) >= dispInterval){
    u8g2.firstPage();
    do {
      spdDiffTime = spdCurrTime - spdPrevTime;
      if(spdDiffTime > 0){        
        wheelSpeed = (0.55 * 3.14 * 3.6 * 1000) / (spdDiffTime);
        if(wheelSpeed >= 100){
          wheelSpeed = 99;
        }
      }else{
        wheelSpeed = 0;
      }
      spdPrevTime = spdCurrTime;

      for(int i = sample - 1; i > 0; i--){
        spdAvg[i] = spdAvg[i-1];
      }
      spdAvg[0] = wheelSpeed;

      float speedSum = 0.0f;
      for(int i = 0; i < sample; i++){
        speedSum += spdAvg[i];
      }
      WheelAvg = speedSum / sample;

      if(digitalRead(switch1) == 0){
        if(isTimerStart){
          timeElapsed = millis() - timeInihold;
          timeSechold = timeElapsed / 1000;
          timeMinhold = timeSechold / 60;
        }else{
          isTimerStart = true;
          timeElapsed = 0;
          timeMinhold = 0;
          timeSechold = 0;
          timeInihold = millis();
        }
      }else{
        isTimerStart = false;
      }

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
