// =============================================================
// ESP32 Speed Meter + Logger (main implementation)
//  - ESP32 + SIM7600 (LTE/GPS)
//  - 128x64 LCD (U8g2)
//  - AXP192 power management (battery monitor)
//
// リファクタリング方針（外部ライブラリや構成は変更しない）
//  - マジックナンバーの定数化／命名の明確化
//  - 初期化/更新/描画の責務分離
//  - ISR共有変数のvolatile化、安全な読み取り
//  - 平均速度計算のバグ修正（都度リセット）
//  - コメント整備（日本語）
// =============================================================

#define TINY_GSM_MODEM_SIM7600      // SIM7600JC LTE モデムを使用
#define SerialMon Serial            // デバッグ用シリアル（115200）
#define SerialAT Serial1            // モデム制御用シリアル
// #define DUMP_AT_COMMANDS          // ATコマンドのダンプ（必要に応じ有効化）

#define TINY_GSM_DEBUG SerialMon    // TinyGSM デバッグ出力
#define USE_GSM                     // GSM機能を使用

#include <Arduino.h>
#include "I2C_AXP192.h"
#include <SPI.h>
#include <U8g2lib.h>

#include <TinyGsmClient.h>
#include <Ticker.h>
#include <ArduinoHttpClient.h>

// ---------- 画面・表示 ----------
// LCD はソフトウェアSPIで駆動
U8G2_ST7565_ERC12864_F_4W_SW_SPI u8g2(
  U8G2_R2,
  /* clock=*/ 33,
  /* data=*/ 14,
  /* cs=*/   15,
  /* dc=*/   2,
  /* reset=*/13
);

// 表示のアナログメーター設定
namespace UiCfg {
  constexpr int needleWidth = 4;
  constexpr int meterMaxDeg = 195;
  constexpr int meterMinDeg = -15;
  constexpr int centerX     = 64;
  constexpr int centerY     = 50;
  constexpr int radius      = 50;
  constexpr int ringWidth   = 10;
}

// 表示更新間隔（ms）
constexpr unsigned long kDispIntervalMs = 100;

// ---------- バッテリー ----------
constexpr unsigned long kBattIntervalMs = 5000;
constexpr float kMaxBattMv = 4200.0f;  // 100%
constexpr float kMinBattMv = 3000.0f;  // 0%

// ---------- 速度計測 ----------
constexpr uint8_t  kHallPin           = 32;    // ホールセンサー割り込み入力
constexpr float    kSpeedMaxKmh       = 70.0f; // 表示上限
constexpr float    kSpeedMinKmh       = 0.0f;  // 表示下限
constexpr uint8_t  kAvgSamples        = 20;    // 移動平均サンプル
constexpr float    kWheelCircumferenceM = 0.55f * 3.1415926f; // 推定周長[m]

// ---------- モデム/電源/LED ピン割り当て ----------
constexpr unsigned long kUsToSFactor   = 1000000ULL; // µs→s（未使用だが保持）
constexpr uint32_t       kSleepSec      = 60;        // ディープスリープ秒（未使用だが保持）
constexpr uint8_t        kUartTxPin     = 27;
constexpr uint8_t        kUartRxPin     = 26;
constexpr uint32_t       kUartBaud      = 115200;
constexpr uint8_t        kPwrKeyPin     = 4;   // SIM7600 PWR-KEY
constexpr uint8_t        kLedPin        = 12;  // 基板LED
constexpr uint8_t        kPowerCtrlPin  = 25;  // SIM7600 電源制御
constexpr uint8_t        kIndPin        = 36;  // SIM7600 ステータス

// ---------- ネットワーク/APN ----------
const char apn[]      = "povo.jp";
const char gprsUser[] = "";
const char gprsPass[] = "";

// ---------- HTTP/dweet.io（必要に応じて使用） ----------
const char kHttpHost[] = "dweet.io";
constexpr int kHttpPort = 80;
String dweetName   = "possibility-realize-galaxy";
String httpPath    = String("/dweet/for/") + dweetName;
String contentType = "application/json";

// ---------- デバイス・グローバル ----------
Ticker tick;                     // LED点滅などの周期処理
TinyGsm modem(SerialAT);         // モデムインスタンス
I2C_AXP192 axp192(I2C_AXP192_DEFAULT_ADDRESS, Wire1);

// 表示の周期管理
unsigned long dispCurrTime = 0;
unsigned long dispPrevTime = 0;

// ネットワーク状態表示フラグ（UI用途）
bool isNetworkConnected  = false;
bool isDatabaseUploaded  = false;  // アップロード有無（未実装: ダミー表示）
bool isLoggingStarted    = true;   // ロギング状態（未実装: ダミー表示）

// バッテリー情報
bool  battIsCharging = false;
float batVoltageMv   = 0.0f;
float batCapacityPct = 0.0f;
float batVbusMv      = 0.0f;
unsigned long battCurrTime = 0;
unsigned long battPrevTime = 0;

// 速度計測（ISR共有: 最新パルスタイムスタンプ）
volatile unsigned long spdCurrTime = 0;  // ISRで更新
unsigned long spdPrevTime = 0;           // 直近ループでの値
float wheelSpeedKmh = 0.0f;              // 瞬間速度
float spdAvg[kAvgSamples] = {0};         // 移動平均バッファ

// GPSデータ
float lat = 0, lon = 0, speed = 0, alt = 0;
int   vsat = 0, usat = 0;
float accuracy = 0;
int   year = 0, month = 0, day = 0;
int   hour = 0, minute = 0, second = 0;
unsigned long gpsInterval = 5000;  // 取得周期（ms）
unsigned long gpsCurrTime = 0;
unsigned long gpsPrevTime = 0;

// ----- ヘルパー関数宣言 -----
void initDisplay();
void initPower();
bool initModemAndNetwork();
void initHallSensor();
void drawMeter(int speedKmh);
void drawBattery(int x, int y, bool isCharge, int percent);
void drawStatusIcons();
void updateBattery();
void updateAndRender();
void updateGps();

// ホールセンサー割り込み: パルス時刻（ms）を記録
void IRAM_ATTR timeInterval(){
  spdCurrTime = millis();
}

// 角度degの円周上座標（メーター描画用）
static int rotX(int cx, int r, int deg) {
  return cx + r * cos(2*PI * (deg / 360.0f) - PI);
}
static int rotY(int cy, int r, int deg) {
  return cy + r * sin(2*PI * (deg / 360.0f) - PI);
}

// アナログメーター描画（0-~km/h）
void drawMeter(int speedKmh){
  using namespace UiCfg;

  // speedKmh をメーター角度に線形マッピング
  const float clamped = constrain((float)speedKmh, kSpeedMinKmh, kSpeedMaxKmh);
  const float t = (clamped - kSpeedMinKmh) / (kSpeedMaxKmh - kSpeedMinKmh); // 0..1
  const int needlePos = (int)(meterMinDeg + t * (float)(meterMaxDeg - meterMinDeg));

  // メーターリング
  u8g2.drawCircle(centerX, centerY, radius, U8G2_DRAW_ALL);
  u8g2.drawCircle(centerX, centerY, radius - ringWidth, U8G2_DRAW_ALL);

  // 針（三角）
  const int x1 = rotX(centerX, radius, needlePos-needleWidth);
  const int y1 = rotY(centerY, radius, needlePos-needleWidth);
  const int x2 = rotX(centerX, radius, needlePos+needleWidth);
  const int y2 = rotY(centerY, radius, needlePos+needleWidth);
  u8g2.drawTriangle(centerX, centerY, x1, y1, x2, y2);

  // 中抜き（内側リング）
  u8g2.setDrawColor(0);
  u8g2.drawDisc(centerX, centerY, radius - (ringWidth + 1), U8G2_DRAW_ALL);
  u8g2.setDrawColor(1);

  // 目盛り（30度刻み）
  for(int deg = meterMinDeg; deg <= meterMaxDeg; deg += 30){
    const int xn1 = rotX(centerX, radius - ringWidth, deg);
    const int yn1 = rotY(centerY, radius - ringWidth, deg);
    const int xn2 = rotX(centerX, radius - (ringWidth + 4), deg);
    const int yn2 = rotY(centerY, radius - (ringWidth + 4), deg);
    u8g2.drawLine(xn1, yn1, xn2, yn2);
  }
}

// バッテリーインジケータ描画
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

// ネットワーク/アップロード/ログ状態アイコン
void drawStatusIcons(){
  u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
  if(isNetworkConnected){
    u8g2.drawGlyph(2,10,0x0051); // Connected
  }else{
    u8g2.drawGlyph(2,10,0x0048); // Disconnected
  }

  if(isDatabaseUploaded){
    u8g2.drawGlyph(12,10,0x0043); // Uploaded
  }else{
    u8g2.drawGlyph(12,10,0x0054); // Not Uploaded
  }

  u8g2.setFont(u8g2_font_open_iconic_play_2x_t);
  if(isLoggingStarted){
    u8g2.drawGlyph(80, 45,0x0045); // Started
  }else{
    u8g2.drawGlyph(80, 45,0x0044); // Paused
  }
}

// ---------- 初期化 ----------
void initDisplay(){
  u8g2.begin();
  u8g2.setContrast(15);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_mr);
  u8g2.setCursor(12,10);
  u8g2.print("Wait...              ");
  u8g2.sendBuffer();
}

void initPower(){
  // AXP192 初期化（ボード依存の電圧設定。必要に応じ調整）
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

  // LED/PWR周辺
  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, LOW);
  pinMode(kPowerCtrlPin, OUTPUT);
  digitalWrite(kPowerCtrlPin, HIGH);

  // SIM7600 PWR-KEY パルス（typ. 500ms）
  pinMode(kPwrKeyPin, OUTPUT);
  digitalWrite(kPwrKeyPin, HIGH);
  delay(500);
  digitalWrite(kPwrKeyPin, LOW);

  // SIM7600 ステータスに応じLED点滅開始
  pinMode(kIndPin, INPUT);
  attachInterrupt(kIndPin, []() {
    detachInterrupt(kIndPin);
    tick.attach_ms(1000, []() {
      digitalWrite(kLedPin, !digitalRead(kLedPin));
    });
  }, CHANGE);
}

bool initModemAndNetwork(){
  SerialMon.println("Initializing modem...");
  u8g2.setCursor(12,10);
  u8g2.print("Initializing modem...");
  u8g2.sendBuffer();

  // UART for SIM7600
  SerialAT.begin(kUartBaud, SERIAL_8N1, kUartRxPin, kUartTxPin);

  if (!modem.init()) {
    SerialMon.println("Modem init failed");
    u8g2.setCursor(12,10);
    u8g2.print("Modem init failed  ");
    u8g2.sendBuffer();
    return false;
  }

  // 2:Auto 13:GSM 38:LTE only 51:GSM+LTE
  bool ok;
  do {
    ok = modem.setNetworkMode(38);
    u8g2.setCursor(12,10);
    u8g2.print("Network: LTE only  ");
    u8g2.sendBuffer();
    delay(500);
  } while (!ok);

  SerialMon.println("Waiting for network...");
  if (!modem.waitForNetwork()) {
    u8g2.setCursor(12,10);
    u8g2.print("Wait for network   ");
    u8g2.sendBuffer();
    return false;
  }

  isNetworkConnected = modem.isNetworkConnected();
  if (isNetworkConnected) {
    SerialMon.println("Network connected");
    u8g2.setCursor(12,10);
    u8g2.print("Network connected  ");
    u8g2.sendBuffer();
  }

  SerialMon.print("Connecting APN: ");
  SerialMon.println(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println("GPRS connect failed");
    return false;
  }

  // ログ用情報
  const bool gprs = modem.isGprsConnected();
  IPAddress local = modem.localIP();
  const int csq = modem.getSignalQuality();
  SerialMon.print("GPRS status: "); SerialMon.println(gprs);
  SerialMon.print("Local IP: ");    SerialMon.println(local);
  SerialMon.print("Signal: ");      SerialMon.println(csq);

  // 画面へ簡易表示
  u8g2.setFont(u8g2_font_5x7_mr);
  u8g2.setCursor(12,20); u8g2.print("Connecting to:  "); u8g2.print(apn);
  u8g2.setCursor(12,28); u8g2.print("Local IP:");
  u8g2.setCursor(20,36); u8g2.print(local);
  u8g2.setCursor(12,44); u8g2.print("Signal quality:"); u8g2.print(csq);
  u8g2.sendBuffer();

  // GPS有効化
  modem.enableGPS();
  return true;
}

void initHallSensor(){
  pinMode(kHallPin, INPUT);
  attachInterrupt(kHallPin, timeInterval, RISING);
}

void setup() {
  // シリアル開始
  Serial.begin(115200);
  delay(10);

  initDisplay();
  initPower();

  // モデム/ネットワーク初期化
  (void)initModemAndNetwork();

  // ホールセンサー割り込み設定
  initHallSensor();

  // 軽く待つ
  delay(500);
}

// 表示・速度・バッテリー更新と描画
void updateAndRender(){
  // 速度計算（パルス間隔 → km/h）
  unsigned long curr;
  noInterrupts();
  curr = spdCurrTime; // 原子スナップショット
  interrupts();

  const unsigned long diffMs = curr - spdPrevTime;
  if (diffMs > 0) {
    // v[km/h] = C[m] / (dt[s]) * 3.6, dt[s] = diffMs/1000
    wheelSpeedKmh = (kWheelCircumferenceM * 3.6f * 1000.0f) / (float)diffMs;
    if (wheelSpeedKmh > 99.0f) wheelSpeedKmh = 99.0f; // 表示は2桁想定
  }
  spdPrevTime = curr;

  // 移動平均（最新を先頭へシフト）
  for (int i = kAvgSamples - 1; i > 0; --i) {
    spdAvg[i] = spdAvg[i - 1];
  }
  spdAvg[0] = wheelSpeedKmh;

  float sum = 0.0f; // バグ修正: ループ毎に初期化
  for (int i = 0; i < kAvgSamples; ++i) sum += spdAvg[i];
  const float avgKmh = sum / (float)kAvgSamples;

  // 画面描画
  char spdText[8];
  dtostrf(avgKmh, 2, 0, spdText); // 整数表示（最大2桁）

  u8g2.firstPage();
  do {
    drawMeter((int)avgKmh);

    u8g2.setFont(u8g2_font_logisoso32_tn);
    u8g2.drawStr(37, 58, spdText);
    u8g2.setFont(u8g2_font_t0_11b_te);
    u8g2.setCursor(78,58);
    u8g2.print("km/h");

    drawBattery(119, 22, battIsCharging, (int)batCapacityPct);
    u8g2.setFont(u8g2_font_5x7_mr);
    u8g2.setCursor(118,30);
    u8g2.print(batCapacityPct, 0);

    drawStatusIcons();
  } while (u8g2.nextPage());
}

// バッテリー情報の更新
void updateBattery(){
  battCurrTime = millis();
  if((battCurrTime - battPrevTime) < kBattIntervalMs) return;

  noInterrupts();
  batVoltageMv = axp192.getBatteryVoltage();
  batVbusMv    = axp192.getVbusVoltage();
  interrupts();

  battIsCharging = (batVbusMv >= 200.0f);
  batCapacityPct = 100.0f * (batVoltageMv - kMinBattMv) / (kMaxBattMv - kMinBattMv);
  if (batCapacityPct < 0)   batCapacityPct = 0;
  if (batCapacityPct > 100) batCapacityPct = 100;
  battPrevTime = battCurrTime;
}

// GPSの取得とログ（UIは簡易）
void updateGps(){
  gpsCurrTime = millis();
  if((gpsCurrTime - gpsPrevTime) < gpsInterval) return;

  if(modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy, &year, &month, &day, &hour, &minute, &second)){
    Serial.printf("Lat:%.6f Lon:%.6f\n", lat, lon);
    Serial.printf("%04d/%02d/%02d %02d:%02d:%02d\n", year, month, day, hour, minute, second);
    // GPS Fix時はLEDを速く点滅
    tick.attach_ms(200, []() { digitalWrite(kLedPin, !digitalRead(kLedPin)); });
  }else{
    Serial.println("GPS no data");
  }

  // 例: dweet.ioへ送信（必要時に有効化）
  // TinyGsmClient client(modem);
  // HttpClient http(client, kHttpHost, kHttpPort);
  // String payload = String("{\"Speed\":\"") + (int)avgKmh + "\"}"; // 送信内容を適宜拡張
  // http.post(httpPath, contentType, payload);
  // int statusCode = http.responseStatusCode();
  // String response = http.responseBody();
  // Serial.printf("HTTP %d: %s\n", statusCode, response.c_str());
  // http.stop();

  gpsPrevTime = gpsCurrTime;
}

void loop() {
  // 表示更新
  dispCurrTime = millis();
  if((dispCurrTime - dispPrevTime) >= kDispIntervalMs){
    updateAndRender();
    dispPrevTime = dispCurrTime;
  }

  // バッテリー更新
  updateBattery();

  // GPS更新
  updateGps();
}

