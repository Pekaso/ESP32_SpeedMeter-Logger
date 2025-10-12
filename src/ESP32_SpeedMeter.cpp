#define TINY_GSM_MODEM_SIM7600 //Use SIM7600JC LTE Module
#define SerialMon Serial //Default speed 115200 baud
#define SerialAT Serial1 //for AT commands to the module
// #define DUMP_AT_COMMANDS //for debug

#define TINY_GSM_DEBUG SerialMon //for serial console

#define sample 30

// Low-speed smoothing feature (disabled by default)
#define ENABLE_LOW_SPEED_SMOOTHING 1
#define LOW_SPEED_THRESHOLD_KMH 5.0f
#define SMOOTH_DECAY_DELAY_MS 300UL
#define SMOOTH_DECAY_TIME_MS 1500UL

#define switch1 23
#define switch2 19
#define sensor 32

#include <Arduino.h>
#include "I2C_AXP192.h"
#include <SPI.h>
#include <U8g2lib.h>
#include "donguri_bitmap.h"

#include <TinyGsmClient.h>
#include <Ticker.h>
#include <ArduinoHttpClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

Ticker tick;

// bitmap moved into header above

U8G2_ST7565_ERC12864_F_4W_SW_SPI u8g2(U8G2_R0,/* clock=*/ 33, /* data=*/ 14, /* cs=*/ 15, /* dc=*/ 2, /* reset=*/ 13);

//GSM credentials
// const char apn[]  = "povo.jp";
// const char gprsUser[] = "";
// const char gprsPass[] = "";
const char apn[]  = "iijmio.jp";
const char gprsUser[] = "mio@iij";
const char gprsPass[] = "iij";


//Modem settings
#define uS_TO_S_FACTOR          1000000ULL  //Conversion factor for micro seconds to seconds 
//#define TIME_TO_SLEEP           60          //Time ESP32 will go to sleep (in seconds) 
#define PIN_TX                  27
#define PIN_RX                  26
#define UART_BAUD               115200
#define PWR_PIN                 4
#define LED_PIN                 12
#define POWER_PIN               25
#define IND_PIN                 36

int systemState = 0;

bool isOnline = false;
int networkState = 0;
unsigned long netInterval = 100;
unsigned long netCurrTime = 0;
unsigned long netPrevTime = 0;

bool LED_STATS = false;

//Update display
unsigned long dispInterval = 100;
unsigned long dispCurrTime = 0;
unsigned long dispPrevTime = 0;

//Network connection
bool isNetworkConnected = false;
bool isDatabaseUploaded = false;
bool isLoggingStarted = false;
int isUploadingCount = 0;

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
bool sensorState = 0;
const float spdMax = 70.0f;
const float spdMin = 0.0f;

const int zeroDet = 3;
int zeroCnt = 0;

volatile unsigned long spdCurrTime = 0;
unsigned long spdPrevTime = 0;
unsigned long spdDiffTime = 0;
float wheelSpeed = 0.0f;
float WheelAvg = 0.0f;
float spdAvg[sample] = {0};
char spdTexbuf[3];

String dataPayload = "";

//Timer
bool isTimerStart = false;
int timeInihold = 0;
int timeElapsed = 0;
int timeMinhold = 0;
int timeSechold = 0;

volatile bool lapPushed = false;
int lapCount = 0;

char timeTextbuf[10][6] = {"00:00",
                          "00:00",
                          "00:00",
                          "00:00",
                          "00:00",
                          "00:00",
                          "00:00",
                          "00:00",
                          "00:00",
                          "00:00"};

//GPS data
float lat       = 0;
float lon       = 0;
float speed     = 0;
float alt       = 0;
int   vsat      = 0;
int   usat      = 0;
float accuracy  = 0;
int   year      = 0;
int   month     = 0;
int   day       = 0;
int   hour      = 0;
int   minute    = 0;
int   second    = 0;

unsigned long gpsInterval = 2000;
unsigned long gpsCurrTime = 0;
unsigned long gpsPrevTime = 0;

//Http Client
const char serverAddress[] = "dweet.io";  // server address
const int port = 80;

String dweetName = "possibility-realize-galaxy";
String path = "/dweet/for/" + dweetName;
String contentType = "application/json";
String postData;

TinyGsmClient client(modem);
// HttpClient    http = HttpClient(client, serverAddress, port);

I2C_AXP192 axp192(I2C_AXP192_DEFAULT_ADDRESS, Wire1);

void IRAM_ATTR timeInterval(){
  spdCurrTime = millis();
  digitalWrite(LED_PIN, LOW);
}

void IRAM_ATTR laptimequery(){
  lapPushed = true;
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

// networkStep
// - Drives SIM7600 initialization and connection state machine (no data send).
static void networkStep() {
  if(!isOnline) return;
  switch(networkState){
    case 0:
      digitalWrite(POWER_PIN, HIGH);
      networkState = 1;
      SerialMon.println("networkState 0 -> 1");
      netInterval = 500;
      netPrevTime = millis();
      digitalWrite(PWR_PIN, HIGH);
      break;
    case 1:
      if((netCurrTime - netPrevTime) >= netInterval){
        digitalWrite(PWR_PIN, LOW);
        networkState = 2;
        SerialMon.println("networkState 1 -> 2");
      }
      break;
    case 2:
      netInterval = 3000;
      netPrevTime = millis();
      networkState = 3;
      SerialMon.println("networkState 2 -> 3");
      break;
    case 3:
      if((netCurrTime - netPrevTime) >= netInterval){
        SerialMon.println("Wait...");
        SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
        networkState = 4;
        SerialMon.println("networkState 3 -> 4");
      }
      break;
    case 4:
      SerialMon.println("Initializing modem...");
      if (!modem.init()) {
        SerialMon.println("Failed to restart modem, delaying 10s and retrying");
      }
      SerialMon.println("enter setNetwork Mode");
      networkState = 5;
      SerialMon.println("networkState 4 -> 5");
      break;
    case 5: {
      bool result = modem.setNetworkMode(38);
      (void)result;
      if (modem.waitResponse(10000L) != 1){
        SerialMon.println("setNetworkMode fail");
      }
      networkState = 6;
      SerialMon.println("networkState 5 -> 6");
      break; }
    case 6:
      SerialMon.print("Connecting to:");
      SerialMon.println(apn);
      modem.gprsConnect(apn, gprsUser, gprsPass);
      netInterval = 10000;
      netPrevTime = millis();
      networkState = 7;
      SerialMon.println("networkState 6 -> 7");
      break;
    case 7:
      if((netCurrTime - netPrevTime) >= netInterval){
        bool res = modem.isGprsConnected();
        SerialMon.print("GPRS status:");
        SerialMon.println(res);

        IPAddress local = modem.localIP();
        SerialMon.print("Local IP:");
        SerialMon.println(local);

        int csq = modem.getSignalQuality();
        SerialMon.print("Signal quality:");
        SerialMon.println(csq);

        netInterval = 2000;
        netPrevTime = millis();
        networkState = 8;
        SerialMon.println("networkState 7 -> 8");
        if(modem.isNetworkConnected()){
          SerialMon.println("Network Initialized");
          isNetworkConnected = true;
        }else{
          digitalWrite(POWER_PIN, LOW);
          digitalWrite(PWR_PIN, HIGH);
          SerialMon.println("Offline mode");
          isNetworkConnected = false;
          isOnline = false;
        }
      }
      break;
    case 8:
      // Connected; data publish handled by networkPublish()
      break;
    case 9:
      // Error state (not used)
      break;
    default:
      break;
  }
}

// networkPublish
// - Sends telemetry to dweet.io when connected and interval elapsed.
static void networkPublish() {
  if(!isOnline || networkState != 8) return;
  if((netCurrTime - netPrevTime) < netInterval) return;

  dataPayload = "/dweet/for/possibility-realize-galaxy?Time="+String(timeTextbuf[0])
                +"&Speed="+String((int)WheelAvg)
                +"&Battery="+String(batCapacity);
  HttpClient http = HttpClient(client, serverAddress, port);
  int err = http.get(dataPayload);
  isDatabaseUploaded = true;
  if (err != 0) {
    SerialMon.println("failed to connect");
    isDatabaseUploaded = false;
  }
  netInterval = 2000;
  netPrevTime = millis();
}

// readButtonsAndTimer
// - Reads start/pause button and updates stopwatch values and formatted text.
static void readButtonsAndTimer(unsigned long now) {
  if(digitalRead(switch1) == 0){
    if(isTimerStart){
      timeElapsed = now - timeInihold;
      timeSechold = timeElapsed / 1000;
      timeMinhold = timeSechold / 60;
    }else{
      isTimerStart = true;
      timeElapsed = 0;
      timeSechold = 0;
      timeMinhold = 0;
      timeInihold = now;
    }
  }else{
    isTimerStart = false;
  }
  sprintf(timeTextbuf[0], "%02d:%02d", timeMinhold, (timeSechold)%60);
}

// computeSpeedAndAverage
// - Converts pulse intervals to speed (km/h), applies optional low-speed smoothing,
//   maintains moving average buffer and updates WheelAvg.
static void computeSpeedAndAverage() {
  static float lastInstantSpeed = 0.0f; // km/h

  spdDiffTime = spdCurrTime - spdPrevTime;
  if(spdDiffTime > 0){
    wheelSpeed = (0.55 * 3.14 * 3.6 * 1000) / (spdDiffTime);
    if(wheelSpeed >= 100){ wheelSpeed = 99; }
    lastInstantSpeed = wheelSpeed;
  }else{
#if ENABLE_LOW_SPEED_SMOOTHING
    unsigned long sincePulse = dispCurrTime - spdPrevTime;
    if(lastInstantSpeed <= LOW_SPEED_THRESHOLD_KMH){
      if(sincePulse <= SMOOTH_DECAY_DELAY_MS){
        wheelSpeed = lastInstantSpeed;
      }else{
        float t = (float)(sincePulse - SMOOTH_DECAY_DELAY_MS) / (float)SMOOTH_DECAY_TIME_MS;
        if(t > 1.0f) t = 1.0f;
        wheelSpeed = lastInstantSpeed * (1.0f - t);
      }
    }else{
      wheelSpeed = 0;
    }
#else
    wheelSpeed = 0;
#endif
  }
  spdPrevTime = spdCurrTime;

  for(int i = sample - 1; i > 0; i--){
    spdAvg[i] = spdAvg[i-1];
  }
  spdAvg[0] = wheelSpeed;

  WheelAvg = 0.0f;
  for(int i = 0; i < sample; i++){
    WheelAvg += spdAvg[i];
  }
  WheelAvg = (float)WheelAvg / sample;
}

// renderUI
// - Renders meter, texts, icons, and initialization overlay.
static void renderUI() {
  if((dispCurrTime - dispPrevTime) < dispInterval) return;

  u8g2.firstPage();
  do {
    digitalWrite(LED_PIN, HIGH);

    drawMeter(WheelAvg);
    dtostrf(WheelAvg, 2, 0, spdTexbuf);

    u8g2.setFont(u8g2_font_logisoso28_tn);
    u8g2.drawStr(39, 47, spdTexbuf);
    u8g2.setFont(u8g2_font_t0_11b_te);
    u8g2.setCursor(76,47);
    u8g2.print("km/h");

    u8g2.setFont(u8g2_font_mercutio_sc_nbp_tn);
    u8g2.drawStr(56, 62, timeTextbuf[0]);

    drawBattery(119, 22, battIsCharge, batCapacity);
    u8g2.setFont(u8g2_font_5x7_mr);
    u8g2.setCursor(118,30);
    u8g2.print(batCapacity, 0);

    u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
    if(isNetworkConnected){
      u8g2.drawGlyph(2,10,0x0051);
    }else{
      u8g2.drawGlyph(2,10,0x0054);
    }

    if(isDatabaseUploaded){
      u8g2.drawGlyph(12,10,0x0043);
      isUploadingCount++;
      if(isUploadingCount >= 10){
        isDatabaseUploaded = false;
        isUploadingCount = 0;
      }
    }

    u8g2.setFont(u8g2_font_open_iconic_play_1x_t);
    if(isTimerStart){
      u8g2.drawGlyph(42, 61,0x0045);
    }else{
      u8g2.drawGlyph(42, 61,0x0044);
    }

    if(isOnline && (networkState <= 4)){
      u8g2.clearBuffer();
      u8g2.setBitmapMode(false /* solid */);
      u8g2.setDrawColor(0);
      u8g2.drawXBM( 0, 0, 128, 47, epd_bitmap_donguri);
      u8g2.setDrawColor(1);
      u8g2.setFont(u8g2_font_samim_12_t_all);
      u8g2.setCursor(30,60);
      u8g2.print("Initializing...");
    }

    if(axp192.getPekPress() == 1){
      SerialMon.println("Shutdown...");
      digitalWrite(POWER_PIN, LOW);
      digitalWrite(PWR_PIN, HIGH);
      u8g2.clearBuffer();
      u8g2.setCursor(40,30);
      u8g2.print("Shutdown...");
      u8g2.sendBuffer();
      delay(500);
      axp192.powerOff();
    }

  } while ( u8g2.nextPage() );

  dispPrevTime = dispCurrTime;
}

// updateBatteryStatus
// - Periodically samples battery voltages and computes percentage and charge state.
static void updateBatteryStatus() {
  if((battCurrTime - battPrevTime) < battInterval) return;
  batVoltage = axp192.getBatteryVoltage();
  batVbusVol = axp192.getVbusVoltage();
  battIsCharge = (batVbusVol >= 200);
  batCapacity = 100 * (batVoltage - minBatVol)/(maxBatVol - minBatVol);
  battPrevTime = battCurrTime;
}

void setup() {
  // put your setup code here, to run once:
 
  u8g2.begin();
  u8g2.setContrast(15);
  u8g2.clearBuffer();

  pinMode(switch1, INPUT_PULLUP);
  delay(100);
  isOnline = digitalRead(switch1);
  
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
  //delay(1000);
  noInterrupts();
  Wire1.begin(21, 22);
  axp192.begin(initDef);
  interrupts();

  Serial.begin(115200);
  delay(10);

  // Onboard LED light, it can be used freely
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // POWER_PIN : This pin controls the power supply of the SIM7600
  pinMode(POWER_PIN, OUTPUT);

  // PWR_PIN ： This Pin is the PWR-KEY of the SIM7600
  // The time of active low level impulse of PWRKEY pin to power on module , type 500 ms
  pinMode(PWR_PIN, OUTPUT);

  //Hall sensor interrupt setting
  pinMode(sensor, INPUT);
  attachInterrupt(sensor, timeInterval, RISING);

  // pinMode(switch2, INPUT_PULLUP);
  // attachInterrupt(switch2, laptimequery, FALLING);

}

void loop() {
  // put your main code here, to run repeated
  unsigned long now = millis();
  dispCurrTime = now;
  netCurrTime = now;
  battCurrTime = now;

  // Inputs & network
  networkStep();
  networkPublish();
  // Inputs processing
  readButtonsAndTimer(now);
  computeSpeedAndAverage();
  // Output rendering
  renderUI();

  updateBatteryStatus();

  // gpsCurrTime = millis();
  // if((gpsCurrTime - gpsPrevTime) >= gpsInterval){
  //   if(modem.isNetworkConnected()){
  //     // if(modem.getGPS(&lat, &lon)){
  //     //   Serial.printf("Lat:%f lon:%f\n", lat, lon);
  //     //   tick.attach_ms(200, []() {
  //     //           digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  //     //   });
  //     // }else{
  //     //   Serial.printf("GPS no data\n");
  //     // }

  //     // Serial.printf("Switch1 (23pin):%d\n", digitalRead(23));

  //     HttpClient    http = HttpClient(client, serverAddress, port);

  //     // Serial.println(timeTextbuf);

  //     //http.connectionKeepAlive();
  //     // dataPayload = "/dweet/for/possibility-realize-galaxy?Time="+String(timeTextbuf)
  //     //                                                       // "&Latitude="+String(lat)+
  //     //                                                       // "&Longtitude="+String(lon)+
  //     //                                                       // "&Time(Sec)="+String((int)timeSechold)+
  //     //                                                       +"&Speed="+String((int)WheelAvg)
  //     //                                                       +"&Battery="+String(batCapacity)
  //     //                                                       ;

  //     int err = http.get(dataPayload);

  //     // int err = http.get("/dweet/for/possibility-realize-galaxy?Speed="+String((int)WheelAvg)+
  //     //                                                       // "&Latitude="+String(lat)+
  //     //                                                       // "&Longtitude="+String(lon)+
  //     //                                                       // "&Time(Sec)="+String((int)timeSechold)+
  //     //                                                       "&Time="+String(minTextbuf)+":"+String(secTextbuf)+
  //     //                                                       "&Battery="+String(batCapacity)
  //     //                                                       );
  
  //     isDatabaseUploaded = true;
  //     if (err != 0) {
  //       SerialMon.println("failed to connect");
  //       isDatabaseUploaded = false;
  //     }
  //   }
  //   gpsPrevTime = gpsCurrTime;
  // }

}
