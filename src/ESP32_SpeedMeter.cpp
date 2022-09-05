#define TINY_GSM_MODEM_SIM7600 //Use SIM7600JC LTE Module
#define SerialMon Serial //Default speed 115200 baud
#define SerialAT Serial1 //for AT commands to the module
// #define DUMP_AT_COMMANDS //for debug

#define TINY_GSM_DEBUG SerialMon //for serial console

#define USE_GSM //Use SIM7600JC for GSM communication

#define sample 20

#include <Arduino.h>
#include "I2C_AXP192.h"
#include <SPI.h>
#include <U8g2lib.h>

#include <TinyGsmClient.h>
#include <Ticker.h>
#include <ArduinoHttpClient.h>
Ticker tick;


U8G2_ST7565_ERC12864_F_4W_SW_SPI u8g2(U8G2_R2,/* clock=*/ 12, /* data=*/ 14, /* cs=*/ 15, /* dc=*/ 2, /* reset=*/ 13);

//GSM credentials
const char apn[]  = "povo.jp";
const char gprsUser[] = "";
const char gprsPass[] = "";

//Modem settings
#define uS_TO_S_FACTOR          1000000ULL  //Conversion factor for micro seconds to seconds 
#define TIME_TO_SLEEP           60          //Time ESP32 will go to sleep (in seconds) 
#define PIN_TX                  27
#define PIN_RX                  26
#define UART_BAUD               115200
#define PWR_PIN                 4
#define LED_PIN                 12
#define POWER_PIN               25
#define IND_PIN                 36

TinyGsm modem(SerialAT);

//Update display
unsigned long dispInterval = 100;
unsigned long dispCurrTime = 0;
unsigned long dispPrevTime = 0;

//Network connection
bool isNetworkConnected = true;
bool isDatabaseUploaded = true;
bool isLoggingStarted = true;

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
unsigned long battInterval = 5000;
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

unsigned long spdCurrTime = 0;
unsigned long spdPrevTime = 0;
float wheelSpeed = 0.0f;
float WheelAvg = 0.0f;
float spdAvg[sample] = {0};
char spdTexbuf[3];

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

unsigned long gpsInterval = 5000;
unsigned long gpsCurrTime = 0;
unsigned long gpsPrevTime = 0;

I2C_AXP192 axp192(I2C_AXP192_DEFAULT_ADDRESS, Wire1);

void IRAM_ATTR timeInterval(){
  spdCurrTime = millis();
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
  // put your setup code here, to run once:
 
  u8g2.begin();
  u8g2.setContrast(15);
  u8g2.clearBuffer();
  
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
  digitalWrite(POWER_PIN, HIGH);

  // PWR_PIN ： This Pin is the PWR-KEY of the SIM7600
  // The time of active low level impulse of PWRKEY pin to power on module , type 500 ms
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(500);
  digitalWrite(PWR_PIN, LOW);

  // IND_PIN: It is connected to the SIM7600 status Pin,
  // through which you can know whether the module starts normally.
  pinMode(IND_PIN, INPUT);

  attachInterrupt(IND_PIN, []() {
    detachInterrupt(IND_PIN);
    // If SIM7600 starts normally, then set the onboard LED to flash once every 1 second
    tick.attach_ms(1000, []() {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    });
  }, CHANGE);

  SerialMon.println("Wait...");
  // u8g2.setFont(u8g2_font_5x7_mr);
  // u8g2.setCursor(12,10);
  // u8g2.print("Wait...              ");
  // u8g2.sendBuffer();

  delay(3000);

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  //モデムの初期化
  // u8g2.setCursor(12,10);
  // u8g2.print("Initializing modem...");
  // u8g2.sendBuffer();

  SerialMon.println("Initializing modem...");
  if (!modem.init()) {
    SerialMon.println("Failed to restart modem, delaying 10s and retrying");
    // u8g2.setCursor(12,10);
    // u8g2.print("Modem init failed  ");
    // u8g2.sendBuffer();
    return;
  }
  SerialMon.println("enter setNetwork Mode");

  //接続開始
  bool result;
  do {
    result = modem.setNetworkMode(38);//2 Automatic, 13 GSM only,  38 LTE only,  51 GSM and LTE only
    // u8g2.setCursor(12,10);
    // u8g2.print("Network: LTE only  ");
    // u8g2.sendBuffer();
    delay(500);
  } while (result != true);

  SerialMon.println("Waiting for network...");
  if (!modem.waitForNetwork()) {
    delay(10000);
    SerialMon.println("waitForNetwork");
    // u8g2.setCursor(12,10);
    // u8g2.print("Wait for network   ");
    // u8g2.sendBuffer();
    return;
  }

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
    // u8g2.setCursor(12,10);
    // u8g2.print("Network connected  ");
    // u8g2.sendBuffer();
  }
  // u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
  // if(isNetworkConnected){
  //   u8g2.drawGlyph(2,10,0x0051); //Connected
  // }else{
  //   u8g2.drawGlyph(2,10,0x0048); //Disconnected
  // }
  // u8g2.sendBuffer();

  SerialMon.print("Connecting to:");
  SerialMon.println(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    delay(10000);
    return;
  }

  bool res = modem.isGprsConnected();
  SerialMon.print("GPRS status:");
  SerialMon.println(res);

  IPAddress local = modem.localIP();
  SerialMon.print("Local IP:");
  SerialMon.println(local);

  int csq = modem.getSignalQuality();
  SerialMon.print("Signal quality:");
  SerialMon.println(csq);

  // u8g2.setFont(u8g2_font_5x7_mr);

  // u8g2.setCursor(12,20);
  // u8g2.print("Connecting to:  ");
  // u8g2.print(apn);

  // u8g2.setCursor(12,28);
  // u8g2.print("Local IP:");
  // u8g2.setCursor(20,36);
  // u8g2.print(local);

  // u8g2.setCursor(12,44);
  // u8g2.print("Signal quality:");
  // u8g2.print(csq);

  // u8g2.sendBuffer();

  //GPS enable
  modem.enableGPS();

  delay(2000);

  //Hall sensor interrupt setting
  pinMode(32, INPUT);
  attachInterrupt(32, timeInterval, RISING);

  // u8g2.clearDisplay();
  delay(100);
}

void loop() {
  // put your main code here, to run repeated
    
  u8g2.firstPage();
  do {
    dispCurrTime = millis();
    if((dispCurrTime - dispPrevTime) >= dispInterval){
      unsigned long spdDiffTime = spdCurrTime - spdPrevTime;
      if(spdDiffTime > 0){        
        wheelSpeed = (0.55 * 3.14 * 3.6 * 1000) / (spdDiffTime);
        if(wheelSpeed >= 100){
          wheelSpeed = 99;
        }
      }
      spdPrevTime = spdCurrTime;

      for(int i = sample - 1; i > 0; i--){
        spdAvg[i] = spdAvg[i-1];
      }
      spdAvg[0] = wheelSpeed;

      for(int i = 0; i < sample; i++){
        WheelAvg += spdAvg[i];
      }
      WheelAvg = (float)WheelAvg/sample;
      drawMeter(WheelAvg);
      dtostrf(WheelAvg, 2, 0, spdTexbuf);
      // drawMeter(wheelSpeed);
      // dtostrf(wheelSpeed, 2, 0, spdTexbuf);

      u8g2.setFont(u8g2_font_logisoso32_tn);
      u8g2.drawStr(37, 58, spdTexbuf);
      u8g2.setFont(u8g2_font_t0_11b_te);
      u8g2.setCursor(78,58);
      u8g2.print("km/h");

      drawBattery(119, 22, battIsCharge, batCapacity);
      u8g2.setFont(u8g2_font_5x7_mr);
      u8g2.setCursor(118,30);
      u8g2.print(batCapacity, 0);

      u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
      if(isNetworkConnected){
        u8g2.drawGlyph(2,10,0x0051); //Connected
      }else{
        u8g2.drawGlyph(2,10,0x0048); //Disconnected
      }

      if(isDatabaseUploaded){
        u8g2.drawGlyph(12,10,0x0043); //Uploaded
      }else{
        u8g2.drawGlyph(12,10,0x0054); //Not Uploaded
      }

      u8g2.setFont(u8g2_font_open_iconic_play_2x_t);
      if(isLoggingStarted){
        u8g2.drawGlyph(80, 45,0x0045); //Started
      }else{
        u8g2.drawGlyph(80, 45,0x0044); //paused
      }
      
      }
    dispPrevTime = dispCurrTime;

    battCurrTime = millis();
    if((battCurrTime - battPrevTime) >= battInterval){
      noInterrupts();
      batVoltage = axp192.getBatteryVoltage();
      batVbusVol = axp192.getVbusVoltage();
      interrupts();
      if(batVbusVol >= 200){
        battIsCharge = true;
      }else{
        battIsCharge = false;
      }
      batCapacity = 100 * (batVoltage - minBatVol)/(maxBatVol - minBatVol);
      battPrevTime = battCurrTime;
      //Serial.printf("BatteryVoltage : %7.2f\n", batVoltage);
    }

  } while ( u8g2.nextPage() );

  // gpsCurrTime = millis();
  // if((gpsCurrTime - gpsPrevTime) >= 5000){
  //   if(modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy, &year, &month, &day, &hour, &minute, &second)){
  //     Serial.printf("Lat:%f lon:%f\n", lat, lon);
  //     Serial.printf("%d/%d/%d ", year, month, day);
  //     Serial.printf("%d:%d:%d \n", hour, minute, second);
  //     tick.attach_ms(200, []() {
  //             digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  //     });
  //   }else{
  //     Serial.printf("GPS no data\n");
  //   }
  //   gpsCurrTime = gpsPrevTime;
  // }

}
