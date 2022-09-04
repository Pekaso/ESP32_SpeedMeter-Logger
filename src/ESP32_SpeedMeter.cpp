#define sample 20

#include <Arduino.h>
#include "I2C_AXP192.h"
#include <SPI.h>
#include <U8g2lib.h>

U8G2_ST7565_ERC12864_F_4W_SW_SPI u8g2(U8G2_R2,/* clock=*/ 12, /* data=*/ 14, /* cs=*/ 15, /* dc=*/ 2, /* reset=*/ 13);

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
  pinMode(27, INPUT);
  attachInterrupt(27, timeInterval, RISING);
  
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
}

void loop() {
  // put your main code here, to run repeatedl

  dispCurrTime = millis();
  if((dispCurrTime - dispPrevTime) >= dispInterval){
    
    u8g2.firstPage();
    do {

      // if(wheelSpeed >= 1){
      //   wheelSpeed = wheelSpeed - 1;
      // }
      if((spdCurrTime - spdPrevTime) > 0){
        wheelSpeed = (0.55 * 3.14 * 3.6 * 1000) / (spdCurrTime - spdPrevTime);
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
      
      } while ( u8g2.nextPage() );
    dispPrevTime = dispCurrTime;

  }

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
    Serial.printf("BatteryVoltage : %7.2f\n", batVoltage);
  }

}
