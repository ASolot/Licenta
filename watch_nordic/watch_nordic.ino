// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

/*
 * Serial Port over BLE
 * Create UART service compatible with Nordic's *nRF Toolbox* and Adafruit's *Bluefruit LE* iOS/Android apps.
 *
 * BLESerial class implements same protocols as Arduino's built-in Serial class and can be used as it's wireless
 * replacement. Data transfers are routed through a BLE service with TX and RX characteristics. To make the
 * service discoverable all UUIDs are NUS (Nordic UART Service) compatible.
 *
 * Please note that TX and RX characteristics use Notify and WriteWithoutResponse, so there's no guarantee
 * that the data will make it to the other end. However, under normal circumstances and reasonable signal
 * strengths everything works well.
 */


// Import libraries (BLEPeripheral depends on SPI)
#include <SPI.h>
#include <BLEPeripheral.h>
#include "BLESerial.h"

// define pins (varies per shield/board)
#define BLE_REQ   10
#define BLE_RDY   2
#define BLE_RST   9

// create ble serial instance, see pinouts above
BLESerial BLESerial(BLE_REQ, BLE_RDY, BLE_RST);

// Import libraries for Screen

#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>

// Configure Screen pins 
#define SHARP_SCK  15
#define SHARP_MOSI 16
#define SHARP_SS   17
#define SHARP_DISP 18

// Set the size of the display here, e.g. 144x168!
Adafruit_SharpMem display(SHARP_SCK, SHARP_MOSI, SHARP_SS, 96, 96);
// The currently-available SHARP Memory Display (144x168 pixels)
// requires > 4K of microcontroller RAM; it WILL NOT WORK on Arduino Uno
// or other <4K "classic" devices!  The original display (96x96 pixels)
// does work there, but is no longer produced.

#define BLACK 0
#define WHITE 1

int minorHalfSize; // 1/2 of lesser of display width or height

bool bRefreshScreen = false;

// RTC

#include "RTCInt.h"

RTCInt rtc;

int hours;
int minutes;
int seconds;

int day;
int month;
int year;

// text displayed
char hourText[10];

// timers 
#include "Timer.h"

TimerClass screenRefreshTimer(3, 0);

void setup() {
  // custom services and characteristics can be added as well
  BLESerial.setLocalName("AS_Watch");

  Serial.begin(115200);
  BLESerial.begin();

  pinMode(SHARP_DISP, OUTPUT);
  digitalWrite(SHARP_DISP, HIGH);

  ////// Display side
  
  // start & clear the display
  display.begin();
  display.clearDisplay();

  // Several shapes are drawn centered on the screen.  Calculate 1/2 of
  // lesser of display width or height, this is used repeatedly later.
  minorHalfSize = min(display.width(), display.height()) / 2;

//  // draw a single pixel
//  display.drawPixel(10, 10, BLACK);
//  display.refresh();
//  delay(500);
//  display.clearDisplay();
//
//  // draw many lines
//  testdrawline();
//  delay(500);
//  display.clearDisplay();

  // draw rectangles
  testdrawrect();
  delay(500);
  display.clearDisplay();

  // draw multiple rectangles
  testfillrect();
  display.refresh();
  delay(500);
  display.clearDisplay();

//  // draw a circle, 10 pixel radius
//  display.fillCircle(display.width()/2, display.height()/2, 10, BLACK);
//  display.refresh();
//  delay(500);
//  display.clearDisplay();
//
//  testdrawroundrect();
//  display.refresh();
//  delay(500);
//  display.clearDisplay();
//
//  testfillroundrect();
//  display.refresh();
//  delay(500);
//  display.clearDisplay();

//  testdrawtriangle();
//  display.refresh();
//  delay(500);
//  display.clearDisplay();
//
//  testfilltriangle();
//  display.refresh();
//  delay(500);
//  display.clearDisplay();
//
//  testdrawchar();
//  display.refresh();
//  for(int i=0; i<4; i++) {
//    display.refresh();
//    delay(500);
//  }

  // init rtc

  hours = 18;
  minutes = 30;
  seconds = 0;

  day = 6;
  month = 6;
  year = 18;
  
  rtc.begin(TIME_H24);

  rtc.setHour(hours,0);
  rtc.setMinute(minutes);
  rtc.setSecond(seconds);

  rtc.setDay(day);
  rtc.setMonth(month);
  rtc.setYear(year);

  sprintf(hourText, "%s", "18:30:25");

  screenRefreshTimer.attachInterrupt(&timerHandler, 1000000);
  //blePollTimer.attachInterrupt(&blePoll, 200000);
}

void refreshScreen(int orientation);

void loop() {
  
  BLESerial.poll();

  //forward();
  loopback();
  //spam();

  if(bRefreshScreen)
  {
    bRefreshScreen = false;
    refreshScreen(1);
  }
  
}

void refreshScreen(int orientation = 1) {

    display.setRotation(orientation);
    display.clearDisplay();
    // text display tests
    display.setTextSize(2);
    display.setTextColor(BLACK);
    display.setCursor(0,35);

    rtc.getDate();
    rtc.getTime();
    sprintf(hourText,"%02d:%02d:%02d\n", rtc.time.hour, rtc.time.minute, rtc.time.second);
    
    display.println(hourText);

    display.setTextSize(1);

    display.setCursor(25, 60);
    display.println("IAQ: 50");
    display.setCursor(10, 75);
    display.println("2346 steps");
    
    // Screen must be refreshed at least once per second

    display.refresh();
}

void timerHandler()
{
  bRefreshScreen = true;
  
  screenRefreshTimer.attachInterrupt(&timerHandler, 1000000); 
}

// forward received from Serial to BLESerial and vice versa
void forward() {
  if (BLESerial && Serial) {
    int byte;
    while ((byte = BLESerial.read()) > 0) Serial.write((char)byte);
    while ((byte = Serial.read()) > 0) BLESerial.write((char)byte);
  }
}

// echo all received data back
void loopback() {
  if (BLESerial) {
    char buffer[30];
    int k = 0;
    int byte;
    while ((byte = BLESerial.read()) > 0) 
    {
      BLESerial.write(byte);
      buffer[k] = byte;
      k++;
    }

      if(strcmp(buffer, "up") == 0)
      {
        refreshScreen(1);
      }
      if(strcmp(buffer, "down") == 0)
      {
        refreshScreen(2);
      }
      if(strstr(buffer, "synchrs") != NULL)
      {
        int syncHours = atoi(buffer+7);
        if(syncHours <= 23 and syncHours >= 0)
        {
          rtc.setHour(syncHours,0);
        }
      }
      
      if(strstr(buffer, "syncmin") != NULL)
      {
        int syncMinutes = atoi(buffer+7);
        if(syncMinutes <= 59 and syncMinutes >= 0)
        {
          rtc.setMinute(syncMinutes);
        }
      }

      if(strstr(buffer, "syncsec") != NULL)
      {
        int syncSeconds = atoi(buffer+7);
        if(syncSeconds <= 59 and syncSeconds >= 0)
        {
          rtc.setSecond(syncSeconds);
        }
      }
      
      if(strstr(buffer, "syncday") != NULL)
      {
        int syncDay = atoi(buffer+7);
        if(syncDay <= 31 and syncDay >= 0)
        {
          rtc.setDay(syncDay);
        }
      }

      if(strstr(buffer, "syncmon") != NULL)
      {
        int syncMonth = atoi(buffer+7);
        if(syncMonth <= 12 and syncMonth >= 0)
        {
          rtc.setMonth(syncMonth);
        }
      }
  }
}

// periodically sent time stamps
void spam() {
  if (BLESerial) {
    BLESerial.print(millis());
    BLESerial.println(" tick-tacks!");
    delay(1000);
  }
}

void testdrawline() {
  for (int i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, BLACK);
    display.refresh();
  }
  for (int i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, BLACK);
    display.refresh();
  }
  delay(250);

  display.clearDisplay();
  for (int i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, BLACK);
    display.refresh();
  }
  for (int i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, BLACK);
    display.refresh();
  }
  delay(250);

  display.clearDisplay();
  for (int i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, BLACK);
    display.refresh();
  }
  for (int i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, BLACK);
    display.refresh();
  }
  delay(250);

  display.clearDisplay();
  for (int i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, BLACK);
    display.refresh();
  }
  for (int i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, BLACK);
    display.refresh();
  }
  delay(250);
}

void testdrawrect(void) {
  for (int i=0; i<minorHalfSize; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, BLACK);
    display.refresh();
  }
}

void testfillrect(void) {
  uint8_t color = BLACK;
  for (int i=0; i<minorHalfSize; i+=3) {
    // alternate colors
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, color&1);
    display.refresh();
    color++;
  }
}

void testdrawroundrect(void) {
  for (int i=0; i<minorHalfSize/2; i+=2) {
    display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i, minorHalfSize/2, BLACK);
    display.refresh();
  }
}

void testfillroundrect(void) {
  uint8_t color = BLACK;
  for (int i=0; i<minorHalfSize/2; i+=2) {
    display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i, minorHalfSize/2, color&1);
    display.refresh();
    color++;
  }
}

void testdrawtriangle(void) {
  for (int i=0; i<minorHalfSize; i+=5) {
    display.drawTriangle(display.width()/2, display.height()/2-i,
                     display.width()/2-i, display.height()/2+i,
                     display.width()/2+i, display.height()/2+i, BLACK);
    display.refresh();
  }
}

void testfilltriangle(void) {
  uint8_t color = BLACK;
  for (int i=minorHalfSize; i>0; i-=5) {
    display.fillTriangle(display.width()/2  , display.height()/2-i,
                         display.width()/2-i, display.height()/2+i,
                         display.width()/2+i, display.height()/2+i, color & 1);
    display.refresh();
    color++;
  }
}

void testdrawchar(void) {
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0,0);
  display.cp437(true);

  for (int i=0; i < 256; i++) {
    if (i == '\n') continue;
    display.write(i);
  }
  display.refresh();
}
