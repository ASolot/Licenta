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

// I2C
#include <Wire.h>
#include <SPI.h>

//// Accelerometer - adafruit lib
//#include <Adafruit_LSM9DS0.h>
//#include <Adafruit_Sensor.h>
//
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(29);

// Accelerometer - SparkFun
#include <SFE_LSM9DS0.h>

///////////////////////
// Example I2C Setup //
///////////////////////
// Comment out this section if you're using SPI
// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
// Create an instance of the LSM9DS0 library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

#define PRINT_CALCULATED
#define PRINT_SPEED 500 // 500 ms between prints

#define PIN_BUTTON1 23
#define PIN_BUTTON2 22

const unsigned long LONG_DELTA = 500ul;               // hold seconds for a long press
const unsigned long DEBOUNCE_DELTA = 30ul;

#define STATE_NORMAL 0
#define STATE_SHORT 1
#define STATE_LONG 2

volatile int  resultButton1 = 0;
volatile int  resultButton2 = 0;

int err = 0;

//TwoWire bus = TwoWire(NRF_TWIM0, NRF_TWIS0, (IRQn_Type)3, 9, 10);
//TwoWire Wire = TwoWire(9,10);

// Menus
#include "menu.h"

// display settings 
bool displayDate = true;
bool displayHourBig = true;
bool displayAirQuality = true;
bool displaySteps = true;
bool displayNotify = true;
bool displayMenu = false;

#define DEVICE_STATE_CLOCK    (0)
#define DEVICE_STATE_MENU     (1)

Menu menu;

int currentDeviceState = DEVICE_STATE_CLOCK;
int lastDeviceState = DEVICE_STATE_CLOCK;

void setupI2C(void)
{
  //Wire.TwoWire(NRF_TWIM0, NRF_TWIS0, (IRQn_Type)3, PIN_WIRE_SDA, PIN_WIRE_SCL);
  //Wire.begin();
}

void setupButtons(void)
{
  pinMode(PIN_BUTTON1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON1), checkButton1, CHANGE);

  pinMode(PIN_BUTTON2, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON2), checkButton2, CHANGE);
}

void setupAccel(void)
{
  uint16_t status = dof.begin();
}

void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  dof.readAccel();
  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(dof.calcAccel(dof.ax), 2);
  Serial.print(", ");
  Serial.print(dof.calcAccel(dof.ay), 2);
  Serial.print(", ");
  Serial.println(dof.calcAccel(dof.az), 2);
#elif defined PRINT_RAW 
  Serial.print(dof.ax);
  Serial.print(", ");
  Serial.print(dof.ay);
  Serial.print(", ");
  Serial.println(dof.az);
#endif

}

void setupBLE(void)
{
  // custom services and characteristics can be added as well
  BLESerial.setLocalName("AS_Watch");

  Serial.begin(115200);
  BLESerial.begin();
}

void setupAccelerometer(void)
{
//  if(!lsm.begin())
//  {
//    /* There was a problem detecting the LSM9DS0 ... check your connections */
//    //Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
//    err = 1;
//    return;
//  }
  //Serial.println(F("Found LSM9DS0 9DOF"));
  
  
//  /* Setup the sensor gain and integration time */
//  // 1.) Set the accelerometer range
//  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
//  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
//  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
//  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
//  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
//  
//  // 2.) Set the magnetometer sensitivity
//  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
//  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
//  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
//  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);
//
//  // 3.) Setup the gyroscope
//  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
//  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
//  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
//
//  lsm.read();
//  Serial.print("Accel X: "); Serial.print((int)lsm.accelData.x); Serial.print(" ");
//  Serial.print("Y: "); Serial.print((int)lsm.accelData.y);       Serial.print(" ");
//  Serial.print("Z: "); Serial.println((int)lsm.accelData.z);     Serial.print(" ");
//  Serial.print("Mag X: "); Serial.print((int)lsm.magData.x);     Serial.print(" ");
//  Serial.print("Y: "); Serial.print((int)lsm.magData.y);         Serial.print(" ");
//  Serial.print("Z: "); Serial.println((int)lsm.magData.z);       Serial.print(" ");
//  Serial.print("Gyro X: "); Serial.print((int)lsm.gyroData.x);   Serial.print(" ");
//  Serial.print("Y: "); Serial.print((int)lsm.gyroData.y);        Serial.print(" ");
//  Serial.print("Z: "); Serial.println((int)lsm.gyroData.z);      Serial.println(" ");
//  Serial.print("Temp: "); Serial.print((int)lsm.temperature); 
}

void setupPollutionSensor(void)
{

}

void setupVibe(void)
{
 
}

void setupRTC(void)
{
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

  screenRefreshTimer.attachInterrupt(&timerHandler, 1000000);
}

void setupMenu(void)
{
  static MenuItem item1(String("Pedometer"), -1);
  menu.addMenuItem(&item1);
  static MenuItem item2(String("Air Quality"), -1);
  menu.addMenuItem(&item2);
  static MenuItem item3(String("Date"), -1);
  menu.addMenuItem(&item3);
  static MenuItem item4(String("Notify"), -1);
  menu.addMenuItem(&item4);
  static MenuItem item5(String("Set clock"), 0);
  menu.addMenuItem(&item5);
}

void setupDisplay(void)
{
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
}

void setup() 
{

  setupBLE();
  //setupI2C();
  setupRTC();
  setupButtons();
  
  setupDisplay();
  setupMenu();
  
  //setupAccelerometer();
  //setupAccel();
  //setupPollutionSensor();

  

}

void refreshScreen(int orientation);
void checkState();

void loop() 
{
  
  BLESerial.poll();

  //forward();
  loopback();
  //spam();

  if(bRefreshScreen)
  {
    bRefreshScreen = false;
    refreshScreen(1);
  }

  checkState();
  
}

void drawMenu()
{
  display.clearDisplay();

  display.setTextColor(BLACK);
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("MENU");
  display.setTextSize(1);
  display.println(" ");

  int selected = menu.getSelectedItemIndex();
  
  for(int i = 0; i < menu.getNumberOfItems(); i++)
  {
    if(i == selected)
    {
      display.print(">");
    }
    else
    {
      display.print(" ");
    }

    display.print(menu.m_menuItemsList[i]->getDescription());
    display.print(" ");
    display.println(menu.m_menuItemsList[i]->getValueDescription());
    
  }
}

void refreshScreen(int orientation = 1) 
{

    display.setRotation(orientation);
    display.clearDisplay();
    
    // text display test

    display.setTextColor(BLACK);
    if(displayMenu)
    {
      drawMenu();
    }
    else
    {

      if(displayDate)
      {
        display.setTextSize(1);
        display.setCursor(0,0);

        display.print(rtc.date.day);      // day
        display.print('/');
        display.print(rtc.date.month);    // month
        display.print('/');
        display.print(rtc.date.year+2000); // year
      }
      
      if(displayHourBig)
      {
        display.setTextSize(2);
        display.setCursor(0,35);
  
        rtc.getDate();
        rtc.getTime();
        sprintf(hourText,"%02d:%02d:%02d\n", rtc.time.hour, rtc.time.minute, rtc.time.second);
        
        display.println(hourText);
      }
  
      if(displayAirQuality)
      {
        display.setTextSize(1);
        display.setCursor(25, 60);
        display.println("IAQ: 50");
      }
      
      if(displaySteps)
      {
        display.setTextSize(1);
        display.setCursor(10, 75);
        display.println("2346 steps");
      }
    }


    // Screen must be refreshed at least once per second
    display.refresh();
}

void updateSettings(void)
{

  displaySteps = (menu.m_menuItemsList[0]->getValue() == -1);
  displayAirQuality = (menu.m_menuItemsList[1]->getValue() == -1);
  displayDate = (menu.m_menuItemsList[2]->getValue() == -1);
  displayNotify = (menu.m_menuItemsList[3]->getValue() == -1);

}

void checkState(void)
{
  
      switch(currentDeviceState)
      {
        case DEVICE_STATE_CLOCK:

          if(resultButton1 == STATE_LONG)
          {
            resultButton1 = STATE_NORMAL;
            lastDeviceState = currentDeviceState;
            currentDeviceState = DEVICE_STATE_MENU;
            displayMenu = displayMenu ^ true;
          }

          break;

          
        case DEVICE_STATE_MENU:
          
          if(resultButton1 == STATE_LONG)
          {
            resultButton1 = STATE_NORMAL;
            lastDeviceState = currentDeviceState;
            currentDeviceState = DEVICE_STATE_CLOCK;
            displayMenu = displayMenu ^ true;

            updateSettings();
          }

          if(resultButton1 == STATE_SHORT)
          {
            resultButton1 = STATE_NORMAL;
            menu.decrementSelectedItem();
          }

          if(resultButton2 == STATE_SHORT)
          {
            resultButton2 = STATE_NORMAL;
            menu.incrementSelectedItem();
          }

          if(resultButton2 == STATE_LONG)
          {
            resultButton2 = STATE_NORMAL;
            menu.action();
          }

          
          break;
      }
  

}

void drawError(char* errorMsg)
{
  display.setRotation(1);

  display.clearDisplay();
    // text display tests
  display.setTextSize(2);
  display.setTextColor(BLACK);
  display.setCursor(0,0);

  display.println(errorMsg);
  
}

void timerHandler(void)
{
  bRefreshScreen = true;
  
  // refresh screen faster in menu
  if(currentDeviceState == DEVICE_STATE_CLOCK)
    screenRefreshTimer.attachInterrupt(&timerHandler, 1000000);
  else
    screenRefreshTimer.attachInterrupt(&timerHandler, 500000);
}

void checkButton1(void)
{
  
  static int lastButton1Status = HIGH;                                   // HIGH indicates the button is NOT pressed
  int button1Status = digitalRead(PIN_BUTTON1);
  boolean Transition = false;
  boolean Released = true;
  boolean timeoutShort = false;
  boolean timeoutLong = false;

  static unsigned long longTime1 = 0ul;
  static unsigned long shortTime1 = 0ul;

  timeoutShort = (millis() > shortTime1);
  timeoutLong = (millis() > longTime1);

  if (button1Status != lastButton1Status) 
  {                         
    shortTime1 = millis() + DEBOUNCE_DELTA;
    longTime1 = millis() + LONG_DELTA;
  }

  Transition = (button1Status != lastButton1Status);        // has the button changed state
  Released = (Transition && (button1Status == HIGH));       // for input pullup circuit

  lastButton1Status = button1Status;

  if ( ! Transition) 
  {                                                                //without a transition, there's no change in input
       // if there has not been a transition, don't change the previous result
       resultButton1 =  STATE_NORMAL | resultButton1;
       return;
  }

  if (timeoutLong && Released) {                                      // long timeout has occurred and the button was just released
       resultButton1 = STATE_LONG | resultButton1;       // ensure the button result reflects a long press
  } else if (timeoutShort && Released) {                          // short timeout has occurred (and not long timeout) and button was just released
      resultButton1 = STATE_SHORT | resultButton1;     // ensure the button result reflects a short press
  } else {                                                                                  // else there is no change in status, return the normal state
      resultButton1 = STATE_NORMAL | resultButton1; // with no change in status, ensure no change in button status
  }
}

void checkButton2(void)
{
  static int lastButton2Status = HIGH;                                   // HIGH indicates the button is NOT pressed
  int button2Status = digitalRead(PIN_BUTTON2);
  boolean Transition = false;
  boolean Released = true;
  boolean timeoutShort = false;
  boolean timeoutLong = false;

  static unsigned long longTime2 = 0ul;
  static unsigned long shortTime2 = 0ul;

  timeoutShort = (millis() > shortTime2);
  timeoutLong = (millis() > longTime2);

  if (button2Status != lastButton2Status) 
  {                         
    shortTime2 = millis() + DEBOUNCE_DELTA;
    longTime2 = millis() + LONG_DELTA;
  }

  Transition = (button2Status != lastButton2Status);        // has the button changed state
  Released = (Transition && (button2Status == HIGH));       // for input pullup circuit

  lastButton2Status = button2Status;

  if ( ! Transition) 
  {                                                                //without a transition, there's no change in input
       // if there has not been a transition, don't change the previous result
       resultButton2 =  STATE_NORMAL | resultButton2;
       return;
  }

  if (timeoutLong && Released) {                                      // long timeout has occurred and the button was just released
       resultButton2 = STATE_LONG | resultButton2;       // ensure the button result reflects a long press
  } else if (timeoutShort && Released) {                          // short timeout has occurred (and not long timeout) and button was just released
      resultButton2 = STATE_SHORT | resultButton2;     // ensure the button result reflects a short press
  } else {                                                                                  // else there is no change in status, return the normal state
      resultButton2 = STATE_NORMAL | resultButton2; // with no change in status, ensure no change in button status
  }
}

// forward received from Serial to BLESerial and vice versa
void forward(void) 
{
  if (BLESerial && Serial) 
  {
    int byte;
    while ((byte = BLESerial.read()) > 0) Serial.write((char)byte);
    while ((byte = Serial.read()) > 0) BLESerial.write((char)byte);
  }
}

// echo all received data back
void loopback(void) 
{
  if (BLESerial) 
  {
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
void spam(void) 
{
  if (BLESerial) 
  {
    BLESerial.print(millis());
    BLESerial.println(" tick-tacks!");
    delay(1000);
  }
}

void testdrawline(void) 
{
  for (int i=0; i<display.width(); i+=4) 
  {
    display.drawLine(0, 0, i, display.height()-1, BLACK);
    display.refresh();
  }
  for (int i=0; i<display.height(); i+=4) 
  {
    display.drawLine(0, 0, display.width()-1, i, BLACK);
    display.refresh();
  }
  delay(250);

  display.clearDisplay();
  for (int i=0; i<display.width(); i+=4) 
  {
    display.drawLine(0, display.height()-1, i, 0, BLACK);
    display.refresh();
  }
  for (int i=display.height()-1; i>=0; i-=4) 
  {
    display.drawLine(0, display.height()-1, display.width()-1, i, BLACK);
    display.refresh();
  }
  delay(250);

  display.clearDisplay();
  for (int i=display.width()-1; i>=0; i-=4) 
  {
    display.drawLine(display.width()-1, display.height()-1, i, 0, BLACK);
    display.refresh();
  }
  for (int i=display.height()-1; i>=0; i-=4) 
  {
    display.drawLine(display.width()-1, display.height()-1, 0, i, BLACK);
    display.refresh();
  }
  delay(250);

  display.clearDisplay();
  for (int i=0; i<display.height(); i+=4) 
  {
    display.drawLine(display.width()-1, 0, 0, i, BLACK);
    display.refresh();
  }
  for (int i=0; i<display.width(); i+=4) 
  {
    display.drawLine(display.width()-1, 0, i, display.height()-1, BLACK);
    display.refresh();
  }
  delay(250);
}

void testdrawrect(void) 
{
  for (int i=0; i<minorHalfSize; i+=2) 
  {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, BLACK);
    display.refresh();
  }
}

void testfillrect(void) 
{
  uint8_t color = BLACK;
  for (int i=0; i<minorHalfSize; i+=3) 
  {
    // alternate colors
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, color&1);
    display.refresh();
    color++;
  }
}

void testdrawroundrect(void) 
{
  for (int i=0; i<minorHalfSize/2; i+=2) 
  {
    display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i, minorHalfSize/2, BLACK);
    display.refresh();
  }
}

void testfillroundrect(void) 
{
  uint8_t color = BLACK;
  for (int i=0; i<minorHalfSize/2; i+=2) 
  {
    display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i, minorHalfSize/2, color&1);
    display.refresh();
    color++;
  }
}

void testdrawtriangle(void) 
{
  for (int i=0; i<minorHalfSize; i+=5) 
  {
    display.drawTriangle(display.width()/2, display.height()/2-i,
                     display.width()/2-i, display.height()/2+i,
                     display.width()/2+i, display.height()/2+i, BLACK);
    display.refresh();
  }
}

void testfilltriangle(void) 
{
  uint8_t color = BLACK;
  for (int i=minorHalfSize; i>0; i-=5)
  {
    display.fillTriangle(display.width()/2  , display.height()/2-i,
                         display.width()/2-i, display.height()/2+i,
                         display.width()/2+i, display.height()/2+i, color & 1);
    display.refresh();
    color++;
  }
}

void testdrawchar(void) 
{
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0,0);
  display.cp437(true);

  for (int i=0; i < 256; i++) 
  {
    if (i == '\n') continue;
    display.write(i);
  }
  display.refresh();
}
