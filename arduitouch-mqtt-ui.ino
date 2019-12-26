/*
 *  Application note: ArduiTouch-MQTT-UI
 *  Version 0.1
 *  Copyright (C) 2019  
 *  DC1RD Rainer Dobler
 *  McBoeck
 *  
 *  Icons from: Sieren Homepoint https://github.com/sieren/Homepoint and https://fontawesome.com/icons?d=gallery
 *  Functions from: Hartmut Wendt  www.zihatec.de
 *  
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/   

/*______Import Libraries_______*/
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <XPT2046_Touchscreen.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <DHTesp.h>
#include "Ticker.h"
#include "usergraphics.h"
#include "gardenhouse.h"
#include "gardenhouse_active.h"
#include "terrace.h"
#include "terrace_active.h"

/*______End of Libraries_______*/


/*__Pin definitions for the ESP8266__*/
#define TFT_CS   5
#define TFT_DC   4
#define TFT_LED  15  
#define TFT_MOSI 23
#define TFT_CLK  18
#define TFT_RST  22
#define TFT_MISO 19
#define TFT_LED  15  

#define DHTPIN 27     // Digital pin connected to the DHT sensor
#define DHTTYPE    DHT22     // DHT 22 (AM2302)

#define BEEPER 21

#define HAVE_TOUCHPAD
#define TOUCH_CS 14
#define TOUCH_IRQ 2

/*_______End of definitions______*/

 

/*____Calibrate Touchscreen_____*/
#define MINPRESSURE 10      // minimum required force for touch event
#define TS_MINX 370
#define TS_MINY 470
#define TS_MAXX 3700
#define TS_MAXY 3600
/*______End of Calibration______*/


/*___Keylock spezific definitions___*/
#define codenum 42    // 42 is the answer for everything, but you can change this to any number between 0 and 999999

/*___End of Keylock spezific definitions___*/


Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);

DHTesp dht;
void tempTask(void *pvParameters);
bool getTemperature();
void triggerGetTemp();
/** Task handle for the light value read task */
TaskHandle_t tempTaskHandle = NULL;
/** Ticker for temperature reading */
Ticker tempTicker;
/** Comfort profile */
ComfortState cf;
/** Flag if task should run */
bool tasksEnabled = false;

bool initTemp() {
  byte resultValue = 0;
  // Initialize temperature sensor
  dht.setup(DHTPIN, DHTesp::DHTTYPE);
  Serial.println("DHT initiated");

  // Start task to get temperature
  xTaskCreatePinnedToCore(
      tempTask,                       /* Function to implement the task */
      "tempTask ",                    /* Name of the task */
      4000,                           /* Stack size in words */
      NULL,                           /* Task input parameter */
      5,                              /* Priority of the task */
      &tempTaskHandle,                /* Task handle. */
      1);                             /* Core where the task should run */

  if (tempTaskHandle == NULL) {
    Serial.println("Failed to start task for temperature update");
    return false;
  } else {
    // Start update of environment data every 20 seconds
    tempTicker.attach(20, triggerGetTemp);
  }
  return true;
}

/**
 * triggerGetTemp
 * Sets flag dhtUpdated to true for handling in loop()
 * called by Ticker getTempTimer
 */
void triggerGetTemp() {
  if (tempTaskHandle != NULL) {
     xTaskResumeFromISR(tempTaskHandle);
  }
}

/**
 * Task to reads temperature from DHT11 sensor
 * @param pvParameters
 *    pointer to task parameters
 */
void tempTask(void *pvParameters) {
  Serial.println("tempTask loop started");
  while (1) // tempTask loop
  {
    if (tasksEnabled) {
      // Get temperature values
      getTemperature();
    }
    // Got sleep again
    vTaskSuspend(NULL);
  }
}

/**
 * getTemperature
 * Reads temperature from DHT sensor
 * @return bool
 *    true if temperature could be aquired
 *    false if aquisition failed
*/
bool getTemperature() {
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dht.getTempAndHumidity();
  // Check if any reads failed and exit early (to try again).
  if (dht.getStatus() != 0) {
    Serial.println("DHT error status: " + String(dht.getStatusString()));
    return false;
  }

  float heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
  float dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);
  float cr = dht.getComfortRatio(cf, newValues.temperature, newValues.humidity);

  String comfortStatus;
  switch(cf) {
    case Comfort_OK:
      comfortStatus = "Comfort_OK";
      break;
    case Comfort_TooHot:
      comfortStatus = "Comfort_TooHot";
      break;
    case Comfort_TooCold:
      comfortStatus = "Comfort_TooCold";
      break;
    case Comfort_TooDry:
      comfortStatus = "Comfort_TooDry";
      break;
    case Comfort_TooHumid:
      comfortStatus = "Comfort_TooHumid";
      break;
    case Comfort_HotAndHumid:
      comfortStatus = "Comfort_HotAndHumid";
      break;
    case Comfort_HotAndDry:
      comfortStatus = "Comfort_HotAndDry";
      break;
    case Comfort_ColdAndHumid:
      comfortStatus = "Comfort_ColdAndHumid";
      break;
    case Comfort_ColdAndDry:
      comfortStatus = "Comfort_ColdAndDry";
      break;
    default:
      comfortStatus = "Unknown:";
      break;
  };

  Serial.println(" T:" + String(newValues.temperature) + " H:" + String(newValues.humidity) + " I:" + String(heatIndex) + " D:" + String(dewPoint) + " " + comfortStatus);
  return true;
}


int X,Y;
char action;
boolean result = false;
bool Touch_pressed = false;
unsigned long delayTime;
TS_Point p;


void setup() {
  Serial.begin(115200); //Use serial monitor for debugging

  Serial.println("DHT ESP32 example with tasks");
  initTemp();
  // Signal end of setup() to tasks
  tasksEnabled = true;


  pinMode(TFT_LED, OUTPUT); // define as output for backlight control

  Serial.println("Init TFT and Touch...");
  tft.begin();
  tft.setRotation(3);
  touch.begin();
  Serial.print("tftx ="); Serial.print(tft.width()); Serial.print(" tfty ="); Serial.println(tft.height());
  tft.fillScreen(ILI9341_BLACK);
  
  IntroScreen();
  digitalWrite(TFT_LED, LOW);    // LOW to turn backlight on; 

  delay(1500);

  digitalWrite(TFT_LED, HIGH);    // HIGH to turn backlight off - will hide the display during drawing
  draw_BoxNButtons(); 
  digitalWrite(TFT_LED, LOW);    // LOW to turn backlight on; 

  //sound configuration
  ledcSetup(0,1E5,12);
  ledcAttachPin(BEEPER,0);

    Serial.println();
}

void loop() {
  // check touch screen for new events
  if (Touch_Event()== true) { 
    X = p.y; Y = p.x;
    Touch_pressed = true;
    
  } else {
    Touch_pressed = false;
  }

  // if touch is pressed detect pressed buttons
  if (Touch_pressed == true) {
    
    DetectButtons();
  

   // hier Auswertung einfügen

   
  }    
  delay(100);
  ledcWriteTone(0,0);

  if (!tasksEnabled) {
    // Wait 2 seconds to let system settle down
    delay(2000);
    // Enable task that will read values from the DHT sensor
    tasksEnabled = true;
    if (tempTaskHandle != NULL) {
      vTaskResume(tempTaskHandle);
    }
  }
  yield();

//END LOOP
}




/********************************************************************//**
 * @brief     detects a touch event and converts touch data 
 * @param[in] None
 * @return    boolean (true = touch pressed, false = touch unpressed) 
 *********************************************************************/
bool Touch_Event() {
  p = touch.getPoint(); 
  delay(1);
  #ifdef touch_yellow_header
    p.x = map(p.x, TS_MINX, TS_MAXX, 320, 0); // yellow header
  #else
    p.x = map(p.x, TS_MINX, TS_MAXX, 0, 320); // black header
  #endif
  p.y = map(p.y, TS_MINY, TS_MAXY, 0, 240);
  if (p.z > MINPRESSURE) return true;  
  return false;  
}



/********************************************************************//**
 * @brief     detecting pressed buttons with the given touchscreen values
 * @param[in] None
 * @return    None
 *********************************************************************/
void DetectButtons()
{
  if (X>120) //Detecting Buttons on Column 1
  {
    if (Y>213) //If Button 1 is pressed
    {Serial.println ("Button 1");
    Button_ACK_Tone();
    }
    
     if (Y>106 && Y<213) //If Button 2 is pressed
    {Serial.println ("Button 2"); 
    Button_ACK_Tone();
    }
    
     if (Y>0 && Y<106) //If Button 3 is pressed
    {Serial.println ("Button 3"); 
    Button_ACK_Tone();
    }
    
  }

  if (X>0 && X<120) //Detecting Buttons on Column 2
  {
    if (Y>213) //If Button 4 is pressed
    {Serial.println ("Button 4");
    Button_ACK_Tone();
    }
    
     if (Y>106 && Y<213) //If Button 5 is pressed
    {Serial.println ("Button 5"); 
    Button_ACK_Tone();
    }
    
     if (Y>0 && Y<106) //If Button 6 is pressed
    {Serial.println ("Button 6"); 
    Button_ACK_Tone();
    }  
  }

}


/********************************************************************//**
 * @brief     shows the intro screen in setup procedure
 * @param[in] None
 * @return    None
 *********************************************************************/
void IntroScreen()
{
  //Draw the Result Box
  tft.fillRect(0, 0, 240, 320, ILI9341_BLACK);

  
  
  tft.setTextSize(0);
  tft.setTextColor(ILI9341_WHITE);
  tft.setFont(&FreeSansBold9pt7b);  

  tft.setCursor(70, 120);
  tft.println("ArduiTouch MQTT UI");
  delay(1000); 

}


/********************************************************************//**
 * @brief     draws the keypad
 * @param[in] None
 * @return    None
 *********************************************************************/
void draw_BoxNButtons()
{
  
   //clear screen black
  tft.fillRect(0, 0, 320, 240, ILI9341_BLACK);
  tft.setFont(0);  

  
  //Draw Horizontal Lines
  
  tft.drawFastHLine(0, 120, 320, ILI9341_CYAN);

  //Draw Vertical Lines
  tft.drawFastVLine(106, 0, 240, ILI9341_CYAN);
  tft.drawFastVLine(213, 0, 240, ILI9341_CYAN);


  tft.drawRGBBitmap(28,35, gardenhouse,50,50);
  tft.drawRGBBitmap(134,35, gardenhouse_active,50,50);
  tft.drawRGBBitmap(240,35, terrace,50,50);
  tft.drawRGBBitmap(28,155, terrace_active,50,50);
  tft.drawRGBBitmap(134,155, terrace_active,50,50);
  tft.drawRGBBitmap(240,155, terrace_active,50,50);

}


/********************************************************************//**
 * @brief     plays ack tone (beep) after button pressing
 * @param[in] None
 * @return    None
 *********************************************************************/
void Button_ACK_Tone(){
  ledcWriteTone(0,4000);
}
