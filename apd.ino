// APD version 1.1
// 7/28/14
// Brian Tice

#include "Wire.h"
#include "BlinkM_funcs.h"
#include <avr/pgmspace.h>  // for progmem stuff
#include <SPI.h>
#include <RTClib.h>
#include <RTC_DS3231.h>


//**TODO: evauluate if I need these globals -bjt
const boolean BLINKM_ARDUINO_POWERED = true;
byte blinkm_addr = 0x09; // the default address of all BlinkMs


RTC_DS3231 RTC;
//#define SQW_FREQ DS3231_SQW_FREQ_1024     //0b00001000   1024Hz
//#define PWM_COUNT 1020   //determines how often the LED flips
//#define LOOP_DELAY 5000 //ms delay time in loop

//#define RTC_SQW_IN 5     // input square wave from RTC into T1 pin (D5)
                               //WE USE TIMER1 so that it does not interfere with Arduino delay() command
//#define INT0_PIN   2     // INT0 pin for 32kHz testing?
//#define LED_PIN    9     // random LED for testing...tie to ground through series resistor..
//#define LED_ONBAORD 13   // Instead of hooking up an LED, the nano has an LED at pin 13.

//volatile long TOGGLE_COUNT = 0;

void setup() {
  
  if( BLINKM_ARDUINO_POWERED )
    BlinkM_beginWithPower();
  else
    BlinkM_begin();

  delay(100); // wait a bit for things to stabilize
  BlinkM_off(0);  // turn everyone off

  //BlinkM_setAddress( blinkm_addr );  // uncomment to set address
  
  RTC.begin();
  DateTime now = RTC.now();
  DateTime compiled = DateTime(__DATE__, __TIME__);
  if (now.unixtime() < compiled.unixtime()) {
    //Serial.println("RTC is older than compile time!  Updating");
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }  
  Serial.begin(19200);

  
  
  lookForBlinkM();


}

void loop() {
 
  BlinkM_fadeToRandomRGB( blinkm_addr, '100','100','100');
  DateTime now = RTC.now();
  
   
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
  
    delay(1000);
  
}

void lookForBlinkM() {
  Serial.print("Looking for a BlinkM: ");
  int a = BlinkM_findFirstI2CDevice();
  if( a == -1 ) {
    Serial.println("No I2C devices found");
  } else { 
    Serial.print("Device found at addr ");
    Serial.println( a, DEC);
    blinkm_addr = a;
  }
}

