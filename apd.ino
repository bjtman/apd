// APD version 1.1.01
// 7/29/14
// Brian Tice

#include <Wire.h>
#include <SPI.h>
#include "BlinkM_funcs.h"

#include <avr/pgmspace.h>  // for progmem stuff
#include <RTClib.h>
#include <RTC_DS3231.h>
#include <TSL2561.h>
#include "LiquidCrystal.h"

//**TODO: evauluate if I need these globals -bjt
const boolean BLINKM_ARDUINO_POWERED = true;
byte blinkm_addr = 0x09; // the default address of all BlinkMs

// The address will be different depending on whether you let
// the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases
// use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively
TSL2561 tsl(TSL2561_ADDR_FLOAT); 
LiquidCrystal lcd(0);
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
  
  Serial.begin(19200);
  if( BLINKM_ARDUINO_POWERED )
    BlinkM_beginWithPower();
  else
    BlinkM_begin();

  delay(100); // wait a bit for things to stabilize
  BlinkM_off(0);  // turn everyone off

  //BlinkM_setAddress( blinkm_addr );  // uncomment to set address
  
  // Initialize lux sensor
  if (tsl.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No sensor?");
    while (1);
  }
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2561_GAIN_0X);         // set no gain (for bright situtations)
  tsl.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // medium integration time (medium light)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_402MS);  // longest integration time (dim light)
  
  RTC.begin();
  DateTime now = RTC.now();
  DateTime compiled = DateTime(__DATE__, __TIME__);
  if (now.unixtime() < compiled.unixtime()) {
    //Serial.println("RTC is older than compile time!  Updating");
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }  
 
  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Anti Predator Device");
  
  
  lookForBlinkM();


}

void loop() {
 
   lcd.setCursor(0, 1);
  BlinkM_fadeToRandomRGB( blinkm_addr, '100','100','100');
  DateTime now = RTC.now();
  
   uint16_t x = tsl.getLuminosity(TSL2561_VISIBLE);     
   int luminosity = x;
    //uint16_t x = tsl.getLuminosity(TSL2561_FULLSPECTRUM);
    //uint16_t x = tsl.getLuminosity(TSL2561_INFRARED);
   
    lcd.print(now.year(), DEC);
    lcd.print('/');
    lcd.print(now.month(), DEC);
    lcd.print('/');
    lcd.print(now.day(), DEC);
    lcd.print(' ');
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    lcd.print(now.minute(), DEC);
    lcd.print(':');
    lcd.print(now.second(), DEC);
    lcd.setCursor(21,3);
   
    //lcd.print('Luminosity: ');
    lcd.print(luminosity);
    delay(100);
  
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

