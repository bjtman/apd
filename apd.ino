// APD version 1.1.01
// 7/29/14
// Brian Tice

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
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

const int chipSelect = 53;

File dataFile;


void setup() {
  
  Serial.begin(19200);
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(53, OUTPUT);     // change this to 53 on a mega
  
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
  lcd.begin(20, 4);
  // Print a message to the LCD.
  lcd.print("Anti Predator Device");
  
  
  lookForBlinkM();

   Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SS, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) ;
  }
  Serial.println("card initialized.");
  
  // Open up the file we're going to log to!
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (! dataFile) {
    Serial.println("error opening datalog.txt");
    // Wait forever since we cant write data
    while (1) ;
  }
   
}

void loop() {
  
 
   lcd.setCursor(0, 1);
  BlinkM_fadeToRandomRGB( blinkm_addr, '100','100','100');
  DateTime now = RTC.now();
  
   uint16_t x = tsl.getLuminosity(TSL2561_VISIBLE);     
 
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
    
    dataFile.print(now.year(), DEC);
    dataFile.print('/');
    dataFile.print(now.month(), DEC);
    dataFile.print('/');
    dataFile.print(now.day(), DEC);
    dataFile.print(' ');
    dataFile.print(now.hour(), DEC);
    dataFile.print(':');
    dataFile.print(now.minute(), DEC);
    dataFile.print(':');
    dataFile.print(now.second(), DEC);
    
    
    lcd.setCursor(0,3);
    lcd.print("                   ");
    lcd.setCursor(0,3);
    lcd.print("Luminosity: ");
    lcd.print(x,DEC);
    Serial.println(x,DEC);
    dataFile.println(x,DEC);
   
    // The following line will 'save' the file to the SD card after every
    // line of data - this will use more power and slow down how much data
    // you can read but it's safer! 
    // If you want to speed up the system, remove the call to flush() and it
    // will save the file only every 512 bytes - every time a sector on the 
    // SD card is filled with data.
    dataFile.flush();
    delay(500);
  
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

