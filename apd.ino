// APD version 1.1.08
// 8/2/14
// Brian Tice

#include <Arduino.h>
#include <Wire.h>            // Need this for I2C support
#include <SPI.h>             // Need this for SPI communication Support
#include <SD.h>              // Need this for microSD card read and write
                             // Note: SDFAT.h is supposedly faster and supports two  SD cards
                             //       Sticking with SD.h for now mostly because the Adafruit 
                             //       Library for VS1053 mp3 player is written using SD.h.

#include "BlinkM_funcs.h"    // Need this for BlinkM routines for RGB led clusters.
                             // This 'library' is a little different that the others in that its 
                             // functions are housed in a file associated with main sketch called BlinkM_funcs.h
                             
#include <avr/pgmspace.h>    // for progmem stuff
#include <RTClib.h>          // Need this for ChronoDot real time clock library functions 
#include <RTC_DS3231.h>      // Need this for the particular Chip used on the ChronoDot 2.0
#include <TSL2561.h>         // Need this for Luminosity sensor support
#include <Adafruit_VS1053.h> // Need this for music player library functions
#include "LiquidCrystal.h"   // Need this for I2C backback and LCD display
                             // Note this is latest version of this library from 
                             // adafruit that includes I2C support

                             // These are the pins used for the breakout example
#define BREAKOUT_RESET   9   // VS1053 reset pin (output)
#define BREAKOUT_CS     10   // VS1053 chip select pin (output)
#define BREAKOUT_DCS     8   // VS1053 Data/command select pin (output)
                             // These are the pins used for the music maker shield
#define SHIELD_CS        7   // VS1053 chip select pin (output)
#define SHIELD_DCS       6   // VS1053 Data/command select pin (output)


#define PIEZO_SOUNDER_PIN    22    // Create constant for Piezo Sounder HiLo alarm
#define MICRO_SD_CHIP_SELECT 53    // Chip select constant for MicroSD datalogger


// These are common pins between breakout and shield
#define CARDCS 4     // Card chip select pin
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

// Create Constants for Keypad items
#define BUTTON_PIN_1     30
#define BUTTON_PIN_2     31
#define BUTTON_PIN_3     32
#define BUTTON_PIN_4     33
#define BUTTON_PIN_5     34


#define PIR_A_LED_PIN 40
#define PIR_B_LED_PIN 41
#define PIR_A_SIGNAL_PIN 2
#define PIR_B_SIGNAL_PIN 18
#define PIR_CALIBRATION_TIME 30


// Class declarations for system

TSL2561 tsl(TSL2561_ADDR_FLOAT);    // Need this for Luminosity sensor. Default I2C address is: 0x09 = TSL2561_ADDR_FLOAT
                                    // This can be changed by soldering jumpers on the sensor
                                    // The address will be different depending on whether you let
                                    // the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases
                                    // use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively
                                    
LiquidCrystal lcd(0);               // Need this to initialize the I2C backpack / LCD combo             
                                    // Connect via i2c, default address #0 (A0-A2 not jumpered)
                                    // I2C address for LCD is 0x00 without soldering jumper headers
                                    // I did not jump A0-A2 which is why lcd is initialized as lcd(0)
                                    
RTC_DS3231 RTC;                     // Need this to create an instance of the RTC_DS3231 class for real time clock
                                    // located in RTC_DS.3231.h
                                    
                                    
Adafruit_VS1053_FilePlayer musicPlayer =     // Need this to create instance of music player class and File Player   

                                             // create breakout-example object!
  
  Adafruit_VS1053_FilePlayer(BREAKOUT_RESET, BREAKOUT_CS, BREAKOUT_DCS, DREQ, CARDCS);
                                             // create shield-example object!
                                             //Adafruit_VS1053_FilePlayer(SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);                                   
                                  
File dataFile;                      // Need this to declare a File instance for use in datalogging.


const boolean BLINKM_ARDUINO_POWERED = true;  // For now this is true. This will change when moving for bench
                                              // testing to field testing

byte blinkm_addr_a = 0x09;          // I2C Address of one of the LED's. LED A
byte blinkm_addr_b = 0x0C;          // I2C Address of one of the LED's. LED B 
byte blinkm_addr_c = 0x0D;          // I2C Address of one of the LED's. LED C





void setup() {
   
  initialize_real_time_clock();
  initialize_and_test_leds();
  initialize_pin_modes();  
  initialize_and_calibrate_PIR_sensor_array();
  initialize_lux_sensor();
  initialize_lcd_backpack_and_screen();
  initialize_datalogging_sd_card();
  initialize_vs1053_music_player();
  Serial.begin(19200);
  
  
  
 
  
  
  
  //lookForBlinkM();

   
  
  
}

void loop() {
  
  
  
  //PIR sensor's setup variables

/////////////////////////////
//VARS
//the time we give the sensor to calibrate (10-60 secs according to the datasheet)
int calibrationTime = 30;

//the time when the sensor outputs a low impulse
long unsigned int lowIn;         

//the amount of milliseconds the sensor has to be low 
//before we assume all motion has stopped
long unsigned int pause = 5000;  

boolean lockLow = true;
boolean takeLowTime;  

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

  // read the state of the pushbutton value:
  buttonState = digitalRead(BUTTON_PIN_1);   
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(PIR_A_LED_PIN, HIGH);
  }
  else {
    // turn LED off:
    digitalWrite(PIR_A_LED_PIN, LOW);
  }
  
  
   if (Serial.available()) {
    char c = Serial.read();
    
    // if we get an 's' on the serial console, stop!
    if (c == 's') {
      musicPlayer.stopPlaying();
      Serial.println("Done playing music");
      SD.end();
      delay(1000);
      SD.begin(MICRO_SD_CHIP_SELECT);
      dataFile = SD.open("datalog.txt", FILE_WRITE);
    }
    
    if (c == 'g') {
      
      // need to switch SD card's here. stop logging. starting playing mp3
      SD.end();
      delay(100);
     
      SD.begin(CARDCS);    // initialise the SD card
  
      Serial.println(F("Playing track 002"));
      musicPlayer.startPlayingFile("track002.mp3");
    }
    
    // if we get an 'p' on the serial console, pause/unpause!
    if (c == 'p' || (buttonState == HIGH)) {
      if (! musicPlayer.paused()) {
        Serial.println("Paused");
        musicPlayer.pausePlaying(true);
        
        // *** PIR READ ROUTINE
        // 
        if(digitalRead(PIR_A_SIGNAL_PIN) == HIGH){
          digitalWrite(PIR_A_SIGNAL_PIN, HIGH);   //the led visualizes the sensors output pin state
      
          if(lockLow){  
            //makes sure we wait for a transition to LOW before any further output is made:
            lockLow = false;            
            Serial.println("---");
            Serial.print("motion detected at ");
            Serial.print(millis()/1000);
            Serial.println(" sec"); 
            delay(50);
            // PIEZO SENSOR ACTUATION ROUTINE
            //
            digitalWrite(PIEZO_SOUNDER_PIN,HIGH);
            delay(500);
            digitalWrite(PIEZO_SOUNDER_PIN,LOW);
            //
            // DONE PIEZO
          }         
          takeLowTime = true;
        }

        if(digitalRead(PIR_A_SIGNAL_PIN) == LOW){       
          digitalWrite(PIR_A_LED_PIN, LOW);  //the led visualizes the sensors output pin state
      
          if(takeLowTime){
            lowIn = millis();          //save the time of the transition from high to LOW
            takeLowTime = false;       //make sure this is only done at the start of a LOW phase
          }
          //if the sensor is low for more than the given pause, 
          //we assume that no more motion is going to happen
          if(!lockLow && millis() - lowIn > pause){  
           //makes sure this block of code is only executed again after 
           //a new motion sequence has been detected
           lockLow = true;                        
           Serial.print("motion ended at ");      //output
           Serial.print((millis() - pause)/1000);
           Serial.println(" sec");
           delay(50);
          }
       }
        
        
        
        
      } else { 
        Serial.println("Resumed");
        musicPlayer.pausePlaying(false);
      }
    }
  }
  
  
  
  
 if (musicPlayer.stopped()) {
   lcd.setCursor(0, 1);
  //BlinkM_fadeToRandomRGB( blinkm_addr_a, '100','100','100');
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
}

void lookForBlinkM() {
  Serial.print("Looking for a BlinkM: ");
  int a = BlinkM_findFirstI2CDevice();
  if( a == -1 ) {
    Serial.println("No I2C devices found");
  } else { 
    Serial.print("Device found at addr ");
    Serial.println( a, DEC);
    blinkm_addr_a = a;
  }
}

void initialize_and_test_leds() {
 if( BLINKM_ARDUINO_POWERED )
    BlinkM_beginWithPower();
  else
    BlinkM_begin();
  
  
  // Test 0x09 LED functionality
  BlinkM_playScript( blinkm_addr_a, 18, 0x00,0x00);
  delay(2000);
  BlinkM_stopScript(blinkm_addr_a);
  //delay(100);
  BlinkM_fadeToRGB(blinkm_addr_a, 0,0,0);
 // delay(100);
  
 
 // delay(100);
  BlinkM_playScript( blinkm_addr_b, 18, 0x00,0x00);
  delay(2000);
  BlinkM_stopScript(blinkm_addr_b);
//  delay(100);
  BlinkM_fadeToRGB(blinkm_addr_b, 0,0,0);
  
  
 // delay(100);
  BlinkM_playScript( blinkm_addr_c, 18, 0x00,0x00);
  delay(2000);
  BlinkM_stopScript(blinkm_addr_c);
//  delay(100);
  BlinkM_fadeToRGB(blinkm_addr_c, 0,0,0);  
}

void initialize_real_time_clock() {
  
  RTC.begin();
  DateTime now = RTC.now();
  DateTime compiled = DateTime(__DATE__, __TIME__);
  if (now.unixtime() < compiled.unixtime()) {
    //Serial.println("RTC is older than compile time!  Updating");
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }  
}

void initialize_pin_modes() {
  
  pinMode(MICRO_SD_CHIP_SELECT, OUTPUT);     
  pinMode(PIEZO_SOUNDER_PIN,OUTPUT);
  pinMode(BUTTON_PIN_1, INPUT);
  pinMode(BUTTON_PIN_2, INPUT);
  pinMode(BUTTON_PIN_3, INPUT);
  pinMode(BUTTON_PIN_4, INPUT);
  pinMode(BUTTON_PIN_5, INPUT);
  pinMode(PIR_A_SIGNAL_PIN, INPUT);
  pinMode(PIR_B_SIGNAL_PIN, INPUT);
  pinMode(PIR_A_LED_PIN, OUTPUT);
  pinMode(PIR_B_LED_PIN, OUTPUT);
  pinMode(SS, OUTPUT);
}

void initialize_and_calibrate_PIR_sensor_array() {
  
  //give the sensor some time to calibrate
  Serial.print("calibrating sensor ");
    for(int i = 0; i < PIR_CALIBRATION_TIME; i++){
      Serial.print(".");
      delay(1000);
      }
    Serial.println(" done");
    Serial.println("SENSOR ACTIVE");
    delay(50);
}

void initialize_lux_sensor() {
  
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
}

void initialize_lcd_backpack_and_screen() {
  
  // set up the LCD's number of rows and columns: 
  lcd.begin(20, 4);
  // Print a message to the LCD.
  lcd.print("Anti Predator Device");
  
}

void initialize_datalogging_sd_card() {

  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
 
  
  // see if the card is present and can be initialized:
  if (!SD.begin(MICRO_SD_CHIP_SELECT)) {
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

void initialize_vs1053_music_player() {
   
  if (! musicPlayer.begin()) { // initialise the music player
        Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
        while (1);
  }
  Serial.println(F("VS1053 found"));
   
  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(20,20);

  // Timer interrupts are not suggested, better to use DREQ interrupt!
  //musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT); // timer int

  // If DREQ is on an interrupt pin (on uno, #2 or #3) we can do background
  // audio playing
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
   musicPlayer.startPlayingFile("track002.mp3");
 }
  
