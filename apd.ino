// APD version 1.1.10
// 8/6/14
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
#include <SFE_TSL2561.h>     // Need this for Luminosity sensor support. changed for more extensive library
                             // from Mike Grusin. The Adafruit library didn't have functions
                             // to set Interrupts and HI/LOW light thresholds for Interrupt. This is better.
#include <Adafruit_VS1053.h> // Need this for music player library functions
#include "LiquidCrystal.h"   // Need this for I2C backback and LCD display
                             // Note this is latest version of this library from 
                             // adafruit that includes I2C support

#include <MenuBackend.h>     //MenuBackend library - need this to run LCD menu routine
                             // Compliments of Alexander Brevig
                             
                             // IMPORTANT: to use the menubackend library by Alexander Brevig download it at
                             // http://www.arduino.cc/playground/uploads/Profiles/MenuBackend_1-4.zip 
                             // and add the next code at line 195
	                     //  void toRoot() {
		             //       setCurrent( &getRoot() );
	                     //  }

 
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


#define STATE_IDLE_POLLING    2
#define STATE_BUTTON_ISR      3
#define STATE_NIGHT_TIME_ISR  4

// Class declarations for system

SFE_TSL2561 light;                  // Need this for Luminosity sensor. Default I2C address is: 0x09 = TSL2561_ADDR_FLOAT
                                    // This can be changed by soldering jumpers on the sensor
                                    // The address will be different depending on whether you let
                                    // the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases
                                    // use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively
                                    
LiquidCrystal lcd(0);               // Need this to initialize the I2C backpack / LCD combo             
                                    // Connect via i2c, default address #0 (A0-A2 not jumpered)
                                    // I2C address for LCD is 0x00 without soldering jumper headers
                                    // I did not jump A0-A2 which is why lcd is initialized as lcd(0)
                                    
                                    // This is a special version of LiquidCrystal.h with overloaded 
                                    // constructor, if only one argument is given, the I2C backpack kicks in
                                    
RTC_DS3231 RTC;                     // Need this to create an instance of the RTC_DS3231 class for real time clock
                                    // located in RTC_DS.3231.h
                                    
                                    
Adafruit_VS1053_FilePlayer musicPlayer =     // Need this to create instance of music player class and File Player   

                                             // create breakout-example object!
  
  Adafruit_VS1053_FilePlayer(BREAKOUT_RESET, BREAKOUT_CS, BREAKOUT_DCS, DREQ, CARDCS);
                                             // create shield-example object!
                                             //Adafruit_VS1053_FilePlayer(SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);                                   
                                  
File dataFile;                      // Need this to declare a File instance for use in datalogging.

                                    
MenuBackend menu = MenuBackend(menuUsed,menuChanged);  //Menu variables
                                    
    MenuItem menu1Item1 = MenuItem("Item1");           //initialize menuitems
      MenuItem menuItem1SubItem1 = MenuItem("Item1SubItem1");
      MenuItem menuItem1SubItem2 = MenuItem("Item1SubItem2");
    MenuItem menu1Item2 = MenuItem("Item2");
      MenuItem menuItem2SubItem1 = MenuItem("Item2SubItem1");
      MenuItem menuItem2SubItem2 = MenuItem("Item2SubItem2");
      MenuItem menuItem3SubItem3 = MenuItem("Item2SubItem3");
    MenuItem menu1Item3 = MenuItem("Item3");

const boolean BLINKM_ARDUINO_POWERED = true;  // For now this is true. This will change when moving for bench
                                              // testing to field testing

byte blinkm_addr_a = 0x09;          // I2C Address of one of the LED's. LED A
byte blinkm_addr_b = 0x0C;          // I2C Address of one of the LED's. LED B 
byte blinkm_addr_c = 0x0D;          // I2C Address of one of the LED's. LED C

volatile int state = 0;
boolean gain;     // Gain setting, 0 = X1, 1 = X16;
unsigned int ms;  // Integration ("shutter") time in milliseconds

//Redundant for LCD menu code, fix later, shouldn't be globals..
// ++
const int buttonPinLeft =   30;      // pin for the Up button
const int buttonPinRight =  31;     // pin for the Down button
const int buttonPinEsc =    32;       // pin for the Esc button
const int buttonPinEnter =  33;     // pin for the Enter button
int lastButtonPushed = 0;

int lastButtonEnterState = LOW;   // the previous reading from the Enter input pin
int lastButtonEscState = LOW;   // the previous reading from the Esc input pin
int lastButtonLeftState = LOW;   // the previous reading from the Left input pin
int lastButtonRightState = LOW;   // the previous reading from the Right input pin


long lastEnterDebounceTime = 0;  // the last time the output pin was toggled
long lastEscDebounceTime = 0;  // the last time the output pin was toggled
long lastLeftDebounceTime = 0;  // the last time the output pin was toggled
long lastRightDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 200;    // the debounce time

// ++

void setup() {
   
  initialize_real_time_clock();
  initialize_and_test_leds();
  initialize_pin_modes();  
  initialize_and_calibrate_PIR_sensor_array();
  initialize_lux_sensor();
  initialize_lcd_backpack_and_screen();
  initialize_datalogging_sd_card();
  initialize_vs1053_music_player();
  initialize_LCD_menu_system();
  Serial.begin(19200);
  attachInterrupt(4,pin_19_ISR,CHANGE);
  state = STATE_IDLE_POLLING;
  
  
}

void loop() {
  

  switch(state) {
    
    case STATE_IDLE_POLLING:
    {
      Serial.println("Just chilling out");
      break;
    }
    case STATE_BUTTON_ISR:
    {
      
      
      
      Serial.println("Whoa dude a button was pressed!");
      
      // Decode buttons
      
        readButtons();  //I splitted button reading and navigation in two procedures because 
        navigateMenus();  //in some situations I want to use the button for other purpose (eg. to change some settings)
     
      state = STATE_BUTTON_ISR;
      
      break;
    }
    
    case STATE_NIGHT_TIME_ISR:
    {
       Serial.println("Oh shit it night time its the right time baby");
  
       light.clearInterrupt();
       light.setInterruptControl(0, 0);
       detachInterrupt(0);                // Turn off interrupt for now, it's night time.
                                          // Turn back on when day is upon us in the African Wild.
                                          // Tibetan Plains
       state = STATE_IDLE_POLLING;
       break;
    }
  }
  
  //PIR sensor's setup variables

/////////////////////////////
//VARS
//the time we give the sensor to calibrate (10-60 secs according to the datasheet)
int calibrationTime = 10;

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
  
   // There are two light sensors on the device, one for visible light
  // and one for infrared. Both sensors are needed for lux calculations.
  
  // Retrieve the data from the device:

  unsigned int data0, data1;
  
  if (light.getData(data0,data1))
  {
    // getData() returned true, communication was successful
    
    Serial.print("data0: ");
    Serial.print(data0);
    Serial.print(" data1: ");
    Serial.print(data1);
  
    // To calculate lux, pass all your settings and readings
    // to the getLux() function.
    
    // The getLux() function will return 1 if the calculation
    // was successful, or 0 if one or both of the sensors was
    // saturated (too much light). If this happens, you can
    // reduce the integration time and/or gain.
    // For more information see the hookup guide at: https://learn.sparkfun.com/tutorials/getting-started-with-the-tsl2561-luminosity-sensor
  
    double lux;    // Resulting lux value
    boolean good;  // True if neither sensor is saturated
   
    // Perform lux calculation:
   
    good = light.getLux(gain,ms,data0,data1,lux);
    
    // Print out the results:
	
    Serial.print(" lux: ");
    Serial.print(lux);
    if (good) Serial.println(" (good)"); else Serial.println(" (BAD)");
  }
  else
  {
    
  }    
 
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
    lcd.print(data0,DEC);
    Serial.println(data0,DEC);
    dataFile.println(data0,DEC);
   
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
  // You can pass nothing to light.begin() for the default I2C address (0x39),
  // or use one of the following presets if you have changed
  // the ADDR jumper on the board:
  
  // TSL2561_ADDR_0 address with '0' shorted on board (0x29)
  // TSL2561_ADDR   default address (0x39)
  // TSL2561_ADDR_1 address with '1' shorted on board (0x49)

  // For more information see the hookup guide at: https://learn.sparkfun.com/tutorials/getting-started-with-the-tsl2561-luminosity-sensor

  light.begin();

  // Get factory ID from sensor:
  // (Just for fun, you don't need to do this to operate the sensor)

  unsigned char ID;
  
  if (light.getID(ID))
  {
    Serial.print("Got factory ID: 0X");
    Serial.print(ID,HEX);
    Serial.println(", should be 0X5X");
  }
  // Most library commands will return true if communications was successful,
  // and false if there was a problem. You can ignore this returned value,
  // or check whether a command worked correctly and retrieve an error code:
  else
  {
   
  }

  // The light sensor has a default integration time of 402ms,
  // and a default gain of low (1X).
  
  // If you would like to change either of these, you can
  // do so using the setTiming() command.
  
  // If gain = false (0), device is set to low gain (1X)
  // If gain = high (1), device is set to high gain (16X)

  gain = 0;

  // If time = 0, integration will be 13.7ms
  // If time = 1, integration will be 101ms
  // If time = 2, integration will be 402ms
  // If time = 3, use manual start / stop to perform your own integration

  unsigned char time = 2;

  // setTiming() will set the third parameter (ms) to the
  // requested integration time in ms (this will be useful later):
  
  Serial.println("Set timing...");
  light.setTiming(gain,time,ms);

  // To start taking measurements, power up the sensor:
  
  Serial.println("Powerup...");
  light.setPowerUp();
  
  // The sensor will now gather light during the integration time.
  // After the specified time, you can retrieve the result from the sensor.
  // Once a measurement occurs, another integration period will start.
  
  light.setInterruptControl(1, 15);    // Enable Interrupt Pin Output
			               // Sets up interrupt operations
			               // If control = 0, interrupt output disabled
			               // If control = 1, use level interrupt, see setInterruptThreshold()
			               // If persist = 0, every integration cycle generates an interrupt
			               // If persist = 1, any value outside of threshold generates an interrupt
			               // If persist = 2 to 15, value must be outside of threshold for 2 to 15 integration cycles
			               // Returns true (1) if successful, false (0) if there was an I2C error
			               // (Also see getError() below)

  light.setInterruptThreshold(300, 1500);  // set LOW and HIGH channel 0 threshholds for Interrupt trigger
			                   // Set interrupt thresholds (channel 0 only)
			                   // low, high: 16-bit threshold values
			                   // Returns true (1) if successful, false (0) if there was an I2C error
			                   // (Also see getError() below)

  attachInterrupt(0,DayNightISR,RISING);  // Attach the interrupt to pin 2.
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
 
 void initialize_LCD_menu_system() {
     //configure menu
  menu.getRoot().add(menu1Item1);
  menu1Item1.addRight(menu1Item2).addRight(menu1Item3);
  menu1Item1.add(menuItem1SubItem1).addRight(menuItem1SubItem2);
  menu1Item2.add(menuItem2SubItem1).addRight(menuItem2SubItem2).addRight(menuItem3SubItem3);
  menu.toRoot();
  lcd.setCursor(0,0);  
  lcd.print("AntiPredator Device");
 }
 
 
 
 
 
 void menuChanged(MenuChangeEvent changed){
  
  MenuItem newMenuItem=changed.to; //get the destination menu
  
  lcd.setCursor(0,1); //set the start position for lcd printing to the second row
  
  if(newMenuItem.getName()==menu.getRoot()){
      lcd.print("Main Menu       ");
  }else if(newMenuItem.getName()=="Item1"){
      lcd.print("Item1           ");
  }else if(newMenuItem.getName()=="Item1SubItem1"){
      lcd.print("Item1SubItem1");
  }else if(newMenuItem.getName()=="Item1SubItem2"){
      lcd.print("Item1SubItem2   ");
  }else if(newMenuItem.getName()=="Item2"){
      lcd.print("Item2           ");
  }else if(newMenuItem.getName()=="Item2SubItem1"){
      lcd.print("Item2SubItem1   ");
  }else if(newMenuItem.getName()=="Item2SubItem2"){
      lcd.print("Item2SubItem2   ");
  }else if(newMenuItem.getName()=="Item2SubItem3"){
      lcd.print("Item2SubItem3   ");
  }else if(newMenuItem.getName()=="Item3"){
      lcd.print("Item3           ");
  }
}

void menuUsed(MenuUseEvent used){
  lcd.setCursor(0,0);  
  lcd.print("You used        ");
  lcd.setCursor(0,1); 
  lcd.print(used.item.getName());
  delay(3000);  //delay to allow message reading
  lcd.setCursor(0,0);  
  lcd.print("APD");
  menu.toRoot();  //back to Main
}
 
void  readButtons(){  //read buttons status
  int reading;
  int buttonEnterState=LOW;             // the current reading from the Enter input pin
  int buttonEscState=LOW;             // the current reading from the input pin
  int buttonLeftState=LOW;             // the current reading from the input pin
  int buttonRightState=LOW;             // the current reading from the input pin

  //Enter button
                  // read the state of the switch into a local variable:
                  reading = digitalRead(buttonPinEnter);

                  // check to see if you just pressed the enter button 
                  // (i.e. the input went from LOW to HIGH),  and you've waited 
                  // long enough since the last press to ignore any noise:  
                
                  // If the switch changed, due to noise or pressing:
                  if (reading != lastButtonEnterState) {
                    // reset the debouncing timer
                    lastEnterDebounceTime = millis();
                  } 
                  
                  if ((millis() - lastEnterDebounceTime) > debounceDelay) {
                    // whatever the reading is at, it's been there for longer
                    // than the debounce delay, so take it as the actual current state:
                    buttonEnterState=reading;
                    lastEnterDebounceTime=millis();
                  }
                  
                  // save the reading.  Next time through the loop,
                  // it'll be the lastButtonState:
                  lastButtonEnterState = reading;
                  

    //Esc button               
                  // read the state of the switch into a local variable:
                  reading = digitalRead(buttonPinEsc);

                  // check to see if you just pressed the Down button 
                  // (i.e. the input went from LOW to HIGH),  and you've waited 
                  // long enough since the last press to ignore any noise:  
                
                  // If the switch changed, due to noise or pressing:
                  if (reading != lastButtonEscState) {
                    // reset the debouncing timer
                    lastEscDebounceTime = millis();
                  } 
                  
                  if ((millis() - lastEscDebounceTime) > debounceDelay) {
                    // whatever the reading is at, it's been there for longer
                    // than the debounce delay, so take it as the actual current state:
                    buttonEscState = reading;
                    lastEscDebounceTime=millis();
                  }
                  
                  // save the reading.  Next time through the loop,
                  // it'll be the lastButtonState:
                  lastButtonEscState = reading; 
                  
                     
   //Down button               
                  // read the state of the switch into a local variable:
                  reading = digitalRead(buttonPinRight);

                  // check to see if you just pressed the Down button 
                  // (i.e. the input went from LOW to HIGH),  and you've waited 
                  // long enough since the last press to ignore any noise:  
                
                  // If the switch changed, due to noise or pressing:
                  if (reading != lastButtonRightState) {
                    // reset the debouncing timer
                    lastRightDebounceTime = millis();
                  } 
                  
                  if ((millis() - lastRightDebounceTime) > debounceDelay) {
                    // whatever the reading is at, it's been there for longer
                    // than the debounce delay, so take it as the actual current state:
                    buttonRightState = reading;
                   lastRightDebounceTime =millis();
                  }
                  
                  // save the reading.  Next time through the loop,
                  // it'll be the lastButtonState:
                  lastButtonRightState = reading;                  
                  
                  
    //Up button               
                  // read the state of the switch into a local variable:
                  reading = digitalRead(buttonPinLeft);

                  // check to see if you just pressed the Down button 
                  // (i.e. the input went from LOW to HIGH),  and you've waited 
                  // long enough since the last press to ignore any noise:  
                
                  // If the switch changed, due to noise or pressing:
                  if (reading != lastButtonLeftState) {
                    // reset the debouncing timer
                    lastLeftDebounceTime = millis();
                  } 
                  
                  if ((millis() - lastLeftDebounceTime) > debounceDelay) {
                    // whatever the reading is at, it's been there for longer
                    // than the debounce delay, so take it as the actual current state:
                    buttonLeftState = reading;
                    lastLeftDebounceTime=millis();;
                  }
                  
                  // save the reading.  Next time through the loop,
                  // it'll be the lastButtonState:
                  lastButtonLeftState = reading;  

                  //records which button has been pressed
                  if (buttonEnterState==HIGH){
                    lastButtonPushed=buttonPinEnter;

                  }else if(buttonEscState==HIGH){
                    lastButtonPushed=buttonPinEsc;

                  }else if(buttonRightState==HIGH){
                    lastButtonPushed=buttonPinRight;

                  }else if(buttonLeftState==HIGH){
                    lastButtonPushed=buttonPinLeft;

                  }else{
                    lastButtonPushed=0;
                  }                  
}

void navigateMenus() {
  MenuItem currentMenu=menu.getCurrent();
  
  switch (lastButtonPushed){
    case buttonPinEnter:
      if(!(currentMenu.moveDown())){  //if the current menu has a child and has been pressed enter then menu navigate to item below
        menu.use();
      }else{  //otherwise, if menu has no child and has been pressed enter the current menu is used
        menu.moveDown();
       } 
      break;
    case buttonPinEsc:
      menu.toRoot();  //back to main
      break;
    case buttonPinRight:
      menu.moveRight();
      break;      
    case buttonPinLeft:
      menu.moveLeft();
      break;      
  }
  
  lastButtonPushed=0; //reset the lastButtonPushed variable
} 
 
 
 void pin_19_ISR() {
   
   state = STATE_BUTTON_ISR;
 }
 
 void DayNightISR() {
 
  state = STATE_NIGHT_TIME_ISR;
  
}
  
