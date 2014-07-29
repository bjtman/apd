// APD version 1.1
// 7/28/14
// Brian Tice

#include "Wire.h"
#include "BlinkM_funcs.h"
#include <avr/pgmspace.h>  // for progmem stuff

//**TODO: evauluate if I need these globals -bjt
const boolean BLINKM_ARDUINO_POWERED = true;
byte blinkm_addr = 0x09; // the default address of all BlinkMs


void setup() {
  
  if( BLINKM_ARDUINO_POWERED )
    BlinkM_beginWithPower();
  else
    BlinkM_begin();

  delay(100); // wait a bit for things to stabilize
  BlinkM_off(0);  // turn everyone off

  //BlinkM_setAddress( blinkm_addr );  // uncomment to set address
    
  Serial.begin(19200);

  
  
  lookForBlinkM();


}

void loop() {
 
  BlinkM_fadeToRandomRGB( blinkm_addr, '100','100','100');

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

