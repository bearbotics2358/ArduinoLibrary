/*
  Light neopixel srips based on target type

*/

// using TOF_protocol version 1.0

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>
#include "TOF_protocol.h"

// comment out the next line for running with an Arduino
// #define PROP_MAKER

// for debugging printouts, uncomment the next line
// #define DEBUG

#define NUMPIXELS 30 // Number of LEDs in strip

#ifdef PROP_MAKER
  #define NEOPIXEL_PIN 5
  #define POWER_PIN    10
#else // Arduino
  #define NEOPIXEL_PIN 5
#endif

// create neopixel strips
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, 4, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NUMPIXELS, 5, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3 = Adafruit_NeoPixel(NUMPIXELS, 6, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip4 = Adafruit_NeoPixel(NUMPIXELS, 6, NEO_GRB + NEO_KHZ800);


#define MAXLEN 256

char rx_buff[MAXLEN];
int rx_index = 0;

int cone_f = 1; // start with a cone


int rainbow_f = 0; // rainbow until target type command received

//ok to change this tolerace over distance tolerance?
void setColor(int index, int val) {
  // index is the index of the LED, from 0 to 3
  // val is the histogram count
  if((val < 10)) { // 0xGGRRBB (color values)
    // Serial.print("red; not enough, too far");
    strip.setPixelColor(index, 0x005000);
    strip2.setPixelColor(index, 0x005000);
    strip3.setPixelColor(index, 0x005000);
    strip4.setPixelColor(index, 0x005000);
  } else if(val < 20) { 
    // Serial.print("yellow; close");
    strip.setPixelColor(index, 0x505000);  
    strip2.setPixelColor(index, 0x505000);  
    strip3.setPixelColor(index, 0x505000);  
    strip4.setPixelColor(index, 0x505000);  
  } else {
    // Serial.print("green; just right");
    strip.setPixelColor(index, 0x500000); 
    strip2.setPixelColor(index, 0x500000); 
    strip3.setPixelColor(index, 0x500000); 
    strip4.setPixelColor(index, 0x500000); 
  }
}

uint32_t Wheel(byte WheelPos) {
  // Input a value 0 to 255 to get a color value.
  // The colours are a transition r - g - b - back to r.
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    // write the pixel values
    strip.show();
    delay(wait);
  }
}

// return true if a command has been received and is stored in rx_buff
int GetCommand()
{
  int ret = 0;
  
  // get command if there is one
  // every time called, and every time through loop, get command chars
  // if available
  // when '\r' (or '\n') found, process command
  // throw away remaining '\r' or '\n'
  while(Serial.available() > 0) {
    rx_buff[rx_index] = Serial.read();
    if((rx_buff[rx_index] == '\r') 
      || (rx_buff[rx_index] == '\n')) {
      // process command
      if(rx_index == 0) {
        // no command
        continue;
      }
      rx_buff[rx_index + 1] = 0;
#ifdef DEBUG      
      Serial.print("cmd = ");
      Serial.println(rx_buff);
#endif

      // process command
      ret = 1;
      
      // reset for next command
      rx_index = 0;
    } else {
      // have not received end of command yet
      if(rx_index < MAXLEN - 1) {
        rx_index++;
      }
    }
  }
  return ret;
}

void ProcessCommand()
{
  // get msg_type and data_len
  int msg_type = atoi(strtok(rx_buff, ","));
  int data_len = atoi(strtok(NULL, ","));
  int data0 = 0;

  switch(msg_type) {
    case RIO_TOF_msgs_enum::TARGET_TYPE:
#ifdef DEBUG    
      Serial.print("TARGET_TYPE command received : ");
#endif      
      if(data_len) {
        data0 = atoi(strtok(NULL, ","));
        cone_f = (data0 == target_type_enum::CONE) ? 1 : 0;
#ifdef DEBUG    
        Serial.print(cone_f ? "CONE" : "CUBE");
#endif      
        // stop rainbow display
        rainbow_f = 0;
        // light strip appropriate color
        if(cone_f) {
          // set to purple
          strip.fill(0x400040);
          strip2.fill(0x400040);
          strip3.fill(0x400040);
          strip4.fill(0x400040);
        } else {
          // set to orange
          strip.fill(0x804000);
          strip2.fill(0x804000);
          strip3.fill(0x804000);
          strip4.fill(0x804000);
        }
        strip.show();
        strip2.show();
        strip3.show();
        strip4.show();
      }
#ifdef DEBUG    
      Serial.println();
#endif      
      break;
    
    default:
#ifdef DEBUG    
      Serial.println("unknown command");
#endif      
      break;
  }
}

void setup()
{
  int i;
  
  Serial.begin(115200);
  delay(1000);
#ifdef DEBUG    
  Serial.println("LED strips control");
#endif      

  // clear rx_buff
//  bzero(rx_buff, MAXLEN);
  memset(rx_buff, MAXLEN, 0);

#ifdef PROP_MAKER
  // Set power pin to output
  pinMode(POWER_PIN, OUTPUT);
  // turn on the Prop-Maker FeatherWing's power pin
  digitalWrite(POWER_PIN, HIGH);
#endif

  // This initializes the NeoPixel library.
  strip.begin(); 
  strip2.begin(); 
  strip3.begin(); 
  strip4.begin(); 

  // initialize LEDs to blue
  for(i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 0x000050);      
    strip2.setPixelColor(i, 0x000050);      
    strip3.setPixelColor(i, 0x000050);
    strip4.setPixelColor(i, 0x000050);
          
  }
  strip.show();                     // Refresh strip
  strip2.show();                     // Refresh strip
  strip3.show();                     // Refresh strip
  strip4.show();                     // Refresh strip
}

void loop()
{
  int i;
  char stemp[MAXLEN];
  
  if(rainbow_f) {
    // cycle a the rainbow with a 20ms wait
    rainbow(10);
  }

  // Get and process command
  if(GetCommand()) {
    ProcessCommand();
  }
  

  delay(5); // Small delay between polling
}
