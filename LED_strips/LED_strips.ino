/*
  Light neopixel srips based on target type

*/

// using TOF_protocol version 1.0

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>
#include "TOF_protocol.h"

#define NUMPIXELS 30 // Number of LEDs in strip

#if defined(__SAMD21G18A__) || defined(__AVR_ATmega32U4__) || defined(NRF52840_XXAA)
  #define NEOPIXEL_PIN 5
  #define POWER_PIN    10
#elif defined(__AVR_ATmega328P__)
  #define NEOPIXEL_PIN 5
  #define POWER_PIN    10
#elif defined(NRF52)
  #define NEOPIXEL_PIN 27
  #define POWER_PIN    11
#elif defined(ESP8266)
  #define NEOPIXEL_PIN 2
  #define POWER_PIN    15
#elif defined(TEENSYDUINO)
  #define NEOPIXEL_PIN 8
  #define POWER_PIN    10
#elif defined(ESP32)
  #define NEOPIXEL_PIN 14
  #define POWER_PIN    33
#endif

// create neopixel strips
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, 4, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NUMPIXELS, 5, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3 = Adafruit_NeoPixel(NUMPIXELS, 6, NEO_GRB + NEO_KHZ800);


#define MAXLEN 256

char rx_buff[MAXLEN];
int rx_index = 0;

int cone_f = target_type_enum::CONE; 

int rainbow_f = 1; // rainbow until target type command received

//ok to change this tolerace over distance tolerance?
void setColor(int index, int val) {
  // index is the index of the LED, from 0 to 3
  // val is the histogram count
  if((val < 10)) { // 0xGGRRBB (color values)
    // Serial.print("red; not enough, too far");
    strip.setPixelColor(index, 0x005000);
    strip2.setPixelColor(index, 0x005000);
    strip3.setPixelColor(index, 0x005000);
  } else if(val < 20) { 
    // Serial.print("yellow; close");
    strip.setPixelColor(index, 0x505000);  
    strip2.setPixelColor(index, 0x505000);  
    strip3.setPixelColor(index, 0x505000);  
  } else {
    // Serial.print("green; just right");
    strip.setPixelColor(index, 0x500000); 
    strip2.setPixelColor(index, 0x500000); 
    strip3.setPixelColor(index, 0x500000); 
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
    // turn on the Prop-Maker FeatherWing's power pin
    digitalWrite(POWER_PIN, HIGH);
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
      Serial.print("cmd = ");
      Serial.println(rx_buff);

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

  switch(msg_type) {
    case RIO_TOF_msgs_enum::TARGET_TYPE:
      Serial.print("TARGET_TYPE command received : "); // 0 is cone, 1 is cube
      if(data_len) {
        cone_f = atoi(strtok(NULL, ","));
        Serial.print(cone_f ? "CUBE" : "CONE");
        rainbow_f = 0;
        if(cone_f) {
          strip.fill(0x400040);
          strip2.fill(0x400040);
          strip3.fill(0x400040);
        } else {
          strip.fill(0x804000);
          strip2.fill(0x804000);
          strip3.fill(0x804000);
        }
        strip.show();
        strip2.show();
        strip3.show();
      }
      Serial.println();
      break;
    
    default:
      Serial.println("unknown command");
  }
}

void setup()
{
  int i;
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  // clear rx_buff
//  bzero(rx_buff, MAXLEN);
  memset(rx_buff, MAXLEN, 0);

  // Set power pin to output
  pinMode(POWER_PIN, OUTPUT);
  // Enable the pin, we're not currently writing to the neopixels.
  digitalWrite(POWER_PIN, HIGH);

  // This initializes the NeoPixel library.
  strip.begin(); 
  strip2.begin(); 
  strip3.begin(); 

  // initialize LEDs to blue
  for(i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 0x000050);      
  }
  strip.show();                     // Refresh strip
  strip2.show();                     // Refresh strip
  strip3.show();                     // Refresh strip
}

void loop()
{
  int i;
  char stemp[MAXLEN];
  
  if(rainbow_f) {
    // cycle a the rainbow with a 20ms wait
    rainbow(20);
  }

  // Get and process command
  if(GetCommand()) {
    ProcessCommand();
  }
  

  delay(100); // Small delay between polling
}
