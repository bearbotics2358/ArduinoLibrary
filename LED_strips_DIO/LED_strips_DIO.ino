/*
  Light neopixel srips based on target type

*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>

// comment out the next line for running with an Arduino
// #define PROP_MAKER

// for debugging printouts, uncomment the next line
 #define DEBUG //previously commented out -- when commented none of the if define DEBUG statements print

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
Adafruit_NeoPixel strip4 = Adafruit_NeoPixel(NUMPIXELS, 7, NEO_GRB + NEO_KHZ800);

// input pin from RoboRIO
int PIN_IN = 8;

#define MAXLEN 256

char rx_buff[MAXLEN];
int rx_index = 0;

int cone_f = 1; // start with a CONE


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

void UpdateLEDS()
{
  // stop rainbow display
  rainbow_f = 0;

  // light strips appropriate color
  if(cone_f) {
    // set to yellow for CONE
    strip.fill(0x400040);
    strip2.fill(0x400040);
    strip3.fill(0x400040);
    strip4.fill(0x400040);
  } else {
    // set to purple for CUBE
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

void setup()
{
  int i;
  
  Serial.begin(115200);
  delay(1000);
#ifdef DEBUG    
  Serial.println("9,1,LED strips control");
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

  // set pin from RoboRIO to input
  pinMode(PIN_IN, INPUT);

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

  delay(2000);
}

void loop()
{
  
  if(rainbow_f) {
    // cycle a the rainbow with a 20ms wait
    rainbow(10);
  }

  // Read input pin from RoboRIO
  // HIGH means CONE
  cone_f = digitalRead(PIN_IN);
  UpdateLEDS();

  delay(5); // Small delay between polling
}
