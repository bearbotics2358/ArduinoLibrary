

// NEOPIXEL BEST PRACTICES for most reliable operation:
// - Add 1000 uF CAPACITOR between NeoPixel strip's + and - connections.
// - MINIMIZE WIRING LENGTH between microcontroller board and first pixel.
// - NeoPi/xel strip's DATA-IN should pass through a 300-500 OHM RESISTOR.
// - AVOID connecting NeoPixels on a LIVE CIRCUIT. If you must, ALWAYS
//   connect GROUND (-) first, then +, then data.
// - When using a 3.3V microcontroller with a 5V-powered NeoPixel strip,
//   a LOGIC-LEVEL CONVERTER on the data line is STRONGLY RECOMMENDED.
// (Skipping these may work OK on your workbench but can fail in the field)

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#include <Wire.h>
#include <SPI.h>
#include <string.h>
#include "TOF_protocol.h"


// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    6

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 150

#define MAXLEN 256

char rx_buff[MAXLEN];
int rx_index = 0;

// Declare our NeoPixel strip object:

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, PIN_EXTERNAL_NEOPIXELS, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
uint32_t colWhite = strip.Color(25,25,25);
uint32_t color = strip.Color(255,58,1); // placeholder name

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
      Serial.print("9,1,cmd = ");
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
      Serial.print("9,1,TARGET_TYPE command received : ");
#endif      
      if(data_len) {
        data0 = atoi(strtok(NULL, ","));
        //cone_f = (data0 == target_type_enum::CONE) ? 1 : 0;
#ifdef DEBUG    
        Serial.print(cone_f ? "CONE" : "CUBE");
#endif      
      
#ifdef DEBUG    
      Serial.println();
#endif      
      break;
    
    default:
#ifdef DEBUG    
      Serial.println("9,1,unknown command");
#endif      
      break;
  }
}
}

void setup() {
  // core1 setup
  Serial.begin(115200);

  pinMode(PIN_EXTERNAL_POWER, OUTPUT);
  digitalWrite(PIN_EXTERNAL_POWER, LOW);

  strip.begin();
  strip.show();
  strip.fill(colWhite);
  strip.setBrightness(50);

  pinMode(PIN_EXTERNAL_BUTTON, INPUT_PULLUP);
}


// loop() function -- runs repeatedly as long as board is on ---------------

void loop() {
  // Fill along the length of the strip in various colors...
  //rainbow(11); loss of comms
  for(int i=14; i<22; i++) {
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.setBrightness(220);
    strip.show();                          //  Update strip to match
  }
}

// Some functions of our own for creating animated effects -----------------

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}



// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}
