

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
#include <avr/power.h>  // Required for 16 MHz Adafruit Trinket
#endif
#include <Wire.h>
#include <SPI.h>
#include <string.h>
#include "Protocol.h"


#define DEBUG 1
// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN 6


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
uint32_t colWhite = strip.Color(25, 25, 25);
uint32_t brightWhite = strip.Color(255, 255, 255);
uint32_t noteColor = strip.Color(255, 58, 1);
uint32_t greenReady = strip.Color(25, 255, 25);
uint32_t bearBlue = strip.Color(5, 15, 255);
uint32_t bearCyan = strip.Color(25, 35, 215);

bool newCommand = false;

// return true if a command has been received and is stored in rx_buff
int GetCommand() {
  int ret = 0;

  // get command if there is one
  // every time called, and every time through loop, get command chars
  // if available
  // when '\r' (or '\n') found, process command
  // throw away remaining '\r' or '\n'
  while (Serial.available() > 0) {
    rx_buff[rx_index] = Serial.read();
    if ((rx_buff[rx_index] == '\r')
        || (rx_buff[rx_index] == '\n')) {
      // process command
      if (rx_index == 0) {
        // no command
        continue;
      }
      // replace with a string terminator to enable printing
      rx_buff[rx_index] = 0;
#ifdef DEBUG
      // Serial.print("9,1,cmd = ");
      // Serial.println(rx_buff);
#endif

      // process command
      ret = 1;

      // reset for next command
      rx_index = 0;
    } else {
      // have not received end of command yet
      if (rx_index < MAXLEN - 1) {
        rx_index++;
      }
    }
  }
  return ret;
}

void ProcessCommand() {
  newCommand = false;
  // get msg_type and data_len
  int msg_type = atoi(strtok(rx_buff, ","));
#ifdef DEBUG
  // Serial.print("msg_type: ");
#endif
  // Serial.println(msg_type);
  int data_len = atoi(strtok(NULL, ","));
  int data0 = 0;

  switch (msg_type) {

    case RIO_msgs_enum::WHITE:
#ifdef DEBUG
      Serial.print("9,0,WHITE command received");
#endif
      strip.clear();
      strip.fill(colWhite);
      strip.show();

#ifdef DEBUG
      Serial.println();
#endif
      break;


    case RIO_msgs_enum::IDLE:
#ifdef DEBUG
      Serial.print("9,1,IDLE command received");
#endif
      alternatingWipe(brightWhite, bearCyan, 5);
      strip.show();

#ifdef DEBUG
      Serial.println();
#endif
      break;


    case RIO_msgs_enum::NO_COMMS:
#ifdef DEBUG
      Serial.print("9,1,NO_COMMS command received");
#endif

      // temp to have another color w/o animations
      strip.clear();
      strip.fill(greenReady);
      strip.show();
      break;
      
      
      // Fill along the length of the strip in various colors...
      rainbow(35);  //loss of comms


#ifdef DEBUG
      Serial.println();
#endif
      break;









    case RIO_msgs_enum::NOTE_ON_BOARD:
#ifdef DEBUG
      Serial.print("9,3,NOTE_ON_BOARD command received");
#endif
      // Fill along the length of the strip in various colors...
      colorWipe(greenReady, 30);  //NOTE ON BOARD
#ifdef DEBUG
      Serial.println();
#endif
      break;



    case RIO_msgs_enum::SHOOTER_READY:
#ifdef DEBUG
      Serial.print("9,5,SHOOTER_READY command received");
#endif
      // Fill along the length of the strip in various colors...
      theaterChase(bearBlue, 65);

#ifdef DEBUG
      Serial.println();
#endif
      break;



    default:
#ifdef DEBUG
      Serial.println("9,0,unknown command");
#endif
      break;
  }
}


void setup() {
  // core1 setup
  Serial.begin(115200);

  pinMode(PIN_EXTERNAL_POWER, OUTPUT);
  digitalWrite(PIN_EXTERNAL_POWER, HIGH);

  strip.begin();
  strip.fill(colWhite);
  strip.setBrightness(50);
  strip.show();

  pinMode(PIN_EXTERNAL_BUTTON, INPUT_PULLUP);
}

void noteAngle(int angle) {
  for (int i = 14; i < 22; i++) {
    strip.setPixelColor(i, noteColor);  //  Set pixel's color (in RAM)
    strip.setBrightness(220);
    strip.show();  //  Update strip to match
  }
}

// loop() function -- runs repeatedly as long as board is on ---------------

void loop() {
  // Fill along the length of the strip in various colors...
  //rainbow(11); loss of comms

  // Get and process command
  if (newCommand || GetCommand()) {
    ProcessCommand();
  }



  delay(5);  // Small delay between polling
}


// Some functions of our own for creating animated effects -----------------

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
    while (!newCommand){
  for (int i = 0; i < strip.numPixels(); i++) {  // For each pixel in strip...
    strip.setPixelColor(i, color);               //  Set pixel's color (in RAM)
    strip.show();                                //  Update strip to match
         if (GetCommand()) {
      newCommand = true;
      break;
     }
    delay(wait);                                 //  Pause for a moment
     }
  }
}

void alternatingWipe(uint32_t color, uint32_t colorTwo, int wait) {
    while (!newCommand){
  for (int i = 0; i < strip.numPixels(); i++) {  // For each pixel in strip...
    if (i % 2 == 0) {
      strip.setPixelColor(i, color);
    } else {
      strip.setPixelColor(i, colorTwo);
    }
    strip.show();  //  Update strip to match
       if (GetCommand()) {
      newCommand = true;
      break;
     }
    delay(wait);   //  Pause for a moment
   }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  while (!newCommand){
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 2048) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show();  // Update strip with new contents
    if (GetCommand()) {
      newCommand = true;
      break;
     }
    delay(wait);   // Pause for a moment
   }
  }
}

//from https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/#google_vignette

void theaterChase(uint32_t color, int wait) {
  while (!newCommand){
  for (int a = 0; a < 10; a++) {   // Repeat 10 times...
    for (int b = 0; b < 3; b++) {  //  'b' counts from 0 to 2...
      strip.clear();               //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for (int c = b; c < strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color);  // Set pixel 'c' to value 'color'
      }
      strip.show();  // Update strip with new contents
       if (GetCommand()) {
      newCommand = true;
      break;
     }
      delay(wait);   // Pause for a moment
     }
    }
  }
}
