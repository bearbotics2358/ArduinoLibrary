

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
#include "protocol2025.h"


#define DEBUG 1
// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN 6


// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 100

#define MAXLEN 900

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
uint32_t algaeAqua = strip.Color(139,247,204);
uint32_t greenReady = strip.Color(25, 155, 25);
uint32_t superPink = strip.Color(255,20,147);//(255,37,169) previously
uint32_t purpleGem = strip.Color(72, 32, 84);
uint32_t matthewRed = strip.Color(123, 10, 3);//(52,0,0) previously
uint32_t piss = strip.Color(190, 161, 4);
uint32_t bearCyan = strip.Color(25, 35, 215);

bool newCommand = false;

// return true if a command has been received and is stored in rx_buff
int GetCommand() {
  int ret = 0;
  int i;

  // get command if there is one
  // every time called, and every time through loop, get command chars
  // if available
  // when '\r' (or '\n') found, process command
  // throw away remaining '\r' or '\n'
  while (!newCommand && (Serial.available() > 0)) {
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

      // print buffer back in HEX for debugging 
      Serial.print("cmd: ");
      for(i = 0; i <= rx_index; i++) {
        Serial.print(rx_buff[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      
      // process command
      ret = 1;
      newCommand = true;

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
  int i;
  
  newCommand = false;
  
      // print buffer back in HEX for debugging 
      Serial.print("cmd: ");
      for(i = 0; i <= rx_index; i++) {
        Serial.print(rx_buff[i], HEX);
        Serial.print(" ");
      }
      
  // get msg_type and data_len
  int msg_type = atoi(strtok(rx_buff, ","));
#ifdef DEBUG
  Serial.print("msg_type: ");
#endif
  Serial.println(msg_type);
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


    case RIO_msgs_enum::MSG_IDLE:
#ifdef DEBUG
      Serial.println("9,1,IDLE command received");
#endif
      alternatingWipe(brightWhite, bearCyan, 1);
      strip.show();

#ifdef DEBUG
      Serial.println();
#endif
      break;


    case RIO_msgs_enum::NO_COMMS:
#ifdef DEBUG
      Serial.println("9,1,NO_COMMS command received");
#endif

      // temp to have another color w/o animations
      // strip.clear();
      // strip.fill(greenReady);
      // strip.show();
      // break;
      
      
      // Fill along the length of the strip in various colors...
      rainbow(65);  //loss of comms


#ifdef DEBUG
      Serial.println();
#endif
      break;









    case RIO_msgs_enum::ELEVATOR_L1:
#ifdef DEBUG
      Serial.print("9,3,ELEVATOR_L1 command received");
#endif
      // Fill along the length of the strip in various colors...
         runningLights(matthewRed,55); 
#ifdef DEBUG
      Serial.println();
#endif
      break;



    case RIO_msgs_enum::ALGAE_HELD:
#ifdef DEBUG
      Serial.print("9,4,ALGAE_HELD command received");
#endif
      // Fill along the length of the strip in various colors...
      theaterChase(algaeAqua, 45);

#ifdef DEBUG
      Serial.println();
#endif
      break;



    case RIO_msgs_enum::ELEVATOR_L2:
#ifdef DEBUG
      Serial.print("9,5,ELEVATOR_L2 command received");
#endif
      // Fill along the length of the strip in various colors...
      radarSweep(purpleGem, 40, 3);

#ifdef DEBUG
      Serial.println();
#endif
      break;



    case RIO_msgs_enum::ELEVATOR_L3:
#ifdef DEBUG
      Serial.print("9,6,ELEVATOR_L3 command received");
#endif
      // Fill along the length of the strip in various colors...
  Strobe(superPink, 2, 50, 50);             

#ifdef DEBUG
      Serial.println();
#endif
      break;



    case RIO_msgs_enum::IDK:
#ifdef DEBUG
      Serial.print("9,7,IDK command received");
#endif
      // Fill along the length of the strip in various colors...
     //fireworks(superPink, 40, 50, 3,20); draft one
     snakeAnimation(piss, 31, 12);

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
  strip.setBrightness(100);
  strip.show();

  pinMode(PIN_EXTERNAL_BUTTON, INPUT_PULLUP);
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
// Helper Function to Dim a Color
uint32_t dimColor(uint32_t color, int factor) {
  // Extract the RGB values from the color
  uint8_t r = (color >> 16) & 0xFF;
  uint8_t g = (color >> 8) & 0xFF;
  uint8_t b = color & 0xFF;

  // Dim the RGB values by the factor
  r = max(0, r - factor);
  g = max(0, g - factor);
  b = max(0, b - factor);

  // Return the dimmed color
  return strip.Color(r, g, b);
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
        
      delay(wait);   // Pause for a moment
       strip.show();  // Update strip with new contents
       if (GetCommand()) {
      newCommand = true;
      break;
     }
   
     }
    }
  }
}

void alternatingChase(uint32_t color, uint32_t colorTwo, int wait) {
  for (int a = 0; a < 90; a++) {   // Repeat 90 times...
    for (int b = 0; b < 6; b++) {  // Loop for offset (6 steps in chase)
      strip.clear();               // Clear all pixels in RAM
      
      // Set pixels with alternating colors starting from offset 'b'
      for (int i = b; i < strip.numPixels(); i += 6) {
        strip.setPixelColor(i, color);          // Primary color for positions
      }
      for (int i = b + 3; i < strip.numPixels(); i += 6) {
        strip.setPixelColor(i, colorTwo);       // Secondary color for alternating positions
      }

      strip.show();  // Update strip to match
      delay(wait);   // Pause for a moment
    }
  }
}
void Strobe(uint32_t color, int StrobeCount, int FlashDelay, int EndPause) {
   while (!newCommand){
  for (int j = 0; j < StrobeCount; j++) {
    setAll(color); // Set all LEDs to the given color
   strip.show();   // Update the strip
    delay(FlashDelay);

    setAll(0);     // Turn off all LEDs
   strip.show();   // Update the strip
    delay(FlashDelay);
  }
  delay(EndPause); // Pause after the strobe effect
    if (GetCommand()) {
      newCommand = true;
      break;
     }
}
}
void fireworks(uint32_t color, int burstSize, int delayTime, int spread,int fadeSteps) {
  strip.clear();
   while (!newCommand){
  for (int i = 0; i < burstSize; i++) {
    int center = random(strip.numPixels());  // Random center pixel for the burst

    // Light up the center pixel and adjacent ones
    for (int offset = -spread; offset <= spread; offset++) {
      int pixel = (center + offset + strip.numPixels()) % strip.numPixels();  // Wrap around
      strip.setPixelColor(pixel, color);  // Set the pixel color
    }

    strip.show();
    delay(delayTime);

     for (int fade = 1; fade <= fadeSteps; fade++) {
      for (int offset = -spread; offset <= spread; offset++) {
        int pixel = (center + offset + strip.numPixels()) % strip.numPixels();
        uint8_t r = (strip.getPixelColor(pixel) >> 16) & 0xFF;
        uint8_t g = (strip.getPixelColor(pixel) >> 8) & 0xFF;
        uint8_t b = strip.getPixelColor(pixel) & 0xFF;

        // Dim the color gradually
        r = (r * (fadeSteps - fade)) / fadeSteps;
        g = (g * (fadeSteps - fade)) / fadeSteps;
        b = (b * (fadeSteps - fade)) / fadeSteps;
      strip.setPixelColor(pixel, 0);  // Turn off the pixel
    }
     }

    strip.show();
    delay(delayTime);
      if (GetCommand()) {
      newCommand = true;
      break;
     }
  }
  }
}



// Helper function to set all LEDs to the same color
void setAll(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
}
void runningLights(uint32_t color, int waveDelay) {
   while (!newCommand){
  for (int j = 0; j < strip.numPixels() * 2; j++) {
    for (int i = 0; i < strip.numPixels(); i++) {
      int intensity = int((sin(i + j * 0.3) * 127 + 128) / 255.0 * 255);
      uint32_t dimmedColor = strip.Color(
        (color >> 16 & 0xFF) * intensity / 255,
        (color >> 8 & 0xFF) * intensity / 255,
        (color & 0xFF) * intensity / 255
      );
      strip.setPixelColor(i, dimmedColor);
    }
    strip.show();
    delay(waveDelay);
    if (GetCommand()) {
      newCommand = true;
      break;
     }
  }
  }
}
void snakeAnimation(uint32_t color, int length, int delayTime) {
  static int head = 0;              // Position of the snake head
  static int direction = 1;         // Snake moves forward (+1) or backward (-1)

  while (!newCommand) {             // Continue until a new command is received
    strip.clear();

       // Draw the snake with a dimming tail
    for (int i = 0; i < length; i++) {
      int pixel = (head - i + strip.numPixels()) % strip.numPixels();
      
      // Calculate dimming factor for the tail
      float dimFactor = 1.0 - (float)i / length;  // Dim factor decreases from head to tail
      uint8_t r = ((color >> 16) & 0xFF) * dimFactor;
      uint8_t g = ((color >> 8) & 0xFF) * dimFactor;
      uint8_t b = (color & 0xFF) * dimFactor;

      strip.setPixelColor(pixel, strip.Color(r, g, b));
    }

    // Move the snake
    head += direction;

    // Handle boundaries
    if (head >= strip.numPixels() || head < 0) {
      direction = -direction;  // Reverse direction at the ends
      head += direction;
    }
      if (GetCommand()) {
      newCommand = true;
      break;
     }

    strip.show();
    delay(delayTime);
  }
}

void radarSweep(uint32_t color, int length, int speed) {
  static int position = 0;

  while (!newCommand) {
    strip.clear();
    for (int i = 0; i < length; i++) {
      int pixel = (position - i + strip.numPixels()) % strip.numPixels();
      float dimFactor = 1.0 - (float)i / length;
      uint8_t r = ((color >> 16) & 0xFF) * dimFactor;
      uint8_t g = ((color >> 8) & 0xFF) * dimFactor;
      uint8_t b = (color & 0xFF) * dimFactor;
      strip.setPixelColor(pixel, strip.Color(r, g, b));
    }

    strip.show();
     if (GetCommand()) {
      newCommand = true;
      break;
     }
    position = (position + 1) % strip.numPixels();  // Move the sweep
    delay(speed);
  }
}

