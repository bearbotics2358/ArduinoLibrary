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
#define LED_PIN 6
#define LED_COUNT 91
#define MAXLEN 546

char rx_buff[MAXLEN];
int rx_index = 0;

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, PIN_EXTERNAL_NEOPIXELS, NEO_GRB + NEO_KHZ800);

uint32_t colWhite = strip.Color(25, 25, 25);
uint32_t brightWhite = strip.Color(255, 255, 255);
uint32_t algaeAqua = strip.Color(139, 247, 204);
uint32_t greenReady = strip.Color(25, 155, 25);
uint32_t superPink = strip.Color(255, 20, 147); //(255,37,169) previously
uint32_t purpleGem = strip.Color(72, 32, 84);
uint32_t matthewRed = strip.Color(123, 10, 3); //(52,0,0) previously
uint32_t piss = strip.Color(190, 161, 4);
uint32_t bearCyan = strip.Color(25, 35, 215);
uint32_t off = strip.Color(0,0,0);

bool newCommand = false;

bool isClimbLeft = false;
bool isClimbRight = false;

int GetCommand() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx_index > 0) {
        rx_buff[rx_index] = '\0';
        Serial.print("Received: ");
        Serial.println(rx_buff);
        rx_index = 0;
        newCommand = true;
        return 1;
      }
    } else if (rx_index < MAXLEN - 1 && isDigit(c)) { // Only accept digits
      rx_buff[rx_index++] = c;
    } else {
        rx_index = 0; // Reset index if non-digit received
    }
  }
  return 0;
}

void ProcessCommand() {
  if (newCommand) {
    newCommand = false;
    Serial.print("Processing command: ");
    Serial.println(rx_buff);

    int msg_type = atoi(rx_buff);

    Serial.print("msg_type: ");
    Serial.println(msg_type);

    strip.clear();  // Clear previous animation

    switch (msg_type) {
      case RIO_msgs_enum::MSG_IDLE:
        Serial.println("9,1,IDLE command received");
        alternatingWipe(brightWhite, bearCyan, 1);
        break;

      case RIO_msgs_enum::NO_COMMS:
        Serial.println("9,2,NO_COMMS command received");
        rainbow(65); //loss of comms
        break;

      case RIO_msgs_enum::ELEVATOR_L1:
        Serial.println("9,3,ELEVATOR_L1 command received");
        runningLights(matthewRed, 55);
        break;

      case RIO_msgs_enum::ELEVATOR_L3_ALGAE:
        Serial.println("9,4,ELEVATOR_L3_ALGAE command received");
        theaterChase(algaeAqua, 45);
        break;

      case RIO_msgs_enum::ELEVATOR_L2:
        Serial.println("9,5,ELEVATOR_L2 command received");
        radarSweep(purpleGem, 40, 3);
        break;

      case RIO_msgs_enum::ELEVATOR_L3:
        Serial.println("9,6,ELEVATOR_L3 command received");
        Breathing(100, superPink, 25, 50);
        break;

      case RIO_msgs_enum::IDK:
        Serial.println("9,7,IDK command received");
        snakeAnimation(piss, 31, 12);
        break;

      case RIO_msgs_enum::CLIMBLEFTTRUE:
        Serial.println("9,8, climber left on command received");
        isClimbLeft = true;
        climberHalves(greenReady, off, 37, 75, isClimbLeft, isClimbRight);
        break;

      case RIO_msgs_enum::CLIMBLEFTFALSE:
        Serial.println("9,9, climber left off command received");
        isClimbLeft = false;
        climberHalves(greenReady, off, 37, 75, isClimbLeft, isClimbRight);
        break;

      case RIO_msgs_enum::CLIMBRIGHTTRUE:
        Serial.println("9,10, climber right on command received");
        isClimbRight = true;
        climberHalves(greenReady, off, 37, 75, isClimbLeft, isClimbRight);
        break;

      case RIO_msgs_enum::CLIMBRIGHTFALSE:
        Serial.println("9,11, climber right off command received");
        isClimbRight = false;
        climberHalves(greenReady, off, 37, 75, isClimbLeft, isClimbRight);
        break;

      case RIO_msgs_enum::CAMERA_RING:
        Serial.println("9,12, camera ring on command received");
        cameraRing(brightWhite, 75, 91, 75);
        break;

      case RIO_msgs_enum::CAMERA_RING_OFF:
        Serial.println("9,13, camera ring off command received");
        cameraRing(off, 75, 91, 75);
        break;

      default:
        Serial.println("9,0,unknown command");
        strip.fill(strip.Color(255, 0, 0));  // Set red for unknown command
        break;
    }

    strip.show();  // Ensure changes are displayed after each command
    Serial.println("LED update complete");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Serial communication initialized");
  delay(100);

  pinMode(PIN_EXTERNAL_POWER, OUTPUT);
  digitalWrite(PIN_EXTERNAL_POWER, HIGH);

  strip.begin();
  strip.setBrightness(100);
  strip.show();
  pinMode(PIN_EXTERNAL_BUTTON, INPUT_PULLUP);
}

void loop() {
  GetCommand();
  ProcessCommand();
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
      int pixel = (head - i * direction + LED_COUNT) % LED_COUNT;
      
      // Calculate dimming factor for the tail
      float dimFactor = 1.0 - (float)i / length;  // Dim factor decreases from head to tail
      uint8_t r = ((color >> 16) & 0xFF) * dimFactor;
      uint8_t g = ((color >> 8) & 0xFF) * dimFactor;
      uint8_t b = (color & 0xFF) * dimFactor;

      strip.setPixelColor(pixel, r, g, b);
    }

    // Move the snake
    head = (head + direction + LED_COUNT) % LED_COUNT;

    // Handle boundaries
    if (head == LED_COUNT - 1 || head == 0) {
      direction = -direction;  // Reverse direction at the ends
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

void partialLighting( uint32_t color, int startPos, int endPos, int wait){
  while (!newCommand){
  for (int i = startPos; i <= endPos; i++){
    strip.setPixelColor(i, color);
    }

    strip.show();
    if (GetCommand()) {
      newCommand = true;
      break;
    }

    delay(wait);
  }
}

//after some testing, updated climber LED function

void climberHalves(uint32_t color, uint32_t falseColor, int halfway, int wait, bool left, bool right){
  while(!newCommand){
    if((left) && (right)){
      colorWipe(color, 10);
    }
    else if(left){
      partialLighting(color, 38, 75, wait);
    }
    else if(right){
      partialLighting(color, 0, halfway, wait);
    }
    else if((!left) && (!right)){
      colorWipe(falseColor, 10);
    }

    if (GetCommand()) {
      newCommand = true;
      break;
    }
    delay(wait);
  }
}


void Breathing(int wait, uint32_t colorOne, int bpm, int currentBrightness) {
  int direction = -1;  // Start by dimming (-1), will switch to brightening (+1)

  while (!newCommand) {  // Infinite loop for continuous breathing
    currentBrightness += (bpm * direction); // Adjust brightness

    // Reverse direction when brightness hits limits
    if (currentBrightness <= 0) {
      currentBrightness = 0;
      direction = 1;  // Switch to increasing brightness
    } 
    else if (currentBrightness >= 255) {
      currentBrightness = 255;
      direction = -1; // Switch to decreasing brightness
    }
    // Apply brightness and color to the entire strip
    strip.setBrightness(currentBrightness);
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, colorOne);
    }
    strip.show();
    if (GetCommand()) {
      newCommand = true;
      break;
    }
    delay(wait);  // Control breathing speed
  }
}

void cameraRing(uint32_t color, int start, int end, int wait){
  while (!newCommand){
    for(int i = start; i < end; i++) {
      strip.setPixelColor(i, color);
    }
    strip.update();
    if (GetCommand()) {
      newCommand = true;
      break;
    }
    delay(wait);
  }
}