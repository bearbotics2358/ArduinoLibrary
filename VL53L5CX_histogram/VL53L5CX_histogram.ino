/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to get all 64 pixels, at 15Hz, comma seperated output.
  This is handy for transmission to visualization programs such as Processing.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_DotStar.h>
#include <string.h>

#define NUMPIXELS 4 // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define DATAPIN    11
#define CLOCKPIN   13
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);


#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; // Used to pretty print output
int imageWidth = 0;      // Used to pretty print output

long measurements = 0;         // Used to calculate actual output rate
long measurementStartTime = 0; // Used to calculate actual output rate

int histogram[11];

void setColor(int index, int val) {
  // index is the index of the LED, from 0 to 3
  // val is the histogram count
  if((val < 10)) { // 0xGGRRBB (color values)
    // Serial.print("red; not enough, too far");
    strip.setPixelColor(index, 0x005000);
  } else if(val < 20) { 
    // Serial.print("yellow; close");
    strip.setPixelColor(index, 0x505000);  
  } else {
    // Serial.print("green; just right");
    strip.setPixelColor(index, 0x500000); 
  }
}

void setup()
{
  int i;
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  strip.begin(); 
  // initialize LEDs to blue
  for(i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 0x000050);      
  }
  strip.show();                     // Refresh strip

  Wire.begin(); // This resets I2C bus to 100kHz
  Wire.setClock(1000000); //Sensor has max I2C freq of 1MHz

  // myImager.setWireMaxPacketSize(128); // Increase default from 32 bytes to 128 - not supported on all platforms

  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");

  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1)
      ;
  }

  myImager.setResolution(8 * 8); // Enable all 64 pads
  // myImager.setResolution(4 * 4); // Enable all 64 pads

  imageResolution = myImager.getResolution(); // Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution);         // Calculate printing width

  // Using 4x4, min frequency is 1Hz and max is 60Hz
  // Using 8x8, min frequency is 1Hz and max is 15Hz
  myImager.setRangingFrequency(15);

  Serial.print("Sharpener(%): ");
  Serial.println(myImager.getSharpenerPercent());

  myImager.setSharpenerPercent(99);

  Serial.print("Sharpener(%): ");
  Serial.println(myImager.getSharpenerPercent());

  myImager.startRanging();

  measurementStartTime = millis();
}

void loop()
{
  int i;
  char stemp[256];
  char smsg[256];
  
  // Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) // Read distance data into array
    {
      for(i = 0; i < 11; i++) {
        histogram[i] = 0;
      }
      
      // The ST library returns the data transposed from zone mapping shown in datasheet
      // Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth)
      {
        for (int x = imageWidth - 1; x >= 0; x--)
        {
          for(i = 0; i < 11; i++) {
            if((i * 100 <= measurementData.distance_mm[x + y]) 
              && (measurementData.distance_mm[x + y] < (i + 1) * 100)){
              histogram[i]++;
            }
            if((i == 10) && (1000 <= measurementData.distance_mm[x + y])) { 
              histogram[i]++;
            }
          }
          // Serial.print(measurementData.distance_mm[x + y]);
          // Serial.print(",");
        }
      }

      strcpy(smsg, "2,11,");    
      for(i = 0; i < 11; i++) {
        // Serial.print(histogram[i]);
        sprintf(stemp, "%d", histogram[i]);
        strcat(smsg, stemp);
        if(i < 10) {
          // Serial.print(",");
          strcat(smsg, ",");
        } else {
          strcat(smsg, "\r\n");
        }
      }
      // Serial.println();
      Serial.print(smsg);
      
      // light some leds
      setColor(0, histogram[3]); // 300 mm <= dist < 400 mm
      setColor(1, histogram[4]); // 400 mm <= dist < 500 mm
      setColor(2, histogram[5]); // 500 mm <= dist < 600 mm
      setColor(3, histogram[6]); // 600 mm <= dist < 700 mm
      strip.show();                     // Refresh strip

      // Uncomment to display actual measurement rate
      // measurements++;
      // float measurementTime = (millis() - measurementStartTime) / 1000.0;
      // Serial.print("rate: ");
      // Serial.print(measurements / measurementTime, 3);
      // Serial.println("Hz");
    }
  }

  delay(100); // Small delay between polling
}
