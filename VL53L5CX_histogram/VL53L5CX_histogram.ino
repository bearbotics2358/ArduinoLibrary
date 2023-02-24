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

// using TOF_protocol version 1.0

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_DotStar.h>
#include <string.h>
#include "TOF_protocol.h"

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
int histogram_f = 1;
int cone_f = 1; // 1 means cone, 0 means cube

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

void send_histogram()
{
  char stemp[256];
  char smsg[256];
  int i;
  
  // copy msg_type and data_length into message
  sprintf(smsg, "%d,11,", TOF_RIO_msgs_enum::HISTOGRAM);    
  for(i = 0; i < 11; i++) {
    sprintf(stemp, "%d", histogram[i]);
    strcat(smsg, stemp);
    if(i < 10) {
      // Serial.print(",");
      strcat(smsg, ",");
    } else {
      strcat(smsg, "\r\n");
    }
  }
  Serial.print(smsg);
}

// calculate range by calculating "center of mass" of histogram buckets
int calc_range(int start, int end)
{
  int i;
  int total = 0;
  float range = 0;

  // total number of pixels observed in this group
  for(i = start; i <= end; i++) {
    total += histogram[i];
  }

  // determine center of mass
  // add up "distance x mass" for each histogram bucket
  for(i = start; i <= end; i++) {
    range += histogram[i] * i;
  }
  // divide by total to get center of mass
  range /= total;
  range *= 100;

  return int(range + 0.5);
}

// determine if in target is in range and calculate range
// send resulting range message
void send_range()
{
  char stemp[256];
  char smsg[256];
  int i;
  int hist_far = 0;
  int hist_close = 0;
  int hist_in_range = 0;
  int target_range_determination = 0;
  int target_range = 0;

  // for now, test code, please update
  // hist[0-3]  pixels < 400 mm
  // hist[4-5]  400 mm < pixels < 600 mm
  // hist[6-10] 600 mm < pixels

  for(i = 0; i < 11; i++) {
    if(i <= 3) {
      hist_close += histogram[i];
    } else if(i <= 5) {
      hist_in_range += histogram[i];
    } else {
      hist_far += histogram[i];
    }
  }

  if(hist_in_range > 30) {
    target_range_determination = target_range_enum::TARGET_IN_RANGE;
    target_range = calc_range(4,5);
  } else if(hist_close > 30) {
    target_range_determination = target_range_enum::TARGET_TOO_CLOSE;
    target_range = calc_range(0,3);
  } else {
    target_range_determination = target_range_enum::TARGET_TOO_FAR;
    target_range = calc_range(6,10);
  }

  sprintf(smsg, "%d,2,%d,%d\r\n", TOF_RIO_msgs_enum::RANGE, target_range_determination, target_range);
  Serial.print(smsg);
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

      // send histogram message
      if(histogram_f) {
        send_histogram();
      }
      send_range();
      
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
