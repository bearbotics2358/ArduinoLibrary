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

#define MAXLEN 256

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; // Used to pretty print output
int imageWidth = 0;      // Used to pretty print output

long measurements = 0;         // Used to calculate actual output rate
long measurementStartTime = 0; // Used to calculate actual output rate

char rx_buff[MAXLEN];
int rx_index = 0;

int histogram[11];
int histogram_f = 0;
int cone_f = target_type_enum::CONE; 
int raw_pixel_data_f = 0;

//ok to change this tolerace over distance tolerance?
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
      Serial.print("9,1,cmd = "); //bugging 3-9
      Serial.println(rx_buff); //this must be important

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
    case RIO_TOF_msgs_enum::TARGET_TYPE: //this is what we care about
      Serial.print("9,1,TARGET_TYPE command received : "); //3-9 bugs
      if(data_len) {
        cone_f = atoi(strtok(NULL, ","));
        Serial.print(cone_f ? "CUBE" : "CONE");
      }
      Serial.println();
      break;

    case RIO_TOF_msgs_enum::HISTOGRAM_ENABLE:
      if(data_len) {
        histogram_f = atoi(strtok(NULL, ","));
      }
      break;
    
    case RIO_TOF_msgs_enum::RAW_PIXEL_DATA_ENABLE:
      if(data_len) {
        raw_pixel_data_f = atoi(strtok(NULL, ","));
      }
      break;
    
    default:
      Serial.println("9,1,unknown command");
  }
}

void send_histogram()
{
  char stemp[MAXLEN];
  char smsg[MAXLEN];
  int i;
  
  // copy msg_type and data_length into message
  sprintf(smsg, "%d,11,", TOF_RIO_msgs_enum::HISTOGRAM);  //sends a message, dont change  
  for(i = 0; i < 11; i++) {
    sprintf(stemp, "%d", histogram[i]);
    strcat(smsg, stemp);
    if(i < 10) {
      //Serial.print(",");
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
  char stemp[MAXLEN];
  char smsg[MAXLEN];
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
  Serial.println("9,1,SparkFun VL53L5CX Imager Example");

  // clear rx_buff
  bzero(rx_buff, MAXLEN);

  strip.begin(); 
  // initialize LEDs to blue
  for(i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 0x000050);      
  }
  strip.show();                     // Refresh strip

  Wire.begin(); // This resets I2C bus to 100kHz
  Wire.setClock(1000000); //Sensor has max I2C freq of 1MHz

  // myImager.setWireMaxPacketSize(128); // Increase default from 32 bytes to 128 - not supported on all platforms

  Serial.println("9,1,Initializing sensor board. This can take up to 10s. Please wait.");

  if (myImager.begin() == false)
  {
    Serial.println(F("9,1,Sensor not found - check your wiring. Freezing"));
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

  Serial.print("9,1,Sharpener(%): ");
  Serial.println(myImager.getSharpenerPercent());

  myImager.setSharpenerPercent(99);

  Serial.print("9,1,Sharpener(%): ");
  Serial.println(myImager.getSharpenerPercent());

  myImager.startRanging();

  measurementStartTime = millis();
}

void loop()
{
  int i;
  char stemp[MAXLEN];
  int first_pixel = 1;
  
  // Get and process command
  if(GetCommand()) {
    ProcessCommand();
  }
  
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
          if(raw_pixel_data_f) {
            // print raw pixel data message
            if(first_pixel) {
              first_pixel = 0;
              // print msg header
              sprintf(stemp, "%d,%d,", TOF_RIO_msgs_enum::RAW_PIXEL_DATA, imageWidth * imageWidth);
              Serial.print(stemp);
            }
            //two lines below previously uncommented            
            Serial.print(measurementData.distance_mm[x + y]); // actually, prints out raw pixel data
            if((y == imageWidth * (imageWidth - 1)) && (x == 0)) {
              // last pixel has been sent
              Serial.println();
            } else {
              Serial.print(",");
            }
            
          }
          //--cone
          // 1st element - 0-5
          // 2nd element - 10-20
          // 3rd element - 40-50 
          //--no cone
          // 1st element - 0
          // 2nd element - 0
          // 3rd element - 10-20
        }
      }

      // send histogram message
      if(histogram_f) {
        send_histogram();
      }
      send_range();
      
      // light some leds
      setColor(0, histogram[1]); // 300 mm <= dist < 400 mm
      setColor(1, histogram[2]); // 400 mm <= dist < 500 mm
      setColor(2, histogram[3]); // 500 mm <= dist < 600 mm
      //setColor(3, histogram[4]); // 600 mm <= dist < 700 mm
      setColor(3, histogram[4]); //cone or no cone
      strip.show();                     // Refresh strip
      //originally 3, 4, 5, 6

      //radius of cone @ 4 in above ground is 2.5 in
      //length of arm (sensor to center of cymbals) is 10 in
      //sensor needs to be at 7.5 inch line on a ruler

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
