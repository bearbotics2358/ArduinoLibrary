/*
	working on creating FeatherCAN, identical software in all FeatherCAN's on the 2025 'bot

  sercom3 is the primary I2C interface


  planning to use sercom2 (2.0 and 2.1 specifically) for 2nd I2C interface - note this precludes using the flash on the bottom of the board
  rather avoid this to allow use of on board flash

  For information on creating a 2nd I2C interface on a SAMD21, see:
  https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-wire

  Looking at the guide, they use sercom1, which maybe is available? Looking at variant.h, it is for EDBG/SPI - what is this?
  We don't need this
  So switching to investigate sercom1
  So this would use pins D11 and D13. But D13 is tied ot the onboard LED
  So NO

  Try sercom5 next, it is also EDBG
  Pins are not available on Feather connector

  Sercom0 pins 0 & 1 look available (pads 2 & 3 are used for UART)
  These route to pins A3 and A4

  Pin  Arduino  SERCOM    
        'Pin'
  -----------------------------------------
  PA4   A3      SERCOM0.0 SDA BLU
  PA5   A4      SERCOM0.1 SCL YEL
  

	REV_encoder_CAN

  This program is for a FeatherCAN board. It was created from Talon_Send_test.
    
  The program:
  - reads the Absolute output from the REV Through Bore Encoder
  REV-11-1271, which is a pulse with a width from 1 usec to 1024 usec,
  representing 0 to 360 degrees
  - converts it to an angle
  - creates a CAN message containing a 16 bit integer that is
  10 x angle in degrees
  - sends the CAN message to the RoboRIO

  Created from Talon_Send_test
  created 3/3/24 BD

*/

// Controlling defines
#define CAN_ENABLED 1
#define PRINT_VALUES 1


#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function
#include <mcp_can.h>
#include <SPI.h>

#include "config.h"
#include "initialization.h"

#include "SAMD21SerialNumber.h"
#include "ThruBoreEncoder.h"
#include "ColorSensorV3.h"
#include "Adafruit_VL53L1X.h"

#include "PackMessage.h"

TwoWire myWire(&sercom0, A3, A4); // 2nd I2C interdace

SAMD21SerialNumber sn;

#define COLOR_SENSOR_MAX 2
rev::ColorSensorV3 cs[COLOR_SENSOR_MAX];

// Bug in VL53L1X driver, cannot not set pins - it should be ok, but it locks up in init if not set
// we don't actually use these functions, and they are not normally wired up anyway
#define IRQ_PIN 1
#define XSHUT_PIN 2

#define TOF_SENSOR_MAX 3
Adafruit_VL53L1X vl53[] = {
  Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN), 
  Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN), 
  Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN)
};

#include <Adafruit_NeoPixel.h>


// CAN0 RESET, INT
#define CAN0_RST 31   // MCP25625 RESET connected to Arduino pin 31
#define CAN0_INT 30  // Set INT to Arduino pin 30

#define NUMPIXELS 2 // Number of LEDs in strip
#define NEO_PIN 9 // NOTE: will have to confirm

// CAN variables
unsigned long t_prev = 0;  // Variable to store last execution time
const unsigned int t_intvl = 10;  // Transmit interval in milliseconds
// const unsigned int t_intvl = 2000;  // Transmit interval in milliseconds

// CAN data to send - init w/ bogus data
byte data[] = {0xAA, 0x55, 0x01, 0x10, 0xFF, 0x12, 0x34, 0x56};

// for CAN receiver
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

#define CAN_ID 0x0A080101

// Create CAN interface using MCP chip
MCP_CAN CAN0(8); // Set MCP25625 CS to Arduino pin 8

int ledPin = 13;      // select the pin for the LED

int board = -1; // will be used to determine which board we are using

// trying to get motor to run
int tx_msg_count = 0;

Adafruit_NeoPixel strip(NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);


// Until TBE is a class:
// angle of encoder
extern float angle_f;

uint32_t proximity[COLOR_SENSOR_MAX];

int16_t distance[TOF_SENSOR_MAX];

void CAN_setup() {
  // CAN chip RESET line
  pinMode(CAN0_RST, OUTPUT);
  // Configuring pin for /INT input
  pinMode(CAN0_INT, INPUT_PULLUP);
  
  // reset CAN chip
  digitalWrite(CAN0_RST, 0);
  delay(100);
  digitalWrite(CAN0_RST, 1);
  delay(500);  

  // Initialize MCP25625 running at 16MHz with a baudrate of 1Mb/s and
  // the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("MCP25625 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP25625...");
  }

  CAN0.setMode(MCP_NORMAL);
}

void Color_sensor_setup() {
  int i;

  for(int i = 0; i < COLOR_SENSOR_MAX; i++) {
    proximity[i] = 0;
  }
  
  // Setup Color Sensors
  if(conf[board].color_sensor_qty) {
    // select I2C bus that Color Sensor is connected to
    for(i = 0; i < conf[board].color_sensor_qty; i++) {
      if(conf[board].color_sensor_bus[i] == 1) {
        cs[i].setWire(&Wire);
      } else if(conf[board].color_sensor_bus[i] == 2) {
        cs[i].setWire(&myWire);
      } else {
        Serial.print("CONFIGURATION ERROR: color_sensor_bus is not valid: ");
        Serial.println(conf[board].color_sensor_bus[i]);
        while(1) {}
      }
  
      while(!cs[i].CheckDeviceID()) {
        Serial.print("Device ID NOT ok for device ");
        Serial.println(i);
        delay(1000);
      }

      cs[i].InitializeDevice();

      // Configure IR LED
      cs[i].ConfigureProximitySensorLED(
        rev::ColorSensorV3::LEDPulseFrequency::k100kHz, rev::ColorSensorV3::LEDCurrent::kPulse100mA, 128);

      Serial.print("MainCtrl after Initialize: 0x");
      cs[i].Read(rev::ColorSensorV3::Register::kMainCtrl, 1, data);
      Serial.println(data[0], HEX);
  
      // Clear the reset flag
      Serial.print("HasReset(): ");
      Serial.println(cs[i].HasReset());
    }
  }

}

void Color_sensor_loop()
{
  uint8_t data[16];
  uint8_t status = 0;
  uint32_t color1 = 0;
  uint32_t red_ch = 0;
  uint32_t green_ch = 0;
  uint32_t blue_ch = 0;
  uint8_t index = 0;
  int i;
  
  for(i = 0; i < conf[board].color_sensor_qty; i++) {
    // wait for Proximity Sensor Ready (and Light Sensor Ready)
    do {
      cs[i].Read(rev::ColorSensorV3::Register::kMainStatus, 1, data);
      status = data[0];
      
      // Serial.print("\nMainStatus: 0x");
      // Serial.print(status, HEX);
      delay(10);
    } while ((status & 1) != 1); // 1 for PS, 9 for LS & PS
  
    Serial.print("MainStatus: 0x");
    Serial.print(status, HEX);

    proximity[i] = cs[i].GetProximity();
    Serial.print("  Proximity: ");
    Serial.print(proximity[i]);
  
    /*
    color1 = cs.GetColor();
    Serial.print("  Color: 0x");
    Serial.print(color1, HEX);
    Serial.print(" , ");
    Serial.print((color1 >> 16) & 0x00ff);
    Serial.print(" , ");
    Serial.print((color1 >> 8) & 0x00ff);
    Serial.print(" , ");
    Serial.print(color1 & 0x00ff);
    */
  
    // manually get colors
    /*
    // individually
    cs.Read(rev::ColorSensorV3::Register::kDataRed, 3, data);
    color1 = ((data[2] << 16) | (data[1]) | data[0]) & 0x0FFFFF;
    Serial.print(" Red: ");
    Serial.print(color1);
  
    cs.Read(rev::ColorSensorV3::Register::kDataGreen, 3, data);
    color1 = ((data[2] << 16) | (data[1]) | data[0]) & 0x0FFFFF;
    Serial.print(" Green: ");
    Serial.print(color1);
  
    cs.Read(rev::ColorSensorV3::Register::kDataBlue, 3, data);
    color1 = ((data[2] << 16) | (data[1]) | data[0]) & 0x0FFFFF;
    Serial.print(" Blue: ");
    Serial.print(color1);
    */
  
    /*
    // get colors all at once
    cs.Read(rev::ColorSensorV3::Register::kDataInfrared, 12, data);
  
    // Red
    index = 9;
    red_ch = ((data[index + 2] << 16) | (data[index + 1]) | data[index]) & 0x0FFFFF;
    Serial.print(" Red: ");
    Serial.print(red_ch);
  
    // Green
    index = 3;
    green_ch = ((data[index + 2] << 16) | (data[index + 1]) | data[index]) & 0x0FFFFF;
    Serial.print(" Green: ");
    Serial.print(green_ch);
  
    // Blue
    index = 6;
    blue_ch = ((data[index + 2] << 16) | (data[index + 1]) | data[index]) & 0x0FFFFF;
    Serial.print(" Blue: ");
    Serial.print(blue_ch);
    */
    
    
    /*
    Serial.print("IR: ");
    Serial.println(cs.GetIR());
    
    Serial.print("MainStatus: 0x");
    cs.Read(rev::ColorSensorV3::Register::kMainStatus, 1, data);
    Serial.println(data[0], HEX);
    */
    
    Serial.println();
  }
}

void TOF_sensor_setup() {
  int i;
  int ret = 0;
  
  // Setup TOF Sensors
  if(conf[board].TOF_qty) {
    if(conf[board].TOF_qty <= 2) {
      // For directly attached to I2C busses on the Feather
      // select I2C bus that TOF Sensor is connected to
      for(i = 0; i < conf[board].TOF_qty; i++) {
        // works every time with the print statements
        // but not without
        // try a delay instead - nope, even 10 seconds didn't work
        Serial.print("about to 'begin' unit ");
        Serial.print(i);
        Serial.print(" on bus ");
        Serial.println(conf[board].TOF_bus[i]);
        // delay(10000);

        
        if(conf[board].TOF_bus[i] == 1) {
          ret = vl53[i].begin(0x29, &Wire);
        } else if(conf[board].TOF_bus[i] == 2) {
          ret = vl53[i].begin(0x29, &myWire);
        }
        
        if(!ret) {
          Serial.print(F("Error on init of VL sensor: "));
          Serial.println(vl53[i].vl_status);
          while (1)       delay(10);
        }
        Serial.println(F("VL53L1X sensor OK!"));
      
        Serial.print(F("Sensor ID: 0x"));
        Serial.println(vl53[i].sensorID(), HEX);
      
        if (! vl53[i].startRanging()) {
          Serial.print(F("Couldn't start ranging: "));
          Serial.println(vl53[i].vl_status);
          while (1)       delay(10);
        }
        Serial.println(F("Ranging started"));
      
        // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
        vl53[i].setTimingBudget(50);
        Serial.print(F("Timing budget (ms): "));
        Serial.println(vl53[i].getTimingBudget());
      }
    } else {
      // all TOF sensors are on I2C mux
      // do necessary setup here
    }
  }
}

void TOF_sensor_loop() {
  int i;
  
  // Read TOF Sensors
  if(conf[board].TOF_qty) {
    if(conf[board].TOF_qty <= 2) {
      // For directly attached to I2C busses on the Feather
      // select I2C bus that TOF Sensor is connected to
      for(i = 0; i < conf[board].TOF_qty; i++) {
        if (vl53[i].dataReady()) {
          // new measurement for the taking!
          distance[i] = vl53[i].distance();
          if (distance[i] == -1) {
            // something went wrong!
            Serial.print(F("Couldn't get distance: "));
            Serial.println(vl53[i].vl_status);
            return;
          }
          Serial.print(F("Distance: "));
          Serial.print(distance[i]);
          Serial.println(" mm");
      
          // data is read out, time for another reading!
          vl53[i].clearInterrupt();
        }
      }
    } else {
      // all TOF sensors are on I2C mux
      // do necessary setup here
    }
  }
}




void setup() {

  int i;

  Serial.begin(115200);

  // wait for serial port connection
  // while(!Serial);

  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);

  delay(5000);

  Serial.println("in setup ...");

  digitalWrite(ledPin, 1);
  
  Wire.begin();
  // Wire.setClock(10000);

  myWire.begin();

  // Assign pins A3 & A4 to SERCOM functionality for 2nd I2C bus
  pinPeripheral(A3, PIO_SERCOM_ALT);
  pinPeripheral(A4, PIO_SERCOM_ALT);

	// Init Thru Bore Encoder if appropriate

	// Init CAN
  CAN_setup();
  
  Serial.println("REV Encoder test to CAN");
  
  // delay(500);

  digitalWrite(ledPin, 1);

	// Decide what we want LEDs to to at powerup
	
  strip.begin(); 
  // LEDs RED at powerup, until board is discovered
  for(i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 0x005000);      
  }
  strip.show();                     // Refresh strip




  initialize_config();

  // determine which board based on SAMD21 serial number
  do {
    // Read FeatherCAN SAMD21 uP serial number
    sn.read();
    sn.print();
    for(int i = 0; i < NUM_OF_CONFIG; i++) {
      if(sn.check(conf[i].sn)) {
        board = i;
        break;
      }
    }
    if(board < 0) {
      // board not found
      // blink leds
      for(i = 0; i < NUMPIXELS; i++) {
        strip.setPixelColor(i, 0x000000);      
      }
      strip.show();
      delay(500);

      for(i = 0; i < NUMPIXELS; i++) {
        strip.setPixelColor(i, 0x005000);      
      }
      strip.show();
      delay(500);
    }
  } while(board < 0);

  Serial.print("Board config: ");
  Serial.println(board);

  // Handle any Color Sensors
  if(conf[board].color_sensor_qty) {
    Color_sensor_setup();
  }

  Serial.println("about to check TOF");
  delay(1000);
  // Handle any TOF Sensors
  if(conf[board].TOF_qty) {
    TOF_sensor_setup();
  }
  
  delay(2000);




#if 0
  // need to update to board types we plan to have
  // are Coral and Climber and Belly Pan using the same message type?
  if(conf[board].type == TIMEOFFLIGHT) {
    //Time of Flight Stuff 
    Serial.print("TIME OF FLIGHT MODE:");
    Serial.println("Adafruit VL53L0X test");
    pinMode(TOF_SHUTDOWN, OUTPUT);
    digitalWrite(TOF_SHUTDOWN, 0); // disables 2nd sensor
    delay(100);
    if (!lox0.begin(TOF_ADDRESS)) {
      Serial.println(F("Failed to boot VL53L0X 0"));
    } else {
      tofExists[0] = 1;
    }
    Serial.println("About to set TOF_SHUTDOWN pin to 1");
    digitalWrite(TOF_SHUTDOWN, 1);
    Serial.println(" pin is set now");
    if (!lox1.begin()) {
      Serial.println(F("Failed to boot VL53L0X 1"));
    } else {
      tofExists[1] = 1;
    }
    // power 
    Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
  }
  else {
    Serial.println("SHOOTER MODE:");
  }

#endif // #if 0

  // initialize LEDs to blue
  for(i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 0x000050);      
  }

  // set LED corresonding to board number to green
  strip.setPixelColor(board, 0x500000);
  strip.show();                     // Refresh strip
  delay(1000);

  // initialize LEDs to blue
  for(i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 0x000050);      
  }

	
  Serial.print("CAN ID: 0x");
  Serial.print((conf[board].canId[0] >> 24) & 0x00ff, HEX);
  Serial.print(" ");
  Serial.print((conf[board].canId[0] >> 16) & 0x00ff, HEX);
  Serial.print(" ");
  Serial.print((conf[board].canId[0] >> 8) & 0x00ff, HEX);
  Serial.print(" ");
  Serial.print((conf[board].canId[0] ) & 0x00ff, HEX);
  Serial.println();





  // for Thru Bore Encoder
  // initialize pulse capture
  init_timer_capture();

  // delay before sending CAN messages
  for(i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, 0);
    delay(500);
    digitalWrite(LED_BUILTIN, 1);
    delay(500);
  }

}

void loop() {
  byte sndStat = 0;
  
#if CAN_ENABLED
  // empty RX buffer
  byte ret = CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
  if(ret == CAN_OK) {
    // Serial.println("Message received");
  }
#endif

  if(millis() < t_prev + t_intvl) {
    return;
  }
  t_prev = millis();



#if 0
  if(conf[board].type == TIMEOFFLIGHT) {
    // Time of Flight Stuff
    VL53L0X_RangingMeasurementData_t measure;
    
    if(tofExists[0]) {
      // Serial.print("Reading a measurement (sensor 0)... ");
      lox0.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

      if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        tofDistance[0] = measure.RangeMilliMeter;
        // Serial.print("Distance (mm): ");
        // Serial.println(measure.RangeMilliMeter);
      } else {
        tofDistance[0] = OUT_OF_RANGE;
        // Serial.println(" out of range ");
      }
    }
    
    if(tofExists[1]) {
      // Serial.print("Reading a measurement (sensor 1)... ");
      lox1.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

      if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        tofDistance[1] = measure.RangeMilliMeter;
        // Serial.print("Distance (mm): ");
        // Serial.println(measure.RangeMilliMeter);
      } else {
        tofDistance[1] = OUT_OF_RANGE;
        // Serial.println(" out of range ");
      }
    }
  }
  
  // pack message for protocol from Feather CAN to RoboRio
  if(conf[board].type == TIMEOFFLIGHT) {
    packMsgTOF();
  }
  else {
    packMsgShooter();
  }

#endif // #if 0






	// Check Thru Bore Encoder
  // read the pulse length from the encoder:

  // Serial.println("About to call TBE_loop()");
  // delay(2000);

  TBE_loop();

  // the above is working!!!!
  // need to fetch the values and do something with them

  Serial.print("Angle: ");
  Serial.print(angle_f);
  Serial.print(" ");

  // Handle any Color Sensors
  if(conf[board].color_sensor_qty) {
    Color_sensor_loop();

    // for now, print the first one
    Serial.print(" prox: ");
    Serial.print(proximity[0]);
  }

  // Handle any Time of Flight Sensors
  TOF_sensor_loop();
  // distance values print in above function

  Serial.println();
  
  // packAngleMsg(angle_f);

  // pack message for protocol from Feather CAN to RoboRio
  if(conf[board].type == CORAL) {
    packCoralMsg(angle_f, proximity[0]);
  } else if(conf[board].type == ALGAE) {
    packAlgaeMsg(angle_f, distance[0]);
  }

#if CAN_ENABLED
  
  // send Extended msg
  // sndStat = CAN0.sendMsgBuf(CAN_ID, 1, 8, data); // ITS THIS ONE!! :)
  sndStat = CAN0.sendMsgBuf(conf[board].canId[0], 1, 8, data); 


#endif // #if CAN_ENABLED


#if PRINT_VALUES
  if(sndStat == CAN_OK) {
    Serial.print("Message Sent Successfully!");
  } else {
    Serial.print("Error Sending Message... sndStat: 0x");
    Serial.print(sndStat, HEX);
    INT8U ret = CAN0.getError();
    Serial.print(" MCP_EFLG: 0x");
    Serial.print(ret, HEX);
  }

  // debug printouts
  Serial.print(" TEC: ");
  Serial.print(CAN0.errorCountTX());

  Serial.print(" REC: ");
  Serial.print(CAN0.errorCountRX());
  
  Serial.println();
#endif

 



  // delay(2000);

  delay(10);
  tx_msg_count++;
}
