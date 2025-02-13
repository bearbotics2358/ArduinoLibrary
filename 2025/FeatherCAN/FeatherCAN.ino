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

#include "PackMessage.h"

TwoWire myWire(&sercom0, A3, A4); // 2nd I2C interdace

SAMD21SerialNumber sn;

#include <Adafruit_NeoPixel.h>


/*
 * TO DO's:
 * - add NeoPixel library, strip, and initialization code
 * 
 * 
 * 
 * 
 * 
 * 
 */


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
byte tx_test_data[] = {0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00};

// for receiver
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




void setup() {

  int i;

  Serial.begin(115200);

  // wait for serial port connection
  while(!Serial);

  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);

  delay(1000);

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




  initialization();
  do {
    sn.read();
    sn.print();
    for(int i = 0; i < NUM_OF_CONFIG; i++) {
      if(sn.check(conf[i].sn)) {
        board = i;
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
  Serial.print((conf[board].canId >> 24) & 0x00ff, HEX);
  Serial.print(" ");
  Serial.print((conf[board].canId >> 16) & 0x00ff, HEX);
  Serial.print(" ");
  Serial.print((conf[board].canId >> 8) & 0x00ff, HEX);
  Serial.print(" ");
  Serial.print((conf[board].canId ) & 0x00ff, HEX);
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
  Serial.println(angle_f);

	packAngleMsg(angle_f);


#if CAN_ENABLED
  
  // send Extended msg
  sndStat = CAN0.sendMsgBuf(CAN_ID, 1, 8, data); // ITS THIS ONE!! :)
  // sndStat = CAN0.sendMsgBuf(conf[board].canId, 1, 8, data); 


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
