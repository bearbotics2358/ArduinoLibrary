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
#define PRINT_VALUES 0


#include <mcp_can.h>
#include <SPI.h>

// #include "config.h"
// #include "initialization.h"

#include "PackMessage.h"

#define COLOR_SENSOR_MAX 2
#define TOF_SENSOR_MAX 3


// CAN0 RESET, INT
#define CAN0_RST 31   // MCP25625 RESET connected to Arduino pin 31
#define CAN0_INT 30  // Set INT to Arduino pin 30

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
#define MAX_CAN_IDS 4
uint32_t canId[] = {0x0a080041, // CORAL
                     0x0a080082, // ALGAE
                     0x0a0800C3, // CLIMBER
                     0x0a080104, // BELLYPAN
  };

// Create CAN interface using MCP chip
MCP_CAN CAN0(8); // Set MCP25625 CS to Arduino pin 8

int ledPin = 13;      // select the pin for the LED

// trying to get motor to run
int tx_msg_count = 0;



// Until TBE is a class:
// angle of encoder
float angle_f;

uint32_t proximity[COLOR_SENSOR_MAX];

uint16_t distance[TOF_SENSOR_MAX];


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
  
  for(int i = 0; i < COLOR_SENSOR_MAX; i++) {
    proximity[i] = 1600;
  }
  
}

void TOF_sensor_setup() {
  int i;
  int ret = 0;

  for(int i = 0; i < TOF_SENSOR_MAX; i++) {
    distance[i] = 0;
  }
  
  
}

void TOF_sensor_loop() {
  int i;
  
  for(int i = 0; i < TOF_SENSOR_MAX; i++) {
    distance[i] = 200;
  }
  
}




void setup() {

  int i;

  Serial.begin(115200);

  // wait for serial port connection
  // while(!Serial);

  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);

  // delay(5000);

  Serial.println("in setup ...");

  digitalWrite(ledPin, 1);
  
  // Init CAN
  CAN_setup();
  
  Serial.println("CAN Simulator for Sensors");
  
  // delay(500);

  digitalWrite(ledPin, 1);


  Color_sensor_setup();

  Serial.println("about to check TOF");
  delay(1000);
  // Handle any TOF Sensors
  TOF_sensor_setup();
  
  // delay(2000);

  // delay before sending CAN messages
  for(i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, 0);
    delay(500);
    digitalWrite(LED_BUILTIN, 1);
    delay(500);
  }

}

void loop() {
  int i;
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


  // Check Thru Bore Encoder
  Serial.print("Angle: ");
  Serial.print(angle_f);
  Serial.print(" ");

  // Handle any Color Sensors
  Color_sensor_loop();
  // for now, print the first one
  Serial.print(" prox: ");
  Serial.print(proximity[0]);

  // Handle any Time of Flight Sensors
  TOF_sensor_loop();
  // distance values print in above function

  Serial.println();
  
  // pack messages for protocol from Feather CAN to RoboRio
  for(i = 0; i < MAX_CAN_IDS; i++) {
    switch(i) {
      case 0: // CORAL
        angle_f = 15.0;
        proximity[0] = 1600;
        packCoralMsg(angle_f, proximity[0]);
        break;

      case 1: // ALGAE
        angle_f = 30.0;
        distance[0] = 250;
        packAlgaeMsg(angle_f, distance[0]);
        break;

      case 2: // CLIMBER          
        angle_f = 45.0;
        proximity[0] = 1625;
        proximity[1] = 1650;
        packClimberMsg(angle_f, proximity);
        break;

      case 3: // BELLYPAN  
        packBellypanTOFMsg(distance);
        break;
    }


#if CAN_ENABLED
    
    // send Extended msg
    // sndStat = CAN0.sendMsgBuf(CAN_ID, 1, 8, data); // ITS THIS ONE!! :)
    sndStat = CAN0.sendMsgBuf(canId[i], 1, 8, data); 
  
  
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
}
