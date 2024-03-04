/*
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

#include "wiring_private.h"
#include <mcp_can.h>
#include <SPI.h>

// Controlling defines
#define CAN_ENABLED 0
#define PRINT_VALUES 1


// CAN0 RESET, INT
#define CAN0_RST 31   // MCP25625 RESET connected to Arduino pin 31
#define CAN0_INT 30  // Set INT to Arduino pin 30

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

/*
The Device Type (5 bits)
The Manufacturer (8 bits)
An API ID (10 bits)
The Device ID (6 bits)

For Team created modules, FIRST says that 
Device Type - HAL_CAN_Dev_kMiscellaneous = 10
Manufacturer - HAL_CAN_Man_kTeamUse = 8
API ID is up to us
Device ID is unique to each module of a specific type (e.g., we can have more than 1 *not* line follower)


CAN ID: (Device Type): TTTTT (Mfr ID): MMMM MMMM (API ID): AA AAAA AAAA (Device ID): DD DDDD 
CAN ID: TTTTT MMMM MMMM AA AAAA AAAA DD DDDD 
CAN ID: T TTTT MMMM MMMM AAAA AAAA AADD DDDD 
CAN ID: 0 1010 0000 1000 AAAA AAAA AADD DDDD 

shooter: 2, tof: 3

shooter:
CAN ID: (Device Type): 01010 (Mfr ID): 0000 1000 (API ID): 00 0000 0010 (Device ID):00 0001 
CAN ID: 01010 0000 1000 00 0000 0010 00 0001 
CAN ID: 0 1010 0000 1000 0000 0000 1000 0001 
which is: 0x0A080081

tof:
CAN ID: 0 1010 0000 1000 0000 0000 1100 0001
which is: 0x0A0800C1 

From CANAPI.cpp:

static int32_t CreateCANId(CANStorage* storage, int32_t apiId) {
  int32_t createdId = 0;
  createdId |= (static_cast<int32_t>(storage->deviceType) & 0x1F) << 24;
  createdId |= (static_cast<int32_t>(storage->manufacturer) & 0xFF) << 16;
  createdId |= (apiId & 0x3FF) << 6;
  createdId |= (storage->deviceId & 0x3F);
  return createdId;
}


*/
#define CAN_ID 0x0a080081

MCP_CAN CAN0(8); // Set MCP25625 CS to Arduino pin 8

int ledPin = 13;      // select the pin for the LED

int pulse_in_pin = 6; // the pin for the connection to the REV encoder pulse output

// trying to get motor to run
int tx_msg_count = 0;

// angle of encoder
float angle_f = 0.0;

// pack data into CAN message
void packMsg()
{
  int angle_i = (int)(angle_f * 10);

  data[0] = (angle_i >> 8) & 0x00ff;
  data[1] = angle_i & 0x00ff;

  data[2] = 0x00;
  data[3] = 0x00;
  
  data[4] = 0x00;
  data[5] = 0x00;

  data[6] = 0x00;
  data[7] = 0x00;
}

void setup() {

  int i;

  Serial.begin(115200);

  // wait for serial port connection
  // while(!Serial);

  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  pinMode(pulse_in_pin, INPUT);

  delay(1000);

  digitalWrite(ledPin, 1);
  
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

  Serial.println("REV Encoder to CAN");
  
  CAN0.setMode(MCP_NORMAL);

  // delay(500);

  digitalWrite(ledPin, 1);

  Serial.print("CAN ID: 0x");
  Serial.print((CAN_ID >> 24) & 0x00ff, HEX);
  Serial.print(" ");
  Serial.print((CAN_ID >> 16) & 0x00ff, HEX);
  Serial.print(" ");
  Serial.print((CAN_ID >> 8) & 0x00ff, HEX);
  Serial.print(" ");
  Serial.print((CAN_ID ) & 0x00ff, HEX);
  Serial.println();

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
  int pulse_len = 0;
  
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

  // read the pulse length from the encoder:
  pulse_len = pulseIn(pulse_in_pin, HIGH);
  // should be 1 us to 1024 us, but my scope says 1050 us period
  angle_f = 360.0 * (pulse_len - 1.0) / 1040.0;
  if(angle_f < 0) {
    angle_f = 0.0;
  }
  if(angle_f >= 360.0) {
    angle_f = 0.0;
  }

  packMsg();

#if PRINT_VALUES
  Serial.print("Pulse length:\t"); 
  Serial.print(pulse_len);
  Serial.print("\t Angle:\t");
  Serial.print(angle_f);
  Serial.println();
#endif



#if CAN_ENABLED
  
  // send Extended msg
  // no: byte sndStat = CAN0.sendMsgBuf(conf[board].canId | 0x80000000, 1, 8, data);
  // byte sndStat = CAN0.sendMsgBuf(conf[board].canId, 1, 8, data); // ITS THIS ONE!! :)
  sndStat = CAN0.sendMsgBuf(CAN_ID, 1, 8, data); // ITS THIS ONE!! :)

  /*
  for(i = 4; i < 6; i++) {
    Serial.println(data[i]);
  }
  */
  
  // byte sndStat = CAN_OK;
    
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

#endif
  
  // send 1 time
  // while(1) {}
  
  // delay(2000);

  delay(10);
  tx_msg_count++;
}
