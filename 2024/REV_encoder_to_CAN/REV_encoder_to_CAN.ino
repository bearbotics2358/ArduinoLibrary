/*
  Talon_Send_test
  Created from Analog Input and CanTalonSRX.cpp
  created 3/27/22 BD

*/

#include "wiring_private.h"
#include <mcp_can.h>
#include <SPI.h>
#include <Adafruit_DotStar.h>
#include "initialization.h"

// Controlling defines
#define CAN_ENABLED 1
#define PRINT_VALUES 1


// CAN0 RESET, INT
#define CAN0_RST 31   // MCP25625 RESET connected to Arduino pin 31
#define CAN0_INT 30  // Set INT to Arduino pin 30

#define NUMPIXELS 4 // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define DATAPIN    11
#define CLOCKPIN   13
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

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

int board = -1; // will be used to determine which board we are using

int sensorPin0 = A0; // select the input pin for the potentiometer
int sensorPin1 = A1;
int sensorPin2 = A2;
int sensorValue0 = 0;  // variable to store the value coming from the sensor
int sensorValueRaw0 = 0;
int sensorValue1 = 0;  
int sensorValueRaw1 = 0;
int sensorValue2 = 0;  
int sensorValueRaw2 = 0;

// trying to get motor to run
int tx_msg_count = 0;


// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
}

 // ATSAMR, for example, doesn't have a DAC
#ifdef DAC
// Wait for synchronization of registers between the clock domains
static __inline__ void syncDAC() __attribute__((always_inline, unused));
static void syncDAC() {
  while (DAC->STATUS.bit.SYNCBUSY == 1)
    ;
}
#endif

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx) {
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}


uint32_t newAnalogRead(uint32_t pin)
{
  uint32_t valueRead = 0;

#if defined(PIN_A6)
  if (pin == 6) {
    pin = PIN_A6;
  } else
#endif
#if defined(PIN_A7)
  if (pin == 7) {
    pin = PIN_A7;
  } else 
#endif
  if (pin <= 5) {
    pin += A0;
  }

  pinPeripheral(pin, PIO_ANALOG);
 //ATSAMR, for example, doesn't have a DAC
#ifdef DAC

    if (pin == A0) { // Disable DAC, if analogWrite(A0,dval) used previously the DAC is enabled
      syncDAC();
    
      DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC
      // DAC->CTRLB.bit.EOEN = 0x00; // The DAC output is turned off.
      syncDAC();

    }

#endif

  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input
  
  // Start conversion
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Store the value
  while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead = ADC->RESULT.reg;

  return valueRead;
}

void averagingOn() {
  // Averaging (see datasheet table in AVGCTRL register description)
  // Serial.print("AVGCTRL.reg before change: 0x");
  // Serial.println(ADC->AVGCTRL.reg, HEX);
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_16 |    // 16 sample averaging)
                     ADC_AVGCTRL_ADJRES(0x4ul);   // Adjusting result (right shift) by 4

  // Serial.print("AVGCTRL.reg after averaging turned on: 0x");
  // Serial.println(ADC->AVGCTRL.reg, HEX);

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  // Serial.print("CTRLB.reg before change: 0x");
  // Serial.println(ADC->CTRLB.reg, HEX);
  ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;         // 16 bits resolution for averaging
  
  // Serial.print("CTRLB.reg after change to 16 bit result: 0x");
  // Serial.println(ADC->CTRLB.reg, HEX);

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains
}

void enableAnalog() {
  // set clock slower due to source resistance
  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  // Serial.print("CTRLB.reg before change: 0x");
  // Serial.println(ADC->CTRLB.reg, HEX);
  ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV64_Val;         // set clock PRESCALER to 64 (default is 32)
  
  // Serial.print("CTRLB.reg after change to PRESCALER / 64: 0x");
  // Serial.println(ADC->CTRLB.reg, HEX);

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  
  // Control A
  /*
   * Bit 1 ENABLE: Enable
   *   0: The ADC is disabled.
   *   1: The ADC is enabled.
   * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
   * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
   * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
   *
   * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
   * configured. The first conversion after the reference is changed must not be used.
   */
  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

}

void setColor(int index, int sensorVal) {
  if((sensorVal < 20) || (sensorVal > 3580)) { // 0xGGRRBB (color values)
    // Serial.print("green; we gucci fam");
    strip.setPixelColor(index, 0x500000); 
  } else if((sensorVal > 20) && (sensorVal <= 1800)) { 
    // Serial.print("red; too far");
    strip.setPixelColor(index, 0x005000);
  } else if((sensorVal > 1800) && (sensorVal < 3580)) {
    // Serial.print("yellow; too far");
    strip.setPixelColor(index, 0x505000);  
  } else {
    // Serial.print("white; there's an error");
    strip.setPixelColor(index, 0x505050);
  }
}

void read_samd21_serial_no(struct serialNum *number)
{
  uint32_t *p;

  p = (uint32_t *)0x0080a00c;
  number->sN[0] = *p;

  p = (uint32_t *)0x0080a040;
  number->sN[1] = *p;

  p = (uint32_t *)0x0080a044;
  number->sN[2] = *p;

  p = (uint32_t *)0x0080a048;
  number->sN[3] = *p;
}

void printSerialNum(struct serialNum number) {
  Serial.print("number(0): 0x"); Serial.println(number.sN[0], HEX);
  Serial.print("number(1): 0x"); Serial.println(number.sN[1], HEX);
  Serial.print("number(2): 0x"); Serial.println(number.sN[2], HEX);
  Serial.print("number(3): 0x"); Serial.println(number.sN[3], HEX);
}

int checkSerialNum(struct serialNum a, struct serialNum b) {
  int ret = 0;

  // Serial.println("a: ");
  // printSerialNum(a);
  // Serial.println("b: ");
  // printSerialNum(b);

  if((a.sN[0] == b.sN[0]) && 
     (a.sN[1] == b.sN[1]) &&
     (a.sN[2] == b.sN[2]) &&
     (a.sN[3] == b.sN[3])) {
    ret = 1;
  }
  // Serial.println(ret);
  return ret;
}

void setup() {

  int i;

  Serial.begin(115200);

//  File file = fatfs.open (, FILE_READ);

  // wait for serial port connection
  // while(!Serial);

  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  // analogReadResolution(10);
  analogReference(AR_EXTERNAL);
  enableAnalog();
  averagingOn();

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

  Serial.println("Analog Input");
  
  CAN0.setMode(MCP_NORMAL);

  // delay(500);

  digitalWrite(ledPin, 1);

  strip.begin(); 
  // LEDs RED at powerup, until board is discovered
  for(i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 0x005000);      
  }
  strip.show();                     // Refresh strip

  struct serialNum serialNumber;

  initialization();
  do {
    read_samd21_serial_no(&serialNumber);
    printSerialNum(serialNumber);
    for(int i = 0; i < NUM_OF_CONFIG; i++) {
      if(checkSerialNum(serialNumber, conf[i].sN)){
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
  } while(0 && (board < 0));  // if board unknown, ignore for Talon Send Test
  // if board unknown, ignore for Talon Send Test
  if(board < 0) {
    board = 0;
  }
  
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

  // read the value from the sensor:
  sensorValueRaw0 = newAnalogRead(sensorPin0); // used to be analogRead(), made new function
  
  // read the value from the sensor:
  sensorValueRaw1 = newAnalogRead(sensorPin1); // used to be analogRead(), made new function
  
  // read the value from the sensor:
  sensorValueRaw2 = newAnalogRead(sensorPin2); // used to be analogRead(), made new function
  

#if PRINT_VALUES
  Serial.println();
  Serial.print("Encoder Values:\t"); 
  Serial.print(sensorValueRaw0);
  Serial.print('\t');
  Serial.print(sensorValueRaw1);
  Serial.print('\t');
  Serial.println();
#endif

#if CAN_ENABLED
  
  // build a Control 1 message
  // #define CONTROL_1 0x02040000
  // Device ID 25, 0x19 -> 0x02040019
  
  typedef struct _TALON_Control_1_General_10ms_t {
    unsigned TokenH:8;
    unsigned TokenL:8;
    unsigned DemandH:8;
    unsigned DemandM:8;
    unsigned DemandL:8;
    unsigned ProfileSlotSelect:1;
    unsigned FeedbackDeviceSelect:4;
    unsigned OverrideLimitSwitchEn:3;
    unsigned RevFeedbackSensor:1;
    unsigned RevMotDuringCloseLoopEn:1;
    unsigned OverrideBrakeType:2;
    unsigned ModeSelect:4;
    unsigned RampThrottle:8;
  } TALON_Control_1_General_10ms_t ;
  
  // Percent Mode, ~10% = 0.10 * 1023 = 102
  // Demand = 0x000066 - looking at enabled_20230315.log, see values like 0x090000
  // ModeSelect = 0
  //   Modes: (see ctre/phoenix/motorocontrol/ControlMode.h: enum class TalonFXControlMode 
  //     0 - Percent Output
  //     1 - Position
  //     2 - Velocity - actually, this might be Voltage Compensated mode
  //     3 - Current
  // Need to set limit switch to kLimitSwitchOverride_UseDefaultsFromFlash, which is 1
  // Basically, everything else is 0
  // so:
  // byte talon_tx_test_data[] = {0x00, 0x00,  0x00, 0x00, 0x66,  0x01, 0x00, 0x00};

  // THE FOLLOWING WORKS!!!
  // byte talon_tx_test_data[] = {0xF7, 0x28,  0x09, 0x00, 0x00,  0x0B, 0x02, 0x00};
  // so does this ~2.5 - 3 A @12V from bench supply (started at 3, then dropped back)
  // byte talon_tx_test_data[] = {0xF7, 0x28,  0x00, 0x00, 0x66,  0x0B, 0x02, 0x00};
  // interesting, seems to be the same as the above
  // byte talon_tx_test_data[] = {0xF7, 0x28,  0x00, 0x00, 0x0a,  0x0B, 0x02, 0x00};
  // changed mode from Velocity to Percent Output, still the same
  // byte talon_tx_test_data[] = {0xF7, 0x28,  0x00, 0x00, 0x0a,  0x0B, 0x00, 0x00};
  // Maybe need to give a Set Mode command?
  // 0B as 01 or 03 does not run at all
  // byte talon_tx_test_data[] = {0x00, 0x00,  0x00, 0x00, 0x66,  0x0B, 0x00, 0x00};
  // following is also about the same
  // byte talon_tx_test_data[] = {0x00, 0x00,  0x09, 0x00, 0x00,  0x0B, 0x00, 0x00};
  // byte talon_tx_test_data[] = {0x00, 0x00,  0x03, 0x00, 0x00,  0x0B, 0x00, 0x00};
  // byte talon_tx_test_data[] = {0x00, 0x00,  0x01, 0x00, 0x00,  0x0B, 0x00, 0x00};
  // byte talon_tx_test_data[] = {0x00, 0x00,  0x01, 0x00, 0x00,  0x0B, 0x02, 0x00};
  byte talon_tx_test_data[] = {0x00, 0x00,  0x00, 0x00, 0x01,  0x0B, 0x00, 0x00};

  // send Extended msg
  // no: byte sndStat = CAN0.sendMsgBuf(conf[board].canId | 0x80000000, 1, 8, data);
  // byte sndStat = CAN0.sendMsgBuf(conf[board].canId, 1, 8, data); // ITS THIS ONE!! :)
  if(tx_msg_count > 10) {
    // CAN ID
    // 0x02040000
    // ID 25 = 0x19
    // Index 2 for Set setpoint (0x0080)
    // was 0x02040019
    sndStat = CAN0.sendMsgBuf(0x02040099, 1, 8, talon_tx_test_data); // ITS THIS ONE!! :)
  }


  /*
    From this link:
    https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/HERO%20C%23/HERO%20Low%20Level%20Percent%20Output%20Example/Program.cs

    Need to also send a Global Enable
    - ArbID is 0x401BF
    - first data byte is 0 for disable, 1 for enable
    - other bytes are 0, length is 8
    - use 29 bit ID

    CAN ID: (Device Type): TTTTT (Mfr ID): MMMM MMMM (API ID): AA AAAA AAAA (Device ID): DD DDDD 
    CAN ID: TTTTT MMMM MMMM AA AAAA AAAA DD DDDD 
    CAN ID: T TTTT MMMM MMMM AAAA AAAA AADD DDDD 
    CAN ID:                4    0    1    B    F
    CAN ID: 0 0000 0000 0100 0000 0001 1011 1111

    https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
    
    Broadcast msg:
    Device Type: 0 for broadcast
    Mfr: 4 for CTRE
    Addr Class: 0
    Addr Index: 6
    Device: 0x3F for Broadcast


    Alternatively, consider this:
    Also, according to that page, the Universal Heartbeat from roboRIo is 0x01011840
    Mfr ID: NI
    RobotController type
    Device ID: 0
    API ID: 0x62
    
  */
  
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


  
  // send Talon Global enable frame
  byte talon_global_enable_data[] = {0x01, 0x00,  0x00, 0x00, 0x00,  0x00, 0x00, 0x00};
  // byte talon_global_enable_data[] = {0x00, 0x00,  0x00, 0x00, 0x00,  0x00, 0x00, 0x00};

  // send Extended msg
  sndStat = CAN0.sendMsgBuf(0x000401BF, 1, 8, talon_global_enable_data);


  // send RoboRIO heartbeat
  // byte roborio_heartbeat_data[] = {0x00, 0x00, 0x00, 0x00,  0x12, 0x00, 0x00, 0x00};
  // byte roborio_heartbeat_data[] = {0x00, 0x00, 0x00, 0x00,  0x48, 0x00, 0x00, 0x00};
  // byte roborio_heartbeat_data[] = {0x00, 0x00, 0x00, 0x00,  0xFF, 0x00, 0x00, 0x00};
  // byte roborio_heartbeat_data[] = {0x00, 0x00, 0x00, 0x00,  0xF8, 0x00, 0x00, 0x00};
  // byte roborio_heartbeat_data[] = {0x00, 0x00, 0x00, 0x00,  0x1F, 0x00, 0x00, 0x00};
  // Extended ID: 0x01011840  DLC: 8  Data: 0xB4 0xB4 0x3C 0xBB 0x13 0x00 0x00 0xFF REC: 0
  byte roborio_heartbeat_data[] = {0xB4, 0xB4, 0x3C, 0xBB,  0x13, 0x00, 0x00, 0xFF};
  // disabled startup
  // Extended ID: 0x01011840  DLC: 8  Data: 0x02 0x31 0x04 0x06 0x00 0x00 0x00 0xFF REC: 0
  byte roborio_disabled_start_data[] = {0x02, 0x31, 0x04, 0x06,  0x00, 0x00, 0x00, 0xFF};
  

  // send Roborio msg
  /*
  if(tx_msg_count == 0) {
    sndStat = CAN0.sendMsgBuf(0x01011840, 1, 8, roborio_disabled_start_data);  
  } else {
    sndStat = CAN0.sendMsgBuf(0x01011840, 1, 8, roborio_heartbeat_data);
  }
  */
                                
#if PRINT_VALUES
  if(sndStat == CAN_OK) {
    Serial.print("Message Sent Successfully!");
  } else {
    Serial.print("Error Sending Message... sndStat: 0x");
    Serial.print(sndStat, HEX);
    ret = CAN0.getError();
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
