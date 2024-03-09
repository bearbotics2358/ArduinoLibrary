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
#define CAN_ENABLED 1
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

line follower: 1 - well, set it aside anyway
shooter: 2
tof: 3
REV Through Bore Encoder angle: 4

shooter:
CAN ID: (Device Type): 01010 (Mfr ID): 0000 1000 (API ID): 00 0000 0010 (Device ID):00 0001 
CAN ID: 01010 0000 1000 00 0000 0010 00 0001 
CAN ID: 0 1010 0000 1000 0000 0000 1000 0001 
which is: 0x0A080081

tof:
CAN ID: 0 1010 0000 1000 0000 0000 1100 0001
which is: 0x0A0800C1 

REV Through Bore Encoder angle:
CAN ID: 01010 0000 1000 00 0000 0100 00 0001 
CAN ID: 0 1010 0000 1000 0000 0001 0000 0001
which is: 0x0A080101 

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

#define CAN_ID 0x0A080101

MCP_CAN CAN0(8); // Set MCP25625 CS to Arduino pin 8

int ledPin = 13;      // select the pin for the LED

int pulse_in_pin = 12; // the pin for the connection to the REV encoder pulse output

// trying to get motor to run
int tx_msg_count = 0;

// angle of encoder
float angle_f = 0.0;

// Setup TC3 to capture pulse-width and period
volatile boolean periodComplete;
volatile uint16_t isrPeriod;
volatile uint16_t isrPulsewidth;
uint16_t period;
uint16_t pulsewidth;


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

void init_timer_capture() {
  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;     // Switch on the event system peripheral

  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |   // Divide the 48MHz system clock by 1 = 48MHz
                     GCLK_GENDIV_ID(5);      // Set division on Generic Clock Generator (GCLK) 5
 
  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                      GCLK_GENCTRL_GENEN |         // Enable GCLK 5
                      GCLK_GENCTRL_SRC_DFLL48M |   // Set the clock source to 48MHz
                      GCLK_GENCTRL_ID(5);          // Set clock source on GCLK 5
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable the generic clock...
                      GCLK_CLKCTRL_GEN_GCLK5 |     // ....on GCLK5
                      GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed the GCLK5 to TCC2 and TC3

  /////////////////////////////////////////////////
  // Test PWM signal 1.25Hz, 70% duty-cycle on D11
  /////////////////////////////////////////////////

  /*
  // Enable the port multiplexer on digital pin D11
  PORT->Group[g_APinDescription[11].ulPort].PINCFG[g_APinDescription[11].ulPin].bit.PMUXEN = 1;
 
  // Set-up the pin as a TCC2 timer output on digital pin D11
  PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg |= PORT_PMUX_PMUXE_E;
  
  TCC2->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;          // Single slope PWM operation         
  while (TCC2->SYNCBUSY.bit.WAVE);                 // Wait for synchronization
 
  TCC2->PER.reg = 7999;                            // Set the frequency of the PWM on TCC2 to 1.25Hz
  while (TCC2->SYNCBUSY.bit.PER);                  // Wait for synchronization

  TCC2->CC[0].reg = 5600;                          // Set duty-cycle to 70%
  while (TCC2->SYNCBUSY.bit.CC0);                  // Wait for synchronization
  
  TCC2->CTRLA.reg = TCC_CTRLA_PRESCSYNC_PRESC |    // Set timer to overflow on prescaler clock (rather than GCLK)
                    TCC_CTRLA_PRESCALER_DIV64;     // 640kHz / 64 = 10kHz
                     
  TCC2->CTRLA.bit.ENABLE = 1;                      // Enable the TCC2 timer 
  while (TCC2->SYNCBUSY.bit.ENABLE);               // Wait for synchronization
  */
  
  ///////////////////////////////////////////////////
  // Timer input on D12, with timer T3 output on D10
  ///////////////////////////////////////////////////
  
  // Enable the port multiplexer on digital pin D12
  PORT->Group[g_APinDescription[12].ulPort].PINCFG[g_APinDescription[12].ulPin].bit.PMUXEN = 1;
 
  // Set-up the pin as an EIC (interrupt) peripheral on digital pin D12
  PORT->Group[g_APinDescription[12].ulPort].PMUX[g_APinDescription[12].ulPin >> 1].reg |= PORT_PMUX_PMUXO_A;
  
  EIC->EVCTRL.reg |= EIC_EVCTRL_EXTINTEO3;          // Enable event output on external interrupt 3
  EIC->CONFIG[0].reg |= EIC_CONFIG_SENSE3_HIGH;     // Set event detecting a HIGH level
  EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT3;         // Disable interrupts on external interrupt 3
  EIC->CTRL.bit.ENABLE = 1;                         // Enable EIC peripheral
  while (EIC->STATUS.bit.SYNCBUSY);                 // Wait for synchronization

  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |                                // Attach the event user (receiver) to channel 0 (n + 1)
                    EVSYS_USER_USER(EVSYS_ID_USER_TC3_EVU);                // Set the event user (receiver) as timer TC3

  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |                // No event edge detection
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                   // Set event path as asynchronous
                       EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_3) |    // Set event generator (sender) as external interrupt 3
                       EVSYS_CHANNEL_CHANNEL(0);                           // Attach the generator (sender) to channel 0

  /*
  // Enable the port multiplexer on digital pin D10
  PORT->Group[g_APinDescription[10].ulPort].PINCFG[g_APinDescription[10].ulPin].bit.PMUXEN = 1;
 
  // Set-up the pin as a TC3 timer output on digital pin D10
  PORT->Group[g_APinDescription[10].ulPort].PMUX[g_APinDescription[10].ulPin >> 1].reg |= PORT_PMUX_PMUXE_E;
  */
  
  TC3->COUNT16.EVCTRL.reg = TC_EVCTRL_TCEI |              // Enable the TC event input
                            /*TC_EVCTRL_TCINV |*/         // Invert the event input
                            TC_EVCTRL_EVACT_PPW;          // Set up the timer for capture: CC0 period, CC1 pulsewidth
  
  TC3->COUNT16.CTRLC.reg = TC_CTRLC_CPTEN1 |              // Enable capture on CC1
                           TC_CTRLC_CPTEN0;               // Enable capture on CC0
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);               // Wait for synchronization

  NVIC_SetPriority(TC3_IRQn, 0);      // Set the Nested Vector Interrupt Controller (NVIC) priority for TC3 to 0 (highest)
  NVIC_EnableIRQ(TC3_IRQn);           // Connect the TC3 timer to the Nested Vector Interrupt Controller (NVIC)

  TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC1 |           // Enable compare channel 1 (CC1) interrupts
                              TC_INTENSET_MC0;            // Enable compare channel 0 (CC0) interrupts

  TC3->COUNT16.CTRLA.reg = TC_CTRLA_PRESCSYNC_PRESC |     // Set timer to overflow on prescaler clock (rather than GCLK)
                           TC_CTRLA_PRESCALER_DIV1;      // Set prescaler to 1, 48 MHz
                 
  TC3->COUNT16.CTRLA.bit.ENABLE = 1;                      // Enable the TC3 timer
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);               // Wait for synchronization
  
}

void setup() {

  int i;

  Serial.begin(115200);

  // wait for serial port connection
  // while(!Serial);

  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  // pinMode(pulse_in_pin, INPUT);

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
  int pulse_len = 0;
  float pulsewidth_f = 0.0;
  float period_f = 0.0;
  float minpulse_f = 0.0;
  float denominator = 0.0;
  
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
  /*
  pulse_len = pulseIn(pulse_in_pin, HIGH);
  // should be 1 us to 1024 us, but my scope says 1050 us period
  angle_f = 360.0 * (pulse_len - 1.0) / 1040.0;
  if(angle_f < 0) {
    angle_f = 0.0;
  }
  if(angle_f >= 360.0) {
    angle_f = 0.0;
  }
  */

  if (periodComplete) {                            // Check if the period is complete
    noInterrupts();                               // Read the new period and pulse-width   
    pulsewidth = isrPulsewidth;
    period = isrPeriod;
    periodComplete = false;                       // Start a new period
    interrupts();

    // angle_f = 360.0 * (1.0 * pulsewidth) /  (1.0 * period);
    // Calculate angle
    // Cycle should be 1025 usec period
    // Low pulse for 1 usec to start
    // Minimum pulse width is 1 usec, indiates 0 degrees
    // Maximum pulse width is 1024 usec, indicates 360 * (1023/1024) degrees
    // Max angle = 360 * (period - 2 * minimum pulse width) / (period - minimum pulse width)
    // Pulsewidth has a range of 0 to 1023
    // And the REV Encoder has a period of 1050 usec
    period_f = period;
    pulsewidth_f = pulsewidth;
    minpulse_f = period_f / 1025.0;
    denominator = period_f - minpulse_f;
    if(denominator > 1) { // make sure we won't divide by 0 due to bogus data
      // angle_f = 360.0 * (pulsewidth_f - minpulse_f) / ((period_f - minpulse_f) - minpulse_f)
      // * ((period_f - minpulse_f) - minpulse_f) / (period_f - minpulse_f)
      angle_f = 360.0 * (pulsewidth_f - minpulse_f) / denominator;
    } else {
      angle_f = 0.0;
    }

    if((angle_f < 0) || (angle_f >= 360.0)) {
      angle_f = 0;
    }
  }

#if PRINT_VALUES
  Serial.print(pulsewidth);                         // Output the results
  Serial.print("\t");
  Serial.print(period);
  Serial.print("\t Angle: ");
  Serial.println(angle_f);
#endif

  packMsg();

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

void TC3_Handler()                                    // Interrupt Service Routine (ISR) for timer TC3
{
  if (TC3->COUNT16.INTFLAG.bit.MC0)                   // Check for match counter 0 (MC0) interrupt
  {
    TC3->COUNT16.READREQ.reg = TC_READREQ_RREQ |      // Enable a read request
                               TC_READREQ_ADDR(0x18); // Offset address of the CC0 register
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);         // Wait for (read) synchronization
    isrPeriod = TC3->COUNT16.CC[0].reg;               // Copy the period
    periodComplete = true;                            // Indicate that the period is complete
  }
 
  if (TC3->COUNT16.INTFLAG.bit.MC1)                   // Check for match counter 1 (MC1) interrupt
  {
    TC3->COUNT16.READREQ.reg = TC_READREQ_RREQ |      // Enable a read request
                               TC_READREQ_ADDR(0x1A); // Offset address of the CC1 register
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);         // Wait for (read) synchronization
    isrPulsewidth = TC3->COUNT16.CC[1].reg;           // Copy the pulse-width
  }
}
