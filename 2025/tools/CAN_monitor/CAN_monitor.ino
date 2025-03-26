// CAN Receive Example
//

#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

unsigned long prevTX = 0;                                        // Variable to store last execution time
const unsigned int invlTX = 1000;                                // One second interval constant

int rx_err_cnt = 0;

#define CAN0_INT 30                              // Set INT to pin PB22 (TXD for Arduino, RXD_INT for mine)
MCP_CAN CAN0(8);                               // Set CS to pin 8

// MCP25625 RESET
#define TXD_RST 31

#define LED 13

byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

int loop_cnt = 0;

#define MAX_ADDRESSES 10
uint32_t addresses[MAX_ADDRESSES];

char s1[256];

void setup()
{
  int i;
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);
  
  Serial.begin(115200);

  // so we can see the startup messages
  while(!Serial) ;
  // delay(2000);


  Serial.println("starting...");

  for(i = 0; i < MAX_ADDRESSES; i++) {
    addresses[i] = 0;
  }

  // CAN chip RESET line
  pinMode(TXD_RST, OUTPUT);
  
  // reset CAN chip
  digitalWrite(TXD_RST, 0);
  delay(100);
  digitalWrite(TXD_RST, 1);
  delay(500);  

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  // Enable clock output so can measure clock frequency externally
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ | MCP_CLKOUT_ENABLE) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
  // CAN0.setMode(MCP_LISTENONLY);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  // enable SOF output on CLKOUT for logic analyzer
  // CAN0.setSOF_output();  
  pinMode(CAN0_INT, INPUT_PULLUP);                            // Configuring pin for /INT input

    
  Serial.println("MCP2515 Library Receive Example...");

  Serial.print("CAN0_INT: ");
  Serial.println(digitalRead(CAN0_INT));

  Serial.print("MCP_CLKOUT_ENABLE: ");
  Serial.println(MCP_CLKOUT_ENABLE);
  
  /*
  Serial.print("delaying 5 seconds ...");
  delay(5000);
  Serial.println("done");
  */

  // clear screen
  Serial.print("\e[H\e[J");
  
  digitalWrite(LED, 0);

  // delay(2000);
}

void loop()
{
  int ret; 
  int i;
  int addr_index;
  uint16_t angle_d = 0;
  float angle_f = 0;
  uint32_t proximity = 0;
  uint16_t distance = 0;
  
  // Serial.print("REC: ");
  // Serial.println(CAN0.errorCountRX());
    
  // Serial.print("CAN0_INT: ");
  // Serial.println(digitalRead(CAN0_INT));
  
  
  // if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  ret = digitalRead(CAN0_INT);
  // ret = CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
  // Serial.print("CAN0 Status: ");
  // Serial.println(ret);

  // if(ret == CAN_OK)
  if(ret == 0)
  {
    // Serial.println("msg recvd");
    
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    if(rxId != 0x00000000) {
      // got something

      // look for address
      addr_index = -1;
      for(i = 0; i < MAX_ADDRESSES; i++) {
        if(rxId == addresses[i]) {
          // found it
          addr_index = i;
          break;     
        }
      }

      if(addr_index == -1) {
        // were not able to find the address, find first empty slot
        for(i = 0; i < MAX_ADDRESSES; i++) {
          if(addresses[i] == 0) {
            // found empty slot
            addr_index = i;
            addresses[addr_index] = rxId;
            break;     
          }
        }
      }

      if(addr_index == -1) {
        // no available slots
        Serial.println("No available address slots");
      } else {
        // place cursor at the beginning of the appropriate line
        sprintf(s1, "\e[%d;1H", addr_index + 1);
        Serial.print(s1);
        /*
        Serial.print("\x1B[[");
        // Serial.print(addr_index + 1);
        Serial.print("1");
        Serial.print(";1H");
        */
        // clear to end of line
        Serial.print("\e[K");
        // print the message            
        if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
          sprintf(msgString, "%.8lX ", (rxId & 0x1FFFFFFF));
        else
          sprintf(msgString, "%.3lX ", rxId);
    
        Serial.print(msgString);
    
        if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
          sprintf(msgString, " REMOTE REQUEST FRAME");
          Serial.print(msgString);
        } else {
          for(byte i = 0; i<len; i++){
            sprintf(msgString, " %.2X", rxBuf[i]);
            Serial.print(msgString);
            if(i == 3) {
              Serial.print(" ");
            }
          }
        }
  
        if((rxId & 0x3ffff000) == 0x0A080000) {
          // Team CAN messages
          // # print("\033[31mRed Text (31)\033[0m")
          // # print("\033[32mGreen Text (32)\033[0m")              
          switch(rxId & 0x3fffffff) {
            case 0x0a080041: // CORAL
              Serial.print("  CORAL \traw angle: ");
              angle_d = ((uint16_t)rxBuf[0] << 8) | (rxBuf[1]);
              angle_f = (1.0 * angle_d) / 10.0;
              Serial.print(angle_f);

              proximity = ((uint16_t)rxBuf[2] << 8) | (rxBuf[3]);
              if(proximity < 1500) {
                // Red
                Serial.print("\e[31m");
              } else {
                // Green
                Serial.print("\e[32m");
              }
              Serial.print("\tproximity: ");
              Serial.print(proximity);
              Serial.print("\e[0m");
              
              break;
                          
            case 0x0a080082: // ALGAE
              Serial.print("  ALGAE \traw angle: ");
              angle_d = (((uint16_t)rxBuf[0] << 8) | (rxBuf[1]));
              angle_f = (1.0 * angle_d) / 10.0;
              Serial.print(angle_f);

              distance = ((uint16_t)rxBuf[2] << 8) | (rxBuf[3]);
              if(distance > 300) {
                // Red
                Serial.print("\e[31m");
              } else {
                // Green
                Serial.print("\e[32m");
              }
              Serial.print("\tdistance: ");
              Serial.print(distance);
              Serial.print("\e[0m");
              
              break;
                          
            case 0x0a0800C3: // CLIMBER
              Serial.print("  CLIMBER \traw angle: ");
              angle_d = (((uint16_t)rxBuf[0] << 8) | (rxBuf[1]));
              angle_f = (1.0 * angle_d) / 10.0;
              Serial.print(angle_f);
              
              proximity = ((uint16_t)rxBuf[2] << 8) | (rxBuf[3]);
              if(proximity < 1500) {
                // Red
                Serial.print("\e[31m");
              } else {
                // Green
                Serial.print("\e[32m");
              }
              Serial.print("\tleft proximity: ");
              Serial.print(proximity);
              Serial.print("\e[0m");
              
              proximity = ((uint16_t)rxBuf[4] << 8) | (rxBuf[5]);
              if(proximity < 1500) {
                // Red
                Serial.print("\e[31m");
              } else {
                // Green
                Serial.print("\e[32m");
              }
              Serial.print("\tright proximity: ");
              Serial.print(proximity);
              Serial.print("\e[0m");
              
              
              break;
                          
            case 0x0a080104: // BELLYPAN
              Serial.print("  BELLYPAN");
              break;
                          
            default:
              Serial.print("  unknown");
              break;
          }
        }
        
        rx_err_cnt = CAN0.errorCountRX();
        if(rx_err_cnt) {
          digitalWrite(LED, 1);
        } else {        
          digitalWrite(LED, 0);
        }
        Serial.println();
      }
    }
  }

  
  // delay(1000);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
