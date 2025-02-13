// CAN.cpp


#if 0


// from setup():
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


	// Print CAN ID
  /*
  Serial.print("CAN ID: 0x");
  Serial.print((CAN_ID >> 24) & 0x00ff, HEX);
  Serial.print(" ");
  Serial.print((CAN_ID >> 16) & 0x00ff, HEX);
  Serial.print(" ");
  Serial.print((CAN_ID >> 8) & 0x00ff, HEX);
  Serial.print(" ");
  Serial.print((CAN_ID ) & 0x00ff, HEX);
  Serial.println();
  */



// from loop():

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


// pack message into format used by 2 steering and 1 shooter to RoboRio
void packMsgShooter()
{
  int i;
  long ltemp = 0;
  int distance = 0;
  
  data[0] = (sensorValue0 >> 8) & 0x00ff;
  data[1] = sensorValue0 & 0x00ff;

  data[2] = (sensorValue1 >> 8) & 0x00ff;
  data[3] = sensorValue1 & 0x00ff;
  
  data[4] = (sensorValue2 >> 8) & 0x00ff;
  data[5] = sensorValue2 & 0x00ff;

  data[6] = 0x00;
  data[7] = 0x00;
}

// pack message into format used by 2 steering and 2 time of flight to RoboRio
void packMsgTOF()
{
  int i;
  long ltemp = 0;
  int distance = 0;
  
  data[0] = (sensorValue0 >> 8) & 0x00ff;
  data[1] = sensorValue0 & 0x00ff;
  
  data[2] = (sensorValue1 >> 8) & 0x00ff;
  data[3] = sensorValue1 & 0x00ff;
  
  data[4] = (tofDistance[0] >> 8) & 0x00ff;
  data[5] = tofDistance[0] & 0x00ff;

  // Then pack the TOF distance into the next 16 bits.
  // This is a unsigned value, in units of mm
  data[6] = (tofDistance[1] >> 8) & 0xff;
  data[7] = tofDistance[1] & 0xff;
}


#endif // #if 0
