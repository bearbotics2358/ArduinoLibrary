// PackMessage.cpp - pack a CAN message

#include "PackMessage.h"

// pack angle data into CAN message
void packAngleMsg(float angle_f)
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

// pack Coral angle data and proximity sensor data into CAN message
void packCoralMsg(float angle_f, uint32_t prox)
{
  int angle_i = (int)(angle_f * 10);

  data[0] = (angle_i >> 8) & 0x00ff;
  data[1] = angle_i & 0x00ff;

  data[2] = (prox >> 8) & 0x00ff;
  data[3] = prox & 0x00ff;
  
  data[4] = 0x00;
  data[5] = 0x00;

  data[6] = 0x00;
  data[7] = 0x00;
}

/*
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
*/
