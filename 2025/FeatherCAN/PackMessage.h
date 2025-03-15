// PackMessage.h - pack a CAN message

#pragma once

#include <Arduino.h>

extern byte data[8];

// pack Coral angle data and proximity sensor data into CAN message
void packCoralMsg(float angle_f, uint32_t prox);

// pack Algae angle data and proximity sensor data into CAN message
void packAlgaeMsg(float angle_f, uint16_t tofDistance);

// pack Climber angle data and proximity sensor data into CAN message
void packClimberMsg(float angle_f, uint32_t prox[]);

// pack Belly pan TOF sensor data into CAN message
void packBellypanTOFMsg(int16_t tofDistance[]);


// pack angle data into CAN message
void packAngleMsg(float angle_f);

// pack message into format used by 2 steering and 1 shooter to RoboRio
void packMsgShooter();

// pack message into format used by 2 steering and 2 time of flight to RoboRio
void packMsgTOF();
