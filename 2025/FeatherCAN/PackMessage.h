// PackMessage.h - pack a CAN message

#pragma once

#include <Arduino.h>

extern byte data[8];

// pack angle data into CAN message
void packAngleMsg(float angle_f);

// pack Coral angle data and proximity sensor data into CAN message
void packCoralMsg(float angle_f, uint32_t prox);

// pack message into format used by 2 steering and 1 shooter to RoboRio
void packMsgShooter();

// pack message into format used by 2 steering and 2 time of flight to RoboRio
void packMsgTOF();
