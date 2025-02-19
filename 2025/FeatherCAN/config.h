// config.h

#pragma once

#include "SAMD21SerialNumber.h"


enum boardType {
  TEST = 0,
  CORAL = 1,
  ALGAE = 2,
  CLIMBER = 3,
  BELLYPAN = 4
} ;

enum CANmessages {
  TEST_MSG = 0, // if needed
  CORAL_MSG = 1, // collector angle and 1 proximity sensor
  ALGAE_MSG = 2, // collector angle and 1 TOF sensor
  CLIMBER_MSG = 3, // climber angle and 2 proximity sensors
  BELLYPAN_TOF = 4, // 2 forward facing and 1 down facing TOF sensors
  BELLYPAN_COLOR = 5 // 2 color sensor values
} ;

struct configuration {
  enum boardType type;
	SAMD21SerialNumber sn;
  int featherCAN;
  int color_sensor_qty;
  int color_sensor_bus[2];
  int TOF_qty;
  int TOF_bus[3];
  uint32_t deviceId[2];
  uint32_t api[2];
  uint32_t canId[2];
} ;
