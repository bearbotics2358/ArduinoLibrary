// config.h

#pragma once

#include "SAMD21SerialNumber.h"


struct calibration {
  int fromLow;
  int fromHigh;
  int offsetAngle; // angle in tenths of degree
} ;

enum boardType {
  SHOOTER = 0,
  TIMEOFFLIGHT = 1
} ;

struct configuration {
  /* 
    we want:
    - serial number
    - number for CAN
    - number for analog interface board
    - CAN ID
    - what type it is
    - an array for each channel calibration
      - (in another structure) from low/high (calibrations on paper)
  */

	SAMD21SerialNumber sn;
  int featherCAN;
  int analogNum;
  uint32_t canId;
  struct calibration calib[3];
  enum boardType type;
} ;
