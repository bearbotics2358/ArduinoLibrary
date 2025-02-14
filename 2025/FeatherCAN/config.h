// config.h

#pragma once

#include "SAMD21SerialNumber.h"


struct calibration {
  int offsetAngle; // angle in tenths of degree
} ;

enum boardType {
  CORAL = 0,
  ALGAE = 1,
  CLIMBER = 2,
  BELLY_PAN = 3
} ;

enum CANmessages {
  ANGLE_n_TOF = 0, // unit angle and up to 3 TOF
  ANGLE_n_PROXIMITY = 1, // unit angle and up to 3 proximity sensors
  TOF = 2, // up to 4 TOF sensors
  COLOR = 3 // up to 2 color sensor value
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
  // int analogNum; // needed for calibration of analog offsets of board
  uint32_t canId;
  // struct calibration calib[3];  // there were 3 analog interfaces on the board
  enum boardType type;
} ;
