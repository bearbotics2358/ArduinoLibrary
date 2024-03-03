
#include "config.h"

// offsets based on how encoders were mounted on the bot
#define LF_OFFSET_ANGLE 647
#define LR_OFFSET_ANGLE 3275
#define RF_OFFSET_ANGLE 1195
#define RR_OFFSET_ANGLE 3402

#define NUM_OF_CONFIG 5

struct configuration conf[NUM_OF_CONFIG];

void initialization() {
  // on right side of bot - purple
  // QWIIC connectors pointing down
  // A0 input to Right Front swerve module
  // A1 input to Right Rear swerve module
  conf[0].featherCAN = 7;
  conf[0].analogNum = 1;
  conf[0].sN.sN[0] = 0xA97F72CD;
  conf[0].sN.sN[1] = 0x50504335;
  conf[0].sN.sN[2] = 0x382E3120;
  conf[0].sN.sN[3] = 0xFF0E2814;
  conf[0].calib[0].fromLow = 9;
  conf[0].calib[0].fromHigh = 4079;
  conf[0].calib[0].offsetAngle = RF_OFFSET_ANGLE;
  conf[0].calib[1].fromLow = 11;
  conf[0].calib[1].fromHigh = 4094;
  conf[0].calib[1].offsetAngle = RR_OFFSET_ANGLE;
  conf[0].calib[2].fromLow = 9;
  conf[0].calib[2].fromHigh = 4087;
  conf[0].calib[2].offsetAngle = 0;
  conf[0].canId = 0x0a080081;
  conf[0].type = SHOOTER;

  // on left side of bot
  // QWIIC connectors pointing up
  // A0 input to Left Front swerve module
  // A1 input to Left Rear swerve module
  conf[1].featherCAN = 8;
  conf[1].analogNum = 2;
  conf[1].sN.sN[0] = 0xB644C4E4;
  conf[1].sN.sN[1] = 0x50504335;
  conf[1].sN.sN[2] = 0x382E3120;
  conf[1].sN.sN[3] = 0xFF0D1A30;
  conf[1].calib[0].fromLow = 10;
  conf[1].calib[0].fromHigh = 4095;
  conf[1].calib[0].offsetAngle = LF_OFFSET_ANGLE;
  conf[1].calib[1].fromLow = 11;
  conf[1].calib[1].fromHigh = 4091;
  conf[1].calib[1].offsetAngle = LR_OFFSET_ANGLE;
  conf[1].calib[2].fromLow = 11;
  conf[1].calib[2].fromHigh = 4094;
  conf[1].calib[2].offsetAngle = 0;
  conf[1].canId = 0x0a080082;
  conf[1].type = SHOOTER;

  // on right side of bot - green
  // QWIIC connectors pointing down
  // A0 input to Right Front swerve module
  // A1 input to Right Rear swerve module
  conf[2].featherCAN = 9;
  conf[2].analogNum = 1;
  conf[2].sN.sN[0] = 0x920E71B5;
  conf[2].sN.sN[1] = 0x50504335;
  conf[2].sN.sN[2] = 0x382E3120;
  conf[2].sN.sN[3] = 0xFF0D2F2F;
  conf[2].calib[0].fromLow = 9;
  conf[2].calib[0].fromHigh = 4079;
  conf[2].calib[0].offsetAngle = RF_OFFSET_ANGLE;
  conf[2].calib[1].fromLow = 11;
  conf[2].calib[1].fromHigh = 4094;
  conf[2].calib[1].offsetAngle = RR_OFFSET_ANGLE;
  conf[2].calib[2].fromLow = 9;
  conf[2].calib[2].fromHigh = 4087;
  conf[2].calib[2].offsetAngle = 0;
  conf[2].canId = 0x0a080083;
  conf[2].type = SHOOTER;


  
  
  
  
  // unit for testing
  // on right side of bot
  // QWIIC connectors pointing down
  // A0 input to Right Front swerve module
  // A1 input to Right Rear swerve module
  conf[3].featherCAN = 7;
  conf[3].analogNum = 1;
  conf[3].sN.sN[0] = 0x8D8DAD73;
  conf[3].sN.sN[1] = 0x51503235;
  conf[3].sN.sN[2] = 0x314A2020;
  conf[3].sN.sN[3] = 0xFF040223;
  conf[3].calib[0].fromLow = 9;
  conf[3].calib[0].fromHigh = 4079;
  conf[3].calib[0].offsetAngle = RF_OFFSET_ANGLE;
  conf[3].calib[1].fromLow = 11;
  conf[3].calib[1].fromHigh = 4094;
  conf[3].calib[1].offsetAngle = RR_OFFSET_ANGLE;
  conf[3].calib[2].fromLow = 9;
  conf[3].calib[2].fromHigh = 4087;
  conf[3].calib[2].offsetAngle = 0;
  conf[3].canId = 0x0a0800C4;
  conf[3].type = TIMEOFFLIGHT;


  // unit for testing - copy 1st unit
    // on right side of bot - purple
  // QWIIC connectors pointing down
  // A0 input to Right Front swerve module
  // A1 input to Right Rear swerve module
  conf[4].featherCAN = 7;
  conf[4].analogNum = 1;
  conf[4].sN.sN[0] = 0x9511C974;
  conf[4].sN.sN[1] = 0x51503235;
  conf[4].sN.sN[2] = 0x314A2020;
  conf[4].sN.sN[3] = 0xFF061532;
  conf[4].calib[0].fromLow = 9;
  conf[4].calib[0].fromHigh = 4079;
  conf[4].calib[0].offsetAngle = RF_OFFSET_ANGLE;
  conf[4].calib[1].fromLow = 11;
  conf[4].calib[1].fromHigh = 4094;
  conf[4].calib[1].offsetAngle = RR_OFFSET_ANGLE;
  conf[4].calib[2].fromLow = 9;
  conf[4].calib[2].fromHigh = 4087;
  conf[4].calib[2].offsetAngle = 0;
  conf[4].canId = 0x0a080081;
  conf[4].type = SHOOTER;


}
