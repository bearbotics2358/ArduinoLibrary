// initialization.h

#pragma once

#include "config.h"

#define NUM_OF_CONFIG 5

struct configuration conf[NUM_OF_CONFIG];

void initialize_config() {

/**********************************************************************************************************/
  // for testing
  conf[0].type = CORAL;
  // to ignore the test unit, use the following line for serial number instead of the real one below it
  conf[0].sn.set((uint32_t)0, (uint32_t)0, (uint32_t)0, (uint32_t)0);
  // conf[0].sn.set((uint32_t)0xA97F72CD, (uint32_t)0x50504335, (uint32_t)0x382E3120, (uint32_t)0xFF0E2814);

  conf[0].color_sensor_qty = 1;
  conf[0].color_sensor_bus[0] = 1;
  conf[0].color_sensor_bus[1] = 0;

  conf[0].using_mux = 0;
  conf[0].mux_bus = 0;

  conf[0].TOF_qty = 0;
  conf[0].TOF_bus[0] = 1;
  conf[0].TOF_bus[1] = 0;

  conf[0].deviceId[0] = 1;
  conf[0].api[0] = 1;
  conf[0].canId[0] = 0x0a080041;
  conf[0].deviceId[1] = 0;
  conf[0].api[1] = 0;
  conf[0].canId[1] = 0;

  // These were the old test values. Replaced these with the above to turn this back into the Coral board
  // conf[0].deviceId[0] = 2;
  // conf[0].api[0] = 2;
  // conf[0].canId[0] = 0x0a080082;
  // conf[0].deviceId[1] = 0;
  // conf[0].api[1] = 0;
  // conf[0].canId[1] = 0;


// CAN ID: 0 1010 0000 1000 AAAA AAAA AADD DDDD

/**********************************************************************************************************/
  conf[1].type = CORAL;
  // FeatherCAN #8
  conf[1].sn.set((uint32_t)0xB644C4E4, (uint32_t)0x50504335, (uint32_t)0x382E3120, (uint32_t)0xFF0D1A30);
  conf[1].featherCAN = 8;

  conf[1].color_sensor_qty = 1;
  conf[1].color_sensor_bus[0] = 1;
  conf[1].color_sensor_bus[1] = 0;

  conf[1].using_mux = 0;
  conf[1].mux_bus = 0;

  conf[1].TOF_qty = 0;
  conf[1].TOF_bus[0] = 0;
  conf[1].TOF_bus[1] = 0;

  conf[1].deviceId[0] = 1;
  conf[1].api[0] = 1;
  conf[1].canId[0] = 0x0a080041;
  conf[1].deviceId[1] = 0;
  conf[1].api[1] = 0;
  conf[1].canId[1] = 0;


/**********************************************************************************************************/
  conf[2].type = ALGAE;
  // FeatherCAN #10
  conf[2].sn.set((uint32_t)0x51E05DD4, (uint32_t)0x51503235, (uint32_t)0x314A2020, (uint32_t)0xFF06293D);
  conf[2].featherCAN = 10;

  conf[2].color_sensor_qty = 0;
  conf[2].color_sensor_bus[0] = 0;
  conf[2].color_sensor_bus[1] = 0;

  conf[2].using_mux = 0;
  conf[2].mux_bus = 0;

  conf[2].TOF_qty = 0;
  conf[2].TOF_bus[0] = 1;
  conf[2].TOF_bus[1] = 0;

  conf[2].deviceId[0] = 2;
  conf[2].api[0] = 2;
  conf[2].canId[0] = 0x0a080082;
  conf[2].deviceId[1] = 0;
  conf[2].api[1] = 0;
  conf[2].canId[1] = 0;


/**********************************************************************************************************/
  conf[3].type = CLIMBER;
  // FeatherCAN #9
  conf[3].sn.set((uint32_t)0x920E71B5, (uint32_t)0x50504335, (uint32_t)0x382E3120, (uint32_t)0xFF0D2F2F);
  conf[3].featherCAN = 9;

  conf[3].color_sensor_qty = 2;
  conf[3].color_sensor_bus[0] = 6;
  conf[3].color_sensor_bus[1] = 7;

  conf[3].using_mux = 1;
  conf[3].mux_bus = 1;

  conf[3].TOF_qty = 0;
  conf[3].TOF_bus[0] = 0;
  conf[3].TOF_bus[1] = 0;

  conf[3].deviceId[0] = 3;
  conf[3].api[0] = 3;
  conf[3].canId[0] = 0x0a0800C3;
  conf[3].deviceId[1] = 0;
  conf[3].api[1] = 0;
  conf[3].canId[1] = 0;


/**********************************************************************************************************/
  conf[4].type = BELLYPAN;
  // FeatherCAN #6
  conf[4].sn.set((uint32_t)0x6FDE2932, (uint32_t)0x51503235, (uint32_t)0x314A2020, (uint32_t)0xFF062D32);
  conf[4].featherCAN = 6;

  conf[4].color_sensor_qty = 0;
  conf[4].color_sensor_bus[0] = 1;
  conf[4].color_sensor_bus[1] = 2;

  conf[4].using_mux = 1;
  conf[4].mux_bus = 1;

  conf[4].TOF_qty = 3;
  conf[4].TOF_bus[0] = 4;
  conf[4].TOF_bus[1] = 3;
  conf[4].TOF_bus[2] = 0;

  conf[4].deviceId[0] = 4;
  conf[4].api[0] = 4;
  conf[4].canId[0] = 0x0a080104;
  conf[4].deviceId[1] = 5;
  conf[4].api[1] = 5;
  conf[4].canId[1] = 0x0a080145;


  
/*
 
Board Serial Numbers:


  // FeatherCAN #6
  conf[2].sn.set((uint32_t)0x6FDE2932, (uint32_t)0x51503235, (uint32_t)0x314A2020, (uint32_t)0xFF062D32);

  // FeatherCAN #7
  conf[2].sn.set((uint32_t)0xA97F72CD, (uint32_t)0x50504335, (uint32_t)0x382E3120, (uint32_t)0xFF0E2814);

  // FeatherCAN #8
  // conf[2].sn.set((uint32_t)0xB644C4E4, (uint32_t)0x50504335, (uint32_t)0x382E3120, (uint32_t)0xFF0D1A30);
  
  // FeatherCAN #9
  // conf[0].sn.set((uint32_t)0x920E71B5, (uint32_t)0x50504335, (uint32_t)0x382E3120, (uint32_t)0xFF0D2F2F);

  // FeatherCAN #10
  conf[0].sn.set((uint32_t)0x51E05DD4, (uint32_t)0x51503235, (uint32_t)0x314A2020, (uint32_t)0xFF06293D);
 
  // FeatherCAN 11 (I think)
  conf[1].sn.set((uint32_t)0xFAFF85AA, (uint32_t)0x51504847, (uint32_t)0x35202020, (uint32_t)0xFF032A3B);
 

  // inital Climber:
  // FeatherCAN #8
  conf[3].sn.set((uint32_t)0xB644C4E4, (uint32_t)0x50504335, (uint32_t)0x382E3120, (uint32_t)0xFF0D1A30);
  conf[3].featherCAN = 8;



 
  // from previous year, other serial numbers:

  conf[4].sN.sN[0] = 0x9511C974;
  conf[4].sN.sN[1] = 0x51503235;
  conf[4].sN.sN[2] = 0x314A2020;
  conf[4].sN.sN[3] = 0xFF061532;

*/
}
