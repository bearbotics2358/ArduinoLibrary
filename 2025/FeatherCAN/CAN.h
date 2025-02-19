// CAN.h

#pragma once

/*
The Device Type (5 bits)
The Manufacturer (8 bits)
An API ID (10 bits)
The Device ID (6 bits)

For Team created modules, FIRST says that 
Device Type - HAL_CAN_Dev_kMiscellaneous = 10
Manufacturer - HAL_CAN_Man_kTeamUse = 8
API ID is up to us
Device ID is unique to each module of a specific type (e.g., we can have more than 1 *not* line follower)


CAN ID: (Device Type): TTTTT (Mfr ID): MMMM MMMM (API ID): AA AAAA AAAA (Device ID): DD DDDD 
CAN ID: TTTTT MMMM MMMM AA AAAA AAAA DD DDDD 
CAN ID: T TTTT MMMM MMMM AAAA AAAA AADD DDDD 
CAN ID: 0 1010 0000 1000 AAAA AAAA AADD DDDD 

line follower: 1 - well, set it aside anyway
shooter: 2
tof: 3
REV Through Bore Encoder angle: 4

shooter:
CAN ID: (Device Type): 01010 (Mfr ID): 0000 1000 (API ID): 00 0000 0010 (Device ID):00 0001 
CAN ID: 01010 0000 1000 00 0000 0010 00 0001 
CAN ID: 0 1010 0000 1000 0000 0000 1000 0001 
which is: 0x0A080081

tof:
CAN ID: 0 1010 0000 1000 0000 0000 1100 0001
which is: 0x0A0800C1 

REV Through Bore Encoder angle:
CAN ID: 01010 0000 1000 00 0000 0100 00 0001 
CAN ID: 0 1010 0000 1000 0000 0001 0000 0001
which is: 0x0A080101 

From CANAPI.cpp:

static int32_t CreateCANId(CANStorage* storage, int32_t apiId) {
  int32_t createdId = 0;
  createdId |= (static_cast<int32_t>(storage->deviceType) & 0x1F) << 24;
  createdId |= (static_cast<int32_t>(storage->manufacturer) & 0xFF) << 16;
  createdId |= (apiId & 0x3FF) << 6;
  createdId |= (storage->deviceId & 0x3F);
  return createdId;
}


*/
