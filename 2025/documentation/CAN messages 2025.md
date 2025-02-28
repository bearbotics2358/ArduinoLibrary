# CAN messages for 2025
It is intended that this document matches the code.  If they differ, the code is correct.  See FeatherCAN/PackMessage.cpp for the implementation.

All values greater than 8 bits are sent MSB first.
All messages are 8 bytes long.
All unused bytes shall be transmitted as 0, and must be ignored by the receiver.

The following are the defined CAN messages for 2025:

## Coral message
Encapsulates the following:
- REV Encoder Angle in degrees x10 for greater resolution
- Proximity sensor reading, a unitless value that increases for closer proximity

data[0] = Angle x10 MSB
data[1] = Angle x10 LSB

data[2] = proximity value MSB
data[3] = proximity value LSB


## Algae message
Encapsulates the following:
- REV Encoder Angle in degrees x10 for greater resolution
- Time of Flight distance reading in mm

data[0] = Angle x10 MSB
data[1] = Angle x10 LSB

data[2] = TOF distance MSB
data[3] = TOF distance LSB


## Climber message
Encapsulates the following:
- REV Encoder Angle in degrees x10 for greater resolution
- Left Proximity sensor reading, a unitless value that increases for closer proximity
- Right Proximity sensor reading, a unitless value that increases for closer proximity

data[0] = Angle x10 MSB
data[1] = Angle x10 LSB

data[2] = Left proximity value MSB
data[3] = Left proximity value LSB

data[4] = Right proximity value MSB
data[5] = Right proximity value LSB


## Bellypan message
Encapsulates the following:
- Front facing Left Time of Flight distance reading in mm
- Front facing Right Time of Flight distance reading in mm
- Down facing Time of Flight distance reading in mm

data[0] = Left TOF distance MSB
data[1] = Left TOF distance LSB

data[2] = Right TOF distance MSB
data[3] = Right TOF distance LSB

data[4] = Down TOF distance MSB
data[5] = Down TOF distance LSB



