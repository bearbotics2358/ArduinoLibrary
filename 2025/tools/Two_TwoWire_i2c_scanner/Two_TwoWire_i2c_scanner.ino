// --------------------------------------
// i2c_scanner for 2 I2C busses
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// 
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function

TwoWire myWire(&sercom0, A3, A4);

bool scanning_primary_bus = 1;

void setup()
{
  Wire.begin();
  // Wire.setClock(10000);

  myWire.begin();

  // Assign pins A3 & A4 to SERCOM functionality
  pinPeripheral(A3, PIO_SERCOM_ALT);
  pinPeripheral(A4, PIO_SERCOM_ALT);

  Serial.begin(115200);
  delay(5000);
  
  Serial.println("\nI2C Scanner for 2 I2C busses");
}


void loop()
{
  byte error, address;
  int nDevices;

  Serial.println(scanning_primary_bus ? "Scannning primary bus ..." : "Scanning secondary bus ...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    if(scanning_primary_bus) {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
    } else {
      myWire.beginTransmission(address);
      error = myWire.endTransmission();
    }

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  scanning_primary_bus = !scanning_primary_bus;
  delay(5000);           // wait 5 seconds for next scan
}
