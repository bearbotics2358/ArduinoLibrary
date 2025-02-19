/* read_SAMD21_serial_no - read and display the serial number of the SAMD21 chip
 *  
 *  2/20/2020 Bob D'Avello
 */

#include "SAMD21SerialNumber.h"

SAMD21SerialNumber sn;

void setup() {
  int i;
  
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

  Serial.begin(9600);
}

void loop() {
  Serial.println();
  sn.read();
  Serial.println("The serial number is: ");
  sn.print();
  
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(4500);              // wait for a second
}
