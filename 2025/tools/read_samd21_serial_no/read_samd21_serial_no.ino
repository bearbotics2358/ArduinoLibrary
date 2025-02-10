/* read_samd21_serial_no - read and display the serial number of the SAMD21 chip
 *  
 *  2/20/2020 Bob D'Avello
 */

#include "samd21_serial_no.h"

uint32_t serial[4];

void setup() {
  int i;
  
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

  for(i = 0; i < 4; i++) {
    serial[i] = 0;
  }

  Serial.begin(9600);
}

void loop() {
  Serial.println();
  read_samd21_serial_no(serial);
  Serial.println("The serial number is: ");
  Serial.print("serial[0]: 0x"); Serial.println(serial[0], HEX);
  Serial.print("serial[1]: 0x"); Serial.println(serial[1], HEX);
  Serial.print("serial[2]: 0x"); Serial.println(serial[2], HEX);
  Serial.print("serial[3]: 0x"); Serial.println(serial[3], HEX);
  
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(4500);              // wait for a second
}
