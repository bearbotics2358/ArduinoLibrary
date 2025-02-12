// SAMD21SerialNumber.cpp - Class to work with SAMD21 factory programmed serial number

#include "SAMD21SerialNumber.h"

void SAMD21SerialNumber::read()
{
  uint32_t *p;

  p = (uint32_t *)0x0080a00c;
  sN[0] = *p;

  p = (uint32_t *)0x0080a040;
  sN[1] = *p;

  p = (uint32_t *)0x0080a044;
  sN[2] = *p;

  p = (uint32_t *)0x0080a048;
  sN[3] = *p;
}

void SAMD21SerialNumber::clear() {
  int i;
  
  for(i = 0; i < 4; i++) {
    sN[i] = 0;
  }
}

void SAMD21SerialNumber::print() {
  Serial.print("number(0): 0x"); Serial.println(sN[0], HEX);
  Serial.print("number(1): 0x"); Serial.println(sN[1], HEX);
  Serial.print("number(2): 0x"); Serial.println(sN[2], HEX);
  Serial.print("number(3): 0x"); Serial.println(sN[3], HEX);
}

int SAMD21SerialNumber::check(SAMD21SerialNumber sn2) {
  int ret = 0;

  if((sN[0] == sn2.sN[0]) && 
     (sN[1] == sn2.sN[1]) &&
     (sN[2] == sn2.sN[2]) &&
     (sN[3] == sn2.sN[3])) {
    ret = 1;
  }

  return ret;
}
