// samd21_serial_no.cpp - read samd21 factory programmed serial number

#include "samd21_serial_no.h"

void read_samd21_serial_num(struct serialNum *pnumber)
{
  uint32_t *p;

  p = (uint32_t *)0x0080a00c;
  pnumber->sN[0] = *p;

  p = (uint32_t *)0x0080a040;
  pnumber->sN[1] = *p;

  p = (uint32_t *)0x0080a044;
  pnumber->sN[2] = *p;

  p = (uint32_t *)0x0080a048;
  pnumber->sN[3] = *p;
}

void clear_samd21_serial_num(struct serialNum *number) {
  int i;
  
  for(i = 0; i < 4; i++) {
    number->sN[i] = 0;
  }
}

void printSerialNum(struct serialNum number) {
  Serial.print("number(0): 0x"); Serial.println(number.sN[0], HEX);
  Serial.print("number(1): 0x"); Serial.println(number.sN[1], HEX);
  Serial.print("number(2): 0x"); Serial.println(number.sN[2], HEX);
  Serial.print("number(3): 0x"); Serial.println(number.sN[3], HEX);
}

int checkSerialNum(struct serialNum a, struct serialNum b) {
  int ret = 0;

  // Serial.println("a: ");
  // printSerialNum(a);
  // Serial.println("b: ");
  // printSerialNum(b);

  if((a.sN[0] == b.sN[0]) && 
     (a.sN[1] == b.sN[1]) &&
     (a.sN[2] == b.sN[2]) &&
     (a.sN[3] == b.sN[3])) {
    ret = 1;
  }
  // Serial.println(ret);
  return ret;
}
