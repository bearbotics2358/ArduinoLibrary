// samd21_serial_no.h - read samd21 factory programmed serial number

#include <Arduino.h>

struct serialNum {
  uint32_t sN[4];
} ;

void clear_samd21_serial_num(struct serialNum *pnumber);
void read_samd21_serial_num(struct serialNum *pnumber);
void printSerialNum(struct serialNum number);
int checkSerialNum(struct serialNum a, struct serialNum b);
