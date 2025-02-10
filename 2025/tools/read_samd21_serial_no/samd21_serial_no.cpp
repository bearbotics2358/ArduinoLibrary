// samd21_serial_no.cpp - read samd21 factory programmed serial number

#include "samd21_serial_no.h"

void read_samd21_serial_no(uint32_t serial[])
{
  uint32_t *p;

  p = (uint32_t *)0x0080a00c;
  serial[0] = *p;

  p = (uint32_t *)0x0080a040;
  serial[1] = *p;

  p = (uint32_t *)0x0080a044;
  serial[2] = *p;

  p = (uint32_t *)0x0080a048;
  serial[3] = *p;
}
 
