// SAMD21SerialNumber.h - Class to work with SAMD21 factory programmed serial number

#pragma once

#include <Arduino.h>


class SAMD21SerialNumber
{
 public:
	SAMD21SerialNumber(){
	  clear();
  }
	~SAMD21SerialNumber(){}
	
	void clear();
	void read();
  void set(uint32_t sn0, uint32_t sn1, uint32_t sn2, uint32_t sn3);
  void set(uint32_t data[4]);
	void print();
	int check(SAMD21SerialNumber sn2);

 private:	
	uint32_t sN[4];

};
