// ThruBoreEncoder.h

#pragma once

#include <Arduino.h>

void init_timer_capture();
void TBE_loop();

void TC3_Handler();  // Interrupt Service Routine (ISR) for timer TC3
