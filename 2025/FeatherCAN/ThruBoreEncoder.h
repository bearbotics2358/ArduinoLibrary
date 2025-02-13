// ThruBoreEncoder.h

#pragma once

void init_timer_capture();
void TC3_Handler();                                    // Interrupt Service Routine (ISR) for timer TC3

void TBE_loop();
