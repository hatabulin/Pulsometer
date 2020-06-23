#ifndef __LED_H
#define __LED_H

#include "stm32f1xx_hal.h"

void LED_On(void);
void LED_Off(void);
int constrain(int, int, int);
void fadeLED(int);
void ledFadeToBeat(void);

#endif
