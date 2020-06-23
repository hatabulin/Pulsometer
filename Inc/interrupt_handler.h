#ifndef __IT_HANDLER_H
#define __IT_HANDLER_H

#include "stm32f1xx_hal.h"
#include "stdbool.h"
//#include "stm32f10x_tim.h"

#define DETECT_SIGNAL_LOW_LEVEL		300 // 513 // 510
#define DETECT_SIGNAL_HIGH_LEVEL	540

#define ADC_WITHOUT_LED	950
#define ADC_WITH_LED	2000 - 2030 // 2000- , 1980 с верхним светом
#define ADC_BLOOD 2080

bool detectTrigger(uint32_t tempSignal);
bool detectState();
void calculateProcess(uint32_t);
void normalProcess(uint32_t tempSignal);
void abnormalProcess(uint32_t tempSignal);

#endif
