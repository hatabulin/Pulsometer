#include "led.h"

extern __IO int fadeRate;             // breath light fadeRate

void LED_On() {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void LED_Off() {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

int constrain(int num, int min, int max) {
	if (num < min) return min;
	else if (num > max) return max;
		else return num;
}

void fadeLED(int fadeRate) {
	LED_On();
	Delay(fadeRate);
	LED_Off();
	Delay(4000 - fadeRate);
}

void ledFadeToBeat() {
	fadeRate -= 25;                         //  set onetime LED fade value
    fadeRate = constrain(fadeRate,0,4000);  //  keep LED fade value from going into negative numbers!
    fadeLED(fadeRate);                      //  fade LED
}
