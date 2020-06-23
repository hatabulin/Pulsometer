/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define ADC_SENSOR_IN0_Pin GPIO_PIN_0
#define ADC_SENSOR_IN0_GPIO_Port GPIOA
#define SENSOR_LED_PIN_Pin GPIO_PIN_1
#define SENSOR_LED_PIN_GPIO_Port GPIOB
#define SENSOR_BUTTON_PIN_Pin GPIO_PIN_10
#define SENSOR_BUTTON_PIN_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define USE_OLED_LCD
#define USE_ADC2USB_BULK_MODE

#define MAXSTRING 20

#define UART_CMD_HELP 			1
#define UART_CMD_GET_ADC 		2
#define UART_CMD_LED_OFF 		3
#define UART_CMD_LED_ON 		4
#define UART_CMD_LOGGING_ON 	5
#define UART_CMD_LOGGING_OFF 	6
#define UART_CMD_LOGGINGAUTO	7

#define UART_PACKET_OK 0
#define UART_PACKET_TOO_LONG 1
#define UART_PACKET_CORRUPT 2
#define UART_BUSY 1
#define UART_CPLT 0

#define CMD_HELP_DEFINE	"help"
#define CMD_GETADC_DEFINE	"get adc"
#define CMD_LEDON_DEFINE	"led:on"
#define CMD_LEDOFF_DEFINE	"led:off"
#define CMD_LOGGINGON_DEFINE "log:on"
#define CMD_LOGGINGOFF_DEFINE "log:off"
#define CMD_LOGGINGAUTO_DEFINE "log:auto"

#define BUTTON_NORMAL_MODE 		1
#define BUTTON_TRIGGERED_MODE 	2

#define ADC0_X_POS	0
#define ADC0_Y_POS	18
#define ADC1_X_POS 	0
#define ADC1_Y_POS  28

#define CR2_TSVREFE_Set             ((uint32_t)0x00800000)
#define CR2_TSVREFE_Reset           ((uint32_t)0xFF7FFFFF)
#define CR2_ADON_Set                ((uint32_t)0x00000001)
#define CR2_ADON_Reset              ((uint32_t)0xFFFFFFFE)
#define CR2_DMA_Set                 ((uint32_t)0x00000100)
#define CR2_DMA_Reset               ((uint32_t)0xFFFFFEFF)
#define CR2_RSTCAL_Set              ((uint32_t)0x00000008)
#define CR2_CAL_Set                 ((uint32_t)0x00000004)
#define CR2_EXTTRIG_SWSTART_Set     ((uint32_t)0x00500000)
#define CR2_EXTTRIG_SWSTART_Reset   ((uint32_t)0xFFAFFFFF)


/* NORMAL: being used state */
/* ABNORMAL: not being used state */
#define NORMAL 1
#define ABNORMAL 0
/* The Threshold used to determine whether the device is being used or not */
/* NORMAL: more than LOWER_TRIGGER and less than UPPER_TRIGGER */
/* ABNORMWL: less than LOWER_TRIGGER and more than UPPER_TRIGGER */
/* TRIGGER_DELTA: the max delta between two normal heart wave */
#define UPPER_TRIGGER 540
#define LOWER_TRIGGER 300 //400
#define TRIGGER_DELTA 150

void sendDataToProcessing(char symbol, int data);
void Delay(uint16_t nCount);
void UartCommandProcessor();
void sendMessage(char string[]);
uint8_t processUartMessage(char* string);
void helpString();
//void Delay(uint16_t nCount);
void setLed(bool buttonIsPressed);
void readAdc();

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
