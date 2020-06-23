
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
#include "std-stm32lib.h"
#ifdef USE_OLED_LCD
#include "ssd1306.h"
#endif
//#include "interrupt_handler.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t UART_NewMessage = 0;
char rxString[MAXSTRING];
uint8_t UART1_rxBuffer = '\000';
bool adcIsRead;
bool buttonIsPressed;
bool loggingEnable, loggingAuto;
#ifdef USE_OLED_LCD
bool printLcdEnable = true;
#endif
uint32_t adc[2] = {0,};

uint8_t currentButtonState;
uint8_t prevState;
uint8_t currentState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#ifdef USE_OLED_LCD
void printBpm(uint8_t bpm, uint8_t lcd_id, bool update) {
	char temp_str[5];
	uint8_t i;
	snprintf(temp_str,sizeof(temp_str),"%u",bpm);
	ssd1306_WriteString(4*18, 0, temp_str, Font_11x18, White, lcd_id);
	for (i = 3 - strlen(temp_str); i>0; i-- ) {
		ssd1306_SetCursor((3-i)*11 + 4*18 ,0, lcd_id);
		ssd1306_WriteChar(32, Font_11x18, White, lcd_id);
	}

	if (update) ssd1306_UpdateScreen(lcd_id);
}

void printAdc(uint8_t adcChannel, uint8_t lcd_id, bool update) {
	char temp_str[10];
	uint8_t i;
	snprintf(temp_str,sizeof(temp_str),"%lu",adc[adcChannel]);
	for (i=0;i< 5 - strlen(temp_str);i++ ) {
		ssd1306_SetCursor(i*7 + 5*7,ADC0_Y_POS, lcd_id);
		ssd1306_WriteChar(32, Font_7x10, White,lcd_id);
	}
	if (adcChannel == 0) ssd1306_WriteString(i*7 + 5*7 ,ADC0_Y_POS,temp_str,Font_7x10,White,lcd_id);
	else if (adcChannel == 1) ssd1306_WriteString(i*7 + 5*7 ,ADC1_Y_POS,temp_str,Font_7x10,White,lcd_id);
	if (update) ssd1306_UpdateScreen(lcd_id);
}

void printMain(uint8_t lcd_id) {
	ssd1306_WriteString(22,0,"BPM:",Font_11x18,White,lcd_id);
	ssd1306_SetCursor(0,0, lcd_id);
	ssd1306_WriteChar(31, Font_16x26, White,lcd_id); // HEART symbol

	ssd1306_WriteString(ADC0_X_POS, ADC0_Y_POS,"ADC0:",Font_7x10,White,lcd_id);
	ssd1306_WriteString(ADC1_X_POS,ADC1_Y_POS,"ADC1:",Font_7x10,White,lcd_id);
	printBpm(0, lcd_id, false);
	printAdc(0, lcd_id, false);
	printAdc(1, lcd_id, true);
}
#endif
void checkButton(uint8_t MODE) {

	switch (MODE) {
	case BUTTON_TRIGGERED_MODE:
		if (HAL_GPIO_ReadPin(SENSOR_BUTTON_PIN_GPIO_Port, SENSOR_BUTTON_PIN_Pin) != GPIO_PIN_SET) {
			if (prevState != GPIO_PIN_RESET) {
				prevState = GPIO_PIN_RESET;
				buttonIsPressed = !buttonIsPressed ;
				setLed(buttonIsPressed);
			}
		} else prevState = GPIO_PIN_SET;
		break;
	case BUTTON_NORMAL_MODE:
		if (HAL_GPIO_ReadPin(SENSOR_BUTTON_PIN_GPIO_Port, SENSOR_BUTTON_PIN_Pin) != GPIO_PIN_SET) {
			if (prevState != GPIO_PIN_RESET) {
				prevState = GPIO_PIN_RESET;
				setLed(true);
				buttonIsPressed = true;
			}
		} else {
			if (prevState != GPIO_PIN_SET) {
				prevState = GPIO_PIN_SET;
				setLed(false);
			  	buttonIsPressed = false;
			}
		}
	}
}

void setLed(bool buttonIsPressed) {

	char temp_str[100];
	snprintf(temp_str,sizeof(temp_str), "led:%s\n\r", buttonIsPressed?"true":"false");
	sendMessage(temp_str);

	if (loggingAuto && buttonIsPressed) {
		loggingEnable = true;
	}

	if (buttonIsPressed) {
//		HAL_ADC_Start_IT(&hadc1);
		HAL_GPIO_WritePin(SENSOR_LED_PIN_GPIO_Port, SENSOR_LED_PIN_Pin, GPIO_PIN_SET);
		TIM_Cmd(TIM2, ENABLE);
		#ifdef USE_OLED_LCD
		printMain(MAIN_LCD);
		#endif
	}
	else {
//		HAL_ADC_Stop_IT(&hadc1);
		TIM_Cmd(TIM2, DISABLE);
		HAL_GPIO_WritePin(SENSOR_LED_PIN_GPIO_Port, SENSOR_LED_PIN_Pin, GPIO_PIN_RESET);
#ifdef USE_OLED_LCD
		ssd1306_Fill(0,MAIN_LCD);
		ssd1306_UpdateScreen(MAIN_LCD);
#endif
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
#ifdef USE_OLED_LCD
	void startTitle(uint8_t lcd_id)
	{
		ssd1306_WriteString(16,0,"Pulsometr",Font_11x18,White,lcd_id);
		ssd1306_UpdateScreen(lcd_id);
		HAL_Delay(50);
		ssd1306_WriteString(15,30,"by XaTa6 v1.0",Font_7x10,White,lcd_id);
		ssd1306_UpdateScreen(lcd_id);
		HAL_Delay(150);
		ssd1306_WriteString(0,41,"Lutsk,UKRAINE,2020",Font_7x10,White,lcd_id);
		ssd1306_UpdateScreen(lcd_id);
		HAL_Delay(2000);
		ssd1306_Fill(0,lcd_id);
		ssd1306_UpdateScreen(lcd_id);
	}
#endif
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(SENSOR_LED_PIN_GPIO_Port, SENSOR_LED_PIN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

  sendMessage("\n\rKernel started on ....\n\r");
  sendMessage("Pulse sensor-usart driver, v1.0 (150420) by dragosha2000@gmx.net, HatSoft (c)\n\rLutsk,UA,2020\n\r");
  sendMessage("adc calibration...\n\r");

  ADC_TempSensorVrefintCmd(ENABLE);
/* Enable ADC1 DMA */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 2); // ñòàðòóåì ÀÖÏ
//  ADC_DMACmd(ADC1, ENABLE);
/* Enable ADC1 */
//  ADC_Cmd(ADC1, ENABLE);
/*¸´Î»Ð£×¼¼Ä´æÆ÷ */
  ADC_ResetCalibration(ADC1);
/*µÈ´ýÐ£×¼¼Ä´æÆ÷¸´Î»Íê³É */
  while(ADC_GetResetCalibrationStatus(ADC1));
/* ADCÐ£×¼ */
  HAL_ADCEx_Calibration_Start(&hadc1);// ADC_StartCalibration(ADC1);
  /* µÈ´ýÐ£×¼Íê³É*/
  while(ADC_GetCalibrationStatus(ADC1));
  /* ÓÉÓÚÃ»ÓÐ²ÉÓÃÍâ²¿´¥·¢£¬ËùÒÔÊ¹ÓÃÈí¼þ´¥·¢ADC×ª»» */
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
//  HAL_ADC_Start_IT(&hadc1);

  TIM_ClearFlag(TIM2,TIM_FLAG_UPDATE);
  TIM_ITConfig(TIM2,TIM_IT_UPDATE,ENABLE);
//  TIM_Cmd(TIM2, ENABLE);

  helpString();
  HAL_GPIO_WritePin(SENSOR_LED_PIN_GPIO_Port, SENSOR_LED_PIN_Pin, GPIO_PIN_RESET);
#ifdef USE_OLED_LCD
  ssd1306_Init(MAIN_LCD);
  ssd1306_UpdateScreen(MAIN_LCD);
  startTitle(MAIN_LCD);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  checkButton(BUTTON_TRIGGERED_MODE); // BUTTON_NORMAL_MODE);

	  if (adcIsRead) {
		  if (loggingEnable) {
			  char temp_str[100];
			  snprintf(temp_str,sizeof(temp_str), "Readed adc data, adc[0]=%lu, adc[1]=%lu, button_state:%s\n\r", adc[0], adc[1], buttonIsPressed?"true":"false");
			  sendMessage(temp_str);
			  //HAL_ADC_Start_IT(&hadc1);// HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 2);
			  adcIsRead = false;
		  } else {
#ifdef USE_OLED_LCD
			  if (printLcdEnable && buttonIsPressed) printAdc(0, MAIN_LCD, true);
//			  printAdc(0, MAIN_LCD, true);
			  HAL_Delay(50);
#endif
		  }
	  }

	  UartCommandProcessor();
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 50000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SENSOR_LED_PIN_GPIO_Port, SENSOR_LED_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_LED_PIN_Pin */
  GPIO_InitStruct.Pin = SENSOR_LED_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR_LED_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_BUTTON_PIN_Pin */
  GPIO_InitStruct.Pin = SENSOR_BUTTON_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SENSOR_BUTTON_PIN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Delay(uint16_t nCount)
{
	while (nCount--);
}

void UartCommandProcessor() {

	char string_temp[50];
	int8_t message_id;
//	char binaryString[(sizeof(int) << 3) + 1];
//	char* temp_str;

	if (UART_NewMessage == 1) {
		snprintf(string_temp,sizeof(string_temp),"<UART0> incoming message:%s\n\r", rxString);
		sendMessage(string_temp);
		message_id = processUartMessage(rxString);
		if (message_id>-1) {
			snprintf(string_temp,sizeof(string_temp),"<UART0> command id:%u\n\r", message_id);
			sendMessage(string_temp);
			switch (message_id) {
			case UART_CMD_GET_ADC:
				readAdc();
				break;
			case UART_CMD_HELP:
				helpString();
				break;
			case UART_CMD_LED_ON:
				setLed(true);
				break;
			case UART_CMD_LED_OFF:
				setLed(false);
				break;
			} // loggingEnable
		} else {
			sendMessage("incorrect command !\n\r");
		}
		UART_NewMessage = 0;
	}
}

uint8_t processUartMessage(char* string) {

	char* index;
//	char str1[6];
	int8_t id = -1;

	index = strstr(string,CMD_HELP_DEFINE);
    if(index!=NULL) return UART_CMD_HELP;
    else {
    	index = strstr(string,CMD_GETADC_DEFINE);
    	if (index!=NULL) {
    		return UART_CMD_GET_ADC;
    	} else {
        	index = strstr(string,CMD_LEDON_DEFINE);
        	if (index!=NULL) {
        		return UART_CMD_LED_ON;
        	} else {
            	index = strstr(string,CMD_LEDOFF_DEFINE);
            	if (index!=NULL) {
            		return UART_CMD_LED_OFF;
            	} else {
                	index = strstr(string,CMD_LOGGINGON_DEFINE);
                	if (index!=NULL) {
                		loggingEnable = true;
                		loggingAuto = false;
                		setLed(buttonIsPressed);
                		return UART_CMD_LOGGING_ON;
                	} else {
                    	index = strstr(string,CMD_LOGGINGOFF_DEFINE);
                    	if (index!=NULL) {
                    		loggingEnable = false;
                    		loggingAuto = false;
                    		setLed(buttonIsPressed);
                    		return UART_CMD_LOGGING_OFF;
                    	} else {
                        	index = strstr(string,CMD_LOGGINGAUTO_DEFINE);
                        	if (index!=NULL) {
                        		loggingAuto = true;
                        		setLed(buttonIsPressed);
                        		return UART_CMD_LOGGINGAUTO;
                        	}
                    	}
                	}
            	}
        	}
    	}
    }
    return id;
}

void helpString() {
	sendMessage("Usage:\n\r"
			"help - this page\n\r"
			"sensor:on, sensor:off - pulse sensor led on/off\n\r"
			"log:on ,log:off, log:auto - enable/disable/auto mode logout to uart\n\r"
			"---\n\r");
}

void eepromWriteInt(uint8_t address, uint16_t data) {

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static short int UART1_rxindex = 0;
	static uint8_t UART1_ErrorFlag = UART_PACKET_OK;

		UART1_rxBuffer = (uint8_t)huart->Instance->DR;
		if (UART1_rxBuffer == 8 || UART1_rxBuffer == 127) // If Backspace or del
		{
			UART1_rxindex--;
			if (UART1_rxindex < 0) UART1_rxindex = 0;
		}

		else if (UART1_rxBuffer == '\n' || UART1_rxBuffer == '\r' || UART1_rxBuffer == '\0') // If Enter
		{
			if (UART1_ErrorFlag == UART_PACKET_OK && UART1_rxindex)
			{
				rxString[UART1_rxindex] = 0;
				UART1_rxindex = 0;
				UART_NewMessage = 1;
			}
			else
			{
				sendMessage("ERROR > UART1 packet too long\n\r");
				UART1_ErrorFlag = UART_PACKET_OK; // reset error state
			}
		}

		else
		{
			if (UART1_rxBuffer != '\r' && UART1_ErrorFlag == UART_PACKET_OK) // Ignore return
			{
				rxString[UART1_rxindex] = UART1_rxBuffer; // Add that character to the string
				UART1_rxindex++;
				if (UART1_rxindex >= MAXSTRING) // User typing too much, we can't have commands that big
				{
					UART1_ErrorFlag = UART_PACKET_TOO_LONG;
					UART1_rxindex = 0;
					rxString[UART1_rxindex] = '\000';
					UART_NewMessage = 1;
				}
			}
		}
		__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
    {
		adcIsRead = true; //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 2);
    }
}

void readAdc() {

}

void sendMessage(char string[]) {
//	snprintf(string_out,sizeof(string_out),string);
//	CDC_Transmit_FS(string, (uint16_t)strlen(string));
	HAL_UART_Transmit(&huart2,(uint8_t*)string,strlen(string), HAL_MAX_DELAY);
}

void sendDataToProcessing(char symbol, int data) {
	char temp_str[50];
	snprintf(temp_str,sizeof(temp_str),"%c=%u\n\r",symbol, data);
	sendMessage(temp_str);
}


//void Delay(uint16_t nCount)
//{
//	while (nCount--);
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
