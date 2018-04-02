/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "et_stm32f_arm_kit_lcd.h"
#include <string.h>
#define Mint 0x1FF8
#define Rose 0xF820
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Menu(void);
void buttemppress(uint16_t posX,uint16_t posY);
void buttemprelease(uint16_t posX,uint16_t posY);
void butspeedpress(uint16_t posX,uint16_t posY);
void butspeedrelease(uint16_t posX,uint16_t posY);
void butgraphpress(uint16_t posX,uint16_t posY);
void butgraphrelease(uint16_t posX,uint16_t posY);
void butpower(uint16_t posX,uint16_t posY);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	LCD_Setup();
	uint16_t posX, posY;
	char pos[50];
	//Menu();
		Menu();	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	posX = TCS_Read_X();
	posY = TCS_Read_Y();
	sprintf(pos, "X = %d Y = %d\r\n", posX, posY); 
	//while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==RESET){}
	//HAL_UART_Transmit(&huart2, (uint8_t*) pos, strlen(pos), 500); 
	if(posX > 200 && posX < 260 && posY > 186 && posY < 210)
	{
		butspeedrelease(posX,posY);
		butgraphrelease(posX,posY);
		buttemppress(posX,posY);
	}
	if(posX > 193 && posX < 265 && posY > 140 && posY < 162)
	{
		buttemprelease(posX,posY);
		butgraphrelease(posX,posY);
		butspeedpress(posX,posY);	
	}
	if(posX > 193 && posX < 265 && posY > 98 && posY < 122)
	{
		buttemprelease(posX,posY);
		butspeedrelease(posX,posY);
		butgraphpress(posX,posY);
	}
	if(posX > 205 && posX < 250 && posY > 28 && posY < 76)
	{		
		butpower(posX,posY);
	}	
  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE5 PE6 PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
void Menu(void){
	LCD_Clear(Black);
	LCD_SetBackColor(Mint);
	LCD_SetTextColor(Mint);
	LCD_DisplayStringLine(Line0, "           ");
	LCD_DisplayStringLine(Line1, "           ");
	LCD_DisplayStringLine(Line2, "           ");
	LCD_DisplayStringLine(Line3, "           ");
	LCD_DisplayStringLine(Line4, "           ");
	LCD_DisplayStringLine(Line5, "           ");
	LCD_DisplayStringLine(Line6, "           ");
	LCD_DisplayStringLine(Line7, "           ");
	LCD_DisplayStringLine(Line8, "           ");
	LCD_DisplayStringLine(Line9, "           ");

	LCD_SetTextColor(Grey);
	for(int i=0;i<5;i++){
		//shadowtemp
	LCD_DrawLine(28-i, 276-i, 25, Vertical);
	LCD_DrawLine(48+i, 210+i, 62, Horizontal);
		//shadowspeed
	LCD_DrawLine(76-i, 285-i, 25, Vertical);
	LCD_DrawLine(96+i, 205+i, 76, Horizontal);
		//shadowgraph
	LCD_DrawLine(124-i, 286-i, 25, Vertical);
	LCD_DrawLine(144+i, 205+i, 77, Horizontal);
	}
	
	for(int i=0;i<5;i++){
	LCD_DrawCircle(191+i, 241+i, 30);	
	}
	
	LCD_SetBackColor(Mint);
	LCD_SetTextColor(Black);
	int y=210;
	LCD_DisplayChar(Line1, y, 'T');
	y+=15;
	LCD_DisplayChar(Line1, y, 'E');
	y+=15;
	LCD_DisplayChar(Line1, y, 'M');
	y+=16;
	LCD_DisplayChar(Line1, y, 'P');
	
	y=205;
	LCD_DisplayChar(Line3, y, 'S');
	y+=15;
	LCD_DisplayChar(Line3, y, 'P');
	y+=15;
	LCD_DisplayChar(Line3, y, 'E');
	y+=15;
	LCD_DisplayChar(Line3, y, 'E');
	y+=15;
	LCD_DisplayChar(Line3, y, 'D');
	
	y=205;	
	LCD_DisplayChar(Line5, y, 'G');
	y+=16;	
	LCD_DisplayChar(Line5, y, 'R');
	y+=15;
	LCD_DisplayChar(Line5, y, 'A');
	y+=15;	
	LCD_DisplayChar(Line5, y, 'P');
	y+=15;
	LCD_DisplayChar(Line5, y, 'H');
	
	LCD_SetTextColor(Rose);
	for(int i=1;i<30;i++){
	LCD_DrawCircle(190, 240, i);
	}
	LCD_SetTextColor(Black);
	for(int i=15;i<20;i++){
	LCD_DrawCircle(190, 240, i);
	}
	for(int i=238;i<243;i++){		
		LCD_DrawLine(165, i, 25, Vertical);		
	}
}


void buttemppress(uint16_t posX,uint16_t posY){

		//clear
		LCD_SetTextColor(Black);
		for(int i=1;i<5;i++){
			LCD_DrawLine(48+i, 210+i, 62, Horizontal); //(y,x)
		}
		for(int i=0;i<4;i++){
			LCD_DrawLine(28-i, 276-i, 25, Vertical);   //(y,x)
		}	
		//shadow	
		LCD_SetTextColor(Grey);
			for(int i=0;i<5;i++){
			LCD_DrawLine(23-i, 209-i, 25, Vertical);
			LCD_DrawLine(19+i, 206+i, 62, Horizontal);
		}
			
		LCD_SetBackColor(Black);
		LCD_SetTextColor(Mint);
		int y=210;
		LCD_DisplayChar(Line1, y, 'T');
		y+=15;
		LCD_DisplayChar(Line1, y, 'E');
		y+=15;
		LCD_DisplayChar(Line1, y, 'M');
		y+=16;
		LCD_DisplayChar(Line1, y, 'P');
			

}

void buttemprelease(uint16_t posX,uint16_t posY){
	
			LCD_SetTextColor(Black);
			for(int i=0;i<5;i++){
				LCD_DrawLine(23-i, 209-i, 25, Vertical);
				LCD_DrawLine(19+i, 206+i, 62, Horizontal);
			}
			
			LCD_SetTextColor(Grey);
			for(int i=0;i<5;i++){
				LCD_DrawLine(28-i, 276-i, 25, Vertical);
				LCD_DrawLine(48+i, 210+i, 62, Horizontal);
			}
			
			LCD_SetBackColor(Mint);
			LCD_SetTextColor(Black);
			int y=210;
			LCD_DisplayChar(Line1, y, 'T');
			y+=15;
			LCD_DisplayChar(Line1, y, 'E');
			y+=15;
			LCD_DisplayChar(Line1, y, 'M');
			y+=16;
			LCD_DisplayChar(Line1, y, 'P');
	
}





void butspeedpress(uint16_t posX,uint16_t posY){
		//clear
		LCD_SetTextColor(Black);
		for(int i=0;i<4;i++){
			LCD_DrawLine(76-i, 285-i, 25, Vertical);   //(y,x)
		}
		for(int i=1;i<5;i++){
			LCD_DrawLine(96+i, 205+i, 76, Horizontal); //(y,x)
		}
		
		//shadow
		LCD_SetTextColor(Grey);
		for(int i=0;i<5;i++){
			LCD_DrawLine(71-i, 204-i, 25, Vertical);
			LCD_DrawLine(67+i, 201+i, 76, Horizontal);
		}			
		LCD_SetBackColor(Black);
		LCD_SetTextColor(Mint);
		int y=205;
		LCD_DisplayChar(Line3, y, 'S');
		y+=15;
		LCD_DisplayChar(Line3, y, 'P');
		y+=15;
		LCD_DisplayChar(Line3, y, 'E');
		y+=15;
		LCD_DisplayChar(Line3, y, 'E');
		y+=15;
		LCD_DisplayChar(Line3, y, 'D');
	
}
void butspeedrelease(uint16_t posX,uint16_t posY){
	 //clear
		LCD_SetTextColor(Black);
			for(int i=0;i<5;i++){
			LCD_DrawLine(71-i, 204-i, 25, Vertical);
			LCD_DrawLine(67+i, 201+i, 76, Horizontal);
		}
		//shadow	
		LCD_SetTextColor(Grey);
		for(int i=0;i<5;i++){
			//shadowspeed
		LCD_DrawLine(76-i, 285-i, 25, Vertical);
		LCD_DrawLine(96+i, 205+i, 76, Horizontal);
		}	
		
		LCD_SetBackColor(Mint);
		LCD_SetTextColor(Black);
		int y=205;
		LCD_DisplayChar(Line3, y, 'S');
		y+=15;
		LCD_DisplayChar(Line3, y, 'P');
		y+=15;
		LCD_DisplayChar(Line3, y, 'E');
		y+=15;
		LCD_DisplayChar(Line3, y, 'E');
		y+=15;
		LCD_DisplayChar(Line3, y, 'D');
}

void butgraphpress(uint16_t posX,uint16_t posY){
	
		//clear
		LCD_SetTextColor(Black);
		for(int i=0;i<4;i++){
			LCD_DrawLine(124-i, 286-i, 25, Vertical);
		}
		for(int i=1;i<5;i++){
			LCD_DrawLine(144+i, 205+i, 77, Horizontal);
		}
		
		//shadow
		LCD_SetTextColor(Grey);
		for(int i=0;i<5;i++){
			LCD_DrawLine(119-i, 204-i, 25, Vertical);
			LCD_DrawLine(115+i, 201+i, 77, Horizontal);
		}		
		LCD_SetBackColor(Black);
		LCD_SetTextColor(Mint);
		int y=205;
		LCD_DisplayChar(Line5, y, 'G');
		y+=16;	
		LCD_DisplayChar(Line5, y, 'R');
		y+=15;
		LCD_DisplayChar(Line5, y, 'A');
		y+=15;	
		LCD_DisplayChar(Line5, y, 'P');
		y+=15;
		LCD_DisplayChar(Line5, y, 'H');
}
void butgraphrelease(uint16_t posX,uint16_t posY){
	//clear
		LCD_SetTextColor(Black);
		for(int i=0;i<5;i++){
			LCD_DrawLine(119-i, 204-i, 25, Vertical);
			LCD_DrawLine(115+i, 201+i, 77, Horizontal);
		}
		//shadow
		LCD_SetTextColor(Grey);
		for(int i=0;i<5;i++){
			LCD_DrawLine(124-i, 286-i, 25, Vertical);
			LCD_DrawLine(144+i, 205+i, 77, Horizontal);
		}
		
		LCD_SetBackColor(Mint);
		LCD_SetTextColor(Black);
		int y=205;
		LCD_DisplayChar(Line5, y, 'G');
		y+=16;	
		LCD_DisplayChar(Line5, y, 'R');
		y+=15;
		LCD_DisplayChar(Line5, y, 'A');
		y+=15;	
		LCD_DisplayChar(Line5, y, 'P');
		y+=15;
		LCD_DisplayChar(Line5, y, 'H');
}
void butpower(uint16_t posX,uint16_t posY){
	if(posX > 205 && posX < 250 && posY > 28 && posY < 76)
	{
		
		LCD_SetTextColor(Black);
		for(int i=1;i<30;i++){
			LCD_DrawCircle(190, 240, i);
		}
		LCD_SetTextColor(Rose);
		for(int i=15;i<20;i++){
			LCD_DrawCircle(190, 240, i);
		}
		for(int i=238;i<243;i++){		
			LCD_DrawLine(165, i, 25, Vertical);		
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
