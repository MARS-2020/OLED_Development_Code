/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void sendCMD(uint8_t *cmd, uint16_t size);
//void sendDATA(uint8_t *data, uint16_t size);
//void sendString(char *string, uint8_t header);
//void clearScreen();

// buffer

//multiplex ratio (#lines active on screen) send cmd 0xA8 to state and then 0x3F to say that we want to use all 64 rows
//display offset. send cmd 0xD3 to state and then 0x__ to set
//adressing mode send cmd 0x20 then send 0x01 for horizontal adressing (page numbers automatically incerase) send 0x10 for line by line dressign (have to set page and col numbers)
//display on send cmd 0xAF


uint8_t turnOn[] = {0xA8, 0x3F, 0xD3, 0x00, 0x20,0x10, 0xAF, 0xAC};// 0xAF}; //need to change
uint8_t orientation[]={0xC8, 0xA1};

uint8_t turnOff[]={0xAE};
//const uint8_t A[] = {0x7E, 0x09, 0x09, 0x09, 0x7E,0x00};
uint8_t dim[] = {0xAC, 0xAB};
//uint8_t contrastLow[]={0x81, 0x0F};
//uint8_t contrastHigh[]={0x81, 0xFF};
//testing
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//turn on screen

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOB, oled_NSS_Pin|ole_RES_Pin, GPIO_PIN_SET);
  sendCMD(turnOn, (uint16_t)sizeof(turnOn));
  sendCMD(orientation, (uint16_t)sizeof(orientation));
  //sendCMD(turnOn2,(uint16_t)sizeof(turnOn));

  // clear screen
  //clearScreen();

  //show screeen for x amount of time

  sendDATA(MARSBMP, (uint16_t)sizeof(MARSBMP));
  HAL_TIM_Base_Start_IT(&htim2);
  //wait x amount of time


 // clearScreen();
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //set page
	  //set colo
	  //wait indef time
/*

	 uint8_t page[] = {0x22, 0x00,0x00};
	  uint8_t col[]= {0x21, 0x00, 0x7F};
	  char* message = "126@* 100%        FIX";

	  sendCMD(page,(uint16_t)sizeof(page));

	  sendCMD(col, (uint16_t)sizeof(col));
	  sendString(message,0x00);

	  page[1]=0x01;
	  page[2]=0x01;
	  message = "                     ";

	  sendCMD(page,(uint16_t)sizeof(page));

	  sendCMD(col, (uint16_t)sizeof(col));
	  sendString(message,0x01);

	  page[1]=0x02;
	  page[2]=0x02;
	  message = "N. ARMSTRONG:";
	  sendCMD(page,(uint16_t)sizeof(page));

	  sendCMD(col, (uint16_t)sizeof(col));
	  sendString(message,0x00);

	  page[1]=0x03;
	  page[2]=0x03;
	  message = "84@*  98% N96M";

	  sendCMD(page,(uint16_t)sizeof(page));

	  sendCMD(col, (uint16_t)sizeof(col));
	  sendString(message,0x00);
	  page[1]=0x05;
	  page[2]=0x05;
	  message = "M. MURPHY:";
	  sendCMD(col, (uint16_t)sizeof(col));

	  sendCMD(page,(uint16_t)sizeof(page));
	  sendString(message, 0x00);
	  message="175@* 100% SW55M";
	  page[1]=0x06;
	  page[2]=0x06;

	  sendCMD(col, (uint16_t)sizeof(col));

	  sendCMD(page,(uint16_t)sizeof(page));
	  sendString(message, 0x00);


*/
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20096;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, oled_NSS_Pin|ole_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(oled_DC_GPIO_Port, oled_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : dim_Pin */
  GPIO_InitStruct.Pin = dim_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(dim_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : oled_NSS_Pin ole_RES_Pin */
  GPIO_InitStruct.Pin = oled_NSS_Pin|ole_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : oled_DC_Pin */
  GPIO_InitStruct.Pin = oled_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(oled_DC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
void sendCMD(uint8_t *cmd, uint16_t size) {
	//set dc low
	HAL_GPIO_WritePin(oled_DC_GPIO_Port,  oled_DC_Pin, GPIO_PIN_RESET);
	//set CS low

	HAL_GPIO_WritePin(oled_NSS_GPIO_Port,  oled_NSS_Pin, GPIO_PIN_RESET);
	//send cmd
	HAL_SPI_Transmit(&hspi2, cmd, size, 1000);
	//set CS high

	HAL_GPIO_WritePin(oled_NSS_GPIO_Port,  oled_NSS_Pin, GPIO_PIN_SET);

}

void sendDATA(uint8_t *data, uint16_t size) {
	//send and go through buffer
	//set dc high

	HAL_GPIO_WritePin(oled_DC_GPIO_Port, oled_DC_Pin, GPIO_PIN_SET);
	//set CS low

	HAL_GPIO_WritePin(oled_NSS_GPIO_Port,  oled_NSS_Pin, GPIO_PIN_RESET);
	//sendData
	//for(int i=0; i<dataSize;i++);
	HAL_SPI_Transmit(&hspi2, data, size, 1000);
	//set CS high
	HAL_GPIO_WritePin(oled_NSS_GPIO_Port,  oled_NSS_Pin, GPIO_PIN_SET);
	//set dc high

	//HAL_GPIO_WritePin(oled_DC_GPIO_Port, oled_DC_Pin, GPIO_PIN_RESET);
}

void clearScreen(){
	for (int i=0; i<1024; i++){
		  sendDATA(space, (uint16_t)sizeof(space));
	  }
}

void sendString(char *string, uint8_t header){

	for(int i =0; string[i]!='\0'; i++){
		uint8_t letter[6];
		uint16_t wordSize = (uint16_t)sizeof(letter);
		//IF STRING I LETTER

		for(int j =0; j<6; j++){
			wordSize = (uint16_t)sizeof(letter);
			if (string[i]>='A' && string[i] <= 'Z'){
				letter[j] = fonts[(string[i]-'A')*6+j];
			}
			else if(string[i] >= '0' && string[i] <= '9'){
				letter[j] = fonts[(string[i]-'0'+26)*6+j];
			}
			else if(string[i]=='%'){
				letter[j] = fonts[36*6+j];
			}
			else if(string[i]==':'){
				letter[j]=fonts[39*6+j];
				wordSize = 2;

			}
			else if(string[i]=='.'){
				letter[j]=fonts[39*6+2+j];
				wordSize = 2;

			}
			else if(string[i]==' '){
				letter[j] = fonts[38*6+j];
				//wordSize=2;
			}
			else if(string[i]=='@'){
				letter[j] = fonts[38*6+j];
				wordSize=2;
			}
			else if(string[i]=='*'){
				letter[j] = fonts[37*6+j];
			}
			letter[j]=letter[j]|header;
		}
		sendDATA(letter, wordSize);
	}
}

void updateScreen(char* hr, char* spo2, char* distance, char* user){

	uint8_t page[] = {0x22, 0x00,0x00};

	uint8_t col[]= {0x21, 0x00, 0x7F};

	//hr col is 0-18
	//spo2 col is - 33-51
	//distance col is for
	if(user[0]=='1'){
		page[1]=0x00;
		page[2]=0x00;
		col[1]=0x00;
		col[2]=0x12;
		sendCMD(page,(uint16_t)sizeof(page));
		sendCMD(col, (uint16_t)sizeof(col));
		sendString(hr,0x00);
		col[1]=0x21;
		col[2]=0x32;
		sendCMD(page,(uint16_t)sizeof(page));

		sendCMD(col, (uint16_t)sizeof(col));
		sendString(spo2,0x00);


	}
	if(user[0]=='2'){

		page[1]=0x03;
		page[2]=0x03;
		col[1]=0x00;
		col[2]=0x12;
		sendCMD(page,(uint16_t)sizeof(page));
		sendCMD(col, (uint16_t)sizeof(col));
		sendString(hr,0x00);
		col[1]=0x21;
		col[2]=0x32;
		sendCMD(col, (uint16_t)sizeof(col));
		sendString(spo2,0x00);
		col[1]=0x41;
		col[2]=0x59;
		sendCMD(col, (uint16_t)sizeof(col));
		sendString(distance,0x00);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
