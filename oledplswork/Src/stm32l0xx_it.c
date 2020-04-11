/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
	uint8_t isDim = 0;
	uint8_t isSelfSetup = 1;
	uint8_t isOtherSetup = 1;
	uint8_t contrastLow[]={0x81, 0x0F};
	uint8_t contrastHigh[]={0x81, 0xFF};
	char hr[]="000";
	char spo2[] = {'0','0','0'};
	char distance[]="N000";
	char user[]={'1'};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and line 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */
	if(isDim){

		sendCMD(contrastHigh, (uint16_t)sizeof(contrastHigh));
		isDim = 0;
	}
	else{

		  sendCMD(contrastLow, (uint16_t)sizeof(contrastLow));
		  isDim=1;
	}
  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

	uint8_t page[] = {0x22, 0x00,0x00};

	uint8_t col[]= {0x21, 0x00, 0x7F};


	char* message = "                      ";

	if(isSelfSetup){
		clearScreen();
		sendCMD(page,(uint16_t)sizeof(page));

		sendCMD(col, (uint16_t)sizeof(col));
		sendString(message,0x00);
		col[1]=0x13;
		col[2]=0x32;
		message = "@* ";

		sendCMD(page,(uint16_t)sizeof(page));

		sendCMD(col, (uint16_t)sizeof(col));
		sendString(message,0x00);

		col[1]=0x33;
		col[2]=0x39;
		message = "%";

		sendCMD(page,(uint16_t)sizeof(page));

		sendCMD(col, (uint16_t)sizeof(col));
		sendString(message,0x00);
		isSelfSetup = 0;


	}
	if(isOtherSetup){
		// user name

		message = "TOTO ";
		page[1]=0x02;
		page[2]=0x02;

		col[1]=0x00;
		col[2]=0x7F;
		sendCMD(page,(uint16_t)sizeof(page));

		sendCMD(col, (uint16_t)sizeof(col));
		sendString(message,0x00);

		page[1]=0x03;
		page[2]=0x03;
		col[1]=0x13;
		col[2]=0x20;
		message = "@* ";
		sendCMD(page,(uint16_t)sizeof(page));
		sendCMD(col, (uint16_t)sizeof(col));
		sendString(message,0x00);
		col[1]=0x33;
		col[2]=0x39;
		message = "%";
		sendCMD(page,(uint16_t)sizeof(page));
		sendCMD(col, (uint16_t)sizeof(col));
		sendString(message,0x00);

		col[1]=0x5A;
		col[2]=0x60;
		message ="M";

		sendCMD(page,(uint16_t)sizeof(page));
		sendCMD(col, (uint16_t)sizeof(col));
		sendString(message,0x00);
		isSelfSetup = 0;
	}
	if(hr[2]=='9'){
		if(hr[1]=='9'){
			if (hr[0]=='9'){
				hr[0]='0';
			}
			else{
				hr[0]=hr[0]+1;
			}
			hr[1]='0';
		}
		else{
			hr[1]=hr[1]+1;
		}
		hr[2]='0';
	}
	else{
		hr[2]=hr[2]+1;
	}

	distance[1]=hr[0];
	distance[2]=hr[1];
	distance[3]=hr[2];
	user[0]='1';
	updateScreen(hr, hr, hr, user);
	user[0]='2';
	updateScreen(hr, hr, distance, user);
	//sedns stuff

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
