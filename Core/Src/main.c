/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM9_Init(void);
static void SpinDCMotor(void);
static void StartDCMotor(void);
//static void Error_Handler(void);
/* USER CODE BEGIN PFP */
GPIO_InitTypeDef ForDirPE3;
GPIO_InitTypeDef RevDirPE1;
GPIO_InitTypeDef MotorSpeedControlPE5;
uint8_t was_button_pressed = FALSE;
uint8_t button_press_count = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM9_Init();
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if ((! (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) || was_button_pressed) && ! button_press_count)
//	  	{
//	  		was_button_pressed = TRUE;
//	  		button_press_count = 1;
//	  	}else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) && (button_press_count == 1))
//	  	{
//	  		was_button_pressed = TRUE;
//	  		button_press_count = 1;
//	  	}else if (! HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) && (button_press_count == 1))
//	  	{
//	  		was_button_pressed = TRUE;
//	  		button_press_count = 2;
//	  	}
//
//	  	if(was_button_pressed && (button_press_count == 1) )
//	  	{
//	  		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, RESET);
//			__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,800);
//	  	}
//	  	else if(was_button_pressed && (button_press_count == 2) )
//	  	{
//	  		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,0);
//	  	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}
/*
 * Code to start the DC Motor when a button press is detected
 */
//static void StartDCMotor(void)
//{
//
//	if ((! (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) || was_button_pressed) && ! button_press_count)
//	{
//		was_button_pressed = TRUE;
//		button_press_count = 1;
//	}else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) && (button_press_count == 1))
//	{
//		was_button_pressed = TRUE;
//		button_press_count = 1;
//	}else if (! HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) && (button_press_count == 1))
//	{
//		was_button_pressed = TRUE;
//		button_press_count = 2;
//	}
//
//	if(was_button_pressed && (button_press_count == 1) )
//	{
//		SpinDCMotor();
//	}
//	else if(was_button_pressed && (button_press_count == 2) )
//	{
////		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);
////		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, RESET);
//	//	TIM9->CCR1 = 50000;
//		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,0);
//	}
//}
/*
 * Code to start the DC Motor
 */
//static void SpinDCMotor(void)
//{
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, RESET);
////	TIM9->CCR1 = 50000;
//	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,800);
//
//}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

	TIM_OC_InitTypeDef timer9_PWM_config;
	memset(&timer9_PWM_config,0,sizeof(timer9_PWM_config));
	htim9.Instance = TIM9;
	htim9.Init.Period = 1000-1;
	htim9.Init.Prescaler = 49;
	if ( HAL_TIM_PWM_Init(&htim9) != HAL_OK)
	{
		Error_Handler();
	}

	timer9_PWM_config.OCMode = TIM_OCMODE_PWM1;
	timer9_PWM_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	timer9_PWM_config.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&htim9, &timer9_PWM_config, TIM_CHANNEL_1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
#define IRQ_NO_EXTI9_5					23
#define NVIC_IRQ_PRI15					15
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef ForDirPE3 = {0};
	GPIO_InitTypeDef RevDirPE1 = {0};
	GPIO_InitTypeDef StartButton = {0};


	// Enable the clock for the GPIOE peripheral
	__HAL_RCC_GPIOE_CLK_ENABLE();
	// Enable the clock for the GPIOB peripheral
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// GPIO Initialization for forward direction
	ForDirPE3.Mode = GPIO_MODE_OUTPUT_PP;
	ForDirPE3.Pin = GPIO_PIN_3;
	ForDirPE3.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &ForDirPE3);

	// GPIO Initialization for reverse direction
	RevDirPE1.Mode = GPIO_MODE_OUTPUT_PP;
	RevDirPE1.Pin = GPIO_PIN_1;
	RevDirPE1.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &RevDirPE1);

	// GPIO Initialization for starting the DC Motor
	StartButton.Pin = GPIO_PIN_9;
	StartButton.Mode = GPIO_MODE_IT_FALLING;
	StartButton.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &StartButton);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);
	//IRQ configurations
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

}
void delay(void)
{
	for (uint32_t i=0;i<=500000/2;i++);
}
void EXTI9_5_IRQHandler(void)
{
	delay();
	uint16_t Speed = 900;
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
	if (button_press_count == 0)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, RESET);
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,Speed);
		button_press_count = 1;

	}else if(button_press_count == 1)
	{
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,0);
		button_press_count = 0;
	}

}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

