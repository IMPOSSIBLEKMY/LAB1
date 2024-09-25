/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void display7SEG_1(int num);
void display7SEG_2(int num);
void Ex5(int *time);
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
  /* USER CODE BEGIN 2 */
  int a = 0;
  int *time = &a;
  HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(RED1_GPIO_Port, RED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(YELLOW1_GPIO_Port, YELLOW1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GREEN1_GPIO_Port, GREEN1_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  Ex5(time);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  HAL_GPIO_WritePin(GPIOA, RED_Pin|YELLOW_Pin|GREEN_Pin|RED1_Pin
                          |YELLOW1_Pin|GREEN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, a_Pin|b_Pin|c_Pin|d1_Pin
                          |e1_Pin|f1_Pin|g1_Pin|d_Pin
                          |e_Pin|f_Pin|g_Pin|a1_Pin
                          |b1_Pin|c1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RED_Pin YELLOW_Pin GREEN_Pin RED1_Pin
                           YELLOW1_Pin GREEN1_Pin */
  GPIO_InitStruct.Pin = RED_Pin|YELLOW_Pin|GREEN_Pin|RED1_Pin
                          |YELLOW1_Pin|GREEN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : a_Pin b_Pin c_Pin d1_Pin
                           e1_Pin f1_Pin g1_Pin d_Pin
                           e_Pin f_Pin g_Pin a1_Pin
                           b1_Pin c1_Pin */
  GPIO_InitStruct.Pin = a_Pin|b_Pin|c_Pin|d1_Pin
                          |e1_Pin|f1_Pin|g1_Pin|d_Pin
                          |e_Pin|f_Pin|g_Pin|a1_Pin
                          |b1_Pin|c1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Ex5(int *time)
{
	 if((10 - (*time)) >= 6)
	 {
		 display7SEG_1((10 - (*time) - 5));
		 if((10 - (*time) - 7) >= 1)
	 	 {
		  	 display7SEG_2((10 - (*time) - 7));
	 	 }
		 else
	  	 {
		  	 display7SEG_2((10 - (*time) - 5));
	  	 }
	 }
	 else
	 {
		 display7SEG_2((10 - (*time) + 5 - 5));
		 if((10 - (*time) + 5 - 7) >= 1)
	 	 {
		  	 display7SEG_1((10 - (*time) + 5 - 7));
	 	 }
		 else
	  	 {
		  	 display7SEG_1((10 - (*time) + 5 - 5));
	  	 }
	 }

	 (*time)++;
	 HAL_Delay(1000);

	 switch (*time)
	 {
	 case 3:
	 {
		 HAL_GPIO_TogglePin(GREEN1_GPIO_Port, GREEN1_Pin);
		 HAL_GPIO_TogglePin(YELLOW1_GPIO_Port, YELLOW1_Pin);
		 break;
	 }
	 case 5:
	 {
		 HAL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);
		 HAL_GPIO_TogglePin(GREEN_GPIO_Port, GREEN_Pin);

		 HAL_GPIO_TogglePin(YELLOW1_GPIO_Port, YELLOW1_Pin);
		 HAL_GPIO_TogglePin(RED1_GPIO_Port, RED1_Pin);
		 break;
	 }
	 case 8:
	 {
		 HAL_GPIO_TogglePin(GREEN_GPIO_Port, GREEN_Pin);
		 HAL_GPIO_TogglePin(YELLOW_GPIO_Port, YELLOW_Pin);
		 break;
	 }
	 case 10:
	 {
		 HAL_GPIO_TogglePin(YELLOW_GPIO_Port, YELLOW_Pin);
		 HAL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);

		 HAL_GPIO_TogglePin(RED1_GPIO_Port, RED1_Pin);
		 HAL_GPIO_TogglePin(GREEN1_GPIO_Port, GREEN1_Pin);
		 (*time) = 0;
		 break;
	 }
	 }

}

void display7SEG_1(int num)
{
	HAL_GPIO_WritePin(a_GPIO_Port, a_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(b_GPIO_Port, b_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(c_GPIO_Port, c_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(d_GPIO_Port, d_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(e_GPIO_Port, e_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(f_GPIO_Port, f_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(g_GPIO_Port, g_Pin, GPIO_PIN_SET);

	switch (num)
	{
	case 0:
	{
		HAL_GPIO_TogglePin(a_GPIO_Port, a_Pin);
		HAL_GPIO_TogglePin(b_GPIO_Port, b_Pin);
		HAL_GPIO_TogglePin(c_GPIO_Port, c_Pin);
		HAL_GPIO_TogglePin(d_GPIO_Port, d_Pin);
		HAL_GPIO_TogglePin(e_GPIO_Port, e_Pin);
		HAL_GPIO_TogglePin(f_GPIO_Port, f_Pin);
		break;
	}
	case 1:
	{
		HAL_GPIO_TogglePin(b_GPIO_Port, b_Pin);
		HAL_GPIO_TogglePin(c_GPIO_Port, c_Pin);
		break;
	}
	case 2:
	{
		HAL_GPIO_TogglePin(a_GPIO_Port, a_Pin);
		HAL_GPIO_TogglePin(b_GPIO_Port, b_Pin);
		HAL_GPIO_TogglePin(d_GPIO_Port, d_Pin);
		HAL_GPIO_TogglePin(e_GPIO_Port, e_Pin);
		HAL_GPIO_TogglePin(g_GPIO_Port, g_Pin);
		break;
	}
	case 3:
	{
		HAL_GPIO_TogglePin(a_GPIO_Port, a_Pin);
		HAL_GPIO_TogglePin(b_GPIO_Port, b_Pin);
		HAL_GPIO_TogglePin(c_GPIO_Port, c_Pin);
		HAL_GPIO_TogglePin(d_GPIO_Port, d_Pin);
		HAL_GPIO_TogglePin(g_GPIO_Port, g_Pin);
		break;
	}
	case 4:
	{
		HAL_GPIO_TogglePin(b_GPIO_Port, b_Pin);
		HAL_GPIO_TogglePin(c_GPIO_Port, c_Pin);
		HAL_GPIO_TogglePin(f_GPIO_Port, f_Pin);
		HAL_GPIO_TogglePin(g_GPIO_Port, g_Pin);
		break;
	}
	case 5:
	{
		HAL_GPIO_TogglePin(a_GPIO_Port, a_Pin);
		HAL_GPIO_TogglePin(c_GPIO_Port, c_Pin);
		HAL_GPIO_TogglePin(d_GPIO_Port, d_Pin);
		HAL_GPIO_TogglePin(f_GPIO_Port, f_Pin);
		HAL_GPIO_TogglePin(g_GPIO_Port, g_Pin);
		break;
	}
	case 6:
	{
		HAL_GPIO_TogglePin(a_GPIO_Port, a_Pin);
		HAL_GPIO_TogglePin(c_GPIO_Port, c_Pin);
		HAL_GPIO_TogglePin(d_GPIO_Port, d_Pin);
		HAL_GPIO_TogglePin(e_GPIO_Port, e_Pin);
		HAL_GPIO_TogglePin(f_GPIO_Port, f_Pin);
		HAL_GPIO_TogglePin(g_GPIO_Port, g_Pin);
		break;
	}
	case 7:
	{
		HAL_GPIO_TogglePin(a_GPIO_Port, a_Pin);
		HAL_GPIO_TogglePin(b_GPIO_Port, b_Pin);
		HAL_GPIO_TogglePin(c_GPIO_Port, c_Pin);
		break;
	}
	case 8:
	{
		HAL_GPIO_TogglePin(a_GPIO_Port, a_Pin);
		HAL_GPIO_TogglePin(b_GPIO_Port, b_Pin);
		HAL_GPIO_TogglePin(c_GPIO_Port, c_Pin);
		HAL_GPIO_TogglePin(d_GPIO_Port, d_Pin);
		HAL_GPIO_TogglePin(e_GPIO_Port, e_Pin);
		HAL_GPIO_TogglePin(f_GPIO_Port, f_Pin);
		HAL_GPIO_TogglePin(g_GPIO_Port, g_Pin);
		break;
	}
	case 9:
	{
		HAL_GPIO_TogglePin(a_GPIO_Port, a_Pin);
		HAL_GPIO_TogglePin(b_GPIO_Port, b_Pin);
		HAL_GPIO_TogglePin(c_GPIO_Port, c_Pin);
		HAL_GPIO_TogglePin(d_GPIO_Port, d_Pin);
		HAL_GPIO_TogglePin(f_GPIO_Port, f_Pin);
		HAL_GPIO_TogglePin(g_GPIO_Port, g_Pin);
		break;
	}
	}
}

void display7SEG_2(int num)
{
	HAL_GPIO_WritePin(a1_GPIO_Port, a1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(b1_GPIO_Port, b1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(c1_GPIO_Port, c1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(d1_GPIO_Port, d1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(e1_GPIO_Port, e1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(f1_GPIO_Port, f1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(g1_GPIO_Port, g1_Pin, GPIO_PIN_SET);

	switch (num)
	{
	case 0:
	{
		HAL_GPIO_TogglePin(a1_GPIO_Port, a1_Pin);
		HAL_GPIO_TogglePin(b1_GPIO_Port, b1_Pin);
		HAL_GPIO_TogglePin(c1_GPIO_Port, c1_Pin);
		HAL_GPIO_TogglePin(d1_GPIO_Port, d1_Pin);
		HAL_GPIO_TogglePin(e1_GPIO_Port, e1_Pin);
		HAL_GPIO_TogglePin(f1_GPIO_Port, f1_Pin);
		break;
	}
	case 1:
	{
		HAL_GPIO_TogglePin(b1_GPIO_Port, b1_Pin);
		HAL_GPIO_TogglePin(c1_GPIO_Port, c1_Pin);
		break;
	}
	case 2:
	{
		HAL_GPIO_TogglePin(a1_GPIO_Port, a1_Pin);
		HAL_GPIO_TogglePin(b1_GPIO_Port, b1_Pin);
		HAL_GPIO_TogglePin(d1_GPIO_Port, d1_Pin);
		HAL_GPIO_TogglePin(e1_GPIO_Port, e1_Pin);
		HAL_GPIO_TogglePin(g1_GPIO_Port, g1_Pin);
		break;
	}
	case 3:
	{
		HAL_GPIO_TogglePin(a1_GPIO_Port, a1_Pin);
		HAL_GPIO_TogglePin(b1_GPIO_Port, b1_Pin);
		HAL_GPIO_TogglePin(c1_GPIO_Port, c1_Pin);
		HAL_GPIO_TogglePin(d1_GPIO_Port, d1_Pin);
		HAL_GPIO_TogglePin(g1_GPIO_Port, g1_Pin);
		break;
	}
	case 4:
	{
		HAL_GPIO_TogglePin(b1_GPIO_Port, b1_Pin);
		HAL_GPIO_TogglePin(c1_GPIO_Port, c1_Pin);
		HAL_GPIO_TogglePin(f1_GPIO_Port, f1_Pin);
		HAL_GPIO_TogglePin(g1_GPIO_Port, g1_Pin);
		break;
	}
	case 5:
	{
		HAL_GPIO_TogglePin(a1_GPIO_Port, a1_Pin);
		HAL_GPIO_TogglePin(c1_GPIO_Port, c1_Pin);
		HAL_GPIO_TogglePin(d1_GPIO_Port, d1_Pin);
		HAL_GPIO_TogglePin(f1_GPIO_Port, f1_Pin);
		HAL_GPIO_TogglePin(g1_GPIO_Port, g1_Pin);
		break;
	}
	case 6:
	{
		HAL_GPIO_TogglePin(a1_GPIO_Port, a1_Pin);
		HAL_GPIO_TogglePin(c1_GPIO_Port, c1_Pin);
		HAL_GPIO_TogglePin(d1_GPIO_Port, d1_Pin);
		HAL_GPIO_TogglePin(e1_GPIO_Port, e1_Pin);
		HAL_GPIO_TogglePin(f1_GPIO_Port, f1_Pin);
		HAL_GPIO_TogglePin(g1_GPIO_Port, g1_Pin);
		break;
	}
	case 7:
	{
		HAL_GPIO_TogglePin(a1_GPIO_Port, a1_Pin);
		HAL_GPIO_TogglePin(b1_GPIO_Port, b1_Pin);
		HAL_GPIO_TogglePin(c1_GPIO_Port, c1_Pin);
		break;
	}
	case 8:
	{
		HAL_GPIO_TogglePin(a1_GPIO_Port, a1_Pin);
		HAL_GPIO_TogglePin(b1_GPIO_Port, b1_Pin);
		HAL_GPIO_TogglePin(c1_GPIO_Port, c1_Pin);
		HAL_GPIO_TogglePin(d1_GPIO_Port, d1_Pin);
		HAL_GPIO_TogglePin(e1_GPIO_Port, e1_Pin);
		HAL_GPIO_TogglePin(f1_GPIO_Port, f1_Pin);
		HAL_GPIO_TogglePin(g1_GPIO_Port, g1_Pin);
		break;
	}
	case 9:
	{
		HAL_GPIO_TogglePin(a1_GPIO_Port, a1_Pin);
		HAL_GPIO_TogglePin(b1_GPIO_Port, b1_Pin);
		HAL_GPIO_TogglePin(c1_GPIO_Port, c1_Pin);
		HAL_GPIO_TogglePin(d1_GPIO_Port, d1_Pin);
		HAL_GPIO_TogglePin(f1_GPIO_Port, f1_Pin);
		HAL_GPIO_TogglePin(g1_GPIO_Port, g1_Pin);
		break;
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
