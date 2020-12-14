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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t aRxBuffer[1];
uint8_t aRxBuffer2[1];
uint16_t pwm_compare1 = 1000;
uint16_t pwm_compare2 = 1500;
uint16_t pwm_compare3 = 1500;
uint16_t pwm_compare6 = 1600;
uint16_t pwm_compare4 = 1500;
uint16_t pwm_compare5 = 1650;
uint16_t pwm_compare9 = 1500;
uint16_t pwm_compare10 = 1500;
uint16_t pwm_compare7 = 1500;
uint16_t pwm_compare8 = 1500;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "stdio.h"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_2);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwm_compare1);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pwm_compare2);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pwm_compare3);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pwm_compare6);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwm_compare4);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pwm_compare5);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm_compare9);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm_compare10);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm_compare7);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwm_compare8);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 1);//HAL库中接收中断函数，内部开启了中断，并将数据存储在aRxBuffer，现在在aRxBuffer就缓存一个Byte，该函数只有接收完才会调用回调函数
  HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer2, 1);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 170-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 170-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 170-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//控制0~180度舵机旋转
//void USR_TIM2_PWM_SetCompare(uint16_t duty)
//{
//   uint16_t pluse = 500;
//   pluse += duty;
////   HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_1);
//   __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, (uint16_t)pluse);
//   pluse = 500;
//
//}

//蓝牙通信
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	 HAL_UART_Transmit(&huart1, ((uint8_t *)aRxBuffer), 1,0xFFFF);
	 HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 1);
	 switch(aRxBuffer[0]){
	 case 0x50:
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		 break;
	 case 0x51:
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		 break;

		 //舵机控制
	 case 0x32://PWM1钳子张开
		 pwm_compare1 += 10;
		 if(pwm_compare1 <= 2400)
		 {
			 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwm_compare1);
		 }
		 break;

	 case 0x31://钳子收缩
		 pwm_compare1 -= 10;
		 if(pwm_compare1 >= 86)
		 {
			 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwm_compare1);
		 }
		 break;
	 case 0x33://PWM2前端控制钳子右运动
		 pwm_compare2 += 10;
		 if(pwm_compare2 <= 2400)
		 {
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pwm_compare2);
		 }
		break;
	 case 0x34://前端控制钳子的左运动
		 pwm_compare2 -= 10;
		 if(pwm_compare2 >= 60)
		 {
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pwm_compare2);
		 }
		break;
	 case 0x35://PWM3前端控制钳子的上运动
		 pwm_compare3 += 10;
		 if(pwm_compare3 <= 2400)
		 {
			 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pwm_compare3);
		 }
		break;
	 case 0x36://前端控制钳子的下运动
		 pwm_compare3 -= 10;
		 if(pwm_compare3 >= 560)
		 {
			 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pwm_compare3);

		 }
	     break;
	 case 0x41://PWM6整体右转
		 pwm_compare6 += 20;
		 if(pwm_compare6 <= 2400)
		 {
			   __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pwm_compare6);
		 }
		break;
	 case 0x42://整体左转
		 pwm_compare6 -= 20;
		 if(pwm_compare6 >= 560)
		 {
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pwm_compare6);
		 }
		break;
	 case 0x37://PWM4、PWM5 向后缩进
		pwm_compare4 += 20;
		pwm_compare5 += 20;
		if(pwm_compare5 <= 2400)
		{
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwm_compare4);
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pwm_compare5);
	    }
		break;

	 case 0x38://向前推进
		 pwm_compare4 -= 20;
		 pwm_compare5 -= 20;
		 if(pwm_compare4 >= 60)
		 {
			 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwm_compare4);
			 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pwm_compare5);
		 }
			break;

	 case 0x53://PWM9 摄像头左转
		 pwm_compare9 += 10;
		 if(pwm_compare9 <= 2400)
		 {
			 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm_compare9);
		 }
		 			 break;
	 case 0x54://摄像头右转
		 pwm_compare9 -= 10;
		 if(pwm_compare9 >= 60)
		 {
		 	 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm_compare9);
		 }
		 			 break;
	 case 0x57:
	 	 pwm_compare9 = 1500;
	 	 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm_compare9);
	 	 break;
	 case 0x55://PWM10摄像头向下转
		 pwm_compare10 += 10;
		 if(pwm_compare10 <= 2400)
		 {
			 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm_compare10);
	     }
			 break;
	 case 0x56://摄像头向上转
		 pwm_compare10 -= 10;
		 if(pwm_compare10 >= 60)
		 {
			 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm_compare10);
		 }
			 break;

	 case 0x43://前进
		 //IN1<=1; IN2<=0; IN3<=0; IN4<=1;
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
		 break;
	 case 0x44://后退
//			 IN1<=0; IN2<=1; IN3<=1; IN4<=0;
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3, GPIO_PIN_RESET);
		 break;
	 case 0x45://右转
		 //IN1<=1; IN2<=0; IN3<=1; IN4<=0;
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_10, GPIO_PIN_RESET);
		break;
	 case 0x46://左转
		//IN1<=0; IN2<=1; IN3<=0; IN4<=1;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);
		break;
	 case 0x47://停止
		//IN1<=0; IN2<=0; IN3<=0; IN4<=0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
		break;

	 case 0x48://高速
		 pwm_compare7 = 2500;
		 pwm_compare8 = 2500;
		 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm_compare7);
		 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwm_compare8);
		 break;
	 case 0x49://中速
		 pwm_compare7 = 2000;
		 pwm_compare8 = 2000;
		 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm_compare7);
		 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwm_compare8);
		 break;

	 case 0x52://慢速
		 pwm_compare7 = 1500;
		 pwm_compare8 = 1500;
		 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm_compare7);
		 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwm_compare8);
		 break;
	 }

	 HAL_UART_Transmit(&huart2, ((uint8_t *)aRxBuffer2), 1,0xFFFF);
	 HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer2, 1);
	 switch(aRxBuffer2[0])
	 {
	 	 case 0x50:
	 	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	 	 break;
	 	 case 0x51:
	 	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	 	 break;
	 //舵机控制
	 	 case 0x32://PWM1钳子张开
	 		 pwm_compare1 += 10;
	 		 if(pwm_compare1 <= 2400)
	 		 {
	 			 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwm_compare1);
	 		 }
	 		 break;

	 	 case 0x31://钳子收缩
	 		 pwm_compare1 -= 10;
	 		 if(pwm_compare1 >= 86)
	 		 {
	 			 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwm_compare1);
	 		 }
	 		 break;
	 	 case 0x33://PWM2前端控制钳子右运动
	 		 pwm_compare2 += 10;
	 		 if(pwm_compare2 <= 2400)
	 		 {
	 			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pwm_compare2);
	 		 }
	 		break;
	 	 case 0x34://前端控制钳子的左运动
	 		 pwm_compare2 -= 10;
	 		 if(pwm_compare2 >= 60)
	 		 {
	 			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pwm_compare2);
	 		 }
	 		break;
	 	 case 0x35://PWM3前端控制钳子的上运动
	 		 pwm_compare3 += 10;
	 		 if(pwm_compare3 <= 2400)
	 		 {
	 			 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pwm_compare3);
	 		 }
	 		break;
	 	 case 0x36://前端控制钳子的下运动
	 		 pwm_compare3 -= 10;
	 		 if(pwm_compare3 >= 560)
	 		 {
	 			 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pwm_compare3);

	 		 }
	 	     break;
	 	 case 0x41://PWM6整体右转
	 		 pwm_compare6 += 20;
	 		 if(pwm_compare6 <= 2400)
	 		 {
	 			   __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pwm_compare6);
	 		 }
	 		break;
	 	 case 0x42://整体左转
	 		 pwm_compare6 -= 20;
	 		 if(pwm_compare6 >= 560)
	 		 {
	 			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pwm_compare6);
	 		 }
	 		break;
	 	 case 0x37://PWM4、PWM5 向后缩进
	 		pwm_compare4 += 20;
	 		pwm_compare5 += 20;
	 		if(pwm_compare5 <= 2400)
	 		{
	 			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwm_compare4);
	 			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pwm_compare5);
	 	    }
	 		break;

	 	 case 0x38://向前推进
	 		 pwm_compare4 -= 20;
	 		 pwm_compare5 -= 20;
	 		 if(pwm_compare4 >= 60)
	 		 {
	 			 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwm_compare4);
	 			 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pwm_compare5);
	 		 }
	 			break;

	 	 case 0x53://PWM9 摄像头左转
	 		 pwm_compare9 += 10;
	 		 if(pwm_compare9 <= 2400)
	 		 {
	 			 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm_compare9);
	 		 }
	 		 			 break;
	 	 case 0x54://摄像头右转
	 		 pwm_compare9 -= 10;
	 		 if(pwm_compare9 >= 60)
	 		 {
	 		 	 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm_compare9);
	 		 }
	 		 			 break;
	 	 case 0x57:
	 		 	 pwm_compare9 = 1500;
	 		 	 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm_compare9);
	 		 	 break;

	 	 case 0x55://PWM10摄像头向下转
	 		 pwm_compare10 += 10;
	 		 if(pwm_compare10 <= 2400)
	 		 {
	 			 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm_compare10);
	 	     }
	 			 break;
	 	 case 0x56://摄像头向上转
	 		 pwm_compare10 -= 10;
	 		 if(pwm_compare10 >= 60)
	 		 {
	 			 __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm_compare10);
	 		 }
	 			 break;
	 	 case 0x43://前进
	 		 //IN1<=1; IN2<=0; IN3<=0; IN4<=1;
	 		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3, GPIO_PIN_SET);
	 		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
	 		 break;
	 	 case 0x44://后退
	 //			 IN1<=0; IN2<=1; IN3<=1; IN4<=0;
	 		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
	 		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3, GPIO_PIN_RESET);
	 		 break;
	 	 case 0x45://右转
	 		 //IN1<=1; IN2<=0; IN3<=1; IN4<=0;
	 		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);
	 		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_10, GPIO_PIN_RESET);
	 		break;
	 	 case 0x46://左转
	 		//IN1<=0; IN2<=1; IN3<=0; IN4<=1;
	 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_10, GPIO_PIN_SET);
	 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);
	 		break;
	 	 case 0x47://停止
	 		//IN1<=0; IN2<=0; IN3<=0; IN4<=0;
	 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
	 		break;

	 	 case 0x48://高速
	 		 pwm_compare7 = 2500;
	 		 pwm_compare8 = 2500;
	 		 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm_compare7);
	 		 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwm_compare8);
	 		 break;
	 	 case 0x49://中速
	 		 pwm_compare7 = 2000;
	 		 pwm_compare8 = 2000;
	 		 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm_compare7);
	 		 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwm_compare8);
	 		 break;

	 	 case 0x52://慢速
	 		 pwm_compare7 = 1500;
	 		 pwm_compare8 = 1500;
	 		 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm_compare7);
	 		 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwm_compare8);
	 		 break;
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
