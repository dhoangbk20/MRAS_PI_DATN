/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  * Author: Le Dinh Hoang
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float f1 = 0,f2 = 0;
uint32_t a,b,c,d,e,f,g,h,t =0;
uint8_t txbuffer[16];//Buffer truyen du lieu tu STM32 toi RaspberryPi. Bao gom 2 toc do cua dong co +000.00/+000.00/n
uint8_t temp_rxbuffer[17];
uint8_t y[24];
uint8_t tx[8];
uint8_t f12bytes[sizeof(float)+1];
uint8_t f22bytes[sizeof(float)];
float bytes2f=0;
double time_sample = 0.005; //Thoi gian lay mau roi rac 5ms
char motorState1 = 0, motorState2 = 0;
uint8_t rxbuffer[17]; //Buffer nhan du lieu tu RaspberryPi thong qua USART2 dung DMA.... +00.00/+00.00/y/n
/*--------Thong so ve dong co trai */ //Motor1 = dong co trai
double set1 = 0,set1_1 = 0;
int32_t setpoint1 = -50, setpoint1_1 = 0,setpoint1_2 =0; //Toc do dat cua dong co trai,setpoint = uc(k), setpoint1_1 = uc(k-1), setpoint1_2 = uc(k-2);
double cnt_encoder1, pre_cnt_encoder1,cnt; //Gia tri doc encoder va gia tri trong qua khu
double motor1Velocity,motor1Velocity1,motor1Velocity2,motor1Velocity3;//Toc do thuc te ngo ra y voi motor1Velocity = y1(k), motor1Velocity1 = y1(k-1), motor1Velocity2 = y1(k-2)...
double motor1Velocity_m,motor1Velocity1_m,motor1Velocity2_m; //Toc do ngo ra ym mo hinh chuan motor1Velocity_m = ym(k), motor1Velocity1_m = ym(k-1);
double e1_c, e1_c1, e1_c2; //Sai so giua setpoint va toc do thuc te e1_c = e1(k), e1_c1 = e1(k-1), e1 = setpoint1 - motor1Velocity;
double e1_m, e1_m1, e1_m2; //Sai so giua ngo ra thuc te va ngo ra mo hinh  chuan e1_m = e1(k), e1_m1 = e1(k-1), motor1Velocity - motor1Velocity_m
double delta1_Kp,delta1_Ki;//Bien gán da thuc trong lut MIT de roi rac
double delta1_Kp_k1, delta1_Kp_k2;
double delta1_Ki_k1,delta1_Ki_k2;
double alpha1,beta1;
double Kp1 = 0 ,Ki1 = 0;
//double Kp1_k1,Ki1_k1,Kd1_k1;
double u1_k,u1_k1,u1_k2;
double gamma1=0.00001; //He so thich nghi
double biendem = 0;
//Luat dieu khien u1_k = u(k), u1_k1 = u(k-1)

/* Thong so ve dong co phai */ //Motor2 = dong co phai
double set2 = 0,set2_1 = 0;
int32_t setpoint2 = 50 ,setpoint2_1,setpoint2_2;; //Toc do dat cua dong co phai
double cnt_encoder2, pre_cnt_encoder2; //Gia tri doc encoder va gia tri trong qua khu
double motor2Velocity,motor2Velocity1 = 0,motor2Velocity2,motor2Velocity3;//Toc do thuc te ngo ra y voi motor2Velocity = y2(k), motor2Velocity1 = y2(k-1), motor2Velocity2 = y2(k-2)...
double motor2Velocity_m,motor2Velocity1_m,motor2Velocity2_m; //Toc do ngo ra ym mo hinh chuan motor2Velocity_m = y2m(k), motor2Velocity1_m = y2m(k-1);
double e2_c, e2_c1, e2_c2; //Sai so giua setpoint va toc do thuc te e2_c = e2(k), e2_c1 = e2(k-1), e2 = setpoint2 - motor2Velocity;
double e2_m, e2_m1, e2_m2; //Sai so giua ngo ra thuc te va ngo ra mo hinh  chuan e2_m = e2(k), e2_m1 = e2(k-1), motor2Velocity - motor2Velocity_m
double delta2_Kp,delta2_Ki;//Bien gán da thuc trong lut MIT de roi rac
double delta2_Kp_k1, delta2_Kp_k2;
double delta2_Ki_k1,delta2_Ki_k2;
double alpha2,beta2;
double Kp2,Ki2;
double Kp2_k1,Ki2_k1;
double u2_k,u2_k1,u2_k2;
double gamma2 = 0.00001; //He so thich nghi

int time_parameter = 0;
double pitch = 0, roll = 0, yaw = 0, yad_radian = 0;
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_UART_Receive_DMA(&huart2, rxbuffer, sizeof(rxbuffer));
// HAL_UART_Transmit_DMA(&huart2, y, sizeof(y));
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8399;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 83;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 99;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart2.Instance)
	{
		biendem++;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart2.Instance)
	{

		if(rxbuffer[14] == 'y') //+00.00/+00.00/y/n
		{
			setpoint1 = (double)(rxbuffer[1]-48)*10 + (double)(rxbuffer[2]-48)
						+ (double)(rxbuffer[4]-48)/10 + (double)(rxbuffer[5]-48)/100;
			setpoint2 = (double)(rxbuffer[8]-48)*10 + (double)(rxbuffer[9]-48)
						+ (double)(rxbuffer[11]-48)/10 + (double)(rxbuffer[12]-48)/100 ;
			if(rxbuffer[0] == '-')
			{
				 setpoint1 = -setpoint1;
			}
			if(rxbuffer[7] == '-')
			{
				setpoint2 = -setpoint2;
			}
		}
		temp_rxbuffer[14] = '0';


	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim6.Instance)
	{
		/* ----------Dong co trai (dong co 1)------------- */
		cnt_encoder1 = __HAL_TIM_GET_COUNTER(&htim3);
		if(abs((int)(cnt_encoder1 - pre_cnt_encoder1)) < 13200) motor1Velocity2 = (cnt_encoder1 - pre_cnt_encoder1)*60.0*100/1320;
		motor1Velocity = 0.9753*motor1Velocity1 + 0.02469*motor1Velocity3; //Bộ l�?c thông thấp
		motor1Velocity1 = motor1Velocity; //y(k-1) = y(k)
		motor1Velocity3 = motor1Velocity2; // y(k-3) = y(k-2);
		pre_cnt_encoder1 = cnt_encoder1;

		e1_c = setpoint1 - motor1Velocity; //Sai số giữa sai số tốc độ đặt và tốc độ thực tế

		motor1Velocity_m = 1.921*motor1Velocity1_m - 0.9231*motor1Velocity2_m + 0.001217*setpoint1 + 0.001185*setpoint1; // Ngõ ra của mô hình chuẩn
		e1_m = motor1Velocity - motor1Velocity1_m; //Sai số giữa tốc độ của đối tượng động cơ DC và tốc độ theo mô hình chuẩn


		delta1_Kp = 0.1225*e1_c1 - 0.1225*e1_c2 + 1.921*delta1_Kp_k1 - 0.9231*delta1_Kp_k2;
		Kp1 += -gamma1*e1_m*delta1_Kp;
		delta1_Ki = 0.0003084*e1_c1 + 0.0003043*e1_c2 + 1.921*delta1_Ki_k1 - 0.9231*delta1_Ki_k2;
		Ki1 += -gamma1*e1_m*delta1_Ki;

		alpha1 = Kp1*(e1_c-e1_c1);
		beta1 = time_sample/2*Ki1*(e1_c+e1_c1);
		u1_k = u1_k1 + alpha1 + beta1;

		motor1Velocity2_m = motor1Velocity1_m; //ym(k-2) = ym(k-1);
		motor1Velocity1_m = motor1Velocity_m; //ym(k-1) = ym(k);

		u1_k1 = u1_k;
		e1_c2 = e1_c1; //e(k-2) = e(k-1)
		e1_c1 = e1_c; //e(k-1) = e(k);

		e1_m2 = e1_m1; //em(k-2) = em(k-1);
		e1_m1 = e1_m; //em(k-1) = em(k);

		delta1_Kp_k2 = delta1_Kp_k1; // delKp(k-2) = delKp(k-1);
		delta1_Kp_k1 = delta1_Kp;

		delta1_Ki_k2 = delta1_Ki_k1;
		delta1_Ki_k1 = delta1_Ki;

	//Thêm khâu bão hòa
		if(setpoint1 == 0) u1_k = 0;
		if(u1_k >= 0)
		{
			if(u1_k > 100) u1_k = 100;
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET); //forward
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, u1_k);
		}
		else if(u1_k < 0)
		{
			if(u1_k < -100) u1_k = -100;
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET); //inverse
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, -u1_k);
		}
		else if(u1_k == 0)
		{
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, u1_k);
		}
//		if(u1_k > 100) u1_k = 100;
//		else if(u1_k < -100) u1_k = -100;
//		if(u1_k > 0)
//		{
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET); //forward
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
//		}
//		else if (u1_k <0 )
//		{
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET); //inverse
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
//			u1_k = -u1_k;
//
//		}else u1_k = 0;


		/* ----------Dong co phai (dong co 2)------------- */
		cnt_encoder2 = __HAL_TIM_GET_COUNTER(&htim4);
		if(abs((int)(cnt_encoder2-pre_cnt_encoder2))<13200) motor2Velocity2 = (cnt_encoder2-pre_cnt_encoder2)*60.0*100/1320;
		motor2Velocity = 0.9753*motor2Velocity1 + 0.02469*motor2Velocity3; //Bộ l�?c thông thấp
		motor2Velocity1 = motor2Velocity; //y(k-1) = y(k)
		motor2Velocity3 = motor2Velocity2; // y(k-3) = y(k-2);
		pre_cnt_encoder2 = cnt_encoder2;

		e2_c = setpoint2 - motor2Velocity; //Sai số giữa sai số tốc độ đặt và tốc độ thực tế

		motor2Velocity_m = 1.921*motor2Velocity1_m - 0.9231*motor2Velocity2_m + 0.001217*setpoint2 + 0.001185*setpoint2; // Ngõ ra của mô hình chuẩn
		e2_m = motor2Velocity - motor2Velocity1_m; //Sai số giữa tốc độ của đối tượng động cơ DC và tốc độ theo mô hình chuẩn


		delta2_Kp = 0.1225*e2_c1 - 0.1225*e2_c2  + 1.921*delta2_Kp_k1 - 0.9231*delta2_Kp_k2;
		Kp2 += -gamma2*e2_m*delta2_Kp;
		delta2_Ki = 0.0003084*e2_c1 + 0.0003043*e2_c2 + 1.921*delta2_Ki_k1 - 0.9231*delta2_Ki_k2;
		Ki2 += -gamma2*e2_m*delta2_Ki;

		alpha2 = Kp2*(e2_c-e2_c1);
		beta2 = time_sample/2*Ki2*(e2_c+e2_c1);
		u2_k = u2_k1 + alpha2 + beta2;

		motor2Velocity2_m = motor2Velocity1_m; //ym(k-2) = ym(k-1);
		motor2Velocity1_m = motor2Velocity_m; //ym(k-1) = ym(k);

		e2_c2 = e2_c1; //e(k-2) = e(k-1)
		e2_c1 = e2_c; //e(k-1) = e(k);

		e2_m2 = e2_m1; //em(k-2) = em(k-1);
		e2_m1 = e2_m; //em(k-1) = em(k);

		delta2_Kp_k2 = delta2_Kp_k1; // delKp(k-2) = delKp(k-1);
		delta2_Kp_k1 = delta2_Kp;

		delta2_Ki_k2 = delta2_Ki_k1;
		delta2_Ki_k1 = delta2_Ki;

		u2_k1 = u2_k;

		//Thêm khâu bão hòa
		if(setpoint2 == 0) u2_k = 0;
		if(u2_k < 0)
		{
			if(u2_k < -100) u2_k = -100;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET); //INVERSE
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, -u2_k);
		}
		else if(u2_k >=0)
		{
			if(u2_k > 100) u2_k = 100;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET); //FORWARD
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, u2_k); //PC7 banh phai
		}
		else if(u2_k == 0)
		{
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,u2_k);
		}



		// Truyen du lieu UART frame truyen +000.00/+000.00/n  +120.00 45.00 -50.00
		//Truyen hang tram
		int16_t left_temp,right_temp;
		int32_t setpoint1_temp, setpoint2_temp;
		if(motor1Velocity1 >= 0)
		{
			left_temp = 100*motor1Velocity1;
			y[0] = '+';
		}
		else
		{
			left_temp = -100*motor1Velocity1;
			y[0] = '-';
		}
		y[1] = left_temp/10000 + 48;
		left_temp %= 10000;
		y[2] = left_temp/1000 + 48;
		left_temp %= 1000;
		y[3] = left_temp/100 + 48;
		left_temp %= 100;
		y[4] = 0x2E;
		y[5] = left_temp/10 + 48;
		y[6] = left_temp%10 + 48;
		y[7] = ',' ;

		if(motor2Velocity1 >= 0)
		{
			right_temp = 100*motor2Velocity1;
			y[8] = '+';
		}
		else
		{
			right_temp = -100*motor2Velocity1;
			y[8] = '-';
		}
		y[9] = right_temp/10000 + 48;
		right_temp %= 10000;
		y[10] = right_temp/1000 + 48;
		right_temp %= 1000;
		y[11] = right_temp/100 + 48;
		right_temp %= 100;
		y[12] = 0x2E;
		y[13] = right_temp/10 + 48;
		y[14] = right_temp%10 + 48;
		if(setpoint1 > 0 && setpoint2 > 0)
		{
			a++;
			y[15] = '+';
			y[16] = setpoint1/10 + 48;
			y[17] = setpoint1%10 + 48;
			y[18] = ',';
			y[19] = '+';
			y[20] = setpoint2/10 + 48;
			y[21] = setpoint2%10 + 48;

		}
		else if(setpoint1 > 0 && setpoint2 < 0)
		{
			b++;
			setpoint2_temp = -setpoint2;
			y[15] = '+';
			y[16] = setpoint1/10 + 48;
			y[17] = setpoint1%10 + 48;
			y[18] = ',';
			y[19] = '-';
			y[20] = setpoint2_temp/10 + 48;
			y[21] = setpoint2_temp%10 + 48;
		}
		else if(setpoint1 < 0 && setpoint2 < 0)
		{
			c++;
			setpoint1_temp = -setpoint1;
			setpoint2_temp = -setpoint2;
			y[15] = '-';
			y[16] = setpoint1_temp/10 + 48;
			y[17] = setpoint1_temp%10 + 48;
			y[18] = ',';
			y[19] = '-';
			y[20] = setpoint2_temp/10 + 48;
			y[21] = setpoint2_temp%10 + 48;
		}
		else if(setpoint1 > 0 && setpoint2 == 0)
		{
			d++;
			y[15] = '+';
			y[16] = setpoint1/10 + 48;
			y[17] = setpoint1%10 + 48;
			y[18] = ',';
			y[19] = '+';
			y[20] = '0';
			y[21] = '0';
		}
		else if(setpoint1 == 0 && setpoint2 > 0)
		{
			e++;
			y[15] = '+';
			y[16] = '0';
			y[17] = '0';
			y[18] = ',';
			y[19] = '+';
			y[20] = setpoint2/10 + 48;
			y[21] = setpoint2%10 + 48;
		}
		else if(setpoint1 < 0 && setpoint2 ==0)
		{	f++;
			setpoint1_temp = -setpoint1;
			y[15] = '-';
			y[16] = setpoint1_temp/10 + 48;
			y[17] = setpoint1_temp%10 + 48;
			y[18] = ',';
			y[19] = '+';
			y[20] = setpoint2/10 + 48;
			y[21] = setpoint2%10 + 48;
		}
		else if(setpoint1 == 0 && setpoint2 <0)
		{
			g++;
			setpoint2_temp = -setpoint2;
			y[15] = '+';
			y[16] = '0';
			y[17] = '0';
			y[18] = ',';
			y[19] = '-';
			y[20] = setpoint2_temp/10 + 48;
			y[21] = setpoint2_temp%10 + 48;
		}
		else if(setpoint1 == 0 && setpoint2 == 0)
		{
			h++;
			y[15] = '+';
			y[16] = '0';
			y[17] = '0';
			y[18] = ',';
			y[19] = '+';
			y[20] = '0';
			y[21] = '0';
		}
		else if(setpoint1 < 0 && setpoint2 > 0)
		{
			t++;
			setpoint1_temp = -setpoint1;
			y[15] = '-';
			y[16] = setpoint1_temp/10 + 48;
			y[17] = setpoint1_temp%10 + 48;
			y[18] = ',';
			y[19] = '+';
			y[20] = setpoint2/10 + 48;
			y[21] = setpoint2%10 + 48;
		}
		y[22] = '/';
		y[23] = '\n' ;
		//HAL_UART_Transmit(&huart2,y,sizeof(y),2000);

		//for(int i = 0;i<16;i++)
		//{
			//txbuffer[i] = y[i];
		//}



	}
	if(htim->Instance == htim7.Instance)
	{
		f1 = -3.1415;
		*(float*)f12bytes=f1;
		bytes2f  = *(float*)(f12bytes);
		f12bytes[4] = '\n';
		HAL_UART_Transmit(&huart2,y,sizeof(y),2000);
		biendem++;

	}


}
/*
void IntToString18(int16_t left,int16_t right, uint8_t *y) //+100.000/+100.000\n
{
		int16_t left_temp,right_temp;

		left_temp = 1000*left;
		y[0] = (left_temp >= 0)?'+':'-';
		y[7] = left_temp % 10 + 0x30;
		left_temp = left_temp / 10;
		y[6] = left_temp % 10 + 0x30;
		left_temp = left_temp / 10;
		y[5] = left_temp % 10 + 0x30;
		left_temp = left_temp / 10;
		y[4] = 0x2E;
		y[3] = left_temp % 10 + 0x30;
		left_temp = left_temp / 10;
		y[2] = left_temp % 10 + 0x30;
		left_temp = left_temp / 10;
		y[1] = left_temp % 10 + 0x30;

		y[8] = '/';

		right_temp = 1000*right;
		y[9] = (right_temp >= 0)?'+':'-';
		y[16] = right_temp % 10 + 0x30;
		right_temp = right_temp / 10;
		y[15] = right_temp % 10 + 0x30;
		right_temp = right_temp / 10;
		y[14] = right_temp % 10 + 0x30;
		right_temp = right_temp / 10;
		y[13] = 0x2E;
		y[12] = right_temp % 10 + 0x30;
		right_temp = right_temp / 10;
		y[11] = right_temp % 10 + 0x30;
		right_temp = right_temp / 10;
		y[10] = right_temp % 10 + 0x30;
		y[17] = '\n';
}
*/
/*
uint16_t StringToInt18(uint16_t *setpoint,uint8_t *y)
{
		int temp,setpoint1,setpoint2;

		temp = y[1] - 0x30;
		setpoint1 = temp*100;
		temp = y[2] - 0x30;
		setpoint1 = setpoint1 + temp*10;
		temp = y[3] - 0x30;
		setpoint1 = setpoint1 + temp;
		//y[4] la dau cham
		temp = y[5] - 0x30;
		setpoint1 = setpoint1 + temp/10;
		temp = y[6] - 0x30;
		setpoint1 = setpoint1 + temp/100;
		temp = y[7] - 0x30;
		setpoint1 = setpoint1 + temp/1000;


		temp = y[10] - 0x30;
		setpoint2 = temp*100;
		temp = y[11] - 0x30;
		setpoint2 = setpoint2 + temp*10;
		temp = y[12] - 0x30;
		setpoint2 = setpoint2 + temp;
		//y[13] la dau cham
		temp = y[14] - 0x30;
		setpoint2 = setpoint2 + temp/10;
		temp = y[15] - 0x30;
		setpoint2 = setpoint2 + temp/100;
		temp = y[16] - 0x30;
		setpoint2 = setpoint2 + temp/1000;

		setpoint1 = (y[0] == '-')?-setpoint1:setpoint1;
		setpoint2 = (y[9] == '-')?-setpoint1:setpoint2;
		setpoint[0] = setpoint1; // left
		setpoint[1] = setpoint2; // right
		setpoint[3] = y[17] - 0x30; //Trang thai On/Off dong co
		return setpoint;
}
*/
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
