/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "SimpleFOC.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// default pwm parameters
#define _PWM_RESOLUTION 12 // 12bit
#define _PWM_RANGE 4095.0// 2^12 -1 = 4095
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
// Simple FOC setup:
BLDCMotor motor = BLDCMotor(15); //pole pair count on motor
BLDCDriver3PWM driver = BLDCDriver3PWM(9,5,6,8); //these pins are currently hard coded
//encoder instance
Encoder encoder = Encoder(2, 3, 800); //these pins, and values are actually hard coded
float start_angle = 0.0; //[rad]
//float vane_pos_inc[] = {0, 0.1, 1.571, 0.556, 3.142, 0.1, 4.712, 0.1};
float vane_pos_inc[] = { 0, 0.010, 0.010, 0.010, 0.010, 0.011, 0.011, 0.011, 0.011,
		0.012, 0.012, 0.012, 0.012, 0.013,0.013, 0.013, 0.013, 0.013, 0.013,
		0.014, 0.014, 0.014, 0.014, 0.014, 0.014, 0.014, 0.015, 0.014, 0.015,
		0.014, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015,
		0.015, 0.015, 0.016, 0.015, 0.015, 0.016, 0.015, 0.015, 0.016, 0.015,
		0.016, 0.015, 0.016, 0.015, 0.015, 0.016, 0.015, 0.016, 0.015, 0.016,
		0.015, 0.016, 0.015, 0.016, 0.015, 0.015, 0.012, 0.020, 0.010, 0.020,
		0.020, 0.010, 0.020, 0.010, 0.020, 0.010, 0.020, 0.010, 0.020, 0.010,
		0.020, 0.010, 0.020, 0.010, 0.020, 0.010, 0.020, 0.010, 0.020, 0.010,
		0.020, 0.010, 0.020, 0.010, 0.020, 0.010, 0.020, 0.010, 0.010, 0.020,
		0.010, 0.020, 0.010, 0.020, 0.010, 0.010, 0.020, 0.010, 0.020, 0.010,
		0.020, 0.010, 0.010, 0.020, 0.010, 0.010, 0.020, 0.010, 0.020, 0.010,
		0.010, 0.020, 0.010, 0.010, 0.020, 0.010, 0.010, 0.020, 0.010, 0.010,
		0.010, 0.020, 0.010, 0.010, 0.020, 0.010, 0.010, 0.010, 0.020, 0.010,
		0.010, 0.010, 0.010, 0.020, 0.010, 0.010, 0.010, 0.010, 0.010, 0.020,
		0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010,
		0.020, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.000, 0.010, 0.010,
		0.010, 0.000, 0.010, 0.010, 0.010, 0.010, 0.010, 0.000, 0.010, 0.010,
		0.000, 0.010, 0.010, 0.000, 0.010, 0.000, 0.010, 0.000, 0.010, 0.000,
		0.010, 0.000, 0.010, 0.000, 0.010, 0.000, 0.010, 0.000, 0.010, 0.000,
		0.000, 0.010, 0.000, 0.000, 0.010, 0.000, 0.010, 0.000, 0.000, 0.010,
		0.000, 0.000, 0.010, 0.000, 0.000, 0.010, 0.000, 0.000, 0.010, 0.000,
		0.000, 0.010, 0.000, 0.000, 0.010, 0.000, 0.000, 0.010, 0.000, 0.000,
		0.010, 0.000, 0.000, 0.010, 0.000, 0.000, 0.010, 0.000, 0.000, 0.010,
		0.000, 0.010, 0.000, 0.000, 0.010, 0.000, 0.000, 0.010, 0.000, 0.000,
		0.010, 0.000, 0.010, 0.000, 0.000, 0.010, 0.000, 0.000, 0.010, 0.000,
		0.010, 0.000, 0.000, 0.010, 0.000, 0.010, 0.000, 0.010, 0.000, 0.000,
		0.010, 0.000, 0.010, 0.000, 0.010, 0.000, 0.010, 0.000, 0.010, 0.000,
		0.010, 0.000, 0.010, 0.000, 0.010, 0.000, 0.010, 0.000, 0.010, 0.000,
		0.010, 0.000, 0.010, 0.000, 0.010, 0.000, 0.010, 0.010, 0.000, 0.010,
		0.000, 0.010, 0.010, 0.000, 0.010, 0.010, 0.000, 0.010, 0.000, 0.010,
		0.010, 0.010, 0.000, 0.010, 0.010, 0.000, 0.010, 0.010, 0.010, 0.000,
		0.010, 0.010, 0.010, 0.000, 0.010, 0.010, 0.010 };
float vane_pos[] = {0, 0.5, 0.2, 0.3, 0.5, 0, 1, 0.5};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
//- hardware specific
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
  // transform duty cycle from [0,1] to [0,4095]
	//TIM1->CCR1 = (int)(dc_a * _PWM_RANGE);
//	TIM1->CCR2 = (int)(dc_b * _PWM_RANGE);
//	TIM4->CCR4 = (int)(dc_c * _PWM_RANGE);

	int pwm_range = htim1.Instance->ARR;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dc_a * pwm_range);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dc_b * pwm_range);
	pwm_range = htim4.Instance->ARR;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, dc_c * pwm_range);
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
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

//  HAL_TIM_Base_Start_IT(&htim1);
//  HAL_TIM_Base_Start_IT(&htim4);

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim4);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  //SimpleFOC initialisations:

//  //open loop velocity control
//  driver.voltage_power_supply = 24; //power supply voltage [V]
//  driver.init();
//  motor.linkDriver(&driver); //link the motor and the driver
//  motor.voltage_limit = 3;
//  motor.velocity_limit = 1;
//  motor.controller = ControlType::velocity_openloop; //set motion control loop to be used
//  motor.init(); //initialise motor

//  //closed loop velocity control:
//  encoder.init(); //initialise encoder sensor hardware
//  motor.linkSensor(&encoder); //link the motor to the sensor
//  driver.voltage_power_supply = 24;
//  driver.init();
//  //link the motor and the driver
//  motor.linkDriver(&driver);
//  //aligning voltage
//  motor.voltage_sensor_align = 3;
//  //index search velocity [rad/s]
//  motor.velocity_index_search = 3;
//  //set motion control loop to be used:
//  motor.controller = ControlType::velocity;
//  //velocity PI controller parameters
//  motor.PID_velocity.P = 0.2;
//  motor.PID_velocity.I = 20;
//  motor.PID_velocity.D = 0;
//  motor.voltage_limit = 6; //default: voltage_power_supply
//  //jerk control using voltage ramp
//  //default value is 300 volts per sec - 0.3V per millisecond
//  motor.PID_velocity.output_ramp = 1000;
//  //velocity low pass filtering time constant
//  motor.LPF_velocity.Tf = 0.01;
//  //initialise motor
//  motor.init();
//  //align sensor and start FOC
//  motor.initFOC();

  //closed loop position control:
  encoder.init();
  motor.linkSensor(&encoder);
  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 3; //[V]
  motor.velocity_index_search = 3; //[rad/s]
  motor.controller = ControlType::angle;

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  motor.voltage_limit = 10;
  motor.PID_velocity.output_ramp = 1000; //[V/s]

  motor.LPF_velocity.Tf = 0.01;
  motor.P_angle.P = 20;
  motor.velocity_limit = 8;

  motor.init();
  motor.initFOC();

  _delay(1000);

  int idx = 0;
  int angle_cnt = 0;
  float target_angle = 0.9;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  //open loop velocity:
//	  motor.move(1);


//	  //closed loop velocity control
//	  motor.loopFOC();
//	  idx += 1;
//	  if ((idx % 1024) == 0) {
//		  motor.move(0.5);
//		  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//	  }
//	  if (idx >= 65536) {
//		  idx = 1;
//	  }

	  //closed loop position control
	  motor.loopFOC();
	  //motor.move(target_angle);
	  idx++;
	  if (idx % 500 == 0) {
		  target_angle += vane_pos_inc[angle_cnt];
		//  target_angle = vane_pos[angle_cnt];
		  angle_cnt++;
		  angle_cnt = (angle_cnt%327);
//		  if (angle_cnt >= 8) {
//			  //angle_cnt = 0;
//			  _delay(10000);
//		  }
		//  target_angle -= 5;
		  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	  }
	  if (idx % 64 == 0) {
		  motor.move(target_angle);
	  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = 4800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim4.Init.Period = 4800;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Pin8_Enable_GPIO_Port, Pin8_Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Pin8_Enable_Pin */
  GPIO_InitStruct.Pin = Pin8_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Pin8_Enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Pin2_EnA_Pin */
  GPIO_InitStruct.Pin = Pin2_EnA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pin2_EnA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Pin3_EnB_Pin */
  GPIO_InitStruct.Pin = Pin3_EnB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pin3_EnB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == Pin2_EnA_Pin) {
		encoder.handleA();
	}
	if (GPIO_Pin == Pin3_EnB_Pin) {
		encoder.handleB();
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
