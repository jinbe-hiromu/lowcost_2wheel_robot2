/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AQM0802.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_COUNTER_PERIOD 10000
#define MOTOR_SPEED_BASE 10
#define SENSOR_MAX_VALUE 4095
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
uint16_t cnt = 0;
uint16_t timer = 0;
uint8_t led_state = GPIO_PIN_SET;
int8_t target_speed = 0;
uint16_t analog[4];
uint16_t mon = 0;

int left_speed = 0;
int right_speed = 0;
// PIDパラメータ（要調整）
float Kp = 1.5;  // Lineトレースは1.0
float Kd = 0.5;

// ライントレース or ウォールトレース
typedef enum {
	Line,
	Wall
}TraceType;

TraceType traceType = Wall;

int left_speed;
int right_speed;

// センサー閾値
int sensor_threshold = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void FS90R_SetSpeed(int8_t speed_l, int8_t speed_r) {
    speed_r *= -1; // 回転方向合わせ

    // 範囲制限
    if (speed_l > 100) speed_l = 100;
    if (speed_l < -100) speed_l = -100;
    if (speed_r > 100) speed_r = 100;
	if (speed_r < -100) speed_r = -100;

    // パルス幅（μs）範囲
    //const uint16_t PULSE_MIN_US = 700; //700
    const uint16_t PWM_PERIOD = 20000; //20ms
    const uint16_t PULSE_MIN_US_MARGIN = 700; //0.7ms
	const uint16_t PULSE_MAX_US_MARGIN = 2300; //2.3ms
    const uint16_t PULSE_NEUTRAL_US = 1500; // 1.5ms
    const uint16_t LEFT_CORRECTION = 0; //0.6ms

    uint32_t pulse_us_l, pulse_us_r;

    if (speed_l >= 0) {
//        pulse_us_l = PULSE_NEUTRAL_US + ((PULSE_NEUTRAL_US - PULSE_MIN_US_MARGIN) * speed_l / 100);
        pulse_us_l = PULSE_NEUTRAL_US + ((PULSE_NEUTRAL_US - PULSE_MIN_US_MARGIN - LEFT_CORRECTION) * speed_l / 100);
    } else {
//        pulse_us_l = PULSE_NEUTRAL_US + ((PULSE_MAX_US_MARGIN - PULSE_NEUTRAL_US) * speed_l / 100);
        pulse_us_l = PULSE_NEUTRAL_US + ((PULSE_MAX_US_MARGIN - PULSE_NEUTRAL_US - LEFT_CORRECTION) * speed_l / 100);
    }

    if (speed_r >= 0) {
  	   pulse_us_r = PULSE_NEUTRAL_US + ((PULSE_NEUTRAL_US - PULSE_MIN_US_MARGIN) * speed_r / 100) ;
   } else {
 	   pulse_us_r = PULSE_NEUTRAL_US + ((PULSE_MAX_US_MARGIN - PULSE_NEUTRAL_US) * speed_r / 100) ;
   }
    mon = pulse_us_r;

    // μs → タイマーのカウント値に変換
    uint32_t pulse_count_l = (pulse_us_l * SERVO_COUNTER_PERIOD) / PWM_PERIOD;
    uint32_t pulse_count_r = (pulse_us_r * SERVO_COUNTER_PERIOD) / PWM_PERIOD;

    // PWM出力
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_count_r);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_count_l);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   timer++;
   if(timer >= 100){
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, led_state);
	   led_state = !led_state;

	   timer = 0;
   }
}

void WRITE_LCD(bool isRunning, TraceType traceType){
	  lcd_clear();

	  lcd_locate(0,0);
	  if(isRunning){
		  lcd_printf("Run");
	  }else{
		  lcd_printf("Stop");
	  }

	  lcd_locate(0,1);
	  if(traceType == Wall){
		  lcd_printf("Wall");
	  }else{
		  lcd_printf("Line");
	  }
}

void WRITE_GAIN_LCD(float Kp, float Kd){
	  lcd_clear();

	  int kp_int = (int)Kp;
	  int kp_dec = (int)(Kp * 10) % 10;

	  lcd_locate(0,0);
	  lcd_printf("Kp = %d.%d", kp_int, kp_dec);

	  int kd_int = (int)Kd;
	  int kd_dec = (int)(Kd * 10) % 10;

	  lcd_locate(0,1);
	  lcd_printf("Kd = %d.%d", kd_int, kd_dec);
}

void ADJUST_GAIN(float* K){
  while(1){
	  HAL_Delay(100); // 制御周期

	  WRITE_GAIN_LCD(Kp, Kd);
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0){
		 if(*K < 3){
			 *K += 0.1;
		 }
	  }
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 0){
		  if(*K > 0){
			  *K -= 0.1;
		  }
	  }
	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0){
		  break;
	  }
  }
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  // タイマー割り込み
  HAL_TIM_Base_Start_IT(&htim14);
  // サーボモータPWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  lcd_init();

//  // センサースイッチング
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 4799); // ひとまず100%

  // ADCスタート
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) analog, 4);

  // PID変数
  float error = 0, last_error = 0;

  // 走行Mode
  bool isRunning = false;
  WRITE_LCD(isRunning, traceType);

  // 重みづけ
  int LineTraceWeights[4] = { 3,  1, -1, -3 }; // ライントレース用（右＋、左−）
  int WallTraceWeights[4] = { -1,  -3, 3, 1 }; // 壁トレース用（右-、左+） 検知方向と同じタイヤが動いて欲しいため左右逆転
  int weights[4]          = { 0,  0,  0,  0 }; // 現在の重み

  lcd_clear();
  lcd_locate(0,0);
  lcd_printf("Hello");
  lcd_locate(0,1);
  lcd_printf("World");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_Delay(100); // 制御周期
//	  i++;
//	  lcd_clear();
//	  lcd_locate(0,1);
//	  lcd_printf("i = %d", i);

	  WRITE_LCD(isRunning, traceType);

	  // 待機モード
	  while(1){
		  HAL_Delay(100); // 制御周期

		  // Start
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0){
			 isRunning = true;
			 WRITE_LCD(isRunning, traceType);
			 break;
		  }

		  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == 0){
			  if(traceType == Line){
				  traceType = Wall;
			  }else{
				  traceType = Line;
			  }
			  WRITE_LCD(isRunning, traceType);
		  }

		  // ゲイン調整モード
		  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0){
			  ADJUST_GAIN(&Kp);
			  ADJUST_GAIN(&Kd);

			  WRITE_LCD(isRunning, traceType);	// ゲイン調整終了後、元の画面に戻す
		  }
	  }

	  // 走行モード
	  while(1){
		  // Stop
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 0){
			  FS90R_SetSpeed(0, 0);
			  isRunning = false;
			  WRITE_LCD(isRunning, traceType);
			  break;
		  }

		int sum_val = 0, sum_weight = 0;

	    // ライントレース or ウォールトレース
	    if(traceType == Line){
	    	sensor_threshold = 1000;
	    	memcpy(weights, LineTraceWeights, sizeof(weights));	// ライントレース

	        for(int i=0; i<4; i++){
	            if(analog[i] > sensor_threshold){
	                sum_val += weights[i];
	                sum_weight++;
	            }
	        }
	    }else{
	    	//sensor_threshold = 4000;
	    	memcpy(weights, WallTraceWeights, sizeof(weights));	// ウォールトレース

	        for(int i=0; i<4; i++){
	        	int sensor_value = SENSOR_MAX_VALUE -analog[i];
	            sum_val += (sensor_value * weights[i]);
	            sum_weight += sensor_value;

	            // 閾値あり
	//            if(analog[i] < sensor_threshold){
	//                sum_val += weights[i];
	//                sum_weight++;
	//            }
	        }
	    }

	    if(sum_weight > 0){
	        error = (float)sum_val / sum_weight; // 平均位置
	    } else {
	        error = last_error; // ライン見失ったら前回値
	    }

	    // PD計算
	    float derivative = error - last_error;
	    float correction = Kp*error + Kd*derivative;

	    // 左右モータ速度
	    left_speed  = MOTOR_SPEED_BASE + correction;
	    right_speed = MOTOR_SPEED_BASE - correction;

	    FS90R_SetSpeed((int8_t)left_speed, (int8_t)right_speed);
	    last_error = error;

	    // 制御周期
	    HAL_Delay(20); // 制御周期
	  }


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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x10D45D7D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  htim3.Init.Prescaler = 95;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
