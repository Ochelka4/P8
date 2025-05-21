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
#include "math.h"
#include "string.h"
#include "stdio.h"
#include <stdbool.h>

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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint32_t last_tick = 0;
int toggle_ISR1 = 0; //Toggle variable for ISR1
int toggle_ISR2 = 0; //Toggle variable for ISR2
int toggle_ISR3 = 0;  //Toggle variable for ISR3
double DC_a; //Duty cycle A
double DC_b; //Duty cycle B
double DC_c;  //Duty cycle C
float pi = 3.1415; //PI
float freq_sin; // frequency with a 2.25 factor for sine wave calculation
float freq_sine = 0.1; // actual frequency for voltage calculation
volatile float N_current = 0; // Current speed ramped towards target speed
float N_target = 0;  // Target speed
float V_RL = 0; // RMS line to line voltage
float V_PP = 0; // peak phase voltage
float boost = 1;  // boost procent 
float V_boost = 0; //Voltage boost
uint16_t sector;  //Variable for which sector we are currently in (SVPWM)
uint16_t V_DC = 540;   //DC voltage
float t = 0;   //Time for manual timer
double T_s = 0.0001;  //Sample time 
uint32_t ReadADC;  //ADC variable from potentiometer (0 - 4096)
double f_clock = 72000000.0;  //Nucleo boards clock frequency
uint16_t Volt = 0;    // Voltage variable for testing amplitude of sine wave
int direction1 = 2; // Default direction (Up)
int direction2 = 4; // Default direction (Up)
uint32_t startTime;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);                   //Red LED
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);     // Brake resistor signal


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_ADC_Start_IT(&hadc1); //Potentiometer trigger
	  HAL_Delay(20); //

	  if (toggle_ISR3 == 1)   // Reset pulse signal | Needed here as HAL_Delay doesn't work in ISR
	  {
		  toggle_ISR3 = 0;
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); //Reset signal pulse on
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);  //Reset signal pulse off
	  }

	  /*
	     //Code used for validation of the system
	  uint32_t time_current = HAL_GetTick() - startTime;
	  uint32_t rampstart = 1000;
	  uint32_t rampend = 3000;
	  if (time_current >= rampstart) N_target = 1183;
	  if (time_current >= rampend) N_target = 0;
		*/

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
     //Timer 3 is set to 10kHz with best possible resolution
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim3.Init.Period = 7200 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_REF_CLK_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_REF_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin PB9 */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  // Enable GPIO Clocks
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  // Configure Button (e.g., PC13 on Nucleo boards)
   GPIO_InitStruct.Pin = GPIO_PIN_13;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Falling edge interrupt
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   // Enable and set EXTI line 15_10 Interrupt to the lowest priority
   HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  // Button ITR
{
    if (GPIO_Pin == GPIO_PIN_13)  // Start / Stop button ISR
    {
    	if (HAL_GetTick() - last_tick > 100)  // 100 ms debounce
    	{
            last_tick = HAL_GetTick();
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);  //Green led
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //Red led

            if (toggle_ISR1 == 0)
            {
            	startTime = HAL_GetTick();
            	HAL_TIM_Base_Start_IT(&htim3);
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //Enable signal for interface card
                toggle_ISR1 = 1;

            }
            else
            {
            	DC_a = 0;       //Set duty cycles and speed to 0 when stopping
            	DC_b = 0;
            	DC_c = 0;
            	N_current = 0;
            	HAL_TIM_Base_Stop_IT(&htim3);
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  //Enable signal for interface card
                toggle_ISR1 = 0;
            }
    	}

    }
    else if (GPIO_Pin == GPIO_PIN_6)  // Directional change button ISR
    {
    	if (HAL_GetTick() - last_tick > 100)  // 100 ms debounce
    	{
    		last_tick = HAL_GetTick();
    		if (N_current == 0)     // Only change direction when speed is 0
    		{
    			if (toggle_ISR2 == 0)
    			{
    			    direction1 = 4;    //Down
    			    direction2 = 2;
    			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);  //Toggle blue led
    			    toggle_ISR2 = 1;
    			}
    			else
    			{
    			    direction1 = 2;     //Up
    			    direction2 = 4;
    			    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);  //Toggle blue led
    			    toggle_ISR2 = 0;
    			}
    		}
    	}

    }
    else if (GPIO_Pin == GPIO_PIN_7)  // Reset VFD button ISR
	{
    	if (HAL_GetTick() - last_tick > 100)   // 100 ms debounce
    	{
    		last_tick = HAL_GetTick();
    		toggle_ISR3 = 1;  // ISR only changes toggle variable as delay function doesn't work here, the rest is in the while loop
    	} 

	}
}

void EXTI15_10_IRQHandler(void)   //Enables the button ITR
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
         uint16_t readADC = HAL_ADC_GetValue(hadc);  //Read ADC signal from potentiometer
         N_target = readADC * 1183 / 4096; // Calculate new speed based on readADC

         //Used for amplitude testing
         //Volt = readADC * 400/4096; // Calculate volt from potentiometer for amplitude testing
         //if (Volt < 2) Volt = 1;   //Sets Volt to 1 if less than 2
         //N_current = 1500 ;


         uint16_t step_up = 24;      // Step size of ramp up for speed control
         uint16_t step_down = 79;    // Step size of ramp down for speed control
         if (N_target > N_current + step_up) //If statement for speed ramp in both directions
             N_current += step_up;
         else if (N_target < N_current - step_down)
             N_current -= step_down;
         else
             N_current = N_target;

    	if (N_current < 11 && N_target < 11) N_current = 0; //Sets the speed to 0 if the RPM current is less than 11
    	if (N_current > 1176 && N_target < 1176) N_current = 1183; //Sets the speed to 1500 if the RPM current is more than 1489

         freq_sin = 2.25*(N_current*4)/120;
         V_RL = freq_sine*8;   //RMS line to line voltage
         V_PP = V_RL*sqrt(2)/sqrt(3);   //peak phase voltage

    }
}


// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)      //Timer ITR to calculate and change the duty cycle
{
	 static int tick_count = 0;    // Counter variable 
	 static double t = 0.0;    //Timer variable for manual timer
	 static double t_step = 1.0/(72000000.0/(7200));   //Step for manual timer

  if (htim == &htim3) {    //Checking for timer 3 overflow
	  tick_count += 1;
	  if (tick_count >= 9) {   //Counter for changing how much duty cycles is calculated
		  tick_count = 0;    //Resets counter

	  	  	  if (freq_sin <= 10.0) boost = 0.03; //Boost at 3% until 10 Hz
	  	  	  else if (freq_sin >= 20.0) boost = 0.0; //Boost at 0% at 20 Hz
	  	  	  else {
	  	  		  float fade = (20.0 - freq_sin) / 10.0;   //Fade out the voltage boost between 10 and 20Hz
	  	  		  boost = 0.03 * fade;
	  	  	  }

	  	    V_boost = (230*boost + (400.0/(sqrtf(3.0)) - 230*boost)/50 * freq_sin) * sqrtf(2.0); //Voltage boost

	  	  	if (V_boost > 400) V_boost = 400; //Over current protection

	  	    float phase = (2.0*pi*freq_sin*t);   //Phase calculation

	  	    //Sine waves
	  	  	float  s_a = V_PP * sinf(phase);
	 		float  s_b = V_PP * sinf(phase - direction1*pi/3.0);
	 		float  s_c = V_PP * sinf(phase - direction2*pi/3.0);

	 		  //Clarke transformation
	 		  float s_alpha = (2.0/3.0)*(s_a - (1.0/2.0)*(s_b + s_c));
	 		  float s_beta = (sqrtf(3.0)/3.0)*(s_b - s_c);

	 		  //Space Vector Coordinates Calculator
	 		  float SV_theta = atan2f(s_beta, s_alpha);

	 		  // Ensures space vector angle is always positive
	 		  if (SV_theta < 0.0)
	 		  {
	 		  	SV_theta = SV_theta + 2.0*pi;
	 		  }

	 		  // Magnitude of Space vector
	 		  float SV_m = sqrtf(s_alpha*s_alpha+s_beta*s_beta);

	 		  //Modulation Index Calculator
	 		  float M = SV_m/(V_DC/sqrtf(3.0));
	 		  if (M > 1.0) M = 1.0; //Overflow protection

	 		  //Sector Calculator
	 		  if (0.0 <= SV_theta && SV_theta < pi/3.0)
	 		  {
	 			  sector = 1;
	 		  }
	 		  else if (pi/3.0 <= SV_theta && SV_theta < 2.0*pi/3.0)
	 		  {
	 			  sector = 2;
	 		  }
	 		  else if (2.0*pi/3.0 <= SV_theta && SV_theta < 3.0*pi/3.0)
	 		  {
	 			  sector = 3;
	 		  }
	 		  else if (3.0*pi/3.0 <= SV_theta && SV_theta < 4.0*pi/3.0)
	 		  {
	 			  sector = 4;
	 		  }
	 		  else if (4*pi/3 <= SV_theta && SV_theta < 5.0*pi/3.0)
	 		  {
	 			  sector = 5;
	 		  }
	 		  else if (5.0*pi/3.0 <= SV_theta && SV_theta < 6.0*pi/3.0)
	 		  {
	 			  sector = 6;
	 		  }

	 		  // Space vector angle from begining of current sector
	 		  float sector_theta = SV_theta - (sector-1.0)*(pi/3.0);

	 		  //Dwell Time Calculator
	 		  float T_1 = T_s*M*sinf(pi/3.0 - sector_theta);
	 		  float T_2 = T_s*M*sinf(sector_theta);
	 		  float T_0 = T_s - (T_1 + T_2);

	 		  //Duty cycle calculator based on current sector
	 		  if (sector == 1)
	 		  {
	 			DC_a = (2*(T_1 + T_2) + T_0)/(2*T_s) * 100;
	 			DC_b = (2*T_2 + T_0)/(2*T_s) * 100;
	 			DC_c = T_0/(2*T_s) * 100;
	 		  }
	 		  else if (sector == 2)
	 		  {
	 			DC_a = (2*T_1 + T_0)/(2*T_s) * 100;
	 			DC_b = (2*(T_1 + T_2) + T_0)/(2*T_s) * 100;
	 			DC_c = T_0/(2*T_s) * 100;
	 		  }
	 		  else if (sector == 3)
	 		  {
	 			DC_a = T_0/(2*T_s) * 100;
	 			DC_b = (2*(T_1 + T_2) + T_0)/(2*T_s) * 100;
	 			DC_c = (2*T_2 + T_0)/(2*T_s) * 100;
	 		  }
	 		  else if (sector == 4)
	 		  {
	 			DC_a = T_0/(2*T_s) * 100;
	 			DC_b = (2*T_1 + T_0)/(2*T_s) * 100;
	 			DC_c = (2*(T_1 + T_2) + T_0)/(2*T_s) * 100;
	 		  }
	 		  else if (sector == 5)
	 		  {
	 			DC_a = (2*T_2 + T_0)/(2*T_s) * 100;
	 			DC_b = T_0/(2*T_s) * 100;
	 			DC_c = (2*(T_1 + T_2) + T_0)/(2*T_s) * 100;
	 		  }
	 		  else if (sector == 6)
	 		  {
	 		  	DC_a = (2*(T_1 + T_2) + T_0)/(2*T_s) * 100;
	 		  	DC_b = T_0/(2*T_s) * 100;
	 		  	DC_c = (2*T_1 + T_0)/(2*T_s) * 100;
	 		  }

	 		 //Dead time
	 		 uint32_t dead_time_ticks = 250;  //Current dead time in ticks

	 		//Converting to ticks
	 		double desired_DC_a_ticks = (DC_a * (__HAL_TIM_GET_AUTORELOAD(&htim3) + 1)) / 100;
	 		double desired_DC_b_ticks = (DC_b * (__HAL_TIM_GET_AUTORELOAD(&htim3) + 1)) / 100;
	 		double desired_DC_c_ticks = (DC_c * (__HAL_TIM_GET_AUTORELOAD(&htim3) + 1)) / 100;

	 		//Apply dead time compensation
	 		if (desired_DC_a_ticks > dead_time_ticks) desired_DC_a_ticks += dead_time_ticks; else desired_DC_a_ticks = 0;  // Prevent underflow
	 		if (desired_DC_b_ticks > dead_time_ticks) desired_DC_b_ticks += dead_time_ticks; else desired_DC_b_ticks = 0;  // Prevent underflow
	 		if (desired_DC_c_ticks > dead_time_ticks) desired_DC_c_ticks += dead_time_ticks; else desired_DC_c_ticks = 0;  // Prevent underflow

	 		//Update PWM channels
	 		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, desired_DC_a_ticks);
	 		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, desired_DC_b_ticks);
	 		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, desired_DC_c_ticks);
	  }
	  t += t_step;   //Manual timer counting up
	    	  if (t >= 1/freq_sin) t = 0.0;   //Changes period of manual timer to be based on current frequency
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
