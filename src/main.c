/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "main.h"
#include "stm32f1xx_it.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

volatile uint8_t output_enabled = 0;
volatile uint8_t button_enable_pressed = 0;
volatile uint8_t button_disable_pressed = 0;
volatile uint16_t adc_value = 0;
volatile uint32_t current_frequency = FREQ_MIN_HZ;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void Update_Frequency(uint32_t frequency);
static uint32_t Map_ADC_to_Frequency(uint16_t adc_val);
static void Send_Status_UART(void);
static void Send_Debug_Info(uint32_t frequency, uint32_t prescaler, uint32_t period);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  /* Start ADC calibration */
  HAL_ADCEx_Calibration_Start(&hadc1);

  /* Start ADC conversion */
  HAL_ADC_Start(&hadc1);

  /* Send startup message */
  char msg[] = "STM32F103 Quadrature Generator v1.0\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  /* Initialize timers with default frequency */
  Update_Frequency(current_frequency);

  /* Validate frequency mapping at key points */
  char test_msg[150];
  uint32_t test_freq_min = Map_ADC_to_Frequency(0);      // Should be 1Hz
  uint32_t test_freq_mid = Map_ADC_to_Frequency(2048);   // Should be ~50kHz
  uint32_t test_freq_max = Map_ADC_to_Frequency(4095);   // Should be 100kHz
  
  snprintf(test_msg, sizeof(test_msg), 
           "Freq mapping test - Min: %luHz, Mid: %luHz, Max: %luHz\r\n",
           test_freq_min, test_freq_mid, test_freq_max);
  HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), HAL_MAX_DELAY);

  /* Infinite loop */
  while (1)
  {
    /* Read ADC value for frequency control */
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
      adc_value = HAL_ADC_GetValue(&hadc1);
      uint32_t new_frequency = Map_ADC_to_Frequency(adc_value);
      
      /* Update frequency if changed */
      if (new_frequency != current_frequency)
      {
        current_frequency = new_frequency;
        Update_Frequency(current_frequency);
      }
      
      /* Restart ADC conversion */
      HAL_ADC_Start(&hadc1);
    }

    /* Handle button presses */
    if (button_enable_pressed)
    {
      button_enable_pressed = 0;
      output_enabled = 1;
      HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
      
      /* Start timers for quadrature output */
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    }

    if (button_disable_pressed)
    {
      button_disable_pressed = 0;
      output_enabled = 0;
      HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
      
      /* Stop timers */
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    }

    /* Send status over UART every 500ms */
    static uint32_t last_uart_send = 0;
    if (HAL_GetTick() - last_uart_send > 500)
    {
      Send_Status_UART();
      last_uart_send = HAL_GetTick();
    }

    HAL_Delay(10);
  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;  // Will be dynamically calculated
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;   // Will be updated based on frequency
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;  // 50% duty cycle - will be updated
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  // Configure channel B (TIM1_CH2) as inverse of channel A for true quadrature
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
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
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;  // Will be dynamically calculated
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;   // Will be updated based on frequency
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
  sConfigOC.Pulse = 500;  // 50% duty cycle - will be updated
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  // Configure channel D (TIM2_CH4) as inverse of channel C for true quadrature
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_ENABLE_Pin BUTTON_DISABLE_Pin */
  GPIO_InitStruct.Pin = BUTTON_ENABLE_Pin|BUTTON_DISABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief Update timer frequencies for quadrature output
  * @param frequency: Target frequency in Hz (1Hz - 100kHz)
  * @retval None
  * 
  * This function configures two timers (TIM1 and TIM2) to generate true quadrature
  * output signals:
  * - TIM1 generates channels A & B (PA8, PA9) where B is inverted from A
  * - TIM2 generates channels C & D (PA10, PA11) where D is inverted from C, with 90° phase lead
  * 
  * True quadrature encoding provides directional information and higher resolution
  * for rotary encoders and motor control applications.
  * 
  * The function dynamically calculates prescaler values to maintain optimal
  * timer resolution across the wide frequency range (1Hz - 100kHz).
  */
static void Update_Frequency(uint32_t frequency)
{
  uint32_t timer_clock = 72000000; // 72MHz system clock
  uint32_t prescaler;
  uint32_t period;
  uint32_t pulse;
  
  // Validate frequency range
  if (frequency < FREQ_MIN_HZ) frequency = FREQ_MIN_HZ;
  if (frequency > FREQ_MAX_HZ) frequency = FREQ_MAX_HZ;
  
  // Calculate optimal prescaler and period for the target frequency
  // We want: timer_clock / ((prescaler + 1) * (period + 1)) = frequency
  // Choose prescaler to keep period in a reasonable range (100-65535) for accuracy
  
  for (prescaler = 0; prescaler < 65536; prescaler++)
  {
    period = (timer_clock / ((prescaler + 1) * frequency)) - 1;
    if (period >= 100 && period <= 65535)
    {
      break;
    }
  }
  
  // Fallback for edge cases
  if (prescaler >= 65536)
  {
    prescaler = 65535;
    period = (timer_clock / ((prescaler + 1) * frequency)) - 1;
    if (period > 65535) period = 65535;
    if (period < 1) period = 1;
  }
  
  pulse = period / 2;  // 50% duty cycle
  
  // Stop all PWM outputs before reconfiguration
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
  
  // Reset both timer counters
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  
  // Update TIM1 configuration (channels A & B)
  __HAL_TIM_SET_PRESCALER(&htim1, prescaler);
  __HAL_TIM_SET_AUTORELOAD(&htim1, period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);    // Channel A
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);    // Channel B (inverted polarity)
  
  // Update TIM2 configuration (channels C & D) with same timing parameters
  __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
  __HAL_TIM_SET_AUTORELOAD(&htim2, period);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);    // Channel C
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse);    // Channel D (inverted polarity)
  
  // Restart timers if output is enabled
  if (output_enabled)
  {
    // Reset both timer counters to ensure synchronization
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    
    // Start TIM1 (channels A & B)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    
    // Start TIM2 (channels C & D) with 90-degree phase lead
    // Setting counter to 25% of period creates 90° phase advance
    // This means C & D will lead A & B by 90 degrees (quarter cycle)
    __HAL_TIM_SET_COUNTER(&htim2, period / 4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  }
  
  // Send debug info for frequency changes (uncomment for debugging)
  // Send_Debug_Info(frequency, prescaler, period);
}

/**
  * @brief Map ADC value to frequency range
  * @param adc_val: ADC reading (0-4095)
  * @retval Mapped frequency in Hz (1Hz - 100kHz)
  */
static uint32_t Map_ADC_to_Frequency(uint16_t adc_val)
{
  // Map ADC value (0-4095) to frequency range (1Hz-100kHz)
  uint32_t freq = FREQ_MIN_HZ + ((uint32_t)(adc_val) * (FREQ_MAX_HZ - FREQ_MIN_HZ)) / ADC_MAX_VALUE;
  return freq;
}

/**
  * @brief Send current status over UART
  * @retval None
  */
static void Send_Status_UART(void)
{
  char buffer[100];
  snprintf(buffer, sizeof(buffer), 
           "ADC: %d, Freq: %luHz, Output: %s\r\n", 
           adc_value, current_frequency, output_enabled ? "ON" : "OFF");
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief Send debug timing information over UART
  * @param frequency: Current frequency setting
  * @param prescaler: Timer prescaler value
  * @param period: Timer period value
  * @retval None
  */
static void Send_Debug_Info(uint32_t frequency, uint32_t prescaler, uint32_t period)
{
  char buffer[150];
  uint32_t actual_freq = 72000000 / ((prescaler + 1) * (period + 1));
  snprintf(buffer, sizeof(buffer), 
           "Target: %luHz, Actual: %luHz, Prescaler: %lu, Period: %lu\r\n", 
           frequency, actual_freq, prescaler, period);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */