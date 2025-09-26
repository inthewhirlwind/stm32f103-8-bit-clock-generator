/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * STM32F103C8T6 Quadrature Clock Generator
  * Generates 4 square wave outputs with 90-degree phase shifts
  * Frequency controllable from 10Hz to 1kHz via potentiometer
  * Enable/disable control via buttons, status LED, UART output
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;  /* For Quadrature outputs A and B */
TIM_HandleTypeDef htim4;  /* For Quadrature outputs C and D */
UART_HandleTypeDef huart1;

/* System state variables */
static uint32_t current_frequency = MIN_FREQUENCY_HZ;
static uint8_t output_enabled = 0;
static uint32_t adc_value = 0;
static char uart_buffer[128];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART1_Init(void);
static void Update_Frequency(uint32_t new_frequency);
static void Enable_Quadrature_Output(void);
static void Disable_Quadrature_Output(void);
static void Send_Status_UART(void);

/* Private user code ---------------------------------------------------------*/

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
  MX_TIM4_Init();
  MX_UART1_Init();

  /* Start ADC */
  HAL_ADC_Start(&hadc1);
  
  /* Send startup message */
  sprintf(uart_buffer, "\r\nSTM32F103 Quadrature Clock Generator v1.0\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
  sprintf(uart_buffer, "Frequency Range: %d Hz - %d Hz\r\n", MIN_FREQUENCY_HZ, MAX_FREQUENCY_HZ);
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  /* Infinite loop */
  while (1)
  {
    /* Read ADC value for frequency control */
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {
      adc_value = HAL_ADC_GetValue(&hadc1);
      
      /* Convert ADC value to frequency (10Hz to 1kHz) */
      uint32_t new_frequency = MIN_FREQUENCY_HZ + 
        ((adc_value * (MAX_FREQUENCY_HZ - MIN_FREQUENCY_HZ)) / ADC_MAX_VALUE);
      
      /* Update frequency if changed significantly (avoid jitter) */
      if (abs((int)new_frequency - (int)current_frequency) > 5)
      {
        current_frequency = new_frequency;
        if (output_enabled)
        {
          Update_Frequency(current_frequency);
        }
      }
    }
    
    /* Check enable button */
    if (HAL_GPIO_ReadPin(ENABLE_BTN_GPIO_Port, ENABLE_BTN_Pin) == GPIO_PIN_RESET)
    {
      if (!output_enabled)
      {
        Enable_Quadrature_Output();
        HAL_Delay(200); /* Debounce delay */
      }
    }
    
    /* Check disable button */
    if (HAL_GPIO_ReadPin(DISABLE_BTN_GPIO_Port, DISABLE_BTN_Pin) == GPIO_PIN_RESET)
    {
      if (output_enabled)
      {
        Disable_Quadrature_Output();
        HAL_Delay(200); /* Debounce delay */
      }
    }
    
    /* Send status via UART every 500ms */
    static uint32_t last_uart_time = 0;
    if (HAL_GetTick() - last_uart_time > 500)
    {
      Send_Status_UART();
      last_uart_time = HAL_GetTick();
    }
    
    HAL_Delay(10); /* Main loop delay */
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; /* 8MHz * 9 = 72MHz */
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function - Timer for quadrature outputs A and B
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
  htim1.Init.Prescaler = 71; /* 72MHz / (71+1) = 1MHz */
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999; /* 1MHz / 1000 = 1kHz */
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Configure PWM channels for quadrature outputs A and B */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500; /* 50% duty cycle */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  
  /* Channel 1 - Quadrature output A (0 degree phase) */
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Channel 4 - Quadrature output B (90 degree phase) */
  sConfigOC.Pulse = 750; /* Phase shift for 90 degrees */
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief TIM4 Initialization Function - Timer for quadrature outputs C and D
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71; /* 72MHz / (71+1) = 1MHz */
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999; /* 1MHz / 1000 = 1kHz */
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
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0; /* TIM1 trigger */
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Configure PWM channels for quadrature outputs C and D */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0; /* 180 degree phase shift */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  
  /* Channel 1 - Quadrature output C (180 degree phase) */
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Channel 2 - Quadrature output D (270 degree phase) */
  sConfigOC.Pulse = 250; /* Phase shift for 270 degrees */
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART1_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = UART_BAUD_RATE;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENABLE_BTN_Pin DISABLE_BTN_Pin */
  GPIO_InitStruct.Pin = ENABLE_BTN_Pin|DISABLE_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : POT_ADC_Pin */
  GPIO_InitStruct.Pin = POT_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(POT_ADC_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief Update the frequency of the quadrature output
  * @param new_frequency: New frequency in Hz
  * @retval None
  */
static void Update_Frequency(uint32_t new_frequency)
{
  /* Calculate new period for timers */
  /* Timer clock = 1MHz, so Period = (1MHz / frequency) - 1 */
  uint32_t period = (1000000 / new_frequency) - 1;
  
  /* Update timer periods */
  __HAL_TIM_SET_AUTORELOAD(&htim1, period);
  __HAL_TIM_SET_AUTORELOAD(&htim4, period);
  
  /* Update PWM pulse values to maintain 50% duty cycle and phase relationships */
  uint32_t pulse_50_percent = period / 2;
  uint32_t pulse_25_percent = period / 4;
  uint32_t pulse_75_percent = (3 * period) / 4;
  
  /* Update channel compare values for quadrature phase relationships */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_50_percent);     /* Output A: 0° */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_75_percent);     /* Output B: 90° */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);                   /* Output C: 180° */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse_25_percent);     /* Output D: 270° */
}

/**
  * @brief Enable quadrature output generation
  * @retval None
  */
static void Enable_Quadrature_Output(void)
{
  output_enabled = 1;
  
  /* Turn on status LED */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
  
  /* Set initial frequency */
  Update_Frequency(current_frequency);
  
  /* Start PWM generation on all channels */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  /* Output A */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  /* Output B */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  /* Output C */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);  /* Output D */
  
  /* Start base timers */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim4);
  
  /* Send enable message */
  sprintf(uart_buffer, "Quadrature output ENABLED at %lu Hz\r\n", current_frequency);
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
}

/**
  * @brief Disable quadrature output generation
  * @retval None
  */
static void Disable_Quadrature_Output(void)
{
  output_enabled = 0;
  
  /* Turn off status LED */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
  
  /* Stop PWM generation on all channels */
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);  /* Output A */
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);  /* Output B */
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);  /* Output C */
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);  /* Output D */
  
  /* Stop base timers */
  HAL_TIM_Base_Stop(&htim1);
  HAL_TIM_Base_Stop(&htim4);
  
  /* Send disable message */
  sprintf(uart_buffer, "Quadrature output DISABLED\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
}

/**
  * @brief Send current status via UART
  * @retval None
  */
static void Send_Status_UART(void)
{
  float voltage = (adc_value * 3.3f) / ADC_MAX_VALUE;
  
  sprintf(uart_buffer, "Status: %s | Freq: %lu Hz | ADC: %lu (%.2fV)\r\n", 
          output_enabled ? "ENABLED" : "DISABLED", 
          current_frequency, 
          adc_value, 
          voltage);
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
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
    /* Toggle status LED rapidly to indicate error */
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    HAL_Delay(100);
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